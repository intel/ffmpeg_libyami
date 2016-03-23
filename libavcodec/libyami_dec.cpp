/*
 * Intel Yet Another Media Infrastructure video decoder/encoder
 *
 * Copyright (c) 2016 Intel Corporation
 *     Zhou Yun(yunx.z.zhou@intel.com)
 *     Jun Zhao(jun.zhao@intel.com)
 *
 * This file is part of FFmpeg.
 *
 * FFmpeg is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * FFmpeg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with FFmpeg; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
 */
#include <pthread.h>
#include <unistd.h>
#include <assert.h>
#include <deque>

extern "C" {
#include "avcodec.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "internal.h"
}

#include "VideoDecoderHost.h"
#include "libyami_dec.h"
#include "libyami_utils.h"

using namespace YamiMediaCodec;

typedef struct {
    VideoFrameRawData *video_raw_data;
    VAImage *va_image;
    SharedPtr<VideoFrame> output_frame;
}YamiDecImage;

static int ff_yami_decode_thread_init(YamiDecContext *s)
{
    int ret = 0; 
    if (!s)
        return -1;
    if ((ret = pthread_mutex_init(&s->ctx_mutex, NULL)) < 0)
        return ret;
    if ((ret = pthread_mutex_init(&s->in_mutex, NULL)) < 0)
        return ret;
    if ((ret = pthread_cond_init(&s->in_cond, NULL)) < 0)
        return ret;
    s->decode_status = DECODE_THREAD_NOT_INIT;
    return 0;
}

static int ff_yami_decode_thread_close(YamiDecContext *s)
{
    if (!s)
        return -1;
    pthread_mutex_lock(&s->ctx_mutex);
    while (s->decode_status != DECODE_THREAD_EXIT
           && s->decode_status != DECODE_THREAD_NOT_INIT) { // if decode thread do not create do not loop
        // potential race condition on s->decode_status
        s->decode_status = DECODE_THREAD_GOT_EOS;
        pthread_mutex_unlock(&s->ctx_mutex);
        pthread_cond_signal(&s->in_cond);
        av_usleep(10000);
        pthread_mutex_lock(&s->ctx_mutex);
    }
    pthread_mutex_unlock(&s->ctx_mutex);
    
    pthread_mutex_destroy(&s->in_mutex);
    pthread_cond_destroy(&s->in_cond);
    return 0;
}

static void *ff_yami_decode_thread(void *arg)
{
    AVCodecContext *avctx = (AVCodecContext *)arg;
    YamiDecContext *s = (YamiDecContext *)avctx->priv_data;

    while (1) {
        VideoDecodeBuffer *in_buffer = NULL;
        // deque one input buffer
        av_log(avctx, AV_LOG_VERBOSE, "decode thread runs one cycle start ... \n");
        pthread_mutex_lock(&s->in_mutex);
        if (s->in_queue->empty()) {
            if (s->decode_status == DECODE_THREAD_GOT_EOS) {
                pthread_mutex_unlock(&s->in_mutex);
                break;
            } else {
                av_log(avctx, AV_LOG_VERBOSE, "decode thread wait because s->in_queue is empty\n");
                pthread_cond_wait(&s->in_cond, &s->in_mutex); // wait if no todo frame is available
            }
        }

        if (s->in_queue->empty()) { // may wake up from EOS/Close
            pthread_mutex_unlock(&s->in_mutex);
            continue;
        }

        av_log(avctx, AV_LOG_VERBOSE, "s->in_queue->size()=%ld\n", s->in_queue->size());
        in_buffer = s->in_queue->front();

        pthread_mutex_unlock(&s->in_mutex);

        // decode one input buffer
        av_log(avctx, AV_LOG_VERBOSE, "try to process one input buffer, in_buffer->data=%p, in_buffer->size=%zu\n", in_buffer->data, in_buffer->size);
        Decode_Status status = s->decoder->decode(in_buffer);
        av_log(avctx, AV_LOG_VERBOSE, "decode() status=%d, decode_count_yami=%d render_count %d\n", status, s->decode_count_yami, s->render_count);

        if (DECODE_FORMAT_CHANGE == status) {
            s->format_info = s->decoder->getFormatInfo();
            av_log(avctx, AV_LOG_VERBOSE, "decode format change %dx%d\n", s->format_info->width,s->format_info->height);
            // resend the buffer to decoder
            status = s->decoder->decode(in_buffer);
            av_log(avctx, AV_LOG_VERBOSE, "decode() status=%d\n",status);
            avctx->width = s->format_info->width;
            avctx->height = s->format_info->height;

        }
        if (status < 0 || !s->format_info) {//if format_info is null means current frame decode failed
            av_log(avctx, AV_LOG_ERROR, "decode error %d\n", status);
            break;
        }
        s->decode_count_yami++;
        s->in_queue->pop_front();
        av_free(in_buffer->data);
        av_free(in_buffer);
    }

    av_log(avctx, AV_LOG_VERBOSE, "decode thread exit\n");
    pthread_mutex_lock(&s->ctx_mutex);
    s->decode_status = DECODE_THREAD_EXIT;
    pthread_mutex_unlock(&s->ctx_mutex);
    return NULL;
}

static void ff_yami_recycle_frame(void *opaque, uint8_t *data)
{
    AVCodecContext *avctx = (AVCodecContext *)opaque;
    YamiDecContext *s = (YamiDecContext *)avctx->priv_data;
    YamiDecImage *yami_image = (YamiDecImage *)data;
    if (!s || !s->decoder || !yami_image) // XXX, use shared pointer for s
        return;
    VideoFrameRawData *yami_frame = yami_image->video_raw_data;
    VAImage *va_image = yami_image->va_image;
    pthread_mutex_lock(&s->ctx_mutex);
    /* XXX: should I delete frame buffer?? */
    if (va_image) {
        VADisplay va_display = ff_vaapi_create_display();
        vaUnmapBuffer(va_display, va_image->buf);
        vaDestroyImage(va_display, va_image->image_id);
    }
    yami_image->output_frame.reset();
    av_free(yami_frame);
    av_free(yami_image);
    pthread_mutex_unlock(&s->ctx_mutex);
    av_log(avctx, AV_LOG_DEBUG, "recycle previous frame: %p\n", yami_frame);
}

static int ff_convert_to_frame(AVCodecContext *avctx, YamiDecImage *from, AVFrame *to)
{
    if(!avctx || !from || !to)
        return -1;
    if (avctx->pix_fmt == AV_PIX_FMT_YAMI) {
        to->pts = from->video_raw_data->timeStamp;

        to->width = avctx->width;
        to->height = avctx->height;

        to->format = AV_PIX_FMT_YAMI; 
        to->extended_data = NULL;

        to->extended_data = to->data;
        /* XXX: put the surface id to data[3] */
        to->data[3] = reinterpret_cast<uint8_t *>(from->video_raw_data);

        to->buf[0] = av_buffer_create((uint8_t *)from,
                                      sizeof(YamiDecImage),
                                      ff_yami_recycle_frame, avctx, 0);
    } else {
        int src_linesize[4] = {0};
        uint8_t *src_data[4] = {0};
        if (avctx->pix_fmt == AV_PIX_FMT_YUV420P) {
            src_linesize[0] = from->video_raw_data->pitch[0];
            src_linesize[1] = from->video_raw_data->pitch[1];
            src_linesize[2] = from->video_raw_data->pitch[2];
            uint8_t* yami_data = reinterpret_cast<uint8_t*>(from->video_raw_data->handle);
            src_data[0] = yami_data + from->video_raw_data->offset[0];
            src_data[1] = yami_data + from->video_raw_data->offset[1];
            src_data[2] = yami_data + from->video_raw_data->offset[2];

            to->data[0] = reinterpret_cast<uint8_t*>(src_data[0]);
            to->data[1] = reinterpret_cast<uint8_t*>(src_data[1]);
            to->data[2] = reinterpret_cast<uint8_t*>(src_data[2]);
            to->linesize[0] = src_linesize[0];
            to->linesize[1] = src_linesize[1];
            to->linesize[2] = src_linesize[2];
        } else {
            src_linesize[0] = from->video_raw_data->pitch[0];
            src_linesize[1] = from->video_raw_data->pitch[1];
            src_linesize[2] = from->video_raw_data->pitch[2];
            uint8_t* yami_data = reinterpret_cast<uint8_t*>(from->video_raw_data->handle);
            src_data[0] = yami_data + from->video_raw_data->offset[0];
            src_data[1] = yami_data + from->video_raw_data->offset[1];

            to->data[0] = reinterpret_cast<uint8_t*>(src_data[0]);
            to->data[1] = reinterpret_cast<uint8_t*>(src_data[1]);
            to->linesize[0] = src_linesize[0];
            to->linesize[1] = src_linesize[1];
        }

        to->pkt_pts = AV_NOPTS_VALUE;
        to->pkt_dts = from->video_raw_data->timeStamp;
        to->pts = AV_NOPTS_VALUE;

        to->width = avctx->width;
        to->height = avctx->height;

        to->format = avctx->pix_fmt;
        to->extended_data = NULL;

        to->extended_data = to->data;


        to->buf[0] = av_buffer_create((uint8_t *) from,
                                      sizeof(YamiDecImage),
                                      ff_yami_recycle_frame, avctx, 0);

    }
    return 0;
}

static int
fill_yami_dec_image(AVCodecContext *avctx, YamiDecImage *yami_dec_image, VideoFrameRawData *yami_frame, VAImage *va_image)
{
    yami_dec_image->video_raw_data = yami_frame;

    yami_frame->memoryType = VIDEO_DATA_MEMORY_TYPE_SURFACE_ID;
    yami_frame->width = yami_dec_image->output_frame->crop.width;
    yami_frame->height = yami_dec_image->output_frame->crop.height;
    yami_frame->fourcc = yami_dec_image->output_frame->fourcc;
    VADisplay va_display = ff_vaapi_create_display();
    yami_frame->handle = reinterpret_cast<intptr_t> (va_display);        // planar data has one fd for now, raw data also uses one pointer (+ offset)
    yami_frame->internalID = yami_dec_image->output_frame->surface; // internal identification for image/surface recycle
    yami_frame->timeStamp = yami_dec_image->output_frame->timeStamp;
    yami_frame->flags = yami_dec_image->output_frame->flags;             //see VIDEO_FRAME_FLAGS_XXX

    if (avctx->pix_fmt != AV_PIX_FMT_YAMI) {
    /*map surface to va_image*/
        VAStatus status;
        vaSyncSurface(va_display, yami_frame->internalID);
        yami_frame->memoryType = VIDEO_DATA_MEMORY_TYPE_RAW_POINTER;
        unsigned char *surface_p = NULL;
        if (avctx->pix_fmt == AV_PIX_FMT_NV12) {
            status = vaDeriveImage(va_display, yami_frame->internalID, va_image);
            if (status != VA_STATUS_SUCCESS) {
                av_log(avctx, AV_LOG_ERROR, "vaDeriveImage: %s\n", vaErrorStr(status));
                return -1;
             }
        } else {
            VAImageFormat image_format;
            image_format.fourcc = VA_FOURCC_I420;
            image_format.byte_order = 1;
            image_format.bits_per_pixel = 12;
            status = vaCreateImage(va_display, &image_format, avctx->width, avctx->height, va_image);
            if (status != VA_STATUS_SUCCESS) {
                av_log(avctx, AV_LOG_ERROR, "vaDeriveImage: %s\n", vaErrorStr(status));
                return -1;
             }
             status = vaGetImage(va_display, yami_frame->internalID, 0, 0, avctx->width, avctx->height, va_image->image_id);
             if (status != VA_STATUS_SUCCESS) {
                av_log(avctx, AV_LOG_ERROR, "vaDeriveImage: %s\n", vaErrorStr(status));
                return -1;
             }
        }
        status = vaMapBuffer(va_display, va_image->buf, (void **) &surface_p);
        if (status != VA_STATUS_SUCCESS) {
            av_log(avctx, AV_LOG_ERROR, "vaDeriveImage: %s\n", vaErrorStr(status));
            return -1;
        }
        yami_frame->handle = reinterpret_cast<intptr_t> (surface_p);
        yami_frame->pitch[0] = va_image->pitches[0];
        yami_frame->pitch[1] = va_image->pitches[1];
        yami_frame->pitch[2] = va_image->pitches[2];
        yami_frame->offset[0] = va_image->offsets[0];
        yami_frame->offset[1] = va_image->offsets[1];
        yami_frame->offset[2] = va_image->offsets[2];

        yami_dec_image->va_image = va_image;

    }
    return 0;
}

int yami_dec_init(AVCodecContext *avctx, const char *mime_type)
{
    YamiDecContext *s = (YamiDecContext *)avctx->priv_data;
    Decode_Status status;

    s->decoder = NULL;
    enum AVPixelFormat pix_fmts[4] =
        {
            AV_PIX_FMT_YAMI,
            AV_PIX_FMT_NV12,
            AV_PIX_FMT_YUV420P,
            AV_PIX_FMT_NONE
        };
    if (avctx->pix_fmt == AV_PIX_FMT_NONE) {
        int ret = ff_get_format(avctx, pix_fmts);
        if (ret < 0)
            return ret;

        avctx->pix_fmt = (AVPixelFormat)ret;
    }
    VADisplay va_display = ff_vaapi_create_display();
    if (!va_display) {
        av_log(avctx, AV_LOG_ERROR, "\nfail to create %s display\n", mime_type);
        return AVERROR_BUG;
    }
    av_log(avctx, AV_LOG_VERBOSE, "yami_dec_init\n");
    s->decoder = createVideoDecoder(mime_type);
    if (!s->decoder) {
        av_log(avctx, AV_LOG_ERROR, "fail to create %s decoder\n", mime_type);
        return AVERROR_BUG;
    }

    NativeDisplay native_display;
    native_display.type = NATIVE_DISPLAY_VA;

    native_display.handle = (intptr_t)va_display;
    s->decoder->setNativeDisplay(&native_display);

    VideoConfigBuffer config_buffer;
    memset(&config_buffer, 0, sizeof(VideoConfigBuffer));
    if (avctx->extradata && avctx->extradata_size && avctx->extradata[0] == 1) {
        config_buffer.data = avctx->extradata;
        config_buffer.size = avctx->extradata_size;
    }
    config_buffer.profile = VAProfileNone;
    status = s->decoder->start(&config_buffer);
    if (status != DECODE_SUCCESS) {
        av_log(avctx, AV_LOG_ERROR, "yami decoder fail to start\n");
        return AVERROR_BUG;
    }

    s->in_queue = new std::deque<VideoDecodeBuffer*>;

#if HAVE_PTHREADS
    if (ff_yami_decode_thread_init(s) < 0)
        return AVERROR(ENOMEM);
#else
    av_log(avctx, AV_LOG_ERROR, "pthread libaray must be supported\n");
    return AVERROR(ENOSYS);
#endif
    
    s->decode_count = 0;
    s->decode_count_yami = 0;
    s->render_count = 0;

    return 0;
}

int yami_dec_frame(AVCodecContext *avctx, void *data,
                   int *got_frame, AVPacket *avpkt)
{
    YamiDecContext *s = (YamiDecContext *)avctx->priv_data;
    if (!s || !s->decoder)
        return -1;
    VideoDecodeBuffer *in_buffer = NULL;
    Decode_Status status = RENDER_NO_AVAILABLE_FRAME;
    VideoFrameRawData *yami_frame = NULL;
    YamiDecImage *yami_dec_image =  NULL;
    AVFrame *frame = (AVFrame *)data;

    av_log(avctx, AV_LOG_VERBOSE, "yami_dec_frame\n");

    // append avpkt to input buffer queue
    in_buffer = (VideoDecodeBuffer *)av_mallocz(sizeof(VideoDecodeBuffer));
    if (!in_buffer)
        return AVERROR(ENOMEM);
    /* avoid avpkt free and data is pointer */
    in_buffer->data = (uint8_t *)av_mallocz(avpkt->size);
    if (!in_buffer->data)
        return AVERROR(ENOMEM);
    memcpy(in_buffer->data, avpkt->data, avpkt->size);
    in_buffer->size = avpkt->size;
    in_buffer->timeStamp = avpkt->pts;

    while (s->decode_status < DECODE_THREAD_GOT_EOS) { // we need enque eos buffer more than once
        pthread_mutex_lock(&s->in_mutex);
        if (s->in_queue->size() < DECODE_QUEUE_SIZE) {
            s->in_queue->push_back(in_buffer);
            av_log(avctx, AV_LOG_VERBOSE, "wakeup decode thread ...\n");
            pthread_cond_signal(&s->in_cond);
            pthread_mutex_unlock(&s->in_mutex);
            break;
        }
        pthread_mutex_unlock(&s->in_mutex);

        av_log(avctx, AV_LOG_DEBUG,
               "s->in_queue->size()=%ld, s->decode_count=%d, s->decode_count_yami=%d, too many buffer are under decoding, wait ...\n",
               s->in_queue->size(), s->decode_count, s->decode_count_yami);
        av_usleep(1000);
    };
    s->decode_count++;

    // decode thread status update
    pthread_mutex_lock(&s->ctx_mutex);
    switch (s->decode_status) {
    case DECODE_THREAD_NOT_INIT:
    case DECODE_THREAD_EXIT:
        if (avpkt->data && avpkt->size) {
            s->decode_status = DECODE_THREAD_RUNING;
            pthread_create(&s->decode_thread_id, NULL, &ff_yami_decode_thread, avctx);
        }
        break;
    case DECODE_THREAD_RUNING:
        if (!avpkt->data || !avpkt->size)
            s->decode_status = DECODE_THREAD_GOT_EOS; // call releaseLock for seek
        break;
    case DECODE_THREAD_GOT_EOS:
        if (s->in_queue->empty())
            s->decode_status = DECODE_THREAD_NOT_INIT;
        break;
    default:
        break;
    }
    pthread_mutex_unlock(&s->ctx_mutex);

    // get an output buffer from yami
    yami_frame = (VideoFrameRawData *)av_mallocz(sizeof(VideoFrameRawData));
    if (!yami_frame)
        return AVERROR(ENOMEM);

    do {
        if (!s->format_info) {
            av_usleep(10000);
            continue;
        }

        yami_dec_image = (YamiDecImage *)av_mallocz(sizeof(YamiDecImage));
        if (!yami_dec_image) {
            av_free(yami_frame);
            yami_dec_image->output_frame.reset();
            return AVERROR(ENOMEM);
        }
        do{
            yami_dec_image->output_frame = s->decoder->getOutput();
            av_log(avctx, AV_LOG_DEBUG, "getoutput() status=%d\n", status);
        } while (!avpkt->data && !yami_dec_image->output_frame && s->in_queue->size() > 0);
        if (yami_dec_image->output_frame) {
            VAImage *va_image =  (VAImage *)av_mallocz(sizeof(VAImage));
            if (fill_yami_dec_image(avctx, yami_dec_image, yami_frame, va_image) < 0) {
                av_free(va_image);
                av_free(yami_frame);
                yami_dec_image->output_frame.reset();
                return AVERROR(ENOMEM);
            };

            status = RENDER_SUCCESS;
            break;
        }
        *got_frame = 0;
        av_free(yami_frame);
        av_free(yami_dec_image);
        return avpkt->size;
    } while (s->decode_status == DECODE_THREAD_RUNING);

    if (status != RENDER_SUCCESS) {
        av_log(avctx, AV_LOG_VERBOSE, "after processed EOS, return\n");
        return avpkt->size;
    }

    // process the output frame
    if (ff_convert_to_frame(avctx, yami_dec_image, frame) < 0)
        av_log(avctx, AV_LOG_VERBOSE, "yami frame convert av_frame failed\n");;

    *got_frame = 1;

    s->render_count++;
    assert(data->buf[0] || !*got_frame);
    av_log(avctx, AV_LOG_VERBOSE,
           "decode_count_yami=%d, decode_count=%d, render_count=%d\n",
           s->decode_count_yami, s->decode_count, s->render_count);

    return avpkt->size;
}

int yami_dec_close(AVCodecContext *avctx)
{
    YamiDecContext *s = (YamiDecContext *)avctx->priv_data;
    
    ff_yami_decode_thread_close(s);
    if (s->decoder) {
        s->decoder->stop();
        releaseVideoDecoder(s->decoder);
        s->decoder = NULL;
    }

    while (!s->in_queue->empty()) {
        VideoDecodeBuffer *in_buffer = s->in_queue->front();
        s->in_queue->pop_front();
        av_free(in_buffer->data);
        av_free(in_buffer);
    }
    delete s->in_queue;
    av_log(avctx, AV_LOG_VERBOSE, "yami_dec_close\n");

    return 0;
}

#if CONFIG_LIBYAMI_H264_DECODER
static av_cold int yami_dec_h264_init(AVCodecContext *avctx)
{
    return yami_dec_init(avctx, YAMI_MIME_H264);
}

static int yami_dec_h264_frame(AVCodecContext *avctx, void *data,
                               int *got_frame, AVPacket *avpkt)
{
    return yami_dec_frame(avctx, data,got_frame, avpkt);
}

static av_cold int yami_dec_h264_close(AVCodecContext *avctx)
{
    return yami_dec_close(avctx);
}

AVCodec ff_libyami_h264_decoder = {
    .name                  = "libyami_h264",
    .long_name             = NULL_IF_CONFIG_SMALL("libyami H.264 decoder"),
    .type                  = AVMEDIA_TYPE_VIDEO,
    .id                    = AV_CODEC_ID_H264,
    .capabilities          = CODEC_CAP_DELAY, // it is not necessary to support multi-threads
    .supported_framerates  = NULL,
    .pix_fmts              = (const enum AVPixelFormat[]) { AV_PIX_FMT_YAMI ,
                                                            AV_PIX_FMT_NV12 ,
                                                            AV_PIX_FMT_YUV420P,
                                                            AV_PIX_FMT_NONE},
    .supported_samplerates = NULL,
    .sample_fmts           = NULL,
    .channel_layouts       = NULL,
#if FF_API_LOWRES
    .max_lowres            = 0,
#endif
    .priv_class            = NULL,
    .profiles              = NULL,
    .priv_data_size        = sizeof(YamiDecContext),
    .next                  = NULL,
    .init_thread_copy      = NULL,
    .update_thread_context = NULL,
    .defaults              = NULL,
    .init_static_data      = NULL,
    .init                  = yami_dec_h264_init,
    .encode_sub            = NULL,
    .encode2               = NULL,
    .decode                = yami_dec_h264_frame,
    .close                 = yami_dec_h264_close,
    .flush                 = NULL, // TODO, add it
    .caps_internal         = FF_CODEC_CAP_SETS_PKT_DTS
};
#endif

#if CONFIG_LIBYAMI_HEVC_DECODER
static av_cold int yami_dec_hevc_init(AVCodecContext *avctx)
{
    return yami_dec_init(avctx, YAMI_MIME_H265);
}

static int yami_dec_hevc_frame(AVCodecContext *avctx, void *data,
                               int *got_frame, AVPacket *avpkt)
{
    return yami_dec_frame(avctx, data, got_frame, avpkt);
}

static av_cold int yami_dec_hevc_close(AVCodecContext *avctx)
{
    return yami_dec_close(avctx);
}

AVCodec ff_libyami_hevc_decoder = {
    .name                  = "libyami_hevc",
    .long_name             = NULL_IF_CONFIG_SMALL("libyami H.265 decoder"),
    .type                  = AVMEDIA_TYPE_VIDEO,
    .id                    = AV_CODEC_ID_HEVC,
    .capabilities          = CODEC_CAP_DELAY, // it is not necessary to support multi-threads
    .supported_framerates  = NULL,
    .pix_fmts              = (const enum AVPixelFormat[]) { AV_PIX_FMT_YAMI ,
                                                            AV_PIX_FMT_NV12 ,
                                                            AV_PIX_FMT_YUV420P,
                                                            AV_PIX_FMT_NONE},
    .supported_samplerates = NULL,
    .sample_fmts           = NULL,
    .channel_layouts       = NULL,
#if FF_API_LOWRES
    .max_lowres            = 0,
#endif
    .priv_class            = NULL,
    .profiles              = NULL,
    .priv_data_size        = sizeof(YamiDecContext),
    .next                  = NULL,
    .init_thread_copy      = NULL,
    .update_thread_context = NULL,
    .defaults              = NULL,
    .init_static_data      = NULL,
    .init                  = yami_dec_hevc_init,
    .encode_sub            = NULL,
    .encode2               = NULL,
    .decode                = yami_dec_hevc_frame,
    .close                 = yami_dec_hevc_close,
    .flush                 = NULL, // TODO, add it
    .caps_internal         = FF_CODEC_CAP_SETS_PKT_DTS
};
#endif

#if CONFIG_LIBYAMI_VP8_DECODER
static av_cold int yami_dec_vp8_init(AVCodecContext *avctx)
{
    return yami_dec_init(avctx, YAMI_MIME_VP8);
}

static int yami_dec_vp8_frame(AVCodecContext *avctx, void *data,
                               int *got_frame, AVPacket *avpkt)
{
    return yami_dec_frame(avctx, data,got_frame, avpkt);
}

static av_cold int yami_dec_vp8_close(AVCodecContext *avctx)
{
    return yami_dec_close(avctx);
}

AVCodec ff_libyami_vp8_decoder = {
    .name                  = "libyami_vp8",
    .long_name             = NULL_IF_CONFIG_SMALL("libyami VP8 decoder"),
    .type                  = AVMEDIA_TYPE_VIDEO,
    .id                    = AV_CODEC_ID_VP8,
    .capabilities          = CODEC_CAP_DELAY, // it is not necessary to support multi-threads
    .supported_framerates  = NULL,
    .pix_fmts              = (const enum AVPixelFormat[]) { AV_PIX_FMT_YAMI ,
                                                            AV_PIX_FMT_NV12 ,
                                                            AV_PIX_FMT_YUV420P,
                                                            AV_PIX_FMT_NONE},
    .supported_samplerates = NULL,
    .sample_fmts           = NULL,
    .channel_layouts       = NULL,
#if FF_API_LOWRES
    .max_lowres            = 0,
#endif
    .priv_class            = NULL,
    .profiles              = NULL,
    .priv_data_size        = sizeof(YamiDecContext),
    .next                  = NULL,
    .init_thread_copy      = NULL,
    .update_thread_context = NULL,
    .defaults              = NULL,
    .init_static_data      = NULL,
    .init                  = yami_dec_vp8_init,
    .encode_sub            = NULL,
    .encode2               = NULL,
    .decode                = yami_dec_vp8_frame,
    .close                 = yami_dec_vp8_close,
    .flush                 = NULL, // TODO, add it
    .caps_internal         = FF_CODEC_CAP_SETS_PKT_DTS
};
#endif
