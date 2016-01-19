/*
 * libyami.cpp -- h264 decoder uses libyami
 *
 *  Copyright (C) 2014 Intel Corporation
 *    Author: Zhao Halley<halley.zhao@intel.com>
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
#include "internal.h"
}
#include "VideoDecoderHost.h"
#include "VideoEncoderHost.h"


using namespace YamiMediaCodec;
#ifndef VA_FOURCC_I420
#define VA_FOURCC_I420 VA_FOURCC('I','4','2','0')
#endif
#define PRINT_DECODE_THREAD(format, ...)  av_log(avctx, AV_LOG_VERBOSE, "## decode thread ## line:%4d " format, __LINE__, ##__VA_ARGS__)

typedef enum {
    DECODE_THREAD_NOT_INIT = 0,
    DECODE_THREAD_RUNING,
    DECODE_THREAD_GOT_EOS,
    DECODE_THREAD_EXIT,
} DecodeThreadStatus;

#define DEBUG_LIBYAMI 0

#define QUEUE_MAX_SIZE 8

struct YamiDecContext {
    AVCodecContext *avctx;
    pthread_mutex_t mutex_; // mutex for decoder->getOutput() and YamiContext itself update (decode_status, etc)

    IVideoDecoder *decoder;
    VideoDataMemoryType output_type;
    const VideoFormatInfo *format_info;
    pthread_t decode_thread_id;
    std::deque<VideoDecodeBuffer*> *in_queue;
    pthread_mutex_t in_mutex; // mutex for in_queue
    pthread_cond_t in_cond;   // decode thread condition wait
    DecodeThreadStatus decode_status;

    // debug use
    int decode_count;
    int decode_count_yami;
    int render_count;
};

#include <fcntl.h>
#include <unistd.h>
#include <va/va_drm.h>

static VADisplay createVADisplay(void)
{
    static VADisplay vadisplay = NULL;

    int fd = open("/dev/dri/card0", O_RDWR);
    if (fd < 0) {
        printf("open card0 failed");
        return NULL;
    }

    if (!vadisplay) {
        vadisplay = vaGetDisplayDRM(fd);
        int majorVersion, minorVersion;
        VAStatus vaStatus = vaInitialize(vadisplay, &majorVersion, &minorVersion);
        if (vaStatus != VA_STATUS_SUCCESS) {
            printf("va init failed, status =  %d", vaStatus);
            close(fd);
            return NULL;
        }
        return vadisplay;
    } else {
        return vadisplay;
    }
}

static av_cold int yami_dec_init(AVCodecContext *avctx)
{
    YamiDecContext *s = (YamiDecContext *) avctx->priv_data;
    Decode_Status status;

    av_log(avctx, AV_LOG_VERBOSE, "yami_dec_init\n");
    s->decoder = createVideoDecoder(YAMI_MIME_H264);
    if (!s->decoder) {
        av_log(avctx, AV_LOG_ERROR, "fail to create libyami h264 decoder\n");
        return -1;
    }

    NativeDisplay native_display;
    native_display.type = NATIVE_DISPLAY_VA;
    VADisplay m_display = createVADisplay();
    native_display.handle = (intptr_t)m_display;
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
        av_log(avctx, AV_LOG_ERROR, "yami h264 decoder fail to start\n");
        return -1;
    }

    switch (avctx->coder_type) {
    case 0:
        s->output_type = VIDEO_DATA_MEMORY_TYPE_RAW_POINTER;
        break;
    case 1:
        s->output_type = VIDEO_DATA_MEMORY_TYPE_DRM_NAME;
        break;
    case 2:
        s->output_type = VIDEO_DATA_MEMORY_TYPE_DMA_BUF;
        break;
    default:
        av_log(avctx, AV_LOG_ERROR, "unknown output frame type: %d",
               avctx->coder_type);
        break;
    }

    s->in_queue = new std::deque<VideoDecodeBuffer*>;
    pthread_mutex_init(&s->mutex_, NULL);
    pthread_mutex_init(&s->in_mutex, NULL);
    pthread_cond_init(&s->in_cond, NULL);
    s->decode_status = DECODE_THREAD_NOT_INIT;

    s->decode_count = 0;
    s->decode_count_yami = 0;
    s->render_count = 0;

#ifdef DEBUG_LIBYAMI
    FILE *fp1 = fopen("./test.yuv", "wb");
    fclose(fp1);
#endif
    return 0;
}

static void* decodeThread(void *arg)
{
    AVCodecContext *avctx = (AVCodecContext *)arg;
    YamiDecContext *s = (YamiDecContext *)avctx->priv_data;

    while (1) {
        VideoDecodeBuffer *in_buffer = NULL;
        // deque one input buffer
        PRINT_DECODE_THREAD("decode thread runs one cycle start ... \n");
        pthread_mutex_lock(&s->in_mutex);
        if (s->in_queue->empty()) {
            if (s->decode_status == DECODE_THREAD_GOT_EOS) {
                pthread_mutex_unlock(&s->in_mutex);
                break;
            } else {
                PRINT_DECODE_THREAD("decode thread wait because s->in_queue is empty\n");
                pthread_cond_wait(&s->in_cond, &s->in_mutex); // wait if no todo frame is available
            }
        }

        if (s->in_queue->empty()) { // may wake up from EOS/Close
            pthread_mutex_unlock(&s->in_mutex);
            continue;
        }

        PRINT_DECODE_THREAD("s->in_queue->size()=%ld\n", s->in_queue->size());
        in_buffer = s->in_queue->front();
        s->in_queue->pop_front();
        pthread_mutex_unlock(&s->in_mutex);

        // decode one input buffer
        PRINT_DECODE_THREAD("try to process one input buffer, in_buffer->data=%p, in_buffer->size=%d\n", in_buffer->data, in_buffer->size);
        Decode_Status status = s->decoder->decode(in_buffer);
        PRINT_DECODE_THREAD("decode() status=%d, decode_count_yami=%d\n", status, s->decode_count_yami);

        if (DECODE_FORMAT_CHANGE == status) {
            s->format_info = s->decoder->getFormatInfo();
            PRINT_DECODE_THREAD("decode format change %dx%d\n",s->format_info->width,s->format_info->height);
            // resend the buffer
            status = s->decoder->decode(in_buffer);
            PRINT_DECODE_THREAD("decode() status=%d\n",status);
            avctx->width = s->format_info->width;
            avctx->height = s->format_info->height;
            avctx->pix_fmt = AV_PIX_FMT_YUV420P;
        }
        if (status < 0){
            av_log(avctx, AV_LOG_ERROR, "decode error %d\n",status);
        }
        s->decode_count_yami++;
        av_free(in_buffer->data);
        av_free(in_buffer);
    }

    PRINT_DECODE_THREAD("decode thread exit\n");
    pthread_mutex_lock(&s->mutex_);
    s->decode_status = DECODE_THREAD_EXIT;
    pthread_mutex_unlock(&s->mutex_);
    return NULL;
}

static void yami_recycle_frame(void *opaque, uint8_t *data)
{
    AVCodecContext *avctx = (AVCodecContext *)opaque;
    YamiDecContext *s = (YamiDecContext *)avctx->priv_data;
    VideoRenderBuffer *frame = (VideoRenderBuffer *)data;

    if (!s || !s->decoder || !frame) // XXX, use shared pointer for s
        return;
    pthread_mutex_lock(&s->mutex_);
    s->decoder->renderDone(frame);
    pthread_mutex_unlock(&s->mutex_);
    av_log(avctx, AV_LOG_DEBUG, "recycle previous frame: %p\n", frame);
}

static int yami_dec_frame(AVCodecContext *avctx, void *data,
                             int *got_frame, AVPacket *avpkt)
{
    YamiDecContext *s = (YamiDecContext *)avctx->priv_data;
    VideoDecodeBuffer *in_buffer = NULL;
    Decode_Status status = RENDER_NO_AVAILABLE_FRAME;
    //VideoFrameRawData *yami_frame = NULL;
    VideoRenderBuffer *yami_frame = NULL;
    AVFrame *frame = (AVFrame *) data;

    av_log(avctx, AV_LOG_VERBOSE, "yami_decode_frame\n");

    // append avpkt to input buffer queue
    in_buffer = (VideoDecodeBuffer *)av_mallocz(sizeof(VideoDecodeBuffer));
    /*in_buffer->data = avpkt->data;*/
    /*avoid avpkt free and data is pointer*/
    in_buffer->data = (uint8_t *)av_mallocz(avpkt->size);
    memcpy(in_buffer->data, avpkt->data, avpkt->size);
    in_buffer->size = avpkt->size;
    in_buffer->timeStamp = avpkt->pts;

    if (avctx->extradata && avctx->extradata_size && avctx->extradata[0] == 1)
        in_buffer->flag |= IS_AVCC;

    while (s->decode_status < DECODE_THREAD_GOT_EOS) { // we need enque eos buffer more than once
        pthread_mutex_lock(&s->in_mutex);
        if (s->in_queue->size()<QUEUE_MAX_SIZE) {
            s->in_queue->push_back(in_buffer);
            av_log(avctx, AV_LOG_VERBOSE, "wakeup decode thread ...\n");
            pthread_cond_signal(&s->in_cond);
            pthread_mutex_unlock(&s->in_mutex);
            break;
        }
        pthread_mutex_unlock(&s->in_mutex);

        av_log(avctx, AV_LOG_DEBUG, "s->in_queue->size()=%ld, s->decode_count=%d, s->decode_count_yami=%d, too many buffer are under decoding, wait ...\n",
        s->in_queue->size(), s->decode_count, s->decode_count_yami);
        usleep(1000);
    };
    s->decode_count++;

    // decode thread status update
    pthread_mutex_lock(&s->mutex_);
    switch (s->decode_status) {
    case DECODE_THREAD_NOT_INIT:
    case DECODE_THREAD_EXIT:
        if (avpkt->data && avpkt->size) {
            s->decode_status = DECODE_THREAD_RUNING;
            pthread_create(&s->decode_thread_id, NULL, &decodeThread, avctx);
        }
        break;
    case DECODE_THREAD_RUNING:
        if (!avpkt->data || ! avpkt->size)
            s->decode_status = DECODE_THREAD_GOT_EOS; // call releaseLock for seek
        break;
    case DECODE_THREAD_GOT_EOS:
        if (s->in_queue->empty())
            s->decode_status = DECODE_THREAD_NOT_INIT;
        break;
    default:
        break;
    }
    pthread_mutex_unlock(&s->mutex_);

    // get an output buffer from yami
    do {
        if (!s->format_info) {
            usleep(10000);
            continue;
        }

        yami_frame = (VideoRenderBuffer *)s->decoder->getOutput(false);
        av_log(avctx, AV_LOG_DEBUG, "getoutput() status=%d\n", status);
        if (yami_frame) {
            status = RENDER_SUCCESS;
            break;
        }

        *got_frame = 0;
        return avpkt->size;
    } while (s->decode_status == DECODE_THREAD_RUNING);

    if (status != RENDER_SUCCESS) {
        av_log(avctx, AV_LOG_VERBOSE, "after processed EOS, return\n");
        return avpkt->size;
    }

#if  DEBUG_LIBYAMI
    VAImage vaImage;
    unsigned char *surface_p = NULL;
    vaDeriveImage(yami_frame->display, yami_frame->surface, &vaImage);
    vaMapBuffer(yami_frame->display, vaImage.buf, (void **) &surface_p);
    unsigned char *swap_buf_ = (unsigned char*) malloc(
        avctx->height * avctx->width * 3 / 2);

    unsigned char *sY = surface_p + vaImage.offsets[0];
    unsigned char *sUV = surface_p + vaImage.offsets[1];
    unsigned char *dY = swap_buf_;
    unsigned char *dU = dY + avctx->height * avctx->width;
    unsigned char *dV = dU + avctx->height * avctx->width / 4;

    for (unsigned int j = 0; j<avctx->height >> 1; j++) {
        memcpy(dY, sY, avctx->width);
        dY += avctx->width;
        sY += vaImage.pitches[0];
        memcpy(dY, sY, avctx->width);
        dY += avctx->width;
        sY += vaImage.pitches[0];

        for (unsigned int i = 0; i < avctx->width >> 1; i++) {
            *dU++ = *sUV++;
            *dV++ = *sUV++;
        }

        sUV += vaImage.pitches[0] - avctx->width;
    }
    FILE *fp1 = fopen("./test.yuv", "ab+");
    fwrite(swap_buf_, 1, avctx->height * avctx->width * 3 / 2, fp1);
    fclose(fp1);
    free(swap_buf_);
    vaUnmapBuffer(yami_frame->display, vaImage.buf);
    vaDestroyImage(yami_frame->display, vaImage.image_id);
#endif

    // process the output frame
    if (s->output_type == VIDEO_DATA_MEMORY_TYPE_DRM_NAME
        || s->output_type == VIDEO_DATA_MEMORY_TYPE_DMA_BUF) {
        frame = (AVFrame *)data;
        ((AVFrame *) data)->extended_data = ((AVFrame *) data)->data;
    } else {
        AVFrame *vframe = av_frame_alloc();

        vframe->pts = yami_frame->timeStamp;


        vframe->width = avctx->width;
        vframe->height = avctx->height;

        vframe->format = AV_PIX_FMT_YUV420P; /* FIXME */
        vframe->extended_data = NULL;

        *(AVFrame *)data = *vframe;
        ((AVFrame *)data)->extended_data = ((AVFrame *)data)->data;
    }

    *got_frame = 1;

    /* XXX: put the surface id to data[3] */
    frame->data[3] = reinterpret_cast<uint8_t *>(yami_frame);
    frame->buf[0] = av_buffer_create((uint8_t *) frame->data[3],
                                     sizeof(VideoRenderBuffer),
                                     yami_recycle_frame, avctx, 0);
    s->render_count++;
    assert(data->buf[0] || !*got_frame);
    av_log(avctx, AV_LOG_VERBOSE,
           "decode_count_yami=%d, decode_count=%d, render_count=%d\n",
           s->decode_count_yami, s->decode_count, s->render_count);

    return avpkt->size;
}

static av_cold int yami_dec_close(AVCodecContext *avctx)
{
    YamiDecContext *s = (YamiDecContext *)avctx->priv_data;

    pthread_mutex_lock(&s->mutex_);
    while (s->decode_status != DECODE_THREAD_EXIT) {
        // potential race condition on s->decode_status
        s->decode_status = DECODE_THREAD_GOT_EOS;
        pthread_mutex_unlock(&s->mutex_);
        pthread_cond_signal(&s->in_cond);
        usleep(10000);
        pthread_mutex_lock(&s->mutex_);
    }
    pthread_mutex_unlock(&s->mutex_);

    if (s->decoder) {
        s->decoder->stop();
        releaseVideoDecoder(s->decoder);
        s->decoder = NULL;
    }

    pthread_mutex_destroy(&s->in_mutex);
    pthread_cond_destroy(&s->in_cond);
    while (!s->in_queue->empty()) {
        VideoDecodeBuffer *in_buffer = s->in_queue->front();
        s->in_queue->pop_front();
        av_free(in_buffer->data);
        av_free(in_buffer);
    }
    delete s->in_queue;
    av_log(avctx, AV_LOG_VERBOSE, "yami_close\n");

    return 0;
}

AVCodec ff_libyami_h264_decoder = {
    .name                  = "libyami_h264",
    .long_name             = NULL_IF_CONFIG_SMALL("libyami H.264"),
    .type                  = AVMEDIA_TYPE_VIDEO,
    .id                    = AV_CODEC_ID_H264,
    .capabilities          = CODEC_CAP_DELAY, // it is not necessary to support multi-threads
    .supported_framerates  = NULL,
    .pix_fmts              = NULL,
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
    .init                  = yami_dec_init,
    .encode_sub            = NULL,
    .encode2               = NULL,
    .decode                = yami_dec_frame,
    .close                 = yami_dec_close,
    .flush                 = NULL, // TODO, add it
    .caps_internal         = FF_CODEC_CAP_SETS_PKT_DTS
};

struct YamiEncContext {
    AVCodecContext *avctx;

    IVideoEncoder *encoder;
    VideoEncOutputBuffer outputBuffer;

    uint8_t *m_buffer;
    uint32_t m_frameSize;

    uint32_t maxOutSize;

    // debug use
    int encode_count;
    int encode_count_yami;
    int render_count;
};

static bool createOutputBuffer(VideoEncOutputBuffer *outputBuffer, int maxOutSize)
{
    outputBuffer->data = static_cast<uint8_t *>(malloc(maxOutSize));
    if (!outputBuffer->data)
        return false;
    outputBuffer->bufferSize = maxOutSize;
    outputBuffer->format = OUTPUT_EVERYTHING;

    return true;
}

static av_cold int yami_enc_init(AVCodecContext *avctx)
{
    YamiEncContext *s = (YamiEncContext *)avctx->priv_data;
    Decode_Status status;

    av_log(avctx, AV_LOG_VERBOSE, "yami_init h264 encoder\n");
    s->encoder = createVideoEncoder(YAMI_MIME_H264);
    if (!s->encoder) {
        av_log(avctx, AV_LOG_ERROR, "fail to create libyami h264 encoder\n");
        return -1;
    }

    NativeDisplay native_display;
    native_display.type = NATIVE_DISPLAY_VA;
    VADisplay m_display = createVADisplay();
    native_display.handle = (intptr_t)m_display;
    s->encoder->setNativeDisplay(&native_display);

    // configure encoding parameters
    VideoParamsCommon encVideoParams;
    encVideoParams.size = sizeof(VideoParamsCommon);
    s->encoder->getParameters(VideoParamsTypeCommon, &encVideoParams);

    encVideoParams.resolution.width  = avctx->width;
    encVideoParams.resolution.height = avctx->height;

    //frame rate parameters.
    // FIXME: used the default encodec params, need to get them from avctx
    encVideoParams.frameRate.frameRateDenom = 1;
    encVideoParams.frameRate.frameRateNum = 30;

    //picture type and bitrate
    encVideoParams.intraPeriod = 30;
    encVideoParams.ipPeriod = 1;
//  encVideoParams.rcParams.bitRate = 6000000;
    encVideoParams.rcParams.initQP = 26;
    encVideoParams.rcMode = RATE_CONTROL_CQP;
    encVideoParams.level = 40;

    // s->encoder->setEncoderParameters(&encVideoParams);
    encVideoParams.size = sizeof(VideoParamsCommon);
    s->encoder->setParameters(VideoParamsTypeCommon, &encVideoParams);

    VideoConfigAVCStreamFormat streamFormat;
    streamFormat.size = sizeof(VideoConfigAVCStreamFormat);
    streamFormat.streamFormat = AVC_STREAM_FORMAT_ANNEXB;
    s->encoder->setParameters(VideoConfigTypeAVCStreamFormat, &streamFormat);

    status = s->encoder->start();
    assert(status == ENCODE_SUCCESS);

    // init output buffer
    s->encoder->getMaxOutSize(&(s->maxOutSize));

    s->encode_count = 0;
    s->encode_count_yami = 0;
    s->render_count = 0;

    return 0;
}

static int yami_enc_frame(AVCodecContext *avctx, AVPacket *pkt,
                          const AVFrame *frame, int *got_packet)
{
    YamiEncContext *s = (YamiEncContext *)avctx->priv_data;
    Decode_Status status;
    if (!frame)
    	return 0;
    VideoRenderBuffer *buffer = (VideoRenderBuffer *)frame->data[3];
    SharedPtr<VideoFrame> yami_frame;

    if (buffer) {
        yami_frame.reset(new VideoFrame);
        yami_frame->surface = (intptr_t)buffer->surface; /* XXX: get decoded surface */
        yami_frame->timeStamp = buffer->timeStamp;
        yami_frame->crop.x = 0;
        yami_frame->crop.y = 0;
        yami_frame->crop.width = avctx->width;
        yami_frame->crop.height = avctx->height;
        yami_frame->flags = 0;
    } else
        return 0;

    status = s->encoder->encode(yami_frame);

    if (!createOutputBuffer(&s->outputBuffer, s->maxOutSize)) {
        fprintf (stderr, "fail to create output\n");

        return -1;
    }

    status = s->encoder->getOutput(&s->outputBuffer, true);
    if (status != ENCODE_SUCCESS)
        return 0;

    s->encode_count++;

    int ret;
    if ((ret = ff_alloc_packet2(avctx, pkt, s->outputBuffer.dataSize, 0)) < 0)
        return ret;
    void *p = pkt->data;
    memcpy(p, s->outputBuffer.data, s->outputBuffer.dataSize);
    *got_packet = 1;

    return 0;
}

static av_cold int yami_enc_close(AVCodecContext *avctx)
{
    YamiEncContext *s = (YamiEncContext *)avctx->priv_data;

    if (s->encoder) {
        s->encoder->stop();
        releaseVideoEncoder(s->encoder);
        s->encoder = NULL;
    }

    av_log(avctx, AV_LOG_VERBOSE, "yami_close\n");

    return 0;
}

AVCodec ff_libyami_h264_encoder = {
    .name                  = "libyami_h264",
    .long_name             = NULL_IF_CONFIG_SMALL("libyami H.264"),
    .type                  = AVMEDIA_TYPE_VIDEO,
    .id                    = AV_CODEC_ID_H264,
    .capabilities          = CODEC_CAP_DELAY, // it is not necessary to support multi-threads
    .supported_framerates  = NULL,
    .pix_fmts              = NULL,
    .supported_samplerates = NULL,
    .sample_fmts           = NULL,
    .channel_layouts       = NULL,
#if FF_API_LOWRES
    .max_lowres            = 0,
#endif
    .priv_class            = NULL,
    .profiles              = NULL,
    .priv_data_size        = sizeof(YamiEncContext),
    .next                  = NULL,
    .init_thread_copy      = NULL,
    .update_thread_context = NULL,
    .defaults              = NULL,
    .init_static_data      = NULL,
    .init                  = yami_enc_init,
    .encode_sub            = NULL,
    .encode2               = yami_enc_frame,
    .decode                = NULL,
    .close                 = yami_enc_close,
    .flush                 = NULL, // TODO, add it
};
