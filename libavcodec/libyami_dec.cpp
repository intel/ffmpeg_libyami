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

#include "libyami.h"
#include "libyami_utils.h"

#define DECODE_TRACE(format, ...)  av_log(avctx, AV_LOG_VERBOSE, "# decode # line:%4d " format, __LINE__, ##__VA_ARGS__)


int yami_dec_init(AVCodecContext *avctx, char *mime_type)
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

        avctx->pix_fmt      = (AVPixelFormat)ret;
    }
    VADisplay m_display = createVADisplay();
    if (!m_display) {
        av_log(avctx, AV_LOG_ERROR, "\nfail to create %s display\n", mime_type);
        return -1;
    }
    av_log(avctx, AV_LOG_VERBOSE, "yami_dec_init\n");
    s->decoder = createVideoDecoder(mime_type);
    if (!s->decoder) {
        av_log(avctx, AV_LOG_ERROR, "fail to create %s decoder\n", mime_type);
        return -1;
    }

    NativeDisplay native_display;
    native_display.type = NATIVE_DISPLAY_VA;

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

    /* XXX: now used this as default, maybe change this with private options */
    s->output_type = VIDEO_DATA_MEMORY_TYPE_RAW_POINTER;

    s->in_queue = new std::deque<VideoDecodeBuffer*>;
    pthread_mutex_init(&s->mutex_, NULL);
    pthread_mutex_init(&s->in_mutex, NULL);
    pthread_cond_init(&s->in_cond, NULL);
    s->decode_status = DECODE_THREAD_NOT_INIT;

    s->decode_count = 0;
    s->decode_count_yami = 0;
    s->render_count = 0;

    return 0;
}

static void *decodeThread(void *arg)
{
    AVCodecContext *avctx = (AVCodecContext *)arg;
    YamiDecContext *s = (YamiDecContext *)avctx->priv_data;

    while (1) {
        VideoDecodeBuffer *in_buffer = NULL;
        // deque one input buffer
        DECODE_TRACE("decode thread runs one cycle start ... \n");
        pthread_mutex_lock(&s->in_mutex);
        if (s->in_queue->empty()) {
            if (s->decode_status == DECODE_THREAD_GOT_EOS) {
                pthread_mutex_unlock(&s->in_mutex);
                break;
            } else {
                DECODE_TRACE("decode thread wait because s->in_queue is empty\n");
                pthread_cond_wait(&s->in_cond, &s->in_mutex); // wait if no todo frame is available
            }
        }

        if (s->in_queue->empty()) { // may wake up from EOS/Close
            pthread_mutex_unlock(&s->in_mutex);
            continue;
        }

        DECODE_TRACE("s->in_queue->size()=%ld\n", s->in_queue->size());
        in_buffer = s->in_queue->front();

        pthread_mutex_unlock(&s->in_mutex);

        // decode one input buffer
        DECODE_TRACE("try to process one input buffer, in_buffer->data=%p, in_buffer->size=%d\n", in_buffer->data, in_buffer->size);
        Decode_Status status = s->decoder->decode(in_buffer);
        DECODE_TRACE("decode() status=%d, decode_count_yami=%d render_count %d\n", status, s->decode_count_yami, s->render_count);

        if (DECODE_FORMAT_CHANGE == status) {
            s->format_info = s->decoder->getFormatInfo();
            DECODE_TRACE("decode format change %dx%d\n", s->format_info->width,s->format_info->height);
            // resend the buffer to decoder
            status = s->decoder->decode(in_buffer);
            DECODE_TRACE("decode() status=%d\n",status);
            avctx->width = s->format_info->width;
            avctx->height = s->format_info->height;

        }
        if (status <= 0 || !s->format_info) {//if format_info is null means current frame decode failed
            av_log(avctx, AV_LOG_ERROR, "decode error %d\n", status);
            break;
        }
        s->decode_count_yami++;
        s->in_queue->pop_front();
        av_free(in_buffer->data);
        av_free(in_buffer);
    }

    DECODE_TRACE("decode thread exit\n");
    pthread_mutex_lock(&s->mutex_);
    s->decode_status = DECODE_THREAD_EXIT;
    pthread_mutex_unlock(&s->mutex_);
    return NULL;
}

static void yami_recycle_frame(void *opaque, uint8_t *data)
{
    AVCodecContext *avctx = (AVCodecContext *)opaque;
    YamiDecContext *s = (YamiDecContext *)avctx->priv_data;
    VideoFrameRawData *yami_frame = (VideoFrameRawData *)data;

    if (!s || !s->decoder || !yami_frame) // XXX, use shared pointer for s
        return;
    pthread_mutex_lock(&s->mutex_);
    s->decoder->renderDone(yami_frame);
    /* XXX: should I delete frame buffer?? */
    av_free(yami_frame);
    pthread_mutex_unlock(&s->mutex_);
    av_log(avctx, AV_LOG_DEBUG, "recycle previous frame: %p\n", yami_frame);
}

static int convert_to_AVFrame(AVCodecContext *avctx, VideoFrameRawData *from, AVFrame *to)
{
    if(!avctx || !from || !to)
        return -1;
    if (avctx->pix_fmt == AV_PIX_FMT_YAMI) {
        to->pts = from->timeStamp;

        to->width = avctx->width;
        to->height = avctx->height;

        to->format = AV_PIX_FMT_YAMI; /* FIXME */
        to->extended_data = NULL;

        to->extended_data = to->data;
        /* XXX: put the surface id to data[3] */
        to->data[3] = reinterpret_cast<uint8_t *>(from);

        to->buf[0] = av_buffer_create((uint8_t *)from,
                                         sizeof(VideoFrameRawData),
                                         yami_recycle_frame, avctx, 0);
    } else {
        int src_linesize[4] = {0};
        uint8_t *src_data[4] = {0};
        if (avctx->pix_fmt == AV_PIX_FMT_YUV420P) {
            src_linesize[0] = from->pitch[0];
            src_linesize[1] = from->pitch[1];
            src_linesize[2] = from->pitch[2];
            uint8_t* yami_data = reinterpret_cast<uint8_t*>(from->handle);
            src_data[0] = yami_data + from->offset[0];
            src_data[1] = yami_data + from->offset[1];
            src_data[2] = yami_data + from->offset[2];

            to->data[0] = reinterpret_cast<uint8_t*>(src_data[0]);
            to->data[1] = reinterpret_cast<uint8_t*>(src_data[1]);
            to->data[2] = reinterpret_cast<uint8_t*>(src_data[2]);
            to->linesize[0] = src_linesize[0];
            to->linesize[1] = src_linesize[1];
            to->linesize[2] = src_linesize[2];
        } else {
            src_linesize[0] = from->pitch[0];
            src_linesize[1] = from->pitch[1];
            src_linesize[2] = from->pitch[2];
            uint8_t* yami_data = reinterpret_cast<uint8_t*>(from->handle);
            src_data[0] = yami_data + from->offset[0];
            src_data[1] = yami_data + from->offset[1];

            to->data[0] = reinterpret_cast<uint8_t*>(src_data[0]);
            to->data[1] = reinterpret_cast<uint8_t*>(src_data[1]);
            to->linesize[0] = src_linesize[0];
            to->linesize[1] = src_linesize[1];
        }

        to->pkt_pts = AV_NOPTS_VALUE;
        to->pkt_dts = from->timeStamp;
        to->pts = AV_NOPTS_VALUE;

        to->width = avctx->width;
        to->height = avctx->height;

        to->format = avctx->pix_fmt;
        to->extended_data = NULL;

        to->extended_data = to->data;


        to->buf[0] = av_buffer_create((uint8_t *) from,
                                                     sizeof(VideoFrameRawData),
                                                     yami_recycle_frame, avctx, 0);

    }
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
    yami_frame = (VideoFrameRawData *)av_mallocz(sizeof(VideoFrameRawData));
    if (!yami_frame)
        return AVERROR(ENOMEM);
    if (avctx->pix_fmt == AV_PIX_FMT_YAMI)
        yami_frame->memoryType = VIDEO_DATA_MEMORY_TYPE_SURFACE_ID;
    else
        yami_frame->memoryType = VIDEO_DATA_MEMORY_TYPE_RAW_POINTER;
    /* FIXME */
    if (avctx->pix_fmt == AV_PIX_FMT_NV12)
        yami_frame->fourcc = VA_FOURCC_NV12;
    else
        yami_frame->fourcc = VA_FOURCC_I420;

    do {
        if (!s->format_info) {
            usleep(10000);
            continue;
        }

        do{
            status = s->decoder->getOutput(yami_frame, false);
            av_log(avctx, AV_LOG_DEBUG, "getoutput() status=%d\n", status);
        } while (!avpkt->data && status != RENDER_SUCCESS && s->in_queue->size() > 0);
        if (status == RENDER_SUCCESS) {
            break;
        }

        *got_frame = 0;
        av_free(yami_frame);
        return avpkt->size;
    } while (s->decode_status == DECODE_THREAD_RUNING);

    if (status != RENDER_SUCCESS) {
        av_log(avctx, AV_LOG_VERBOSE, "after processed EOS, return\n");
        return avpkt->size;
    }

    // process the output frame
    if (convert_to_AVFrame(avctx, yami_frame, frame) < 0)
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

    pthread_mutex_lock(&s->mutex_);
    while (s->decode_status != DECODE_THREAD_EXIT 
           && s->decode_status != DECODE_THREAD_NOT_INIT) { //if decode thread do not create do not loop
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

