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

#define DEBUG_LIBYAMI 0

struct YamiDecContext {
    AVCodecContext *avctx;

    IVideoDecoder *decoder;
    VideoDataMemoryType output_type;
    const VideoFormatInfo *format_info;

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

    s->decode_count = 0;
    s->decode_count_yami = 0;
    s->render_count = 0;

    return 0;
}

static void yami_recycle_frame(void *opaque, uint8_t *data)
{
    AVCodecContext *avctx = (AVCodecContext *) opaque;
    YamiDecContext *s = (YamiDecContext *) avctx->priv_data;
    VideoRenderBuffer *frame = (VideoRenderBuffer *) data;

    if (!s->decoder || !frame) // XXX, use shared pointer for s
        return;
    s->decoder->renderDone(frame);
    av_log(avctx, AV_LOG_DEBUG, "recycle previous frame: %p\n", frame);
}

static int yami_dec_frame(AVCodecContext *avctx, void *data,
                             int *got_frame, AVPacket *avpkt)
{
    YamiDecContext *s = (YamiDecContext *) avctx->priv_data;
    VideoDecodeBuffer *in_buffer = NULL;
    Decode_Status status = RENDER_NO_AVAILABLE_FRAME;
    //VideoFrameRawData *yami_frame = NULL;
    VideoRenderBuffer *yami_frame = NULL;
    AVFrame *frame = (AVFrame *) data;

    av_log(avctx, AV_LOG_VERBOSE, "yami_decode_frame\n");

    // append avpkt to input buffer queue
    in_buffer = (VideoDecodeBuffer *)av_mallocz(sizeof(VideoDecodeBuffer));
    in_buffer->data = avpkt->data;
    in_buffer->size = avpkt->size;
    in_buffer->timeStamp = avpkt->pts;
    if (avctx->extradata && avctx->extradata_size && avctx->extradata[0] == 1)
        in_buffer->flag |= IS_AVCC;

    status = s->decoder->decode(in_buffer);
    PRINT_DECODE_THREAD("decode() status=%d, decode_count_yami=%d\n",
                        status, s->decode_count_yami);

    if (DECODE_FORMAT_CHANGE == status) {
        s->format_info = s->decoder->getFormatInfo();
        PRINT_DECODE_THREAD("decode format change %dx%d\n",
                            s->format_info->width, s->format_info->height);
        // resend the buffer
        status = s->decoder->decode(in_buffer);
        PRINT_DECODE_THREAD("decode() status=%d\n", status);
        avctx->width = s->format_info->width;
        avctx->height = s->format_info->height;
        avctx->pix_fmt = AV_PIX_FMT_YUV420P;
    }
    s->decode_count_yami++;
    s->decode_count++;

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
    } while (0);

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

        *(AVFrame *) data = *vframe;
        ((AVFrame *) data)->extended_data = ((AVFrame *) data)->data;
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

    if (s->decoder) {
        s->decoder->stop();
        releaseVideoDecoder(s->decoder);
        s->decoder = NULL;
    }

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
    outputBuffer->data = static_cast<uint8_t*>(malloc(maxOutSize));
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
    YamiEncContext *s = (YamiEncContext *) avctx->priv_data;
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
        yami_frame->flags = 1;
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
    YamiEncContext *s = (YamiEncContext *) avctx->priv_data;

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
