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

extern "C" {
#include "avcodec.h"
#include "libavutil/avassert.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavutil/time.h"
#include "libavutil/internal.h"
#include "internal.h"
}

#include "VideoEncoderHost.h"
#include "libyami_internal.h"
#include "libyami_enc.h"
#include "libyami.h"
using namespace YamiMediaCodec;

static int ff_convert_to_yami(AVCodecContext *avctx, AVFrame *from, YamiImage *to)
{
    int pix_fmt = VA_FOURCC_NV12;
    if (avctx->pix_fmt == AV_PIX_FMT_YUV420P) {
        pix_fmt =  VA_FOURCC_I420;
    } else if (avctx->pix_fmt == AV_PIX_FMT_NV12) {
        pix_fmt =  VA_FOURCC_NV12;
    } else {
        av_log(avctx, AV_LOG_VERBOSE, "used the un-support format ... \n");
    }
    to->output_frame = ff_vaapi_create_surface(VA_RT_FORMAT_YUV420, pix_fmt, avctx->width, avctx->height);
    ff_vaapi_load_image(to->output_frame, from);
    if (from->key_frame)
        to->output_frame->flags |= VIDEO_FRAME_FLAGS_KEY;
    to->va_display = ff_vaapi_create_display();
    from->data[3] = reinterpret_cast<uint8_t *>(to);
    return 0;
}

static void ff_yami_encode_frame(void *handle, void *args)
{
    YamiThreadContext<AVFrame *> *ytc = (YamiThreadContext<AVFrame *> *)handle;
    YamiEncContext *s = (YamiEncContext *)ytc->priv;
    AVCodecContext *avctx = s->avctx;

    AVFrame *frame = (AVFrame *)args;
    /* deque one input buffer */
    av_log(avctx, AV_LOG_VERBOSE, "encode thread runs one cycle start ... \n");
    /* encode one input buffer */
    Encode_Status status;
    YamiImage *yami_image = NULL;
    if (frame->format != AV_PIX_FMT_YAMI) { /* non zero-copy mode */
        yami_image = (YamiImage *)av_mallocz(sizeof(YamiImage));
        if (ff_convert_to_yami(avctx, frame, yami_image) < 0)
            av_log(avctx, AV_LOG_ERROR,
               "av_convert_to_yami convert frame failed\n");
    } else { /* zero-copy mode */
        yami_image = (YamiImage *)frame->data[3];
        /* encode use the AVFrame pts */
        yami_image->output_frame->timeStamp = frame->pts;
    }
    /* handle encoder busy case */
    do {
         status = s->encoder->encode(yami_image->output_frame);
    } while (status == ENCODE_IS_BUSY);
    av_log(avctx, AV_LOG_VERBOSE, "encode status %d, encode count %d\n",
           status, s->encode_count_yami);
    if (status < 0) {
        av_log(avctx, AV_LOG_ERROR,
               "encode error %d frame %d\n", status , s->encode_count_yami);
    }
    s->encode_count_yami++;
    if (ff_yami_push_outdata(ytc, frame) != 0)
        av_log(avctx, AV_LOG_ERROR,
                     "ff_yami_push_outdata failed\n");
}

static int ff_yami_encode_thread_init(YamiEncContext *s)
{
    if (!s)
        return -1;
    s->ctx = (YamiThreadContext<AVFrame *> *)av_mallocz(sizeof(YamiThreadContext<AVFrame *>));
    if (!s->ctx)
        return -1;
    s->ctx->process_data_cb = ff_yami_encode_frame;
    s->ctx->priv = s;
    s->ctx->max_queue_size = ENCODE_QUEUE_SIZE;
    if (ff_yami_thread_init(s->ctx) != 0)
        return -1;
    return 0;
}


static bool
ff_out_buffer_create(VideoEncOutputBuffer *enc_out_buf, int max_out_size)
{
    enc_out_buf->data = static_cast<uint8_t *>(malloc(max_out_size));
    if (!enc_out_buf->data)
        return false;
    enc_out_buf->bufferSize = max_out_size;
    enc_out_buf->format = OUTPUT_EVERYTHING;
    return true;
}

static const char *get_mime(AVCodecID id)
{
    switch (id) {
    case AV_CODEC_ID_H264:
        return YAMI_MIME_H264;
    case AV_CODEC_ID_VP8:
        return YAMI_MIME_VP8;
    case AV_CODEC_ID_HEVC:
        return YAMI_MIME_H265;
    default:
        av_assert0(!"Invalid codec ID!");
        return 0;
    }
}

static void ff_out_buffer_destroy(VideoEncOutputBuffer *enc_out_buf)
{
    if (enc_out_buf->data)
        free(enc_out_buf->data);
}

static int yami_enc_init(AVCodecContext *avctx)
{
    YamiEncContext *s = (YamiEncContext *) avctx->priv_data;
    Encode_Status status;
    enum AVPixelFormat pix_fmts[5] =
        {
            AV_PIX_FMT_NV12,
            AV_PIX_FMT_P010,
            AV_PIX_FMT_YUV420P,
            AV_PIX_FMT_YAMI,
            AV_PIX_FMT_NONE
        };
    if (avctx->pix_fmt == AV_PIX_FMT_NONE) {
        int ret = ff_get_format(avctx, pix_fmts);
        if (ret < 0)
            return ret;
        avctx->pix_fmt      = (AVPixelFormat)ret;
    }

    if (avctx->codec_id == AV_CODEC_ID_H264 && avctx->width % 2 != 0
        || avctx->height % 2 != 0) {
        av_log(avctx, AV_LOG_ERROR,
                "width or height not divisible by 2 (%dx%d) .\n",
               avctx->width,avctx->height);
        return AVERROR(EINVAL);
    }
    av_log(avctx, AV_LOG_VERBOSE, "yami_enc_init\n");
    const char *mime_type = get_mime(avctx->codec_id);
    s->encoder = createVideoEncoder(mime_type);
    if (!s->encoder) {
        av_log(avctx, AV_LOG_ERROR, "fail to create libyami encoder\n");
        return AVERROR_BUG;
    }
    NativeDisplay native_display;
    native_display.type = NATIVE_DISPLAY_VA;
    VADisplay va_display = ff_vaapi_create_display();
    native_display.handle = (intptr_t)va_display;
    s->encoder->setNativeDisplay(&native_display);

    /* configure encoding parameters */
    VideoParamsCommon encVideoParams;
    encVideoParams.size = sizeof(VideoParamsCommon);
    s->encoder->getParameters(VideoParamsTypeCommon, &encVideoParams);
    encVideoParams.resolution.width  = avctx->width;
    encVideoParams.resolution.height = avctx->height;
    /* frame rate setting */
    if (avctx->framerate.den > 0 && avctx->framerate.num > 0) {
        encVideoParams.frameRate.frameRateDenom = avctx->framerate.den;
        encVideoParams.frameRate.frameRateNum = avctx->framerate.num;
    } else {
        encVideoParams.frameRate.frameRateNum = avctx->time_base.den;
        encVideoParams.frameRate.frameRateDenom = avctx->time_base.num;
    }
    /* picture type and bitrate setting */
    encVideoParams.intraPeriod = av_clip(avctx->gop_size, 1, 250);
    s->ip_period = encVideoParams.ipPeriod = avctx->max_b_frames < 2 ? 1 : 3;
    s->max_inqueue_size = FFMAX(encVideoParams.ipPeriod, ENCODE_QUEUE_SIZE);

    /* ratecontrol method selected
    When ‘global_quality’ is specified, a quality-based mode is used.
    Specifically this means either
        - CQP - constant quantizer scale, when the ‘qscale’ codec
        flag is also set (the ‘-qscale’ avconv option).
    Otherwise, a bitrate-based mode is used. For all of those, you
    should specify at least the desired average bitrate with the ‘b’ option.
        - CBR - constant bitrate, when ‘maxrate’ is specified and
        equal to the average bitrate.
        - VBR - variable bitrate, when ‘maxrate’ is specified, but
        is higher than the average bitrate.
     */
    const char *rc_desc;
    float quant;
    int want_qscale = !!(avctx->flags & AV_CODEC_FLAG_QSCALE);

    if (want_qscale) {
        encVideoParams.rcMode = RATE_CONTROL_CQP;
        quant = avctx->global_quality / FF_QP2LAMBDA;
        encVideoParams.rcParams.initQP = av_clip(quant, 1, 52);

        rc_desc = "constant quantization parameter (CQP)";
    } else if (avctx->rc_max_rate > avctx->bit_rate) {
        encVideoParams.rcMode = RATE_CONTROL_VBR;
        encVideoParams.rcParams.bitRate = avctx->rc_max_rate;

        encVideoParams.rcParams.targetPercentage = (100 * avctx->bit_rate)/avctx->rc_max_rate;
        rc_desc = "variable bitrate (VBR)";

        av_log(avctx, AV_LOG_WARNING,
               "Using the %s ratecontrol method, but driver not support it.\n", rc_desc);
    } else if (avctx->rc_max_rate == avctx->bit_rate) {
        encVideoParams.rcMode = RATE_CONTROL_CBR;
        encVideoParams.rcParams.bitRate = avctx->bit_rate;
        encVideoParams.rcParams.targetPercentage = 100;

        rc_desc = "constant bitrate (CBR)";
    } else {
        encVideoParams.rcMode = RATE_CONTROL_CQP;
        encVideoParams.rcParams.initQP = 26;

        rc_desc = "constant quantization parameter (CQP) as default";
    }

    av_log(avctx, AV_LOG_VERBOSE, "Using the %s ratecontrol method\n", rc_desc);

    if (s->level){
        encVideoParams.level = atoi(s->level);
    } else {
        encVideoParams.level = 40;
    }

    if (avctx->codec_id == AV_CODEC_ID_H264) {
        encVideoParams.profile = VAProfileH264Main;
        if (s->profile) {
            if (!strcmp(s->profile , "high")) {
                encVideoParams.profile = VAProfileH264High;
            } else if(!strcmp(s->profile , "main")) {
                encVideoParams.profile = VAProfileH264Main;
            } else if(!strcmp(s->profile , "baseline")) {
                encVideoParams.profile = VAProfileH264Baseline;
            }
        } else {
            av_log(avctx, AV_LOG_WARNING, "Using the main profile as default.\n");
        }
    }
    encVideoParams.size = sizeof(VideoParamsCommon);
    s->encoder->setParameters(VideoParamsTypeCommon, &encVideoParams);

    if (avctx->codec_id == AV_CODEC_ID_H264) {
        VideoConfigAVCStreamFormat streamFormat;
        streamFormat.size = sizeof(VideoConfigAVCStreamFormat);
        streamFormat.streamFormat = AVC_STREAM_FORMAT_ANNEXB;
        s->encoder->setParameters(VideoConfigTypeAVCStreamFormat, &streamFormat);
    }

#if HAVE_PTHREADS
    if (ff_yami_encode_thread_init(s) < 0)
        return AVERROR(ENOMEM);
#else
    av_log(avctx, AV_LOG_ERROR, "pthread libaray must be supported\n");
    return AVERROR(ENOSYS);
#endif
    status = s->encoder->start();
    if (status != ENCODE_SUCCESS) {
        av_log(avctx, AV_LOG_ERROR, "yami encoder fail to start\n");
        return AVERROR_BUG;
    }
    /* init encoder output buffer */
    s->encoder->getMaxOutSize(&(s->max_out_size));

    if (!ff_out_buffer_create(&s->enc_out_buf, s->max_out_size)) {
        av_log(avctx, AV_LOG_ERROR, "fail to create output\n");
        return AVERROR(ENOMEM);
    }
    s->enc_frame_size = FFALIGN(avctx->width, 32) * FFALIGN(avctx->height, 32) * 3;
    s->enc_frame_buf = static_cast<uint8_t *>(av_mallocz(s->enc_frame_size));

    s->encode_count = 0;
    s->encode_count_yami = 0;
    s->render_count = 0;
    av_log(avctx, AV_LOG_DEBUG, "yami_enc_init\n");
    return 0;
}

static int yami_enc_frame(AVCodecContext *avctx, AVPacket *pkt,
                          const AVFrame *frame, int *got_packet)
{
    YamiEncContext *s = (YamiEncContext *)avctx->priv_data;
    s->avctx = avctx;
    Encode_Status status;
    int ret;
    if(!s->encoder)
        return -1;
    if (frame) {
        AVFrame *qframe = av_frame_alloc();
        if (!qframe) {
            return AVERROR(ENOMEM);
        }
        /* av_frame_ref the src frame and av_frame_unref in encode thread */
        ret = av_frame_ref(qframe, frame);
        if (ret < 0)
            return ret;
        ff_yami_push_data(s->ctx, qframe);
        s->encode_count++;
    }
    if (!frame  && ff_yami_read_thread_status(s->ctx) <= YAMI_THREAD_GOT_EOS)
        ff_yami_set_stream_eof(s->ctx);
    if (frame  && ff_yami_read_thread_status(s->ctx) >= YAMI_THREAD_GOT_EOS)
        ff_yami_set_stream_run(s->ctx);
    ff_yami_thread_create (s->ctx);
    do {
        status = s->encoder->getOutput(&s->enc_out_buf, true);
    } while (!frame && status != ENCODE_SUCCESS && ff_yami_read_thread_status(s->ctx) != YAMI_THREAD_FLUSH_OUT);
    if (status != ENCODE_SUCCESS)
        return 0;
    if ((ret = ff_alloc_packet2(avctx, pkt, s->enc_out_buf.dataSize, 0)) < 0)
        return ret;

    AVFrame *qframe = ff_yami_pop_outdata (s->ctx);
    if (qframe) {
            pkt->pts = s->enc_out_buf.timeStamp;
            /* XXX: DTS must be smaller than PTS, used ip_period as offset */
            pkt->dts = qframe->pts - s->ip_period;
            if (qframe->format != AV_PIX_FMT_YAMI) {
                YamiImage *yami_image = (YamiImage *)qframe->data[3];
                ff_vaapi_destory_surface(yami_image->output_frame);
                yami_image->output_frame.reset();
                av_free(yami_image);
            };
            av_frame_free(&qframe);
    }

    s->render_count++;
    /* get extradata when build the first frame */
    int offset = 0;
    if (avctx->codec_id == AV_CODEC_ID_H264) {
        if (avctx->flags & AV_CODEC_FLAG_GLOBAL_HEADER && !avctx->extradata) {
            /* find start code */
            uint8_t *ptr = s->enc_out_buf.data;
            for (uint32_t i = 0; i < s->enc_out_buf.dataSize; i++) {
                if (*(ptr + i) == 0x0 && *(ptr + i + 1) == 0x0
                    && *(ptr + i + 2) == 0x0 && *(ptr + i + 3) == 0x1
                    && (*(ptr + i + 4) & 0x1f) == 5) {
                    offset = i;
                    break;
                }
            }
            avctx->extradata = (uint8_t *) av_mallocz(
                offset + AV_INPUT_BUFFER_PADDING_SIZE);
            memcpy(avctx->extradata, s->enc_out_buf.data, offset);
            avctx->extradata_size = offset;
        }
    }
    void *p = pkt->data;
    memcpy(p, s->enc_out_buf.data + offset,
           s->enc_out_buf.dataSize - offset);
    pkt->size = s->enc_out_buf.dataSize - offset;

    if (s->enc_out_buf.flag & ENCODE_BUFFERFLAG_SYNCFRAME)
        pkt->flags |= AV_PKT_FLAG_KEY;
    *got_packet = 1;

    return 0;
}

static int yami_enc_close(AVCodecContext *avctx)
{
    YamiEncContext *s = (YamiEncContext *)avctx->priv_data;
    ff_out_buffer_destroy(&s->enc_out_buf);
    if (ff_yami_thread_close(s->ctx) != 0) {
            av_log(avctx, AV_LOG_ERROR, "ff_yami_thread_close failed\n");
    }
    if (s->encoder) {
        s->encoder->stop();
        releaseVideoEncoder(s->encoder);
        s->encoder = NULL;
    }
    av_free(s->enc_frame_buf);
    s->enc_frame_size = 0;
    av_log(avctx, AV_LOG_DEBUG, "yami_enc_close\n");
    return 0;
}

#define OFFSET(x) offsetof(YamiEncContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "profile",       "Set profile restrictions ", OFFSET(profile),       AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE},
    { "level",         "Specify level (as defined by Annex A)", OFFSET(level), AV_OPT_TYPE_STRING, {.str=NULL}, 0, 0, VE},
    { NULL },
};

#define YAMI_ENC(NAME, ID) \
static const AVClass yami_enc_##NAME##_class = { \
    .class_name = "libyami_" #NAME, \
    .item_name  = av_default_item_name, \
    .option     = options, \
    .version    = LIBAVUTIL_VERSION_INT, \
}; \
AVCodec ff_libyami_##NAME##_encoder = { \
    /* name */                  "libyami_" #NAME, \
    /* long_name */             NULL_IF_CONFIG_SMALL(#NAME " (libyami)"), \
    /* type */                  AVMEDIA_TYPE_VIDEO, \
    /* id */                    ID, \
    /* capabilities */          CODEC_CAP_DELAY, \
    /* supported_framerates */  NULL, \
    /* pix_fmts */              (const enum AVPixelFormat[]) { AV_PIX_FMT_YAMI, \
                                                            AV_PIX_FMT_NV12, \
                                                            AV_PIX_FMT_YUV420P, \
                                                            AV_PIX_FMT_NONE}, \
    /* supported_samplerates */ NULL, \
    /* sample_fmts */           NULL, \
    /* channel_layouts */       NULL, \
    /* max_lowres */            0, \
    /* priv_class */            &yami_enc_##NAME##_class, \
    /* profiles */              NULL, \
    /* priv_data_size */        sizeof(YamiEncContext), \
    /* next */                  NULL, \
    /* init_thread_copy */      NULL, \
    /* update_thread_context */ NULL, \
    /* defaults */              NULL, \
    /* init_static_data */      NULL, \
    /* init */                  yami_enc_init, \
    /* encode_sub */            NULL, \
    /* encode2 */               yami_enc_frame, \
    /* decode */                NULL, \
    /* close */                 yami_enc_close, \
};

YAMI_ENC(h264, AV_CODEC_ID_H264)
YAMI_ENC(hevc, AV_CODEC_ID_HEVC)
YAMI_ENC(vp8, AV_CODEC_ID_VP8)
