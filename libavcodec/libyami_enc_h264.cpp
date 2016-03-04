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

static av_cold int yami_enc_h264_init(AVCodecContext *avctx)
{
    if (avctx->width%2 != 0 || avctx->height%2 != 0) {
        av_log(avctx, AV_LOG_ERROR,
                      "width or height not divisible by 2 (%dx%d) .\n",
                       avctx->width, avctx->height);
        return AVERROR(EINVAL);
    }
    return yami_enc_init(avctx);
}

static int yami_enc_h264_frame(AVCodecContext *avctx, AVPacket *pkt,
                          const AVFrame *frame, int *got_packet)
{
    return yami_enc_frame(avctx, pkt, frame, got_packet);
}

static av_cold int yami_enc_h264_close(AVCodecContext *avctx)
{
    return yami_enc_close(avctx);
}

static const AVCodecDefault yami_enc_264_defaults[] = {
    { (uint8_t *)("b"),                (uint8_t *)("2M") },
    { (uint8_t *)("g"),                (uint8_t *)("30") },
    { NULL },
};

#define OFFSET(x) offsetof(YamiEncContext, x)
#define VE AV_OPT_FLAG_VIDEO_PARAM | AV_OPT_FLAG_ENCODING_PARAM
static const AVOption options[] = {
    { "profile",       "Set profile restrictions ", OFFSET(profile),       AV_OPT_TYPE_STRING, { 0 }, 0, 0, VE},
    { "level",         "Specify level (as defined by Annex A)", OFFSET(level), AV_OPT_TYPE_STRING, {.str=NULL}, 0, 0, VE},
    { "rcmode",        "rate control mode", OFFSET(rcmod), AV_OPT_TYPE_STRING, {.str=NULL}, 0, 0, VE},
    { "qp",            "Constant quantization parameter rate control method",OFFSET(cqp),        AV_OPT_TYPE_INT,    { .i64 = 26 }, 1, 52, VE },
//    { "cavlc",            NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 0 },  INT_MIN, INT_MAX, VE, "coder" },
//    { "cabac",            NULL, 0, AV_OPT_TYPE_CONST, { .i64 = 1 },  INT_MIN, INT_MAX, VE, "coder" },
    { NULL },
};

static const AVClass yami_enc_h264_class = {
    .class_name = "libyami_h264",
    .item_name  = av_default_item_name,
    .option     = options,
    .version    = LIBAVUTIL_VERSION_INT,
};

AVCodec ff_libyami_h264_encoder = {
    .name                  = "libyami_h264",
    .long_name             = NULL_IF_CONFIG_SMALL("libyami H.264 encoder"),
    .type                  = AVMEDIA_TYPE_VIDEO,
    .id                    = AV_CODEC_ID_H264,
    .capabilities          = CODEC_CAP_DELAY, // it is not necessary to support multi-threads
    .supported_framerates  = NULL,
    .pix_fmts              = (const enum AVPixelFormat[]) { AV_PIX_FMT_YAMI,
                                                            AV_PIX_FMT_NV12,
                                                            AV_PIX_FMT_YUV420P,
                                                            AV_PIX_FMT_NONE},
    .supported_samplerates = NULL,
    .sample_fmts           = NULL,
    .channel_layouts       = NULL,
#if FF_API_LOWRES
    .max_lowres            = 0,
#endif
    .priv_class            = &yami_enc_h264_class,
    .profiles              = NULL,
    .priv_data_size        = sizeof(YamiEncContext),
    .next                  = NULL,
    .init_thread_copy      = NULL,
    .update_thread_context = NULL,
    .defaults              = yami_enc_264_defaults,
    .init_static_data      = NULL,
    .init                  = yami_enc_h264_init,
    .encode_sub            = NULL,
    .encode2               = yami_enc_h264_frame,
    .decode                = NULL,
    .close                 = yami_enc_h264_close,
    .flush                 = NULL, // TODO, add it
};
