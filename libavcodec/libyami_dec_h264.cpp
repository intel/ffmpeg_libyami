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
