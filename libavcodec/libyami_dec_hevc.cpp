/*
 * libyami_dec_hevc.cpp
 *
 *  Created on: 2016.2.19
 *      Author: yunzhoux
 */

#include "libyami.h"

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


