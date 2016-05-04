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
#include "libavutil/dict.h"
#include "libavutil/mem.h"
#include "libavutil/opt.h"

#include "ffmpeg.h"

int yami_transcode_init(InputStream *inst, OutputStream *ost)
{
    InputStream *ist;
    const enum AVPixelFormat *pix_fmt;

    AVDictionaryEntry *e;
    const AVOption *opt;
    int flags = 0;

    int i;

    if (ost && inst && 0 == strncmp(ost->enc_ctx->codec->name, "libyami", strlen("libyami")) &&
        0 == strncmp(inst->dec_ctx->codec->name, "libyami", strlen("libyami"))) {
        /* check if the encoder supports LIBYAMI */
        if (!ost->enc->pix_fmts)
            return 0;
        for (pix_fmt = ost->enc->pix_fmts; *pix_fmt != AV_PIX_FMT_NONE; pix_fmt++)
            if (*pix_fmt == AV_PIX_FMT_YAMI)
                break;
        if (*pix_fmt == AV_PIX_FMT_NONE)
            return 0;

        if (ost->source_index < 0)
            return 0;

        /* check if the decoder supports libyami and the output only goes to this stream */
        ist = input_streams[ost->source_index];
        if ((ist->nb_filters && !force_yami_pipeline) ||
            !ist->dec || !ist->dec->pix_fmts)
            return 0;
        for (pix_fmt = ist->dec->pix_fmts; *pix_fmt != AV_PIX_FMT_NONE; pix_fmt++)
            if (*pix_fmt == AV_PIX_FMT_YAMI)
                break;
        if (*pix_fmt == AV_PIX_FMT_NONE)
            return 0;

        for (i = 0; i < nb_output_streams; i++)
            if (output_streams[i] != ost &&
                output_streams[i]->source_index == ost->source_index)
                return 0;

        av_log(NULL, AV_LOG_VERBOSE, "Setting up libyami transcoding\n");

        e = av_dict_get(ost->encoder_opts, "flags", NULL, 0);
        opt = av_opt_find(ost->enc_ctx, "flags", NULL, 0, 0);
        if (e && opt)
            av_opt_eval_flags(ost->enc_ctx, opt, e->value, &flags);

        ost->enc_ctx->pix_fmt         = AV_PIX_FMT_YAMI;

        ist->dec_ctx->pix_fmt         = AV_PIX_FMT_YAMI;
        ist->resample_pix_fmt         = AV_PIX_FMT_YAMI;
    }

    return 0;
}
