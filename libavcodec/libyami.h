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

#ifndef LIBAVCODEC_LIBYAMI_H_
#define LIBAVCODEC_LIBYAMI_H_

#include <pthread.h>
#include <unistd.h>
#include <assert.h>
#include <deque>

extern "C" {
#include "avcodec.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "internal.h"
}

#include "VideoDecoderHost.h"
#include "VideoEncoderHost.h"

#include "libyami_dec.h"
#include "libyami_enc.h"


#ifndef VA_FOURCC_I420
#define VA_FOURCC_I420 VA_FOURCC('I','4','2','0')
#endif

#ifndef VA_FOURCC_NV12
#define VA_FOURCC_NV12 VA_FOURCC('N','V','1','2')
#endif


int yami_dec_init(AVCodecContext *avctx, const char *mime_type);

int yami_dec_frame(AVCodecContext *avctx, void *data,
                   int *got_frame, AVPacket *avpkt);

int yami_dec_close(AVCodecContext *avctx);

int yami_enc_init(AVCodecContext *avctx, const char *mime_type);

int yami_enc_frame(AVCodecContext *avctx, AVPacket *pkt,
                   const AVFrame *frame, int *got_packet);

int yami_enc_close(AVCodecContext *avctx);


#endif /* LIBAVCODEC_LIBYAMI_H_ */
