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
#ifndef LIBAVCODEC_LIBYAMI_ENC_H_
#define LIBAVCODEC_LIBYAMI_ENC_H_

typedef enum {
    ENCODE_THREAD_NOT_INIT = 0,
    ENCODE_THREAD_RUNING,
    ENCODE_THREAD_GOT_EOS,
    ENCODE_THREAD_EXIT,
} EncodeThreadStatus;

struct YamiEncContext {
    AVCodecContext *avctx;

    YamiMediaCodec::IVideoEncoder *encoder;
    VideoEncOutputBuffer enc_out_buf;

    uint32_t max_inqueue_size;
    YamiThreadContext<AVFrame*> *ytc;

    uint8_t *enc_frame_buf;
    uint32_t enc_frame_size;
    /***video commom param*****/
    uint32_t cqp;           // qp value 0-52
    uint32_t frame_rate;    // frame rate trasfer the time stamp
    char *rcmod;            // rate control mode CQP|CBR|VBR
    uint32_t gop;           // group of picture 1-250
    uint32_t ip_period;     // max b frame 0-only I 1-IP 3-IPBB
    char *level;            // level 40|41|50|51
    char *profile;          // profile main|baseline|high
    /*******************/

    uint32_t max_out_size;

    // debug use
    int encode_count;
    int encode_count_yami;
    int render_count;
};

#endif /* LIBAVCODEC_LIBYAMI_ENC_H_ */
