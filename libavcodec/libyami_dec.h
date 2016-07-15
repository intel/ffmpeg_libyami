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

#ifndef LIBAVCODEC_LIBYAMI_DEC_H_
#define LIBAVCODEC_LIBYAMI_DEC_H_


#define DECODE_QUEUE_SIZE 8

typedef enum {
    DECODE_THREAD_NOT_INIT = 0,
    DECODE_THREAD_RUNING,
    DECODE_THREAD_GOT_EOS,
    DECODE_THREAD_EXIT,
} DecodeThreadStatus;

struct YamiDecContext {
    AVCodecContext *avctx;
    pthread_mutex_t ctx_mutex; // mutex for decoder->getOutput() and YamiContext itself update (decode_status, etc)

    YamiMediaCodec::IVideoDecoder *decoder;
    //VideoDataMemoryType output_type;
    const VideoFormatInfo *format_info;
    pthread_t decode_thread_id;
    std::deque<VideoDecodeBuffer *> *in_queue;
    pthread_mutex_t in_mutex; // mutex for in_queue
    std::deque<YamiImage *> *out_queue;
    pthread_mutex_t out_mutex; // mutex for out_queue
    pthread_cond_t in_cond;   // decode thread condition wait
    DecodeThreadStatus decode_status;

    //the pts is no value use this value
    int duration;
    // debug use
    int decode_count;
    int decode_count_yami;
    int render_count;
    int recycle_count;
};

#endif /* LIBAVCODEC_LIBYAMI_DEC_H_ */
