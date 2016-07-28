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


#include <va/va_drm.h>
#if HAVE_VAAPI_X11
#include <va/va_x11.h>
#endif

#ifndef VA_FOURCC_I420
#define VA_FOURCC_I420 VA_FOURCC('I','4','2','0')
#endif

#ifndef VA_FOURCC_NV12
#define VA_FOURCC_NV12 VA_FOURCC('N','V','1','2')
#endif

typedef struct {
    SharedPtr<VideoFrame> output_frame;
    VADisplay va_display;
/*if map the output_frame to buffer will use follow value*/
}YamiImage;

VADisplay ff_vaapi_create_display(void);
SharedPtr<VideoFrame> ff_vaapi_create_surface(uint32_t rt_fmt, int pix_fmt, uint32_t w, uint32_t h);
bool ff_vaapi_destory_surface(SharedPtr<VideoFrame>& frame);
bool ff_vaapi_load_image(SharedPtr<VideoFrame>& frame, AVFrame *in);
bool ff_vaapi_get_image(SharedPtr<VideoFrame>& frame, AVFrame *out);
bool ff_check_vaapi_status(VAStatus status, const char *msg);

YamiStatus ff_yami_alloc_surface (SurfaceAllocator* thiz, SurfaceAllocParams* params);
YamiStatus ff_yami_free_surface (SurfaceAllocator* thiz, SurfaceAllocParams* params);
void ff_yami_unref_surface (SurfaceAllocator* thiz);

#define DECODE_QUEUE_SIZE 8
#define ENCODE_QUEUE_SIZE 4

#define EXTRA_SIZE (DECODE_QUEUE_SIZE + ENCODE_QUEUE_SIZE + 2) //EXTRA_SIZE must great than DEC_QUE+ENC_QUE+DBP-19 or the thread will be block

#endif /* LIBAVCODEC_LIBYAMI_H_ */
