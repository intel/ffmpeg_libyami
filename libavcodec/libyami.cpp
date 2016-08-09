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

#include "config.h"

extern "C" {
#include "avcodec.h"
#include "libavutil/imgutils.h"
#include "internal.h"
}

#include "VideoCommonDefs.h"
#include "libyami.h"

#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#define HAVE_VAAPI_DRM 1

#if HAVE_VAAPI_X11
#include <X11/Xlib.h>
#endif

VADisplay ff_vaapi_create_display(void)
{
    static VADisplay display = NULL;

    if (!display) {
#if !HAVE_VAAPI_DRM
        const char *device = NULL;
        // Try to open the device as an X11 display.
        Display *x11_display = XOpenDisplay(device);
        if (!x11_display) {
            return NULL;
        } else {
            display = vaGetDisplay(x11_display);
            if (!display) {
                XCloseDisplay(x11_display);
            }
        }
#else
        const char *devices[] = {
            "/dev/dri/renderD128",
            "/dev/dri/card0",
            NULL
        };
        // Try to open the device as a DRM path.
        int i;
        int drm_fd;
        for (i = 0; !display && devices[i]; i++) {
            drm_fd = open(devices[i], O_RDWR);
            if (drm_fd < 0)
                continue;

            display = vaGetDisplayDRM(drm_fd);
            if (!display)
                close(drm_fd);
        }
#endif
        if (!display)
            return NULL;
        int majorVersion, minorVersion;
        VAStatus vaStatus = vaInitialize(display, &majorVersion, &minorVersion);
        if (vaStatus != VA_STATUS_SUCCESS) {
#if HAVE_VAAPI_DRM
            close(drm_fd);
#endif
            display = NULL;
            return NULL;
        }
        return display;
    } else {
        return display;
    }
}

/*
 * Used SSE4 MOVNTDQA instruction improving performance of data copies from
 * Uncacheable Speculative Write Combining (USWC) memory to ordinary write back (WB)
 * system memory.
 * https://software.intel.com/en-us/articles/copying-accelerated-video-decode-frame-buffers/
 */
#if HAVE_SSE4
#define COPY16(dstp, srcp, load, store) \
    __asm__ volatile (                  \
        load "  0(%[src]), %%xmm1\n"    \
        store " %%xmm1,    0(%[dst])\n" \
        : : [dst]"r"(dstp), [src]"r"(srcp) : "memory", "xmm1")

#define COPY128(dstp, srcp, load, store) \
    __asm__ volatile (                   \
        load "  0(%[src]), %%xmm1\n"     \
        load " 16(%[src]), %%xmm2\n"     \
        load " 32(%[src]), %%xmm3\n"     \
        load " 48(%[src]), %%xmm4\n"     \
        load " 64(%[src]), %%xmm5\n"     \
        load " 80(%[src]), %%xmm6\n"     \
        load " 96(%[src]), %%xmm7\n"     \
        load " 112(%[src]), %%xmm8\n"    \
        store " %%xmm1,    0(%[dst])\n"  \
        store " %%xmm2,   16(%[dst])\n"  \
        store " %%xmm3,   32(%[dst])\n"  \
        store " %%xmm4,   48(%[dst])\n"  \
        store " %%xmm5,   64(%[dst])\n"  \
        store " %%xmm6,   80(%[dst])\n"  \
        store " %%xmm7,   96(%[dst])\n"  \
        store " %%xmm8,   112(%[dst])\n" \
        : : [dst]"r"(dstp), [src]"r"(srcp) : "memory", "xmm1", "xmm2", "xmm3", "xmm4", "xmm5", "xmm6", "xmm7", "xmm8")

void *ff_copy_from_uswc(void *dst, void *src, size_t size)
{
    char aligned;
    int remain;
    int i, round;
    uint8_t *pDst, *pSrc;

    if (dst == NULL || src == NULL || size == 0) {
        return NULL;
    }

    aligned = (((size_t) dst) | ((size_t) src)) & 0x0F;

    if (aligned != 0) {
        return NULL;
    }

    pDst = (uint8_t *) dst;
    pSrc = (uint8_t *) src;
    remain = size & 0x7F;
    round = size >> 7;

    __asm__ volatile ("mfence");

    for (i = 0; i < round; i++) {
        COPY128(pDst, pSrc, "movntdqa", "movdqa");
        pSrc += 128;
        pDst += 128;
    }

    if (remain >= 16) {
        size = remain;
        remain = size & 0xF;
        round = size >> 4;

        for (i = 0; i < round; i++) {
            COPY16(pDst, pSrc, "movntdqa", "movdqa");
            pSrc += 16;
            pDst += 16;
        }
    }

    if (remain > 0) {
        char *ps = (char *)(pSrc);
        char *pd = (char *)(pDst);

        for (i = 0; i < remain; i++) {
            pd[i] = ps[i];
        }
    }
    __asm__ volatile ("mfence");

    return dst;
}
#else
void *ff_copy_from_uswc(void *dst, void *src, size_t size)
{
    return memcpy(dst, src, size);
}
#endif

bool ff_check_vaapi_status(VAStatus status, const char *msg)
{
    if (status != VA_STATUS_SUCCESS) {
        av_log(NULL, AV_LOG_ERROR, "%s: %s", msg, vaErrorStr(status));
        return false;
    }
    return true;
}

SharedPtr<VideoFrame> ff_vaapi_create_surface(uint32_t rt_fmt, int pix_fmt, uint32_t w, uint32_t h)
{
    SharedPtr<VideoFrame> frame;
    VAStatus status;
    VASurfaceID id;
    VASurfaceAttrib attrib;

    VADisplay m_vaDisplay = ff_vaapi_create_display();

    attrib.type =  VASurfaceAttribPixelFormat;
    attrib.flags = VA_SURFACE_ATTRIB_SETTABLE;
    attrib.value.type = VAGenericValueTypeInteger;
    attrib.value.value.i = pix_fmt;

    status = vaCreateSurfaces(m_vaDisplay, rt_fmt, w, h, &id, 1, &attrib, 1);
    if (!ff_check_vaapi_status(status, "vaCreateSurfaces"))
        return frame;
    frame.reset(new VideoFrame);
    memset(frame.get(), 0 , sizeof(VideoFrame));
    frame->surface = (intptr_t)id;
    frame->crop.x = frame->crop.y = 0;
    frame->crop.width = w;
    frame->crop.height = h;
    frame->fourcc = pix_fmt;

    return frame;
}

bool ff_vaapi_destory_surface(SharedPtr<VideoFrame>& frame)
{
    VADisplay m_vaDisplay = ff_vaapi_create_display();
    VASurfaceID id = (VASurfaceID)(frame->surface);
    VAStatus status = vaDestroySurfaces((VADisplay)m_vaDisplay, &id, 1);
    if (!ff_check_vaapi_status(status, "vaDestroySurfaces"))
        return false;

    return true;
}

bool ff_vaapi_load_image(SharedPtr<VideoFrame>& frame, AVFrame *in)
{
    VASurfaceID surface = (VASurfaceID)frame->surface;
    VAImage image;

    uint32_t dest_linesize[4] = {0};
    const uint8_t *src_data[4];
    uint8_t *dest_data[4];

    VADisplay m_vaDisplay = ff_vaapi_create_display();

    VAStatus status = vaDeriveImage(m_vaDisplay, surface, &image);
    if (!ff_check_vaapi_status(status, "vaDeriveImage"))
        return false;

    uint8_t *buf = NULL;
    status = vaMapBuffer(m_vaDisplay, image.buf, (void**)&buf);
    if (!ff_check_vaapi_status(status, "vaMapBuffer")) {
        vaDestroyImage(m_vaDisplay, image.image_id);
        return false;
    }

    src_data[0] = in->data[0];
    src_data[1] = in->data[1];
    src_data[2] = in->data[2];

    dest_data[0] = buf + image.offsets[0];
    dest_data[1] = buf + image.offsets[1];
    dest_data[2] = buf + image.offsets[2];

    if (in->format == AV_PIX_FMT_YUV420P) {
        dest_linesize[0] = image.pitches[0];
        dest_linesize[1] = image.pitches[1];
        dest_linesize[2] = image.pitches[2];
    } else if (in->format == AV_PIX_FMT_NV12) {
        dest_linesize[0] = image.pitches[0];
        dest_linesize[1] = image.pitches[1];
        dest_linesize[2] = image.pitches[2];
    } else {
        av_log(NULL, AV_LOG_ERROR, "Unsupported the pixel format : %s.\n", av_pix_fmt_desc_get((AVPixelFormat)in->format)->name);
        return false;
    }

    av_image_copy(dest_data, (int *)dest_linesize, src_data,
                  (int *)in->linesize, (AVPixelFormat)in->format,
                  in->width, in->height);
    frame->timeStamp = in->pts;

    ff_check_vaapi_status(vaUnmapBuffer(m_vaDisplay, image.buf), "vaUnmapBuffer");
    ff_check_vaapi_status(vaDestroyImage(m_vaDisplay, image.image_id), "vaDestroyImage");
    return true;
}

bool ff_vaapi_get_image(SharedPtr<VideoFrame>& frame, AVFrame *out)
{
    VASurfaceID surface = (VASurfaceID)frame->surface;
    VAImage image;
    VAStatus status;
    uint32_t src_linesize[4] = { 0 };
    uint32_t dest_linesize[4] = { 0 };
    const uint8_t *src_data[4];
    uint8_t *dest_data[4];

    VADisplay m_vaDisplay = ff_vaapi_create_display();

    if (out->format == AV_PIX_FMT_NV12) {
        status = vaDeriveImage(m_vaDisplay, surface, &image);
        if (!ff_check_vaapi_status(status, "vaDeriveImage"))
            return false;
    } else {
        VAImageFormat image_format;
        image_format.fourcc = VA_FOURCC_I420;
        image_format.byte_order = 1;
        image_format.bits_per_pixel = 12;
        status = vaCreateImage(m_vaDisplay, &image_format, frame->crop.width, frame->crop.height, &image);
        if (!ff_check_vaapi_status(status, "vaCreateImage"))
            return false;
        status = vaGetImage(m_vaDisplay, surface, 0, 0, out->width, out->height, image.image_id);
        if (!ff_check_vaapi_status(status, "vaGetImage"))
            return false;
    }

    uint8_t *buf = NULL;
    status = vaMapBuffer(m_vaDisplay, image.buf, (void**)&buf);
    if (!ff_check_vaapi_status(status, "vaMapBuffer")) {
        vaDestroyImage(m_vaDisplay, image.image_id);
        return false;
    }

    dest_data[0] = out->data[0];
    dest_data[1] = out->data[1];
    dest_data[2] = out->data[2];

    int plane_size = image.data_size;
    uint8_t *plane_buf = (uint8_t *)av_malloc(FFMAX(image.width * image.height * 3, plane_size));
    if (!plane_buf)
        return false;

    ff_copy_from_uswc((void *)plane_buf, (void *)buf, plane_size);

    src_data[0] = plane_buf + image.offsets[0];
    src_data[1] = plane_buf + image.offsets[1];
    src_data[2] = plane_buf + image.offsets[2];

    if (out->format == AV_PIX_FMT_YUV420P) {
        dest_linesize[0] = out->linesize[0];
        dest_linesize[1] = out->linesize[1];
        dest_linesize[2] = out->linesize[2];

        src_linesize[0] = image.pitches[0];
        src_linesize[1] = image.pitches[1];
        src_linesize[2] = image.pitches[2];
    } else if (out->format == AV_PIX_FMT_NV12) {
        dest_linesize[0] = out->linesize[0];
        dest_linesize[1] = out->linesize[1];
        dest_linesize[2] = out->linesize[2];

        src_linesize[0] = image.pitches[0];
        src_linesize[1] = image.pitches[1];
        src_linesize[2] = image.pitches[2];
    } else {
        av_log(NULL, AV_LOG_ERROR, "Unsupported the pixel format : %s.\n", av_pix_fmt_desc_get((AVPixelFormat)out->format)->name);
        return false;
    }

    av_image_copy(dest_data, (int *)dest_linesize, src_data,
                  (int *)src_linesize, (AVPixelFormat)out->format,
                  out->width, out->height);

    av_free(plane_buf);

    ff_check_vaapi_status(vaUnmapBuffer(m_vaDisplay, image.buf), "vaUnmapBuffer");
    ff_check_vaapi_status(vaDestroyImage(m_vaDisplay, image.image_id), "vaDestroyImage");
    return true;
}

YamiStatus ff_yami_alloc_surface (SurfaceAllocator* thiz, SurfaceAllocParams* params)
{
    if (!params)
        return YAMI_INVALID_PARAM;
    uint32_t size = params->size;
    uint32_t width = params->width;
    uint32_t height = params->height;
    if (!width || !height || !size)
        return YAMI_INVALID_PARAM;

    size += EXTRA_SIZE;

    VASurfaceID* v = new VASurfaceID[size];
    VAStatus status = vaCreateSurfaces(ff_vaapi_create_display(), VA_RT_FORMAT_YUV420, width,
                                       height, &v[0], size, NULL, 0);
    if (!ff_check_vaapi_status(status, "vaCreateSurfaces"))
        return YAMI_FAIL;

    params->surfaces = new intptr_t[size];
    for (uint32_t i = 0; i < size; i++) {
        params->surfaces[i] = (intptr_t)v[i];
    }
    params->size = size;
    return YAMI_SUCCESS;
}

YamiStatus ff_yami_free_surface (SurfaceAllocator* thiz, SurfaceAllocParams* params)
{
    if (!params || !params->size || !params->surfaces)
        return YAMI_INVALID_PARAM;
    uint32_t size = params->size;
    VADisplay m_vaDisplay = ff_vaapi_create_display();
    VASurfaceID *surfaces = new VASurfaceID[size];
    for (uint32_t i = 0; i < size; i++) {
        surfaces[i] = params->surfaces[i];
    }
    VAStatus status = vaDestroySurfaces((VADisplay) m_vaDisplay, &surfaces[0], size);
    delete[] surfaces;
    if (!ff_check_vaapi_status(status, "vaDestroySurfaces"))
        return YAMI_FAIL;

    delete[] params->surfaces;
    return YAMI_SUCCESS;
}

void ff_yami_unref_surface (SurfaceAllocator* thiz)
{
    //TODO
}
