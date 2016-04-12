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
#include "libyami_utils.h"

#include <fcntl.h>
#include <unistd.h>

#define HAVE_VAAPI_DRM 1

#if HAVE_VAAPI_X11
#include <X11/Xlib.h>
#endif

#if HAVE_SSE4
#include "fast_copy.h"
#endif

VADisplay ff_vaapi_create_display(void)
{
    static VADisplay display = NULL;

    if (!display) {
#if !HAVE_VAAPI_DRM
        const char *device = NULL;/*FIXME*/
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
        const char *device = "/dev/dri/card0";/*FIXME*/
        // Try to open the device as a DRM path.
        int drm_fd = open(device, O_RDWR);
        if (drm_fd < 0) {
            return NULL;
        } else {
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

inline bool ff_check_vaapi_status(VAStatus status, const char *msg)
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
    frame->surface = (intptr_t)id;
    frame->crop.x = frame->crop.y = 0;
    frame->crop.width = w;
    frame->crop.height = h;

    return frame;
}

bool ff_vaapi_delete_surface(SharedPtr<VideoFrame>& frame)
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
    }

    av_image_copy(dest_data, (int *)dest_linesize, src_data,
                  (int *)in->linesize, (AVPixelFormat)in->format,
                  in->width, in->height);

    ff_check_vaapi_status(vaUnmapBuffer(m_vaDisplay, image.buf), "vaUnmapBuffer");
    ff_check_vaapi_status(vaDestroyImage(m_vaDisplay, image.image_id), "vaDestroyImage");
    return true;
}

bool ff_vaapi_get_image(SharedPtr<VideoFrame>& frame, AVFrame *out)
{
    VASurfaceID surface = (VASurfaceID)frame->surface;
    VAImage image;

    uint32_t src_linesize[4] = { 0 };
    uint32_t dest_linesize[4] = { 0 };
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

    dest_data[0] = out->data[0];
    dest_data[1] = out->data[1];
    dest_data[2] = out->data[2];

    int plane_size = image.offsets[1] / 2 + image.offsets[1];
    uint8_t *plane_buf = (uint8_t *)av_malloc(image.width * image.height * 3);
    if (!plane_buf)
        return false;
#if HAVE_SSE4
    fast_copy((void *)plane_buf, (void *)buf, plane_size);
#else
    memcpy(plane_buf, buf, plane_size);
#endif

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
    }

    /*
     * TODO:
     * Need to improving performance of data copies from Uncacheable
     * Speculative Write Combining(USWC) memory to ordinary Write Back(WB)
     * system memory
     * https://software.intel.com/en-us/articles/copying-accelerated-video-decode-frame-buffers/
     */
    av_image_copy(dest_data, (int *)dest_linesize, src_data,
                  (int *)src_linesize, (AVPixelFormat)out->format,
                  out->width, out->height);

    av_free(plane_buf);

    ff_check_vaapi_status(vaUnmapBuffer(m_vaDisplay, image.buf), "vaUnmapBuffer");
    ff_check_vaapi_status(vaDestroyImage(m_vaDisplay, image.image_id), "vaDestroyImage");
    return true;
}
