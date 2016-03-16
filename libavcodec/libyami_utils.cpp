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

#include "libyami_utils.h"

#define HAVE_VAAPI_X11 0

#if HAVE_VAAPI_X11
#include <X11/Xlib.h>
#endif

static VADisplay ff_get_display(void)
{
    static VADisplay display = NULL;
    
    if(!display) {
#if HAVE_VAAPI_X11
        const char *device = NULL;/*FIXME*/
            // Try to open the device as an X11 display.
        Display *x11_display = XOpenDisplay(device);
        if(!x11_display) {
            return NULL;
        } else {
            display = vaGetDisplay(x11_display);
            if(!display) {
                XCloseDisplay(x11_display);
            } 
        }
#else
        const char *device = "/dev/dri/card0";/*FIXME*/
        // Try to open the device as a DRM path.
        int drm_fd = open(device, O_RDWR);
        if(drm_fd < 0) {
            return NULL;
        } else {
            display = vaGetDisplayDRM(drm_fd);
            if(!display) 
                close(drm_fd);
        }    
#endif
        if (!display)
            return NULL;
        int majorVersion, minorVersion;
        VAStatus vaStatus = vaInitialize(display, &majorVersion, &minorVersion);
        if (vaStatus != VA_STATUS_SUCCESS) {
#if !HAVE_VAAPI_X11
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

VADisplay ff_vaapi_create_display(void)
{
    return ff_get_display();
}
