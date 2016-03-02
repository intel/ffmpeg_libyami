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

static VADisplay getVADisplay(void)
{
    static VADisplay vadisplay = NULL;


    if (!vadisplay) {
        int fd = open("/dev/dri/card0", O_RDWR);
        if (fd < 0) {
//            av_log(NULL, AV_LOG_ERROR, "open card0 failed");
            return NULL;
        }
        vadisplay = vaGetDisplayDRM(fd);
        int majorVersion, minorVersion;
        VAStatus vaStatus = vaInitialize(vadisplay, &majorVersion, &minorVersion);
        if (vaStatus != VA_STATUS_SUCCESS) {
//            av_log(NULL, AV_LOG_ERROR, "va init failed, status =  %d", vaStatus);
            close(fd);
            vadisplay = NULL;
            return NULL;
        }
        return vadisplay;
    } else {
        return vadisplay;
    }
}

VADisplay createVADisplay(void)
{
    return getVADisplay();
}
