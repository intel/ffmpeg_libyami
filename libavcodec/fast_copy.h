/*
 * Copyright (c) 2016 Intel Corporation
 *     Jun Zhao(jun.zhao@intel.com)
 *     Zhou Yun(yunx.z.zhou@intel.com)
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
#ifndef LIBAVCODEC_FAST_COPY_H_
#define LIBAVCODEC_FAST_COPY_H_

#include <stdio.h>
void *fast_copy( void *dst, void *src, size_t size );

#endif /* LIBAVCODEC_FAST_COPY_H_ */

