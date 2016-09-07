/*
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

#ifndef AVUTIL_FASTCOPY_H
#define AVUTIL_FASTCOPY_H

#include <stddef.h>
#include <stdint.h>
#include "common.h"

typedef void (*av_fastcopy_uswc_fn)(void *dst, void *src, size_t size);

typedef struct FastCopyAccel {
    av_fastcopy_uswc_fn fastcopy_fn;
    int register_bits;
    int align_bits;
    int offset_bits;
} FastCopyAccel;

void av_fastcopy_uswc_init();

FastCopyAccel *av_fastcopy_uswc_get_fn();

#endif
