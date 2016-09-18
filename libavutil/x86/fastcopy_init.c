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

#include "config.h"

#include "fastcopy.h"
#include "cpu.h"

void *ff_copy_mem_from_uswc_avx(void *dst, void *src, size_t size);

void *ff_copy_mem_from_uswc_sse4(void *dst, void *src, size_t size);

void ff_fastcopy_uswc_init_x86(FastCopyAccel *fc)
{
    int cpu_flags = av_get_cpu_flags();

    if (EXTERNAL_SSE4(cpu_flags)) {
        fc->fastcopy_fn = ff_copy_mem_from_uswc_sse4;
        fc->register_bits = 128;
        fc->align_bits = 0x0F;
        fc->offset_bits = 7;
    }

    if (EXTERNAL_AVX(cpu_flags)) {
        fc->fastcopy_fn = ff_copy_mem_from_uswc_avx;
        fc->register_bits = 256;
        fc->align_bits = 0x1F;
        fc->offset_bits = 8;
    }

}
