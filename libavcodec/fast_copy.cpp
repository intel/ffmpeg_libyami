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

#include "fast_copy.h"
#include "libavutil/common.h"
#include "internal.h"

/*
 * Used SSE4 MOVNTDQA instruction improving performance of data copies from
 * Uncacheable Speculative Write Combining (USWC) memory to ordinary write back (WB)
 * system memory.
 * https://software.intel.com/en-us/articles/copying-accelerated-video-decode-frame-buffers/
 */

#define COPY16(dstp, srcp, load, store) \
    asm volatile (                      \
        load "  0(%[src]), %%xmm1\n"    \
        store " %%xmm1,    0(%[dst])\n" \
        : : [dst]"r"(dstp), [src]"r"(srcp) : "memory", "xmm1")

#define COPY128(dstp, srcp, load, store) \
    asm volatile (                       \
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

void *fast_copy(void *dst, void *src, size_t size)
{
    char aligned;
    int remain;
    int i, round;
    uint8_t *pDst, *pSrc;

    if (dst == NULL || src == NULL) {
        return NULL;
    }

    aligned = (((size_t) dst) | ((size_t) src)) & 0x0F;

    if (aligned != 0) {
        printf("No data aligned!!!\n");
        return NULL;
    }

    pDst = (uint8_t *) dst;
    pSrc = (uint8_t *) src;
    remain = size & 0x7F;
    round = size >> 7;

    asm volatile ("mfence");

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
    asm volatile ("mfence");

    return dst;
}

