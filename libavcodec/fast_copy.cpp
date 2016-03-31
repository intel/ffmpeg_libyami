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
#include <smmintrin.h>

/*
 * Used SSE4 MOVNTDQA instruction improving performance of data copies from 
 * Uncacheable Speculative Write Combining (USWC) memory to ordinary write back (WB) 
 * system memory.
 * https://software.intel.com/en-us/articles/copying-accelerated-video-decode-frame-buffers/
 */
void *fast_copy(void *dst, void *src, size_t size)
{
    char aligned;
    int remain;
    int i, round;
    __m128i x0, x1, x2, x3, x4, x5, x6, x7;
    __m128i *pDst, *pSrc;

    if (dst == NULL || src == NULL) {
        return NULL;
    }

    aligned = (((size_t) dst) | ((size_t) src)) & 0x0F;

    if (aligned != 0) {
        printf("No data aligned!!!\n");
        return NULL;
    }

    pDst = (__m128i *) dst;
    pSrc = (__m128i *) src;
    remain = size & 0x7F;
    round = size >> 7;
    _mm_mfence();

    for (i = 0; i < round; i++) {
        x0 = _mm_stream_load_si128(pSrc + 0);
        x1 = _mm_stream_load_si128(pSrc + 1);
        x2 = _mm_stream_load_si128(pSrc + 2);
        x3 = _mm_stream_load_si128(pSrc + 3);
        x4 = _mm_stream_load_si128(pSrc + 4);
        x5 = _mm_stream_load_si128(pSrc + 5);
        x6 = _mm_stream_load_si128(pSrc + 6);
        x7 = _mm_stream_load_si128(pSrc + 7);

        _mm_store_si128(pDst + 0, x0);
        _mm_store_si128(pDst + 1, x1);
        _mm_store_si128(pDst + 2, x2);
        _mm_store_si128(pDst + 3, x3);
        _mm_store_si128(pDst + 4, x4);
        _mm_store_si128(pDst + 5, x5);
        _mm_store_si128(pDst + 6, x6);
        _mm_store_si128(pDst + 7, x7);
        pSrc += 8;
        pDst += 8;
    }

    if (remain >= 16) {
        size = remain;
        remain = size & 0xF;
        round = size >> 4;

        for (i = 0; i < round; i++) {
            x0 = _mm_stream_load_si128(pSrc + 0);
            pSrc += 1;
            _mm_store_si128(pDst, x0);
            pDst += 1;
        }
    }

    if ( remain > 0 ) {
        char *ps = (char *)(pSrc);
        char *pd = (char *)(pDst);

        for (i = 0; i < remain; i++) {
            pd[i] = ps[i];
        }
    }

    return dst;
}

