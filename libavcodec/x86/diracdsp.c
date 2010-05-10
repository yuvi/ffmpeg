/*
 * This file is part of FFmpeg.
 * Copyright (c) 2010 David Conrad
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

#include "dsputil_mmx.h"

#define HPEL_FILTER(mmsize, ext) \
void ff_dirac_hpel_filter_v_ ## ext(uint8_t *, uint8_t *, int, int); \
void ff_dirac_hpel_filter_h_ ## ext(uint8_t *, uint8_t *, int); \
\
static void dirac_hpel_filter_ ## ext(uint8_t *dsth, uint8_t *dstv, uint8_t *dstc, \
                                      uint8_t *src, int stride, int width, int height) \
{ \
    while( height-- ) \
    { \
        ff_dirac_hpel_filter_v_ ## ext(dstv-mmsize, src-mmsize, stride, width+mmsize); \
        ff_dirac_hpel_filter_h_ ## ext(dsth, src, width); \
        ff_dirac_hpel_filter_h_ ## ext(dstc, dstv, width); \
\
        dsth += stride; \
        dstv += stride; \
        dstc += stride; \
        src  += stride; \
    } \
}

HPEL_FILTER(8, mmx)
HPEL_FILTER(16, sse2)

void ff_diracdsp_init_mmx(DSPContext* dsp, AVCodecContext *avctx)
{
    mm_flags = mm_support();

    if (!ARCH_X86_64) {
        dsp->dirac_hpel_filter = dirac_hpel_filter_mmx;
    }

    if (mm_flags & FF_MM_SSE2) {
        dsp->dirac_hpel_filter = dirac_hpel_filter_sse2;
    }
}
