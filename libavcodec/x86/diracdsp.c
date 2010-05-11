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
#include "diracdsp.h"

#define HPEL_FILTER(MMSIZE, EXT) \
void ff_dirac_hpel_filter_v_ ## EXT(uint8_t *, uint8_t *, int, int);\
void ff_dirac_hpel_filter_h_ ## EXT(uint8_t *, uint8_t *, int);\
\
static void dirac_hpel_filter_ ## EXT(uint8_t *dsth, uint8_t *dstv, uint8_t *dstc,\
                                      uint8_t *src, int stride, int width, int height)\
{\
    while( height-- )\
    {\
        ff_dirac_hpel_filter_v_ ## EXT(dstv-MMSIZE, src-MMSIZE, stride, width+MMSIZE);\
        ff_dirac_hpel_filter_h_ ## EXT(dsth, src, width);\
        ff_dirac_hpel_filter_h_ ## EXT(dstc, dstv, width);\
\
        dsth += stride;\
        dstv += stride;\
        dstc += stride;\
        src  += stride;\
    }\
}

#if !ARCH_X86_64
HPEL_FILTER(8, mmx)
#endif
HPEL_FILTER(16, sse2)

#define PIXFUNC(PFX, IDX, EXT) \
    dsp->PFX ## _dirac_pixels_tab[0][IDX] = ff_ ## PFX ## _dirac_pixels8_ ## EXT; \
    dsp->PFX ## _dirac_pixels_tab[1][IDX] = ff_ ## PFX ## _dirac_pixels16_ ## EXT; \
    dsp->PFX ## _dirac_pixels_tab[2][IDX] = ff_ ## PFX ## _dirac_pixels32_ ## EXT

void ff_diracdsp_init_mmx(DSPContext* dsp, AVCodecContext *avctx)
{
    mm_flags = mm_support();

#if !ARCH_X86_64
    dsp->dirac_hpel_filter = dirac_hpel_filter_mmx;
#endif

    PIXFUNC(put, 0, mmx);
    PIXFUNC(avg, 0, mmx);

    if (mm_flags & FF_MM_MMX2) {
        PIXFUNC(avg, 0, mmx2);
    }

    if (mm_flags & FF_MM_SSE2) {
        dsp->dirac_hpel_filter = dirac_hpel_filter_sse2;

        dsp->put_dirac_pixels_tab[1][1] = ff_put_dirac_pixels16_sse2;
        dsp->avg_dirac_pixels_tab[1][1] = ff_avg_dirac_pixels16_sse2;
        dsp->put_dirac_pixels_tab[1][2] = ff_put_dirac_pixels32_sse2;
        dsp->avg_dirac_pixels_tab[1][2] = ff_avg_dirac_pixels32_sse2;
    }
}
