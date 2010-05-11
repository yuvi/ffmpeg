/*
 * Copyright (C) 2009 David Conrad
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

#include "dsputil.h"
#include "diracdsp.h"

#define FILTER(src, stride) \
  ((21*((src)[ 0*stride] + (src)[1*stride]) \
    -7*((src)[-1*stride] + (src)[2*stride]) \
    +3*((src)[-2*stride] + (src)[3*stride]) \
    -1*((src)[-3*stride] + (src)[4*stride]) + 16) >> 5)

static void dirac_hpel_filter(uint8_t *dsth, uint8_t *dstv, uint8_t *dstc, uint8_t *src,
                              int stride, int width, int height)
{
    int x, y;

    for (y = 0; y < height; y++) {
        for (x = -3; x < width+4; x++)
            dstv[x] = av_clip_uint8(FILTER(src+x, stride));

        for (x = 0; x < width; x++)
            dstc[x] = av_clip_uint8(FILTER(dstv+x, 1));

        for (x = 0; x < width; x++)
            dsth[x] = av_clip_uint8(FILTER(src+x, 1));

        src  += stride;
        dsth += stride;
        dstv += stride;
        dstc += stride;
    }
}

#define PIXOP_BILINEAR(PFX, OP, WIDTH)\
static void ff_ ## PFX ## _dirac_pixels ## WIDTH ## _bilinear_c(uint8_t *dst, uint8_t *src[5], int stride, int h)\
{\
    int x;\
    uint8_t *s0 = src[0];\
    uint8_t *s1 = src[1];\
    uint8_t *s2 = src[2];\
    uint8_t *s3 = src[3];\
    uint8_t *w  = src[4];\
\
    while (h--) {\
        for (x = 0; x < WIDTH; x++) {\
            OP(dst[x], (s0[x]*w[0] + s1[x]*w[1] + s2[x]*w[2] + s3[x]*w[3] + 8) >> 4);\
        }\
\
        dst += stride;\
        s0 += stride;\
        s1 += stride;\
        s2 += stride;\
        s3 += stride;\
    }\
}

#define OP_PUT(dst, val) (dst) = (val)
#define OP_AVG(dst, val) (dst) = (((dst) + (val) + 1)>>1)

PIXOP_BILINEAR(put, OP_PUT, 8)
PIXOP_BILINEAR(put, OP_PUT, 16)
PIXOP_BILINEAR(put, OP_PUT, 32)
PIXOP_BILINEAR(avg, OP_AVG, 8)
PIXOP_BILINEAR(avg, OP_AVG, 16)
PIXOP_BILINEAR(avg, OP_AVG, 32)

#define PIXFUNC(PFX, WIDTH) \
    dsp->PFX ## _dirac_pixels_tab[WIDTH>>3][0] = ff_ ## PFX ## _dirac_pixels ## WIDTH ## _c; \
    dsp->PFX ## _dirac_pixels_tab[WIDTH>>3][1] = ff_ ## PFX ## _dirac_pixels ## WIDTH ## _l2_c; \
    dsp->PFX ## _dirac_pixels_tab[WIDTH>>3][2] = ff_ ## PFX ## _dirac_pixels ## WIDTH ## _l4_c; \
    dsp->PFX ## _dirac_pixels_tab[WIDTH>>3][3] = ff_ ## PFX ## _dirac_pixels ## WIDTH ## _bilinear_c

void ff_diracdsp_init(DSPContext *dsp, AVCodecContext *avctx)
{
    dsp->dirac_hpel_filter = dirac_hpel_filter;

    PIXFUNC(put, 8);
    PIXFUNC(put, 16);
    PIXFUNC(put, 32);
    PIXFUNC(avg, 8);
    PIXFUNC(avg, 16);
    PIXFUNC(avg, 32);
}
