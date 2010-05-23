/**
 * VP8 compatible video decoder
 *
 * Copyright (C) 2010 David Conrad
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

// TODO: Maybe add dequant
static void vp8_luma_dc_idct_c(DCTELEM block[4][4][16])
{
    int i, t0, t1, t2, t3;

    for (i = 0; i < 4; i++) {
        t0 = *block[0][i] + *block[3][i];
        t1 = *block[1][i] + *block[2][i];
        t2 = *block[1][i] - *block[2][i];
        t3 = *block[0][i] - *block[3][i];

        *block[0][i] = t0 + t3;
        *block[1][i] = t1 + t2;
        *block[2][i] = t1 - t2;
        *block[3][i] = t0 - t3;
    }

    for (i = 0; i < 4; i++) {
        t0 = *block[i][0] + *block[i][3];
        t1 = *block[i][1] + *block[i][2];
        t2 = *block[i][1] - *block[i][2];
        t3 = *block[i][0] - *block[i][3];

        *block[i][0] = (t0 + t3 + 3) >> 3;
        *block[i][1] = (t1 + t2 + 3) >> 3;
        *block[i][2] = (t1 - t2 + 3) >> 3;
        *block[i][3] = (t0 - t3 + 3) >> 3;
    }
}

#define MUL_20091(a) ((((a)*20091) >> 16) + (a))
#define MUL_35468(a)  (((a)*35468) >> 16)

static void vp8_idct_add_c(uint8_t *dst, DCTELEM block[16], int stride)
{
    int i, t0, t1, t2, t3;
    DCTELEM tmp[16];

    for (i = 0; i < 4; i++) {
        t0 = block[0*4+i] + block[2*4+i];
        t1 = block[0*4+i] - block[2*4+i];
        t2 = MUL_35468(block[1*4+i]) - MUL_20091(block[3*4+i]);
        t3 = MUL_20091(block[1*4+i]) + MUL_35468(block[3*4+i]);

        tmp[i*4+0] = t0 + t3;
        tmp[i*4+1] = t1 + t2;
        tmp[i*4+2] = t1 - t2;
        tmp[i*4+3] = t0 - t3;
    }

    for (i = 0; i < 4; i++) {
        t0 = tmp[0*4+i] + tmp[2*4+i];
        t1 = tmp[0*4+i] - tmp[2*4+i];
        t2 = MUL_35468(tmp[1*4+i]) - MUL_20091(tmp[3*4+i]);
        t3 = MUL_20091(tmp[1*4+i]) + MUL_35468(tmp[3*4+i]);

        dst[0] = av_clip_uint8(dst[0] + ((t0 + t3 + 4) >> 3));
        dst[1] = av_clip_uint8(dst[1] + ((t1 + t2 + 4) >> 3));
        dst[2] = av_clip_uint8(dst[2] + ((t1 - t2 + 4) >> 3));
        dst[3] = av_clip_uint8(dst[3] + ((t0 - t3 + 4) >> 3));
        dst += stride;
    }
}
