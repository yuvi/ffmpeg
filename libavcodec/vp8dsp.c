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
// I like the original with dst == src better, but the coeff decode seems more
// suited for the second
#if 0
static void vp8_luma_dc_wht_c(DCTELEM block[4][4][16])
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
#else
static void vp8_luma_dc_wht_c(DCTELEM block[4][4][16], DCTELEM dc[16])
{
    int i, t0, t1, t2, t3;

    for (i = 0; i < 4; i++) {
        t0 = dc[0*4+i] + dc[3*4+i];
        t1 = dc[1*4+i] + dc[2*4+i];
        t2 = dc[1*4+i] - dc[2*4+i];
        t3 = dc[0*4+i] - dc[3*4+i];

        dc[0*4+i] = t0 + t3;
        dc[1*4+i] = t1 + t2;
        dc[2*4+i] = t1 - t2;
        dc[3*4+i] = t0 - t3;
    }

    for (i = 0; i < 4; i++) {
        t0 = dc[i*4+0] + dc[i*4+3];
        t1 = dc[i*4+1] + dc[i*4+2];
        t2 = dc[i*4+1] - dc[i*4+2];
        t3 = dc[i*4+0] - dc[i*4+3];

        *block[i][0] = (t0 + t3 + 3) >> 3;
        *block[i][1] = (t1 + t2 + 3) >> 3;
        *block[i][2] = (t1 - t2 + 3) >> 3;
        *block[i][3] = (t0 - t3 + 3) >> 3;
    }
}
#endif


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

// because I like only having two parameters to pass functions...
#define LOAD_PIXELS\
    int av_unused p3 = p[-4*stride];\
    int av_unused p2 = p[-3*stride];\
    int av_unused p1 = p[-2*stride];\
    int av_unused p0 = p[-1*stride];\
    int av_unused q0 = p[ 0*stride];\
    int av_unused q1 = p[ 1*stride];\
    int av_unused q2 = p[ 2*stride];\
    int av_unused q3 = p[ 3*stride];

#define clip_int8(x) av_clip(x, -128, 127)

static void filter_common(uint8_t *p, int stride, int is4tap)
{
    LOAD_PIXELS
    int a, b;

    a = 3*(q0 - p0);

    if (is4tap)
        a += clip_int8(p1 - q1);

    a = clip_int8(a);

    b = (a & 7) ? -1 : 0;
    a = clip_int8(a+4) >> 3;

    // the ref doesn't clamp here, do we need to?
    p[-1*stride] += a+b;
    p[ 0*stride] -= a;

    // assuming this is equivalent to !hv in subblock_filter and
    // 4tap is true everywhere else
    if (!is4tap) {
        p[-2*stride] += (a+1)>>1;
        p[ 1*stride] -= (a+1)>>1;
    }
}

static int simple_limit(uint8_t *p, int stride, int flim)
{
    LOAD_PIXELS
    return 2*FFABS(p0-q0) + FFABS(p1-q1)/2 <= flim;
}

/**
 * I - limit for interior difference
 * E - limit at the edge
 */
static int normal_limit(uint8_t *p, int stride, int I, int E)
{
    LOAD_PIXELS
    return simple_limit(p, stride, E)
        && FFABS(p3-p2) <= I && FFABS(p2-p1) <= I && FFABS(p1-p0) <= I
        && FFABS(q3-q2) <= I && FFABS(q2-q1) <= I && FFABS(q1-q0) <= I;
}

// high edge variance
static int hev(uint8_t *p, int stride, int thresh)
{
    LOAD_PIXELS
    return FFABS(p1-p0) > thresh || FFABS(q1-q0) > thresh;
}

static void filter_mb(uint8_t *p, int stride,
                      int hev_thresh, int flim_I, int flim_E)
{
    int a0, a1, a2, w;

    if (normal_limit(p, stride, flim_I, flim_E)) {
        if (!hev(p, stride, hev_thresh)) {
            LOAD_PIXELS

            w = clip_int8(p1-q1);
            w = clip_int8(w + 3*(q0-p0));

            a0 = clip_int8(27*w + 63) >> 7;
            a1 = clip_int8(18*w + 63) >> 7;
            a2 = clip_int8( 9*w + 63) >> 7;

            p[-3*stride] += a2;
            p[-2*stride] += a1;
            p[-1*stride] += a0;
            p[ 0*stride] -= a0;
            p[ 1*stride] -= a1;
            p[ 2*stride] -= a2;
        } else
            filter_common(p, stride, 1);
    }
}

static void vp8_v_loop_filter16_simple_c(uint8_t *dst, int stride, int flim)
{
    int i;

    for (i = 0; i < 16; i++)
        if (simple_limit(dst+i, stride, flim))
            filter_common(dst+i, stride, 1);
}

static void vp8_v_subblock_filter16_c(uint8_t *dst, int stride,
                                      int hev_thresh, int flim_I, int flim_E)
{
    int i;

    for (i = 0; i < 16; i++)
        if (normal_limit(dst, stride, flim_I, flim_E)) {
            int hv = hev(dst, stride, hev_thresh);
            filter_common(dst, stride, hv);
        }
}

// todo: 15.4 calc of the parameters

av_cold void ff_vp8dsp_init(DSPContext* dsp, AVCodecContext *avctx)
{
    dsp->vp8_luma_dc_wht = vp8_luma_dc_wht_c;
    dsp->vp8_idct_add    = vp8_idct_add_c;
}
