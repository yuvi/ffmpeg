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
#include "dwt.h"

/*
   -1  3 -7 21 21 -7  3 -1
-1           o  o
 3           o  o
-7           o  o
21  o  o  o  x  x  x  x  x
21  o  o  o  x  x  x  x  x
-7           x  x
 3           x  x
-1           x  x

*/

#define FILTER(src, stride) \
  ((21*((src)[ 0*stride] + (src)[1*stride]) \
    -7*((src)[-1*stride] + (src)[2*stride]) \
    +3*((src)[-2*stride] + (src)[3*stride]) \
    -1*((src)[-3*stride] + (src)[4*stride]) + 16) >> 5)

static void dirac_hpel_filter(uint8_t *hpel_planes[4], int stride, int width, int height)
{
    int x, y;
    uint8_t *src  = hpel_planes[0];
    uint8_t *dsth = hpel_planes[1];
    uint8_t *dstv = hpel_planes[2];
    uint8_t *dstc = hpel_planes[3];

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

static inline void mc_copy(uint8_t *dst, int dst_stride,
                           uint8_t *src, int src_stride,
                           int width, int height)
{
    int x, y;
    for (y = 0; y < height>>1; y++) {
#if HAVE_FAST_64BIT
        for (x = 0; x < width>>3; x++) {
            AV_WN64A(dst+x,            AV_RN64(src+x));
            AV_WN64A(dst+x+dst_stride, AV_RN64(src+x+src_stride));
        }
#else
        for (x = 0; x < width>>2; x++) {
            AV_WN32A(dst+x,            AV_RN32(src+x));
            AV_WN32A(dst+x+dst_stride, AV_RN32(src+x+src_stride));
        }
#endif
        dst += 2*dst_stride;
        src += 2*src_stride;
    }
}

static inline void mc_avg2(uint8_t *dst,    int dst_stride,
                           uint8_t *src[2], int src_stride,
                           int width, int height)
{
    int x, y;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            dst[x] = (src[0][x] + src[1][x] + 1)>>1;
        dst += dst_stride;
        src[0] += src_stride;
        src[1] += src_stride;
    }
}

static inline void mc_avg4(uint8_t *dst,    int dst_stride,
                           uint8_t *src[4], int src_stride,
                           int width, int height)
{
    int x, y;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            dst[x] = (src[0][x] + src[1][x] + src[2][x] + src[3][x] + 2)>>2;
        dst += dst_stride;
        src[0] += src_stride;
        src[1] += src_stride;
        src[2] += src_stride;
        src[3] += src_stride;
    }
}

static void put_dirac_fpel(uint8_t *dst,    int dst_stride,
                           uint8_t *src[4], int src_stride,
                           int offset, int mvx, int mvy,
                           int width, int height)
{
    uint8_t *src0 = src[0] + offset + mvy * src_stride + mvx;
    mc_copy(dst, dst_stride, src0, src_stride, width, height);
}

static void avg_dirac_fpel(uint8_t *dst,    int dst_stride,
                           uint8_t *src[4], int src_stride,
                           int offset, int mvx, int mvy,
                           int width, int height)
{
    uint8_t *src0 = src[0] + offset + mvy * src_stride + mvx;
    int x, y;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            dst[x] = (src0[x] + dst[x] + 1)>>1;

        dst += dst_stride;
        src0 += src_stride;
    }
}

static void dirac_mc_hpel(uint8_t *dst,    int dst_stride,
                          uint8_t *src[4], int src_stride,
                          int mvx, int mvy,
                          int width, int height)
{
    int dxy = ((mvy&1)<<1) + (mvx&1);
    uint8_t *src0 = src[dxy] + (mvy>>1) * src_stride + (mvx>>1);
    mc_copy(dst, dst_stride, src0, src_stride, width, height);
}

/*
   -1  3 -7 21 21 -7  3 -1
-1           o  o
 3           o  o
-7           o  o
21  o  o  o  x  x  x  x  x
21  o  o  o  x  x  x  x  x
-7           x  x
 3           x  x
-1           x  x




0 2 1 2 0
2 2 2 2 2
1 2 1 2 1
2 2 2 2 2
0 2 1 2 0

[0] 0,1 [1] 0,3 [0]
1,0 1,1 1,2 1,3
[2] 2,1 [3] 2,3 [2]
3,0 3,1 3,2 3,3
[0]     [1]     [0]

[0]  1  [1]  3  [0]
 4   5   6   7
[2]  9  [3] 10  [2]
12  13  14  15
[0]     [1]     [0]

*/

static const uint8_t ref[16][4] = {
    {0,0,0,0}, {0,1,0,1}, {1,1,1,1}, {1,0,1,0},
    {0,2,0,2}, {0,2,1,3}, {1,3,1,3}, {1,3,0,2},
    {2,2,2,2}, {2,3,2,3}, {3,3,3,3}, {3,2,3,2},
    {2,0,2,0}, {2,0,3,1}, {3,1,3,1}, {3,1,2,0}
};

static const int ref0[16] = { 0,0,1,1, 0,0,1,0, 2,2,3,3, 2,2,3,2};
static const int ref1[16] = { 0,1,1,0, 2,3,3,3, 2,3,3,2, 0,1,1,1};

static void dirac_mc_qpel(uint8_t *dst,    int dst_stride,
                          uint8_t *src[4], int src_stride,
                          int mvx, int mvy,
                          int width, int height)
{
    int i, dxy = ((mvy&3)<<2) + (mvx&3);
    int offset = (mvy>>2) * src_stride + (mvx>>2);
    int dxy_hpel = (mvy&2) + ((mvx&2)>>1);
    uint8_t *src_act[4];

    if (!(dxy&5)) { // hpel
        src_act[0] = src[dxy_hpel] + offset;
        mc_copy(dst, dst_stride, src_act[0], src_stride, width, height);
    } else if ((dxy&5) < 5) { // qpel, avg of 2
                              // probably not worth the special case in C
        src_act[0] = src[ref0[dxy]] + offset;
        src_act[1] = src[ref1[dxy]] + offset;
        mc_avg2(dst, dst_stride, src_act, src_stride, width, height);
    } else { // qpel, avg of 4
        for (i = 0; i < 4; i++)
            src_act[i] = src[i] + offset;
        src_act[0] += ((mvy&3) == 3) ? src_stride : 0 + ((mvx&3) == 3);
        src_act[1] += ((mvy&3) == 3) ? src_stride : 0;
        src_act[2] +=                                   ((mvx&3) == 3);
        mc_avg4(dst, dst_stride, src_act, src_stride, width, height);
    }
}

/*
    0 1 2 3 4 5 6 7

0   f 3 2 3 h 3 2 3 f
1   3 3 3 3 3 3 3 3 3
2   2 3 2 3 2 3 2 3 2
3   3 3 3 3 3 3 3 3 3
4   v 3 2 3 c 3 2 3 v
5   3 3 3 3 3 3 3 3 3
6   2 3 2 3 2 3 2 3 2
7   3 3 3 3 3 3 3 3 3
    f 3 2 3 h 3 2 3 f

*/

static void dirac_mc_epel(uint8_t *dst,    int dst_stride,
                          uint8_t *src[4], int src_stride,
                          int mvx, int mvy,
                          int width, int height)
{
    int x, y;
    int mx = mvx&3;
    int my = mvy&3;
    int offset = (mvy>>3)*src_stride + (mvx>>3);
    int A = (4-mx)*(4-my);
    int B = (  mx)*(4-my);
    int C = (4-mx)*(  my);
    int D = (  mx)*(  my);

    uint8_t *src0 = src[0] + offset + ((mvy&4)>>2)*src_stride + ((mvx&4)>>2);
    uint8_t *src1 = src[1] + offset + ((mvy&4)>>2)*src_stride;
    uint8_t *src2 = src[2] + offset +                           ((mvx&4)>>2);
    uint8_t *src3 = src[3] + offset;

    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            dst[x] = (A*src0[x] + B*src1[x] + C*src2[x] + D*src3[x] + 8)>>4;
        dst  += dst_stride;
        src0 += src_stride;
        src1 += src_stride;
        src2 += src_stride;
        src3 += src_stride;
    }
}

static void dirac_add_obmc(uint8_t *dst, int dst_stride,
                           uint8_t *obmc_curr, uint8_t *obmc_last, int obmc_stride,
                           uint8_t *obmc_weights, int xblen, int yblen, int xbsep, int ybsep)
{
#if 0
    for (y = 0; y < yblen - ybsep; y++) {gi
        for (x = 0; x < xblen - xbsep; x++) {
            
        }
        for (; x < xbsep; x++) {
            
        }
    }
#endif
}

void ff_diracdsp_init(DSPContext* dsp, AVCodecContext *avctx)
{
    dsp->dirac_hpel_filter = dirac_hpel_filter;
    dsp->put_dirac_tab[0] = put_dirac_fpel;
    dsp->avg_dirac_tab[0] = avg_dirac_fpel;
}
