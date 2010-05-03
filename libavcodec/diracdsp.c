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

static void dirac_hpel_filter(uint8_t *dsth, uint8_t *dstv, uint8_t *dstc, uint8_t *src,
                              int stride, int width, int height)
{
    int x, y;

    for (y = 0; y < height; y++) {
        for (x = -3; x < width+4; x++)
            dstv[x] = av_clip_uint8(FILTER(src+x, stride));

        for (x = 0; x < width; x++)
            dsth[x] = av_clip_uint8(FILTER(src+x, 1));

        for (x = 0; x < width; x++)
            dstc[x] = av_clip_uint8(FILTER(dstv+x, 1));

        src  += stride;
        dsth += stride;
        dstv += stride;
        dstc += stride;
    }
}

#define TAPFILTER(pix, d) ((pix)[x-2*d] + (pix)[x+3*d] - 5*((pix)[x-d] + (pix)[x+2*d]) + 20*((pix)[x] + (pix)[x+d]))
static void hpel_filter( uint8_t *dsth, uint8_t *dstv, uint8_t *dstc, uint8_t *src,
                         int stride, int width, int height )
{
    int16_t buf[width+8];
    for( int y = 0; y < height; y++ )
    {
        for( int x = -2; x < width+3; x++ )
        {
            int v = TAPFILTER(src,stride);
            dstv[x] = av_clip_uint8( (v + 16) >> 5 );
            buf[x+2] = v;
        }
        for( int x = 0; x < width; x++ )
            dstc[x] = av_clip_uint8( (TAPFILTER(buf+2,1) + 512) >> 10 );
        for( int x = 0; x < width; x++ )
            dsth[x] = av_clip_uint8( (TAPFILTER(src,1) + 16) >> 5 );
        dsth += stride;
        dstv += stride;
        dstc += stride;
        src += stride;
    }
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

void ff_diracdsp_init(DSPContext* dsp, AVCodecContext *avctx)
{
    dsp->dirac_hpel_filter = dirac_hpel_filter;
    // dsp->dirac_hpel_filter = hpel_filter;
}
