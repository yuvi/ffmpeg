/*
 * Copyright (C) 2004 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (C) 2008 David Conrad
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

#include "avcodec.h"
#include "dsputil.h"
#include "dwt.h"

static inline int mirror(int v, int m){
    if     (v<0) return -v;
    else if(v>m) return 2*m-v;
    else         return v;
}

static inline int extend(int v, int m){
    if     (v<0) return 0;
    else if(v>m) return m;
    else         return v;
}

static inline int extend_odd(int v, int m){
    if     (v<1) return 1;
    else if(v>m) return m;
    else         return v;
}

#define COMPOSE_53iL0(b0, b1, b2)\
    (b1 - ((b0 + b2 + 2) >> 2))

#define COMPOSE_DIRAC53iH0(b0, b1, b2)\
    (b1 + ((b0 + b2 + 1) >> 1))

#define COMPOSE_DD97iH0(b0, b1, b2, b3, b4)\
    (b2 + ((-b0 + 9*b1 + 9*b3 - b4 + 8) >> 4))

#define COMPOSE_DD137iL0(b0, b1, b2, b3, b4)\
    (b2 - ((-b0 + 9*b1 + 9*b3 - b4 + 16) >> 5))

#define COMPOSE_HAARiL0(b0, b1)\
    (b0 - ((b1 + 1) >> 1))

#define COMPOSE_HAARiH0(b0, b1)\
    (b0 + b1)

#define COMPOSE_DAUB97iL1(b0, b1, b2)\
    (b1 - ((1817*(b0 + b2) + 2048) >> 12))

#define COMPOSE_DAUB97iH1(b0, b1, b2)\
    (b1 - ((3616*(b0 + b2) + 2048) >> 12))

#define COMPOSE_DAUB97iL0(b0, b1, b2)\
    (b1 + (( 217*(b0 + b2) + 2048) >> 12))

#define COMPOSE_DAUB97iH0(b0, b1, b2)\
    (b1 + ((6497*(b0 + b2) + 2048) >> 12))

static av_always_inline
void interleave(IDWTELEM *dst, IDWTELEM *src0, IDWTELEM *src1, int width,
                int add, int shift){
    int i;
    for (i = 0; i < width>>1; i++) {
        dst[2*i  ] = (src0[i] + add) >> shift;
        dst[2*i+1] = (src1[i] + add) >> shift;
    }
}

static void horizontal_compose_dirac53i(IDWTELEM *b, int w){
    IDWTELEM temp[w];
    const int w2= w >> 1;
    int x;

    temp[0] = COMPOSE_53iL0(b[w2], b[0], b[w2]);
    for (x = 0; x < w2-1; x++) {
        temp[x+1 ] = COMPOSE_53iL0     (b[x+w2], b[x+1 ], b[x+w2+1]);
        temp[x+w2] = COMPOSE_DIRAC53iH0(temp[x], b[x+w2], temp[x+1]);
    }
    temp[w-1] = COMPOSE_DIRAC53iH0(temp[w2-1], b[w-1], temp[w2-1]);

    interleave(b, temp, temp+w2, w, 1, 1);
}

static void vertical_compose53iL0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
                                  int width){
    int i;

    for(i=0; i<width; i++){
        b1[i] = COMPOSE_53iL0(b0[i], b1[i], b2[i]);
    }
}

static void vertical_compose_dirac53iH0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
                                        int width){
    int i;

    for(i=0; i<width; i++){
        b1[i] = COMPOSE_DIRAC53iH0(b0[i], b1[i], b2[i]);
    }
}

static void spatial_compose_dirac53i_dy(DWTCompose *cs, IDWTELEM *buffer,
                                        int width, int height, int stride){
    int y= cs->y;
    IDWTELEM *b0= cs->b0;
    IDWTELEM *b1= cs->b1;
    IDWTELEM *b2= buffer + mirror(y+1, height-1)*stride;
    IDWTELEM *b3= buffer + mirror(y+2, height-1)*stride;

        if(y+1<(unsigned)height) vertical_compose53iL0      (b1, b2, b3, width);
        if(y+0<(unsigned)height) vertical_compose_dirac53iH0(b0, b1, b2, width);

        if(y-1<(unsigned)height) horizontal_compose_dirac53i(b0, width);
        if(y+0<(unsigned)height) horizontal_compose_dirac53i(b1, width);

    cs->b0 = b2;
    cs->b1 = b3;
    cs->y += 2;
}

static void horizontal_compose_dd97i(IDWTELEM *b, int w){
    IDWTELEM temp[w];
    const int w2 = w >> 1;
    int x;

    temp[0] = COMPOSE_53iL0(b[w2], b[0], b[w2]);
    for (x = 0; x < w2-1; x++)
        temp[x+1] = COMPOSE_53iL0(b[x+w2], b[x+1], b[x+w2+1]);

    temp[w2] = COMPOSE_DD97iH0(temp[0], temp[0], b[w2], temp[1], temp[2]);
    for (x = 0; x < w2-2; x++)
        temp[x+w2+1] = COMPOSE_DD97iH0(temp[x], temp[x+1], b[x+w2+1], temp[x+2], temp[x+3]);

    temp[w-2] = COMPOSE_DD97iH0(temp[w2-3], temp[w2-2], b[w-2], temp[w2-1], temp[w2-1]);
    temp[w-1] = COMPOSE_DD97iH0(temp[w2-2], temp[w2-1], b[w-1], temp[w2-1], temp[w2-1]);

    interleave(b, temp, temp+w2, w, 1, 1);
}

static void vertical_compose_dd97iH0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
                                  IDWTELEM *b3, IDWTELEM *b4, int width){
    int i;

    for(i=0; i<width; i++){
        b2[i] = COMPOSE_DD97iH0(b0[i], b1[i], b2[i], b3[i], b4[i]);
    }
}

static void spatial_compose_dd97i_dy(DWTCompose *cs, IDWTELEM *buffer,
                                     int width, int height, int stride){
    int y = cs->y;
    IDWTELEM *b0= cs->b0;
    IDWTELEM *b1= cs->b1;
    IDWTELEM *b2= cs->b2;
    IDWTELEM *b3= cs->b3;
    IDWTELEM *b4= cs->b4;
    IDWTELEM *b5= cs->b5;
    IDWTELEM *b6= buffer + extend(y+5, height-2)*stride;
    IDWTELEM *b7= buffer + mirror(y+6, height-1)*stride;

        if(y+5<(unsigned)height) vertical_compose53iL0   (    b5, b6, b7,     width);
        if(y+1<(unsigned)height) vertical_compose_dd97iH0(b0, b2, b3, b4, b6, width);

        if(y-1<(unsigned)height) horizontal_compose_dd97i(b0, width);
        if(y+0<(unsigned)height) horizontal_compose_dd97i(b1, width);

    cs->b0=b2;
    cs->b1=b3;
    cs->b2=b4;
    cs->b3=b5;
    cs->b4=b6;
    cs->b5=b7;
    cs->y += 2;
}

static void horizontal_compose_dd137i(IDWTELEM *b, int w){
    IDWTELEM temp[w];
    const int w2 = w >> 1;
    int x;

    temp[0] = COMPOSE_DD137iL0(b[w2], b[w2], b[0], b[w2  ], b[w2+1]);
    temp[1] = COMPOSE_DD137iL0(b[w2], b[w2], b[1], b[w2+1], b[w2+2]);
    for (x = 0; x < w2-3; x++)
        temp[x+2] = COMPOSE_DD137iL0(b[x+w2], b[x+w2+1], b[x+2], b[x+w2+2], b[x+w2+3]);
    temp[w2-1] = COMPOSE_DD137iL0(b[w-3], b[w-2], b[w2-1], b[w-1], b[w-1]);

    temp[w2] = COMPOSE_DD97iH0(temp[0], temp[0], b[w2], temp[1], temp[2]);
    for (x = 0; x < w2-2; x++)
        temp[x+w2+1] = COMPOSE_DD97iH0(temp[x], temp[x+1], b[x+w2+1], temp[x+2], temp[x+3]);
    temp[w-2] = COMPOSE_DD97iH0(temp[w2-3], temp[w2-2], b[w-2], temp[w2-1], temp[w2-1]);
    temp[w-1] = COMPOSE_DD97iH0(temp[w2-2], temp[w2-1], b[w-1], temp[w2-1], temp[w2-1]);

    interleave(b, temp, temp+w2, w, 1, 1);
}

static void vertical_compose_dd137iL0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
                                      IDWTELEM *b3, IDWTELEM *b4, int width){
    int i;

    for(i=0; i<width; i++){
        b2[i] = COMPOSE_DD137iL0(b0[i], b1[i], b2[i], b3[i], b4[i]);
    }
}

static void spatial_compose_dd137i_dy(DWTCompose *cs, IDWTELEM *buffer,
                                      int width, int height, int stride){
    int y = cs->y;
    IDWTELEM *b0= cs->b0;
    IDWTELEM *b1= cs->b1;
    IDWTELEM *b2= cs->b2;
    IDWTELEM *b3= cs->b3;
    IDWTELEM *b4= cs->b4;
    IDWTELEM *b5= cs->b5;
    IDWTELEM *b6= cs->b6;
    IDWTELEM *b7= cs->b7;
    IDWTELEM *b8= buffer + extend    (y+7, height-2)*stride;
    IDWTELEM *b9= buffer + extend_odd(y+8, height-1)*stride;

        if(y+5<(unsigned)height) vertical_compose_dd137iL0(b3, b5, b6, b7, b9, width);
        if(y+1<(unsigned)height) vertical_compose_dd97iH0 (b0, b2, b3, b4, b6, width);

        if(y-1<(unsigned)height) horizontal_compose_dd137i(b0, width);
        if(y+0<(unsigned)height) horizontal_compose_dd137i(b1, width);

    cs->b0=b2;
    cs->b1=b3;
    cs->b2=b4;
    cs->b3=b5;
    cs->b4=b6;
    cs->b5=b7;
    cs->b6=b8;
    cs->b7=b9;
    cs->y += 2;
}

static void horizontal_compose_haari(IDWTELEM *b, int w, int shift){
    IDWTELEM temp[w];
    const int w2= w >> 1;
    int x;

    for (x = 0; x < w2; x++) {
        temp[x   ] = COMPOSE_HAARiL0(b[x   ], b[x+w2]);
        temp[x+w2] = COMPOSE_HAARiH0(b[x+w2], temp[x]);
    }

    interleave(b, temp, temp+w2, w, shift, shift);
}

static void vertical_compose_haariL0(IDWTELEM *b0, IDWTELEM *b1, int width){
    int i;

    for(i=0; i<width; i++){
        b0[i] = COMPOSE_HAARiL0(b0[i], b1[i]);
    }
}

static void vertical_compose_haariH0(IDWTELEM *b0, IDWTELEM *b1, int width){
    int i;

    for(i=0; i<width; i++){
        b0[i] = COMPOSE_HAARiH0(b0[i], b1[i]);
    }
}

static void spatial_compose_haari_dy(DWTCompose *cs, IDWTELEM *buffer,
                                      int width, int height, int stride, int shift){
    int y = cs->y;
    IDWTELEM *b0 = buffer + (y-1)*stride;
    IDWTELEM *b1 = buffer + (y  )*stride;

        if(y-1<(unsigned)height) vertical_compose_haariL0(b0, b1, width);
        if(y+0<(unsigned)height) vertical_compose_haariH0(b1, b0, width);

        if(y-1<(unsigned)height) horizontal_compose_haari(b0, width, shift);
        if(y+0<(unsigned)height) horizontal_compose_haari(b1, width, shift);

    cs->y += 2;
}

static void horizontal_compose_daub97i(IDWTELEM *b, int w) {
    IDWTELEM temp[w];
    const int w2 = w >> 1;
    int x, b0, b1, b2;

    temp[0] = COMPOSE_DAUB97iL1(b[w2], b[0], b[w2]);
    for (x = 0; x < w2-1; x++) {
        temp[ x+1] = COMPOSE_DAUB97iL1(b[x+w2], b[ x+1], b[x+w2+1]);
        temp[w2+x] = COMPOSE_DAUB97iH1(temp[x], b[w2+x], temp[x+1]);
    }
    temp[w-1] = COMPOSE_DAUB97iH1(temp[w2-1], b[w-1], temp[w2-1]);

    // second stage combined with interleave and shift
    b0 = b2 = COMPOSE_DAUB97iL0(temp[w2], temp[0], temp[w2]);
    b[0] = (b0 + 1) >> 1;
    for (x = 0; x < w2-1; x++) {
        b2 = COMPOSE_DAUB97iL0(temp[x+w2], temp[ x+1], temp[x+w2+1 ]);
        b1 = COMPOSE_DAUB97iH0(        b0, temp[w2+x], b2);
        b[2*x + 1] = (b1 + 1) >> 1;
        b[2*(x+1)] = (b2 + 1) >> 1;
        b0 = b2;
    }
    b[w-1] = (COMPOSE_DAUB97iH0(b2, temp[w-1], b2) + 1) >> 1;
}

static void vertical_compose_daub97iH0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2, int width){
    int i;

    for(i=0; i<width; i++){
        b1[i] = COMPOSE_DAUB97iH0(b0[i], b1[i], b2[i]);
    }
}

static void vertical_compose_daub97iH1(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2, int width){
    int i;

    for(i=0; i<width; i++){
        b1[i] = COMPOSE_DAUB97iH1(b0[i], b1[i], b2[i]);
    }
}

static void vertical_compose_daub97iL0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2, int width){
    int i;

    for(i=0; i<width; i++){
        b1[i] = COMPOSE_DAUB97iL0(b0[i], b1[i], b2[i]);
    }
}

static void vertical_compose_daub97iL1(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2, int width){
    int i;

    for(i=0; i<width; i++){
        b1[i] = COMPOSE_DAUB97iL1(b0[i], b1[i], b2[i]);
    }
}

static void spatial_compose_daub97i_dy(DWTCompose *cs, IDWTELEM *buffer, int width, int height, int stride){
    int y = cs->y;
    IDWTELEM *b0= cs->b0;
    IDWTELEM *b1= cs->b1;
    IDWTELEM *b2= cs->b2;
    IDWTELEM *b3= cs->b3;
    IDWTELEM *b4= buffer + mirror(y+3, height-1)*stride;
    IDWTELEM *b5= buffer + mirror(y+4, height-1)*stride;

        if(y+3<(unsigned)height) vertical_compose_daub97iL1(b3, b4, b5, width);
        if(y+2<(unsigned)height) vertical_compose_daub97iH1(b2, b3, b4, width);
        if(y+1<(unsigned)height) vertical_compose_daub97iL0(b1, b2, b3, width);
        if(y+0<(unsigned)height) vertical_compose_daub97iH0(b0, b1, b2, width);

        if(y-1<(unsigned)height) horizontal_compose_daub97i(b0, width);
        if(y+0<(unsigned)height) horizontal_compose_daub97i(b1, width);

    cs->b0=b2;
    cs->b1=b3;
    cs->b2=b4;
    cs->b3=b5;
    cs->y += 2;
}

static void spatial_compose97i_init(DWTCompose *cs, IDWTELEM *buffer, int height, int stride){
    cs->b0 = buffer + mirror(-3-1, height-1)*stride;
    cs->b1 = buffer + mirror(-3  , height-1)*stride;
    cs->b2 = buffer + mirror(-3+1, height-1)*stride;
    cs->b3 = buffer + mirror(-3+2, height-1)*stride;
    cs->y = -3;
}

static void spatial_compose53i_init(DWTCompose *cs, IDWTELEM *buffer,
                                    int height, int stride){
    cs->b0 = buffer + mirror(-1-1, height-1)*stride;
    cs->b1 = buffer + mirror(-1  , height-1)*stride;
    cs->y = -1;
}

static void spatial_compose_dd97i_init(DWTCompose *cs, IDWTELEM *buffer,
                                       int height, int stride){
    cs->b0 = buffer + extend(-5-1, height-2)*stride;
    cs->b1 = buffer + mirror(-5  , height-1)*stride;
    cs->b2 = buffer + extend(-5+1, height-2)*stride;
    cs->b3 = buffer + mirror(-5+2, height-1)*stride;
    cs->b4 = buffer + extend(-5+3, height-2)*stride;
    cs->b5 = buffer + mirror(-5+4, height-1)*stride;
    cs->y = -5;
}

static void spatial_compose_dd137i_init(DWTCompose *cs, IDWTELEM *buffer,
                                        int height, int stride){
    cs->b0 = buffer + extend    (-5-1, height-2)*stride;
    cs->b1 = buffer + extend_odd(-5  , height-1)*stride;
    cs->b2 = buffer + extend    (-5+1, height-2)*stride;
    cs->b3 = buffer + extend_odd(-5+2, height-1)*stride;
    cs->b4 = buffer + extend    (-5+3, height-2)*stride;
    cs->b5 = buffer + extend_odd(-5+4, height-1)*stride;
    cs->b6 = buffer + extend    (-5+5, height-2)*stride;
    cs->b7 = buffer + extend_odd(-5+6, height-1)*stride;
    cs->y = -5;
}

static void spatial_compose_haari_init(DWTCompose *cs, IDWTELEM *buffer,
                                       int height, int stride){
    cs->y = 1;
}

void ff_spatial_idwt_init2(DWTCompose *cs, IDWTELEM *buffer, int width, int height,
                           int stride, int type, int decomposition_count){
    int level;
    for(level=decomposition_count-1; level>=0; level--){
        int hl = height >> level;
        int stride_l = stride << level;

        switch(type){
        case DWT_DIRAC_DD9_7:
            spatial_compose_dd97i_init(cs+level, buffer, hl, stride_l);
            break;
        case DWT_DIRAC_LEGALL5_3:
            spatial_compose53i_init(cs+level, buffer, hl, stride_l);
            break;
        case DWT_DIRAC_DD13_7:
            spatial_compose_dd137i_init(cs+level, buffer, hl, stride_l);
            break;
        case DWT_DIRAC_HAAR0:
        case DWT_DIRAC_HAAR1:
            spatial_compose_haari_init(cs+level, buffer, hl, stride_l);
            break;
        case DWT_DIRAC_DAUB9_7:
            spatial_compose97i_init(cs+level, buffer, hl, stride_l);
            break;
        }
    }
}

void ff_spatial_idwt_slice2(DWTCompose *cs, IDWTELEM *buffer, int width, int height,
                            int stride, int type, int decomposition_count, int y){
    const int support[] = {5, 3, 7, 3, 7, 3, 3, 0, 5};
    int level;

    for(level=decomposition_count-1; level>=0; level--){
        int wl = width  >> level;
        int hl = height >> level;
        int stride_l = stride << level;

        while(cs[level].y <= FFMIN((y>>level)+support[type], height>>level)){
            switch(type){
            case DWT_DIRAC_DD9_7:
                spatial_compose_dd97i_dy(cs+level, buffer, wl, hl, stride_l);
                break;
            case DWT_DIRAC_LEGALL5_3:
                spatial_compose_dirac53i_dy(cs+level, buffer, wl, hl, stride_l);
                break;
            case DWT_DIRAC_DD13_7:
                spatial_compose_dd137i_dy(cs+level, buffer, wl, hl, stride_l);
                break;
            case DWT_DIRAC_HAAR0:
                spatial_compose_haari_dy(cs+level, buffer, wl, hl, stride_l, 0);
                break;
            case DWT_DIRAC_HAAR1:
                spatial_compose_haari_dy(cs+level, buffer, wl, hl, stride_l, 1);
                break;
            case DWT_DIRAC_DAUB9_7:
                spatial_compose_daub97i_dy(cs+level, buffer, wl, hl, stride_l);
                break;
            }
        }
    }
}

void ff_spatial_idwt2(IDWTELEM *buffer, int width, int height,
                      int stride, int type, int decomposition_count){
    DWTCompose cs[decomposition_count];
    int y;

    ff_spatial_idwt_init2(cs, buffer, width, height, stride,
                          type, decomposition_count);
    for(y=0; y<height; y+=4)
        ff_spatial_idwt_slice2(cs, buffer, width, height, stride,
                               type, decomposition_count, y);
}
