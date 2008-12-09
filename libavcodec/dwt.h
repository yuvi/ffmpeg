/*
 * Copyright (C) 2004 Michael Niedermayer <michaelni@gmx.at>
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

#ifndef AVCODEC_DWT_H
#define AVCODEC_DWT_H

#include "avcodec.h"
#include "dsputil.h"

typedef struct {
    int y;
    IDWTELEM *b0;
    IDWTELEM *b1;
    IDWTELEM *b2;
    IDWTELEM *b3;
    IDWTELEM *b4;
    IDWTELEM *b5;
} dwt_compose_t;

#define DWT_SNOW_DAUB9_7        0
#define DWT_SNOW_LEGALL5_3      1
#define DWT_DIRAC_DD9_7         2
#define DWT_DIRAC_LEGALL5_3     3
#define DWT_DIRAC_DD13_7        4
#define DWT_DIRAC_HAAR0         5
#define DWT_DIRAC_HAAR1         6
#define DWT_DIRAC_FIDELITY      7
#define DWT_DIRAC_DAUB9_7       8

void ff_spatial_idwt2(IDWTELEM *buffer, int width, int height, int stride,
                      int type, int decomposition_count);

void ff_spatial_idwt_slice2(dwt_compose_t *cs, IDWTELEM *buffer, int width, int height,
                            int stride, int type, int decomposition_count, int y);

#endif /* AVCODEC_DWT_H */
