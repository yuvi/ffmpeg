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

#define MAX_DWT_TAPS 8

typedef struct {
    int y;
    IDWTELEM *b[MAX_DWT_TAPS];
} DWTCompose;

enum dwt_type {
    DWT_SNOW_DAUB9_7,
    DWT_SNOW_LEGALL5_3,
    DWT_DIRAC_DD9_7,
    DWT_DIRAC_LEGALL5_3,
    DWT_DIRAC_DD13_7,
    DWT_DIRAC_HAAR0,
    DWT_DIRAC_HAAR1,
    DWT_DIRAC_FIDELITY,
    DWT_DIRAC_DAUB9_7,
    DWT_NUM_TYPES
};

void ff_spatial_idwt2(IDWTELEM *buffer, int width, int height, int stride,
                      enum dwt_type type, int decomposition_count);

void ff_spatial_idwt_slice2(DWTCompose *cs, IDWTELEM *buffer, int width, int height,
                            int stride, enum dwt_type type, int decomposition_count, int y);

#endif /* AVCODEC_DWT_H */
