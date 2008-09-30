/* -*-  indent-tabs-mode:nil; c-basic-offset:4;  -*- */
/*
 * Copyright (C) 2007 Marco Gerards <marco@gnu.org>
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

/**
 * @file dirac_arith.h
 * Dirac support interfaces
 * @author Marco Gerards <marco@gnu.org>
 */

#ifndef AVCODEC_DIRAC_WAVELET_H
#define AVCODEC_DIRAC_WAVELET_H

#include "avcodec.h"

int dirac_subband_idwt_53(AVCodecContext *avctx, int width, int height,
                          int padded_width, int16_t *data, int16_t *synth,
                          int level);

int dirac_subband_idwt_95(AVCodecContext *avctx, int width, int height,
                          int padded_width, int16_t *data, int16_t *synth,
                          int level);

int dirac_subband_dwt_53(AVCodecContext *avctx, int width, int height,
                         int padded_width, int16_t *data, int level);

int dirac_subband_dwt_95(AVCodecContext *avctx, int width, int height,
                         int padded_width, int16_t *data, int level);

#endif /* AVCODEC_DIRACWAVELET_H */
