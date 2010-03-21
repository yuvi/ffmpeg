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
 * @file libavcodec/dirac_arith.h
 * Arithmetic decoder for Dirac
 * @author Marco Gerards <marco@gnu.org>
 */

#ifndef AVCODEC_DIRAC_ARITH_H
#define AVCODEC_DIRAC_ARITH_H

#include "get_bits.h"

enum dirac_arith_contexts {
    CTX_ZPZN_F1,
    CTX_ZPNN_F1,
    CTX_NPZN_F1,
    CTX_NPNN_F1,
    CTX_ZP_F2,
    CTX_ZP_F3,
    CTX_ZP_F4,
    CTX_ZP_F5,
    CTX_ZP_F6,
    CTX_NP_F2,
    CTX_NP_F3,
    CTX_NP_F4,
    CTX_NP_F5,
    CTX_NP_F6,
    CTX_COEFF_DATA,
    CTX_SIGN_NEG,
    CTX_SIGN_ZERO,
    CTX_SIGN_POS,
    CTX_ZERO_BLOCK,
    CTX_DELTA_Q_F,
    CTX_DELTA_Q_DATA,
    CTX_DELTA_Q_SIGN,

    CTX_SB_F1,
    CTX_SB_F2,
    CTX_SB_DATA,
    CTX_PMODE_REF1,
    CTX_PMODE_REF2,
    CTX_GLOBAL_BLOCK,
    CTX_MV_F1,
    CTX_MV_F2,
    CTX_MV_F3,
    CTX_MV_F4,
    CTX_MV_F5,
    CTX_MV_DATA,
    CTX_MV_SIGN,
    CTX_DC_F1,
    CTX_DC_F2,
    CTX_DC_DATA,
    CTX_DC_SIGN,

    DIRAC_CTX_COUNT
};

typedef struct dirac_arith {
    unsigned low;
    unsigned range;
    unsigned counter;

    const uint8_t *bytestream_start;
    const uint8_t *bytestream;
    const uint8_t *bytestream_end;

    uint16_t contexts[DIRAC_CTX_COUNT];
} dirac_arith;

void ff_dirac_init_arith_decoder(dirac_arith *arith, GetBitContext *gb, int length);

int dirac_get_arith_bit(dirac_arith *arith, int ctx);
int dirac_get_arith_uint(dirac_arith *arith, int follow_ctx, int data_ctx);
int dirac_get_arith_int(dirac_arith *arith, int follow_ctx, int data_ctx);

#endif /* AVCODEC_DIRAC_ARITH_H */
