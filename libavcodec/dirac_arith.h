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
 * Arithmetic decoder for Dirac
 * @author Marco Gerards <marco@gnu.org>
 */

#ifndef AVCODEC_DIRAC_ARITH_H
#define AVCODEC_DIRAC_ARITH_H

#include "bitstream.h"

#define ARITH_CONTEXT_COUNT (ARITH_CONTEXT_DC_SIGN + 1)

enum arith_context_indices {
    ARITH_CONTEXT_SIGN_ZERO,
    ARITH_CONTEXT_SIGN_POS,
    ARITH_CONTEXT_SIGN_NEG,
    ARITH_CONTEXT_ZPZN_F1,
    ARITH_CONTEXT_ZPNN_F1,
    ARITH_CONTEXT_ZP_F2,
    ARITH_CONTEXT_ZP_F3,
    ARITH_CONTEXT_ZP_F4,
    ARITH_CONTEXT_ZP_F5,
    ARITH_CONTEXT_ZP_F6,
    ARITH_CONTEXT_NPZN_F1,
    ARITH_CONTEXT_NPNN_F1,
    ARITH_CONTEXT_NP_F2,
    ARITH_CONTEXT_NP_F3,
    ARITH_CONTEXT_NP_F4,
    ARITH_CONTEXT_NP_F5,
    ARITH_CONTEXT_NP_F6,
    ARITH_CONTEXT_COEFF_DATA,
    ARITH_CONTEXT_ZERO_BLOCK,
    ARITH_CONTEXT_Q_OFFSET_FOLLOW,
    ARITH_CONTEXT_Q_OFFSET_DATA,
    ARITH_CONTEXT_Q_OFFSET_SIGN,

    ARITH_CONTEXT_SB_F1,
    ARITH_CONTEXT_SB_F2,
    ARITH_CONTEXT_SB_DATA,
    ARITH_CONTEXT_PMODE_REF1,
    ARITH_CONTEXT_PMODE_REF2,
    ARITH_CONTEXT_GLOBAL_BLOCK,
    ARITH_CONTEXT_VECTOR_F1,
    ARITH_CONTEXT_VECTOR_F2,
    ARITH_CONTEXT_VECTOR_F3,
    ARITH_CONTEXT_VECTOR_F4,
    ARITH_CONTEXT_VECTOR_F5,
    ARITH_CONTEXT_VECTOR_DATA,
    ARITH_CONTEXT_VECTOR_SIGN,
    ARITH_CONTEXT_DC_F1,
    ARITH_CONTEXT_DC_F2,
    ARITH_CONTEXT_DC_DATA,
    ARITH_CONTEXT_DC_SIGN
};

typedef struct dirac_arith_state {
    /* Arithmetic decoding.  */
    unsigned int low;
    unsigned int range;
    unsigned int code;
    unsigned int bits_left;
    int carry;
    unsigned int contexts[ARITH_CONTEXT_COUNT];

    GetBitContext *gb;
    PutBitContext *pb;
} *dirac_arith_state_t;

struct dirac_arith_context_set {
    unsigned int follow[6];     ///< the first follow contexts
    unsigned int follow_length; ///< the amount of follow contexts in follow
    unsigned int data;          ///< context to read data
    unsigned int sign;          ///< context to read the sign
};

void dirac_arith_init(dirac_arith_state_t arith,
                      GetBitContext *gb, int length);

void dirac_arith_coder_init(dirac_arith_state_t arith, PutBitContext *pb);

int dirac_arith_get_bit(dirac_arith_state_t arith, int context);

void dirac_arith_put_bit(dirac_arith_state_t arith, int bit, int context);

unsigned int dirac_arith_read_uint(dirac_arith_state_t arith,
                                   struct dirac_arith_context_set *context_set);

int dirac_arith_read_int(dirac_arith_state_t arith,
                         struct dirac_arith_context_set *context_set);

void dirac_arith_write_uint(dirac_arith_state_t arith,
                            struct dirac_arith_context_set *context_set,
                            unsigned int i);

void dirac_arith_write_int(dirac_arith_state_t arith,
                           struct dirac_arith_context_set *context_set,
                           int i);

void dirac_arith_flush(dirac_arith_state_t arith);

void dirac_arith_coder_flush(dirac_arith_state_t arith);

#endif // AVCODEC_DIRAC_ARITH_H
