/*
 * Copyright (C) 2007 Marco Gerards <marco@gnu.org>
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

    DIRAC_CTX_COUNT
};

// Dirac resets the arith decoder between decoding various types of data,
// so many contexts are never used simultaneously. Thus, we can reduce
// the number of contexts needed by reusing them.
#define CTX_SB_F1        CTX_ZP_F5
#define CTX_SB_DATA      0
#define CTX_PMODE_REF1   0
#define CTX_PMODE_REF2   1
#define CTX_GLOBAL_BLOCK 2
#define CTX_MV_F1        CTX_ZP_F2
#define CTX_MV_DATA      0
#define CTX_DC_F1        CTX_ZP_F5
#define CTX_DC_DATA      0

typedef struct {
    unsigned low;
    unsigned range;
    unsigned counter;

    const uint8_t *bytestream_start;
    const uint8_t *bytestream;
    const uint8_t *bytestream_end;

    uint16_t contexts[DIRAC_CTX_COUNT];
} DiracArith;

extern const uint8_t ff_dirac_next_ctx[DIRAC_CTX_COUNT];
extern const uint16_t ff_dirac_prob[256];

static inline void renorm_arith_decoder(DiracArith *c)
{
    while (c->range <= 0x4000) {
        c->low   <<= 1;
        c->range <<= 1;

        if (!--c->counter) {
            c->low += AV_RB16(c->bytestream);
            c->bytestream += 2;

            // the spec defines overread bits to be 1
            if (c->bytestream > c->bytestream_end) {
                c->low |= 0xff;
                if (c->bytestream > c->bytestream_end+1)
                    c->low |= 0xff00;
                c->bytestream = c->bytestream_end;
            }
            c->counter = 16;
        }
    }
}

static inline int dirac_get_arith_bit(DiracArith *c, int ctx)
{
    int prob_zero = c->contexts[ctx];
    int range_times_prob, ret;

    range_times_prob = (c->range * prob_zero) >> 16;
    ret = (c->low >> 16) >= range_times_prob;

    if (ret) {
        c->low   -= range_times_prob << 16;
        c->range -= range_times_prob;
        c->contexts[ctx] -= ff_dirac_prob[prob_zero>>8];
    } else {
        c->range  = range_times_prob;
        c->contexts[ctx] += ff_dirac_prob[255 - (prob_zero>>8)];
    }

    renorm_arith_decoder(c);
    return ret;
}

static inline int dirac_get_arith_uint(DiracArith *c, int follow_ctx, int data_ctx)
{
    int ret = 1;
    while (!dirac_get_arith_bit(c, follow_ctx)) {
        ret <<= 1;
        ret += dirac_get_arith_bit(c, data_ctx);
        follow_ctx = ff_dirac_next_ctx[follow_ctx];
    }
    return ret-1;
}

static inline int dirac_get_arith_int(DiracArith *c, int follow_ctx, int data_ctx)
{
    int ret = dirac_get_arith_uint(c, follow_ctx, data_ctx);
    if (ret && dirac_get_arith_bit(c, data_ctx+1))
        ret = -ret;
    return ret;
}

void ff_dirac_init_arith_decoder(DiracArith *c, GetBitContext *gb, int length);

#endif /* AVCODEC_DIRAC_ARITH_H */
