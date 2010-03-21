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

#ifndef AVCODEC_DIRAC_H
#define AVCODEC_DIRAC_H

/**
 * @file libavcodec/dirac.h
 * Interfaces to Dirac Decoder/Encoder
 * @author Marco Gerards <marco@gnu.org>
 */

#include "avcodec.h"
#include "get_bits.h"
#include "dirac_arith.h"
#include "dsputil.h"
#include "dwt.h"

#define CALC_PADDING(size, depth) \
         (((size + (1 << depth) - 1) >> depth) << depth)

#define DIVRNDUP(a, b) (((a) + (b) - 1) / (b))

typedef struct {
    unsigned width;
    unsigned height;
    uint8_t chroma_format;          ///< 0: 444  1: 422  2: 420

    uint8_t interlaced;
    uint8_t top_field_first;

    uint8_t frame_rate_index;       ///< index into dirac_frame_rate[]
    uint8_t aspect_ratio_index;     ///< index into dirac_aspect_ratio[]

    uint16_t clean_width;
    uint16_t clean_height;
    uint16_t clean_left_offset;
    uint16_t clean_right_offset;

    uint8_t pixel_range_index;      ///< index into dirac_pixel_range_presets[]
    uint8_t color_spec_index;       ///< index into dirac_color_spec_presets[]
} dirac_source_params;

#define DIRAC_REF_MASK_REF1   1
#define DIRAC_REF_MASK_REF2   2
#define DIRAC_REF_MASK_GLOBAL 4

struct dirac_blockmotion {
    uint8_t use_ref;
    int16_t vect[2][2];
    int16_t dc[3];
};

#define MAX_REFERENCE_FRAMES 8
#define MAX_DELAY 4
#define MAX_FRAMES (MAX_REFERENCE_FRAMES + MAX_DELAY + 1)
#define MAX_BLOCKSIZE 64    ///< maximum blen
#define MAX_QUANT 57        ///< 57 is the last quant not to always overflow int16

typedef struct SubBand {
    int level;
    int orientation;
    int stride;
    int width;
    int height;
    IDWTELEM *ibuf;
    struct SubBand *parent;
} SubBand;

typedef struct Plane {
    int width;
    int height;
    int padded_width;
    int padded_height;
    SubBand band[MAX_DECOMPOSITIONS][4];

    IDWTELEM *idwt_buf;
    IDWTELEM *idwt_buf_base;
    int idwt_stride;

    uint8_t xbsep;
    uint8_t xblen;
    uint8_t ybsep;
    uint8_t yblen;

    uint8_t xoffset;
    uint8_t yoffset;
    uint8_t total_wt_bits;
    uint8_t current_blwidth;
    uint8_t current_blheight;
} Plane;

typedef struct DiracContext {
    AVCodecContext *avctx;
    DSPContext dsp;
    GetBitContext gb;
    dirac_arith arith;
    dirac_source_params source;
    int seen_sequence_header;
    Plane plane[3];
    int chroma_x_shift;
    int chroma_y_shift;

    int zero_res;             ///< zero residue flag
    int is_arith;             ///< whether coeffs use arith or golomb coding
    int low_delay;            ///< use the low delay syntax
    int globalmc_flag;        ///< use global motion compensation flag
    int num_refs;             ///< number of reference pictures

    // wavelet decoding
    uint8_t wavelet_depth;    ///< depth of the IDWT
    unsigned wavelet_idx;

    /** schroedinger 1.0.8 and newer stores quant delta for all codeblocks */
    unsigned new_delta_quant;
    unsigned codeblock_mode;

    struct {
        unsigned width;
        unsigned height;
    } codeblock[MAX_DECOMPOSITIONS+1];

    struct {
        unsigned x_slices;
        unsigned y_slices;
        AVRational slice_bytes;
        uint8_t quant[MAX_DECOMPOSITIONS][4];
    } lowdelay;

    struct {
        unsigned int pan_tilt[2];       ///< pan/tilt vector
        unsigned int zrs[2][2];         ///< zoom/rotate/shear matrix
        int perspective[2];             ///< perspective vector
        unsigned int zrs_exp;
        unsigned int perspective_exp;
    } globalmc;

    // motion compensation
    uint8_t mv_precision;
    int16_t picture_weight_ref1;
    int16_t picture_weight_ref2;
    unsigned int picture_weight_precision;

    int blwidth;              ///< number of blocks (horizontally)
    int blheight;             ///< number of blocks (vertically)
    int sbwidth;              ///< number of superblocks (horizontally)
    int sbheight;             ///< number of superblocks (vertically)

    uint8_t *sbsplit;
    struct dirac_blockmotion *blmotion;

    int16_t *mcpic;
    int16_t spatialwt[MAX_BLOCKSIZE*MAX_BLOCKSIZE];
    int8_t *refdata[2];
    int refwidth;
    int refheight;

    // TODO: interpolate after decoding a ref frame
    uint8_t *hpel_planes[2][3][4];
    uint8_t *obmc_buffer;
    DECLARE_ALIGNED(16, uint8_t, obmc_weight)[2][MAX_BLOCKSIZE*MAX_BLOCKSIZE];

    AVFrame *current_picture;
    AVFrame *ref_pics[2];

    AVFrame *ref_frames[MAX_REFERENCE_FRAMES+1];
    AVFrame *delay_frames[MAX_DELAY+1];
    AVFrame all_frames[MAX_FRAMES];
} DiracContext;

enum dirac_parse_code {
    pc_seq_header         = 0x00,
    pc_eos                = 0x10,
    pc_aux_data           = 0x20,
    pc_padding            = 0x60,
    pc_intra_ref          = 0x0c
};

enum dirac_subband {
    subband_ll = 0,
    subband_hl = 1,
    subband_lh = 2,
    subband_hh = 3
};

static const int avgsplit[7] = { 0, 0, 1, 1, 1, 2, 2 };

static inline int split_prediction(DiracContext *s, int x, int y)
{
    if (x == 0 && y == 0)
        return 0;
    else if (y == 0)
        return s->sbsplit[ y      * s->sbwidth + x - 1];
    else if (x == 0)
        return s->sbsplit[(y - 1) * s->sbwidth + x    ];

    return avgsplit[s->sbsplit[(y - 1) * s->sbwidth + x    ]
                  + s->sbsplit[ y      * s->sbwidth + x - 1]
                  + s->sbsplit[(y - 1) * s->sbwidth + x - 1]];
}

/**
 * Mode prediction
 *
 * @param x    horizontal position of the MC block
 * @param y    vertical position of the MC block
 * @param ref reference frame
 */
static inline
int mode_prediction(DiracContext *s, int x, int y, int refmask, int refshift)
{
    int cnt;

    if (x == 0 && y == 0)
        return 0;
    else if (y == 0)
        return ((s->blmotion[ y      * s->blwidth + x - 1].use_ref & refmask)
                >> refshift);
    else if (x == 0)
        return ((s->blmotion[(y - 1) * s->blwidth + x    ].use_ref & refmask)
                >> refshift);

    /* Return the majority. */
    cnt = (s->blmotion[ y      * s->blwidth + x - 1].use_ref & refmask)
        + (s->blmotion[(y - 1) * s->blwidth + x    ].use_ref & refmask)
        + (s->blmotion[(y - 1) * s->blwidth + x - 1].use_ref & refmask);
    cnt >>= refshift;

    return cnt >> 1;
}

/**
 * Predict the motion vector
 *
 * @param x    horizontal position of the MC block
 * @param y    vertical position of the MC block
 * @param ref reference frame
 * @param dir direction horizontal=0, vertical=1
 */
static inline
int motion_vector_prediction(DiracContext *s, int x, int y, int ref, int dir)
{
    int cnt = 0;
    int left = 0, top = 0, lefttop = 0;
    const int refmask = ref + 1;
    const int mask = refmask | DIRAC_REF_MASK_GLOBAL;
    struct dirac_blockmotion *block = &s->blmotion[y * s->blwidth + x];

    if (x > 0) {
        /* Test if the block to the left has a motion vector for this
           reference frame. */
        if ((block[-1].use_ref & mask) == refmask) {
            left = block[-1].vect[ref][dir];
            cnt++;
        }

        /* This is the only reference, return it. */
        if (y == 0)
            return left;
    }

    if (y > 0) {
        /* Test if the block above the current one has a motion vector
           for this reference frame. */
        if ((block[-s->blwidth].use_ref & mask) == refmask) {
            top = block[-s->blwidth].vect[ref][dir];
            cnt++;
        }

        /* This is the only reference, return it. */
        if (x == 0)
            return top;
        else if (x > 0) {
            /* Test if the block above the current one has a motion vector
               for this reference frame. */
            if ((block[-s->blwidth - 1].use_ref & mask) == refmask) {
                lefttop = block[-s->blwidth - 1].vect[ref][dir];
                cnt++;
            }
        }
    }

    /* No references for the prediction. */
    if (cnt == 0)
        return 0;

    if (cnt == 1)
        return left + top + lefttop;

    /* Return the median of two motion vectors. */
    if (cnt == 2)
        return (left + top + lefttop + 1) >> 1;

    /* Return the median of three motion vectors. */
    return mid_pred(left, top, lefttop);
}

static inline
int block_dc_prediction(DiracContext *s, int x, int y, int comp)
{
    int total = 0;
    int cnt = 0;
    int sign;

    if (x > 0) {
        if (!(s->blmotion[y * s->blwidth + x - 1].use_ref & 3)) {
            total += s->blmotion[y * s->blwidth + x - 1].dc[comp];
            cnt++;
        }
    }

    if (y > 0) {
        if (!(s->blmotion[(y - 1) * s->blwidth + x].use_ref & 3)) {
            total += s->blmotion[(y - 1) * s->blwidth + x].dc[comp];
            cnt++;
        }
    }

    if (x > 0 && y > 0) {
        if (!(s->blmotion[(y - 1) * s->blwidth + x - 1].use_ref & 3)) {
            total += s->blmotion[(y - 1) * s->blwidth + x - 1].dc[comp];
            cnt++;
        }
    }

    if (cnt == 0)
        return 0;

    sign = FFSIGN(total);
    total = FFABS(total);

    /* Return the average of all DC values that were counted. */
    return sign * (total + (cnt >> 1)) / cnt;
}

int dirac_motion_compensation(DiracContext *s, int comp);

int ff_dirac_parse_sequence_header(AVCodecContext *avctx, GetBitContext *gb,
                                   dirac_source_params *source);

extern uint8_t ff_dirac_default_qmat[][4][4];

extern const int ff_dirac_qscale_tab[MAX_QUANT+1];
extern const int ff_dirac_qoffset_intra_tab[MAX_QUANT+1];
extern const int ff_dirac_qoffset_inter_tab[MAX_QUANT+1];

#endif /* AVCODEC_DIRAC_H */
