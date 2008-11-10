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

#ifndef AVCODEC_DIRAC_H
#define AVCODEC_DIRAC_H

/**
 * @file dirac.h
 * Interfaces to Dirac Decoder/Encoder
 * @author Marco Gerards <marco@gnu.org>
 */

#undef DEBUG

#include "avcodec.h"
#include "bitstream.h"
#include "dirac_arith.h"

typedef enum {
    COLOR_PRIMARY_HDTV,         ///< ITU-R BT. 709, also computer/web/sRGB
    COLOR_PRIMARY_SDTV_525,     ///< SMPTE 170M, 525 primaries
    COLOR_PRIMARY_SDTV_625,     ///< EBU Tech 3213-E, 625 primaries
    COLOR_PRIMARY_DCINEMA,      ///< SMPTE 428.1, CIE XYZ
} color_primary_t;

typedef enum {
    COLOR_MATRIX_HDTV,          ///< ITU-R BT.709, also computer/web
    COLOR_MATRIX_SDTV,          ///< ITU-R BT.601
    COLOR_MATRIX_REVERSIBLE,    ///< ITU-T H.264
} color_matrix_t;

typedef enum {
    TRANSFER_FUNC_TV,
    TRANSFER_FUNC_EXTENDED_GAMUT,
    TRANSFER_FUNC_LINEAR,
    TRANSFER_FUNC_DCI_GAMMA
} transfer_func_t;

typedef struct {
    color_primary_t primaries;
    color_matrix_t  matrix;
    transfer_func_t transfer_function;
} color_specification;

#define DIRAC_SIGN(x) ((x) > 0 ? 2 : ((x) < 0 ? 1 : 0))
#define DIRAC_PARSE_INFO_PREFIX 0x42424344
#define CALC_PADDING(size, depth) \
         (((size + (1 << depth) - 1) >> depth) << depth)


typedef struct {
    /* Information about the frames.  */
    unsigned int luma_width;           ///< the luma component width
    unsigned int luma_height;          ///< the luma component height
    /** Choma format: 0: 4:4:4, 1: 4:2:2, 2: 4:2:0 */
    unsigned int chroma_format;

    /* Interlacing.  */
    char interlaced;                   ///< flag for interlacing
    char top_field_first;

    unsigned int frame_rate_index;     ///< index into dirac_frame_rate[]
    unsigned int aspect_ratio_index;   ///< index into dirac_aspect_ratio[]

    /* Clean area.  */
    uint16_t clean_width;
    uint16_t clean_height;
    uint16_t clean_left_offset;
    uint16_t clean_right_offset;

    unsigned int signal_range_index;   ///< index into dirac_signal_range[]
    unsigned int color_spec_index;     ///< index into ff_dirac_color_spec_presets[]

    /* Calculated:  */
    unsigned int chroma_width;         ///< the chroma component width
    unsigned int chroma_height;        ///< the chroma component height
    unsigned int luma_depth;
    unsigned int chroma_depth;

    AVRational frame_rate;
    AVRational aspect_ratio;

    /* Luma and chroma offsets.  */
    uint16_t luma_offset;
    uint16_t luma_excursion;
    uint16_t chroma_offset;
    uint16_t chroma_excursion;

    color_specification color_spec;
    float k_r;
    float k_b; /* XXX: ??? */
} dirac_source_params;

struct decoding_parameters {
    uint8_t wavelet_depth;          ///< depth of the IDWT

    uint8_t luma_xbsep;
    uint8_t luma_xblen;
    uint8_t luma_ybsep;
    uint8_t luma_yblen;

    uint8_t mv_precision;

    int16_t picture_weight_ref1;
    int16_t picture_weight_ref2;
    unsigned int picture_weight_precision;

    uint8_t chroma_xbsep;
    uint8_t chroma_xblen;
    uint8_t chroma_ybsep;
    uint8_t chroma_yblen;
};

struct globalmc_parameters {
    unsigned int pan_tilt[2];                   ///< pan/tilt vector
    unsigned int zrs[2][2];                     ///< zoom/rotate/shear matrix
    int perspective[2];                         ///< perspective vector
    unsigned int zrs_exp;
    unsigned int perspective_exp;
};

extern struct dirac_arith_context_set ff_dirac_context_set_split;
extern struct dirac_arith_context_set ff_dirac_context_set_mv;
extern struct dirac_arith_context_set ff_dirac_context_set_dc;
extern struct dirac_arith_context_set ff_dirac_context_sets_waveletcoeff[];

typedef int16_t vect_t[2];

#define DIRAC_REF_MASK_REF1   1
#define DIRAC_REF_MASK_REF2   2
#define DIRAC_REF_MASK_GLOBAL 4

struct dirac_blockmotion {
    uint8_t use_ref;
    vect_t vect[2];
    int16_t dc[3];
};

/* XXX */
#define REFFRAME_CNT 20

struct reference_frame {
    AVFrame frame;
    int8_t *halfpel[3];
};

typedef struct DiracContext {
    unsigned int profile;
    unsigned int level;

    AVCodecContext *avctx;
    GetBitContext gb;

    PutBitContext pb;
    int next_parse_code;
    char *encodebuf;
    int prev_size;

    AVFrame picture;

    uint32_t picnum;
    int refcnt;
    struct reference_frame refframes[REFFRAME_CNT]; /* XXX */

    int retirecnt;
    uint32_t retireframe[REFFRAME_CNT];
    int16_t *mcpic;

    dirac_source_params source;
    struct decoding_parameters decoding;

    unsigned int codeblock_mode;
    unsigned int codeblocksh[7]; /* XXX: 7 levels.  */
    unsigned int codeblocksv[7]; /* XXX: 7 levels.  */

    int padded_luma_width;    ///< padded luma width
    int padded_luma_height;   ///< padded luma height
    int padded_chroma_width;  ///< padded chroma width
    int padded_chroma_height; ///< padded chroma height

    int chroma_hshift;        ///< horizontal bits to shift for choma
    int chroma_vshift;        ///< vertical bits to shift for choma

    int blwidth;              ///< number of blocks (horizontally)
    int blheight;             ///< number of blocks (vertically)
    int sbwidth;              ///< number of superblocks (horizontally)
    int sbheight;             ///< number of superblocks (vertically)

    int zero_res;             ///< zero residue flag

    int refs;                 ///< number of reference pictures
    int globalmc_flag;        ///< use global motion compensation flag
    /** global motion compensation parameters */
    struct globalmc_parameters globalmc;
    uint32_t ref[2];          ///< reference pictures
    int16_t *spatialwt;

    int8_t *refdata[2];
    int refwidth;
    int refheight;

    unsigned int wavelet_idx;

    /* Current component.  */
    int padded_width;         ///< padded width of the current component
    int padded_height;        ///< padded height of the current component
    int width;
    int height;
    int xbsep;
    int ybsep;
    int xblen;
    int yblen;
    int xoffset;
    int yoffset;
    int total_wt_bits;
    int current_blwidth;
    int current_blheight;

    int *sbsplit;     // XXX: int8_t
    struct dirac_blockmotion *blmotion;

    /** State of arithmetic decoding.  */
    struct dirac_arith_state arith;
} DiracContext;

typedef enum {
    pc_seq_header         = 0x00,
    pc_eos                = 0x10,
    pc_aux_data           = 0x20,
    pc_padding            = 0x60,
    pc_intra_ref          = 0x0c
} parse_code_t;

typedef enum {
    subband_ll = 0,
    subband_hl = 1,
    subband_lh = 2,
    subband_hh = 3
} subband_t;

/**
 * Calculate the width of a subband on a given level
 *
 * @param level subband level
 * @return subband width
 */
static int inline subband_width(DiracContext *s, int level)
{
    if (level == 0)
        return s->padded_width >> s->decoding.wavelet_depth;
    return s->padded_width >> (s->decoding.wavelet_depth - level + 1);
}

/**
 * Calculate the height of a subband on a given level
 *
 * @param level subband level
 * @return height of the subband
 */
static int inline subband_height(DiracContext *s, int level)
{
    if (level == 0)
        return s->padded_height >> s->decoding.wavelet_depth;
    return s->padded_height >> (s->decoding.wavelet_depth - level + 1);
}

static int inline coeff_quant_factor(int idx)
{
    uint64_t base;
    idx = FFMAX(idx, 0);
    base = 1 << (idx / 4);
    switch(idx & 3) {
    case 0:
        return base << 2;
    case 1:
        return (503829 * base + 52958) / 105917;
    case 2:
        return (665857 * base + 58854) / 117708;
    case 3:
        return (440253 * base + 32722) / 65444;
    }
    return 0; /* XXX: should never be reached */
}

static int inline coeff_quant_offset(DiracContext *s, int idx)
{
    if (idx == 0)
        return 1;

    if (s->refs == 0) {
        if (idx == 1)
            return 2;
        else
            return (coeff_quant_factor(idx) + 1) >> 1;
    }

    return (coeff_quant_factor(idx) * 3 + 4) / 8;
}

/**
 * Calculate the horizontal position of a coefficient given a level,
 * orientation and horizontal position within the subband.
 *
 * @param level subband level
 * @param orientation orientation of the subband within the level
 * @param x position within the subband
 * @return horizontal position within the coefficient array
 */
static int inline coeff_posx(DiracContext *s, int level,
                             subband_t orientation, int x)
{
    if (orientation == subband_hl || orientation == subband_hh)
        return subband_width(s, level) + x;

    return x;
}

/**
 * Calculate the vertical position of a coefficient given a level,
 * orientation and vertical position within the subband.
 *
 * @param level subband level
 * @param orientation orientation of the subband within the level
 * @param y position within the subband
 * @return vertical position within the coefficient array
 */
static inline
int coeff_posy(DiracContext *s, int level, subband_t orientation, int y)
{
    if (orientation == subband_lh || orientation == subband_hh)
        return subband_height(s, level) + y;

    return y;
}

static inline
int zero_neighbourhood(DiracContext *s, int16_t *data, int v, int h)
{
    /* Check if there is a zero to the left and top left of this
       coefficient.  */
    if (v > 0 && (data[-s->padded_width]
                  || ( h > 0 && data[-s->padded_width - 1])))
        return 0;
    else if (h > 0 && data[- 1])
        return 0;

    return 1;
}

/**
 * Determine the most efficient context to use for arithmetic decoding
 * of this coefficient (given by a position in a subband).
 *
 * @param current coefficient
 * @param v vertical position of the coefficient
 * @param h horizontal position of the coefficient
 * @return prediction for the sign: -1 when negative, 1 when positive, 0 when 0
 */
static inline
int sign_predict(DiracContext *s, int16_t *data, subband_t orientation,
                 int v, int h)
{
    if (orientation == subband_hl && v > 0)
        return DIRAC_SIGN(data[-s->padded_width]);
    else if (orientation == subband_lh && h > 0)
        return DIRAC_SIGN(data[-1]);
    else
        return 0;
}

static inline
int intra_dc_coeff_prediction(DiracContext *s, int16_t *coeff, int x, int y)
{
    int pred;
    if (x > 0 && y > 0) {
        pred = (coeff[-1]
                + coeff[-s->padded_width]
                + coeff[-s->padded_width - 1]);
        if (pred > 0)
            pred = (pred + 1) / 3;
        else /* XXX: For now just do what the reference
                implementation does.  Check this.  */
            pred = -((-pred)+1)/3;
    } else if (x > 0) {
        /* Just use the coefficient left of this one.  */
                pred = coeff[-1];
    } else if (y > 0)
        pred = coeff[-s->padded_width];
    else
        pred = 0;

    return pred;
}

struct dirac_block_params {
    int xblen;
    int yblen;
    int xbsep;
    int ybsep;
};

extern const struct dirac_block_params ff_dirac_block_param_defaults[];

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

    /* Return the majority.  */
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
           reference frame.  */
        if ((block[-1].use_ref & mask) == refmask) {
            left = block[-1].vect[ref][dir];
            cnt++;
        }

        /* This is the only reference, return it.  */
        if (y == 0)
            return left;
    }

    if (y > 0) {
        /* Test if the block above the current one has a motion vector
           for this reference frame.  */
        if ((block[-s->blwidth].use_ref & mask) == refmask) {
            top = block[-s->blwidth].vect[ref][dir];
            cnt++;
        }

        /* This is the only reference, return it.  */
        if (x == 0)
            return top;
        else if (x > 0) {
            /* Test if the block above the current one has a motion vector
               for this reference frame.  */
            if ((block[-s->blwidth - 1].use_ref & mask) == refmask) {
                lefttop = block[-s->blwidth - 1].vect[ref][dir];
                cnt++;
            }
        }
    }

    /* No references for the prediction.  */
    if (cnt == 0)
        return 0;

    if (cnt == 1)
        return left + top + lefttop;

    /* Return the median of two motion vectors.  */
    if (cnt == 2)
        return (left + top + lefttop + 1) >> 1;

    /* Return the median of three motion vectors.  */
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

    /* Return the average of all DC values that were counted.  */
    return sign * (total + (cnt >> 1)) / cnt;
}

int dirac_reference_frame_idx(DiracContext *s, int frameno);

int dirac_motion_compensation(DiracContext *s, int16_t *coeffs, int comp);

int dirac_decode_frame(AVCodecContext *avctx, void *data, int *data_size,
                       const uint8_t *buf, int buf_size);

void dirac_dump_source_parameters(AVCodecContext *avctx);

int ff_dirac_parse_sequence_header(DiracContext *s);

#endif /* AVCODEC_DIRAC_H */
