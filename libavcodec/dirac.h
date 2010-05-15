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
 * @file
 * Interfaces to Dirac Decoder/Encoder
 * @author Marco Gerards <marco@gnu.org>
 */

#include "avcodec.h"
#include "get_bits.h"
#include "dirac_arith.h"
#include "dsputil.h"
#include "dwt.h"

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

int ff_dirac_parse_sequence_header(AVCodecContext *avctx, GetBitContext *gb,
                                   dirac_source_params *source);


#define CALC_PADDING(size, depth) \
         (((size + (1 << depth) - 1) >> depth) << depth)

#define DIVRNDUP(a, b) (((a) + (b) - 1) / (b))

typedef struct {
    FF_COMMON_FRAME

    int interpolated[3];    ///< 1 if hpel[] is valid
    uint8_t *hpel[3][4];
    uint8_t *hpel_base[3][4];
} DiracFrame;

#define DIRAC_REF_MASK_REF1   1
#define DIRAC_REF_MASK_REF2   2
#define DIRAC_REF_MASK_GLOBAL 4

typedef struct {
    union {
        int16_t mv[2][2];
        int16_t dc[3];
    } u;
    uint8_t ref;
} DiracBlock;

/**
 * The spec limits the number of wavelet decompositions to 4 for both
 * level 1 (VC-2) and 128 (long-gop default).
 * 5 decompositions is the maximum before >16-bit buffers are needed.
 * Schroedinger allows this for DD 9,7 and 13,7 wavelets only, limiting
 * the others to 4 decompositions (or 3 for the fidelity filter).
 *
 * We use this instead of MAX_DECOMPOSITIONS to save some memory.
 */
#define MAX_DWT_LEVELS 5

#define MAX_REFERENCE_FRAMES 8
#define MAX_DELAY 5+1
#define MAX_FRAMES (MAX_REFERENCE_FRAMES + MAX_DELAY + 1)
#define MAX_BLOCKSIZE 32    ///< maximum blen
#define MAX_QUANT 57        ///< 57 is the last quant to not always overflow int16

typedef struct SubBand {
    int level;
    int orientation;
    int stride;
    int width;
    int height;
    unsigned quant;
    IDWTELEM *ibuf;
    struct SubBand *parent;

    // for low delay
    unsigned length;
    const uint8_t *coeff_data;
} SubBand;

typedef struct Plane {
    int width;
    int height;
    int stride;

    int idwt_width;
    int idwt_height;
    int idwt_stride;
    IDWTELEM *idwt_buf;
    IDWTELEM *idwt_buf_base;
    IDWTELEM *idwt_tmp;

    // block length
    uint8_t xblen;
    uint8_t yblen;
    // block separation (block n+1 starts after this many pixels in block n)
    uint8_t xbsep;
    uint8_t ybsep;
    // amount of overspill on each edge (half of the overlap between blocks)
    uint8_t xoffset;
    uint8_t yoffset;

    SubBand band[MAX_DWT_LEVELS][4];
} Plane;

typedef struct DiracContext {
    AVCodecContext *avctx;
    DSPContext dsp;
    GetBitContext gb;
    dirac_source_params source;
    int seen_sequence_header;
    Plane plane[3];
    int chroma_x_shift;
    int chroma_y_shift;

    int zero_res;             ///< zero residue flag
    int is_arith;             ///< whether coeffs use arith or golomb coding
    int low_delay;            ///< use the low delay syntax
    int globalmc_flag;        ///< use global motion compensation
    int num_refs;             ///< number of reference pictures

    // wavelet decoding
    unsigned wavelet_depth;   ///< depth of the IDWT
    unsigned wavelet_idx;

    /** schroedinger 1.0.8 and newer stores quant delta for all codeblocks */
    unsigned new_delta_quant;
    unsigned codeblock_mode;

    struct {
        unsigned width;
        unsigned height;
    } codeblock[MAX_DWT_LEVELS+1];

    struct {
        unsigned num_x;         ///< number of horizontal slices
        unsigned num_y;         ///< number of vertical slices
        AVRational bytes;       ///< average bytes per slice
        uint8_t quant[MAX_DWT_LEVELS][4];
    } lowdelay;

    struct {
        int pan_tilt[2];        ///< pan/tilt vector
        int zrs[2][2];          ///< zoom/rotate/shear matrix
        int perspective[2];     ///< perspective vector
        unsigned zrs_exp;
        unsigned perspective_exp;
    } globalmc[2];

    // motion compensation
    uint8_t mv_precision;
    int16_t picture_weight_ref1;
    int16_t picture_weight_ref2;
    unsigned picture_weight_precision;

    int blwidth;              ///< number of blocks (horizontally)
    int blheight;             ///< number of blocks (vertically)
    int sbwidth;              ///< number of superblocks (horizontally)
    int sbheight;             ///< number of superblocks (vertically)

    uint8_t *sbsplit;
    DiracBlock *blmotion;

    int16_t spatialwt[MAX_BLOCKSIZE*MAX_BLOCKSIZE];

    uint8_t *edge_emu_buffer[4];
    uint8_t *edge_emu_buffer_base;

    // TODO: try obmc_buf[]
    uint16_t *mctmp;
    uint8_t *mcscratch;

#define OBMC_TL 0
#define OBMC_TR 1
#define OBMC_BL 2
#define OBMC_BR 3
    // [ref][OBMC_*]
    uint16_t *obmc_buf[4];
    uint16_t *obmc_scratch;
    int obmc_stride;

    /**
     * The obmc weights for the current block row
     * [0] is for the 1st block, [2] is for the last block
     */
    // uint8_t *obmc_weight[3];

    DECLARE_ALIGNED(16, uint8_t, obmc_weight)[3][MAX_BLOCKSIZE*MAX_BLOCKSIZE];

    void (*put_pixels_tab[4])(uint8_t *dst, uint8_t *src[5], int stride, int h);
    void (*avg_pixels_tab[4])(uint8_t *dst, uint8_t *src[5], int stride, int h);

    DiracFrame *current_picture;
    DiracFrame *ref_pics[2];

    DiracFrame *ref_frames[MAX_REFERENCE_FRAMES+1];
    DiracFrame *delay_frames[MAX_DELAY+1];
    DiracFrame all_frames[MAX_FRAMES];
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

#endif /* AVCODEC_DIRAC_H */
