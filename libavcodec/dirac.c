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
 * @file dirac.c
 * Dirac Decoder
 * @author Marco Gerards <marco@gnu.org>
 */

#define DEBUG 1

#include "avcodec.h"
#include "dsputil.h"
#include "bitstream.h"
#include "golomb.h"
#include "dirac_arith.h"

typedef enum {
    TRANSFER_FUNC_TV,
    TRANSFER_FUNC_EXTENDED_GAMUT,
    TRANSFER_FUNC_LINEAR,
    TRANSFER_FUNC_DCI_GAMMA
} transfer_func_t;

#define DIRAC_SIGN(x) ((x == 0) ? 0 : FFSIGN(x))

struct source_parameters
{
    /* Interlacing.  */
    int interlaced;                     ///< flag for interlacing
    int top_field_first;
    int sequential_fields;

    AVRational frame_rate;             ///< frame rate

    AVRational aspect_ratio;           ///< aspect ratio

    /* Clean area.  */
    int clean_width;
    int clean_height;
    int clean_left_offset;
    int clean_right_offset;

    /* Luma and chroma offsets.  */
    int luma_offset;
    int luma_excursion;
    int chroma_offset;
    int chroma_excursion;

    int color_spec;
    int color_primaries; /* XXX: ??? */

    float k_r;
    float k_b; /* XXX: ??? */

    transfer_func_t transfer_function;
};

struct sequence_parameters
{
    /* Information about the frames.  */
    int luma_width;                    ///< width of the luma component
    int luma_height;                   ///< height of the luma component
    /** Choma format: 0: 4:4:4, 1: 4:2:2, 2: 4:2:0 */
    int chroma_format;
    int video_depth;                   ///< depth in bits

    /* Calculated:  */
    int chroma_width;                  ///< width of the chroma component
    int chroma_height;                 ///< height of the chroma component
};

struct decoding_parameters
{
    int wavelet_depth;                 ///< depth of the IDWT
    int wavelet_idx_intra;             ///< wavelet transform for intra frames
    int wavelet_idx_inter;             ///< wavelet transform for inter frames

    int luma_xbsep;
    int luma_xblen;
    int luma_ybsep;
    int luma_yblen;

    int mv_precision;

    int picture_weight_ref1;
    int picture_weight_ref2;
    int picture_weight_precision;

    /* Codeblocks h*v.  */
    int intra_hlevel_012, intra_vlevel_012;
    int intra_hlevel_other, intra_vlevel_other;
    int inter_hlevel_01, inter_vlevel_01;
    int inter_hlevel_2, inter_vlevel_2;
    int inter_hlevel_other, inter_vlevel_other;

    int slice_width;
    int slide_height;
    int slice_bits;

    /* Calculated.  */
    int chroma_xbsep;
    int chroma_xblen;
    int chroma_ybsep;
    int chroma_yblen;
};

struct globalmc_parameters {
    int b[2];                          ///< b vector
    int A[2][2];                       ///< A matrix
    int c[2];                          ///< c vector
    int zrs_exp;
    int perspective_exp;
};

/* Defaults for sequence parameters.  */
static const struct sequence_parameters sequence_parameters_defaults[13] =
{
    /* Width   Height   Chroma format   Depth  */
    {  640,    480,     2,              8  },
    {  176,    120,     2,              8  },
    {  176,    144,     2,              8  },
    {  352,    240,     2,              8  },
    {  352,    288,     2,              8  },
    {  704,    480,     2,              8  },
    {  704,    576,     2,              8  },

    {  720,    480,     2,              8  },
    {  720,    576,     2,              8  },
    {  1280,   720,     2,              8  },
    {  1920,   1080,    2,              8  },
    {  2048,   1556,    0,              16 },
    {  4096,   3112,    0,              16 },
};

/* Defaults for source parameters.  */
static const struct source_parameters source_parameters_defaults[13] =
{
    { 0, 1, 0, {30, 1},        {1, 1},   640,  480,  0, 0, 0,  255,   128,   254,   0, 0, 0.2126, 0.0722, TRANSFER_FUNC_TV },
    { 0, 1, 0, {15000, 1001},  {10, 11}, 176,  120,  0, 0, 0,  255,   128,   254,   1, 0, 0.299,  0.144,  TRANSFER_FUNC_TV },
    { 0, 1, 0, {25, 2},        {12, 11}, 176,  144,  0, 0, 0,  255,   128,   254,   2, 0, 0.299,  0.144,  TRANSFER_FUNC_TV },
    { 0, 1, 0, {15000, 1001},  {10, 11}, 352,  240,  0, 0, 0,  255,   128,   254,   1, 0, 0.299,  0.144,  TRANSFER_FUNC_TV },
    { 0, 1, 0, {25, 2},        {12, 11}, 352,  288,  0, 0, 0,  255,   128,   254,   2, 0, 0.299,  0.144,  TRANSFER_FUNC_TV },
    { 0, 1, 0, {15000, 1001},  {10, 11}, 704,  480,  0, 0, 0,  255,   128,   254,   1, 0, 0.299,  0.144,  TRANSFER_FUNC_TV },
    { 0, 1, 0, {25, 2},        {12, 11}, 704,  576,  0, 0, 0,  255,   128,   254,   2, 0, 0.299,  0.144,  TRANSFER_FUNC_TV },

    { 0, 1, 0, {24000, 1001},  {10, 11}, 720,  480,  0, 0, 16, 235,   128,   224,   1, 0, 0.299,  0.144,  TRANSFER_FUNC_TV },
    { 0, 1, 0, {35, 1},        {12, 11}, 720,  576,  0, 0, 16, 235,   128,   224,   2, 0, 0.299,  0.144,  TRANSFER_FUNC_TV },
    { 0, 1, 0, {24, 1},        {1, 1},   1280, 720,  0, 0, 16, 235,   128,   224,   0, 0, 0.2126, 0.0722, TRANSFER_FUNC_TV },
    { 0, 1, 0, {24, 1},        {1, 1},   1920, 1080, 0, 0, 16, 235,   128,   224,   0, 0, 0.2126, 0.0722, TRANSFER_FUNC_TV },
    { 0, 1, 0, {24, 1},        {1, 1},   2048, 1536, 0, 0, 0,  65535, 32768, 65534, 3, 0, 0.25,   0.25,   TRANSFER_FUNC_LINEAR },
    { 0, 1, 0, {24, 1},        {1, 1},   4096, 3072, 0, 0, 0,  65535, 32768, 65534, 3, 0, 0.25,   0.25,   TRANSFER_FUNC_LINEAR },
};

/* Defaults for decoding parameters.  */
static const struct decoding_parameters decoding_parameters_defaults[13] =
{
    { 4, 0, 1, 8,  12, 8,  12, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 32, 32, 512  },
    { 4, 0, 1, 4,   8, 4,   8, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 16, 16, 512  },
    { 4, 0, 1, 4,   8, 4,   8, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 16, 16, 512  },
    { 4, 0, 1, 8,  12, 8,  12, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 32, 32, 512  },
    { 4, 0, 1, 8,  12, 8,  12, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 32, 32, 512  },
    { 4, 0, 1, 8,  12, 8,  12, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 32, 32, 512  },
    { 4, 0, 1, 8,  12, 8,  12, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 32, 32, 512  },

    { 4, 0, 1, 8,  12, 8,  12, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 32, 32, 512  },
    { 4, 0, 1, 8,  12, 8,  12, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 32, 32, 512  },
    { 4, 0, 1, 12, 16, 12, 16, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 48, 48, 768  },
    { 4, 0, 1, 16, 24, 16, 24, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 48, 48, 1024 },
    { 4, 6, 1, 16, 24, 16, 24, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 48, 48, 1024 },
    { 4, 6, 0, 16, 24, 16, 24, 2, 1, 1, 1, 1, 1, 4, 3, 1, 1, 8, 6, 12, 8, 48, 48, 1024 }
};

static const AVRational preset_frame_rates[8] =
{
    {24000, 1001}, {24, 1}, {25, 1}, {30000, 1001},
    {30, 1}, {50, 1}, {60000, 1001}, {60, 1}
};

static const AVRational preset_aspect_ratios[3] =
{
    {1, 1}, {10, 11}, {12, 11}
};

static const int preset_luma_offset[3] = { 0, 16, 64 };
static const int preset_luma_excursion[3] = { 255, 235, 876 };
static const int preset_chroma_offset[3] = { 128, 128, 512 };
static const int preset_chroma_excursion[3] = { 255, 224, 896 };

static const int preset_primaries[4] = { 0, 1, 2, 3 };
static const int preset_matrix[4] = {0, 1, 1, 2 };
static const transfer_func_t preset_transfer_func[3] =
{
    TRANSFER_FUNC_TV, TRANSFER_FUNC_TV, TRANSFER_FUNC_DCI_GAMMA
};
static const float preset_kr[3] = { 0.2126, 0.299, 0 /* XXX */ };
static const float preset_kb[3] = {0.0722, 0.114, 0 /* XXX */ };

struct dirac_blockmotion {
    int use_ref[2];
    int use_global;
    int ref1[2];
    int ref2[2];
    int dc[3];
};

/* XXX */
#define REFFRAME_CNT 20

typedef struct DiracContext {
    int access_unit;
    unsigned int profile;
    unsigned int level;

    GetBitContext *gb;

    AVFrame picture;

    uint32_t picnum;
    int refcnt;
    AVFrame refframes[REFFRAME_CNT]; /* XXX */

    int retirecnt;
    uint32_t retireframe[REFFRAME_CNT];

    struct source_parameters source;
    struct sequence_parameters sequence;
    struct decoding_parameters decoding;

    struct decoding_parameters frame_decoding;

    int codeblocksh[7]; /* XXX: 7 levels.  */
    int codeblocksv[7]; /* XXX: 7 levels.  */

    int padded_luma_width;    ///< padded luma width
    int padded_luma_height;   ///< padded luma height
    int padded_chroma_width;  ///< padded chroma width
    int padded_chroma_height; ///< padded chroma height

    int chroma_hratio;        ///< horizontal ratio of choma
    int chroma_vratio;        ///< vertical ratio of choma

    int blwidth;              ///< amount of blocks (horizontally)
    int blheight;             ///< amount of blocks (vertically)
    int sbwidth;              ///< amount of superblocks (horizontally)
    int sbheight;             ///< amount of superblocks (vertically)

    int zero_res;             ///< zero residue flag

    int refs;                 ///< amount of reference pictures
    int globalmc_flag;        ///< use global motion compensation flag
    /** global motion compensation parameters */
    struct globalmc_parameters globalmc;
    uint32_t ref1;            ///< first reference picture
    uint32_t ref2;            ///< second reference picture

    uint8_t *ref1data;
    int ref1width;
    int ref1height;
    uint8_t *ref2data;
    int ref2width;
    int ref2height;

    /* Current component.  */
    int padded_width;         ///< padded width of the current component
    int padded_height;        ///< padded height of the current component

    int *sbsplit;
    struct dirac_blockmotion *blmotion;

    /** State of arithmetic decoding.  */
    struct dirac_arith_state arith;
} DiracContext;

static int decode_init(AVCodecContext *avctx){
    av_log_set_level(AV_LOG_DEBUG);
    return 0;
}

static int decode_end(AVCodecContext *avctx)
{
    // DiracContext *s = avctx->priv_data;

    return 0;
}


typedef enum {
    pc_access_unit_header = 0x00,
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
 * Dump the sequence parameters.  DEBUG needs to be defined.
 */
static void dump_sequence_parameters(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    struct sequence_parameters *seq = &s->sequence;
    const char *chroma_format_str[] = { "4:4:4", "4:2:2", "4:2:0" };

    dprintf(avctx, "-----------------------------------------------------\n");
    dprintf(avctx, "        Dumping the sequence parameters:\n");
    dprintf(avctx, "-----------------------------------------------------\n");


    dprintf(avctx, "Luma size=%dx%d\n",
            seq->luma_width, seq->luma_height);
    dprintf(avctx, "Chroma size=%dx%d, format: %s\n",
            seq->chroma_width, seq->chroma_height,
            chroma_format_str[seq->chroma_format]);
    dprintf(avctx, "Video depth: %d bpp\n", seq->video_depth);

    dprintf(avctx, "-----------------------------------------------------\n");

}

/**
 * Dump the source parameters.  DEBUG needs to be defined.
 */
static void dump_source_parameters(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    struct source_parameters *source = &s->source;

    dprintf(avctx, "-----------------------------------------------------\n");
    dprintf(avctx, "        Dumping source parameters:\n");
    dprintf(avctx, "-----------------------------------------------------\n");

    if (! source->interlaced)
        dprintf(avctx, "No interlacing\n");
    else
        dprintf(avctx, "Interlacing: top fields first=%d\n, seq. fields=%d\n",
                source->top_field_first, source->sequential_fields);

    dprintf(avctx, "Frame rate: %d/%d = %f\n",
            source->frame_rate.num, source->frame_rate.den,
            (double) source->frame_rate.num / source->frame_rate.den);
    dprintf(avctx, "Aspect ratio: %d/%d = %f\n",
            source->aspect_ratio.num, source->aspect_ratio.den,
            (double) source->aspect_ratio.num / source->aspect_ratio.den);

    dprintf(avctx, "Clean space: loff=%d, roff=%d, size=%dx%d\n",
            source->clean_left_offset, source->clean_right_offset,
            source->clean_width, source->clean_height);

    dprintf(avctx, "Luma offset=%d, Luma excursion=%d\n",
            source->luma_offset, source->luma_excursion);
    dprintf(avctx, "Croma offset=%d, Chroma excursion=%d\n",
            source->chroma_offset, source->chroma_excursion);

    /* XXX: This list is incomplete, add the other members.  */

    dprintf(avctx, "-----------------------------------------------------\n");
}


/**
 * Parse the sequence parameters in the access unit header
 */
static void parse_sequence_parameters(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;

    /* Override the luma dimensions.  */
    if (get_bits(gb, 1)) {
        s->sequence.luma_width = dirac_get_ue_golomb(gb);
        s->sequence.luma_height = dirac_get_ue_golomb(gb);
    }

    /* Override the chroma format.  */
    if (get_bits(gb, 1))
        s->sequence.chroma_format = dirac_get_ue_golomb(gb);

    /* Override the chroma dimensions.  */
    switch (s->sequence.chroma_format) {
    case 0:
        /* 4:4:4 */
        s->sequence.chroma_width = s->sequence.luma_width;
        s->sequence.chroma_height = s->sequence.luma_height;
        s->chroma_hratio = 1;
        s->chroma_vratio = 1;
        break;

    case 1:
        /* 4:2:2 */
        s->sequence.chroma_width = s->sequence.luma_width >> 1;
        s->sequence.chroma_height = s->sequence.luma_height;
        s->chroma_hratio = 1;
        s->chroma_vratio = 2;
        break;

    case 2:
        /* 4:2:0 */
        s->sequence.chroma_width = s->sequence.luma_width >> 1;
        s->sequence.chroma_height = s->sequence.luma_height >> 1;
        s->chroma_hratio = 2;
        s->chroma_vratio = 2;
        break;
    }

    /* Override the video depth.  */
    if (get_bits(gb, 1))
        s->sequence.video_depth = dirac_get_ue_golomb(gb);
}

/**
 * Parse the source parameters in the access unit header
 */
static void parse_source_parameters(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;

    /* Access Unit Source parameters.  */
    if (get_bits(gb, 1)) {
        /* Interlace.  */
        s->source.interlaced = get_bits(gb, 1);

        if (s->source.interlaced) {
            if (get_bits(gb, 1))
                s->source.top_field_first = get_bits(gb, 1);

            if (get_bits(gb, 1))
                s->source.sequential_fields = get_bits(gb, 1);
        }
    }

    /* Framerate.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_get_ue_golomb(gb);
        if (! idx) {
            s->source.frame_rate.num = dirac_get_ue_golomb(gb);
            s->source.frame_rate.den = dirac_get_ue_golomb(gb);
        } else {
            /* Use a pre-set framerate.  */
            s->source.frame_rate = preset_frame_rates[idx - 1];
        }
    }

    /* Override aspect ratio.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_get_ue_golomb(gb);
        if (! idx) {
            s->source.aspect_ratio.num = dirac_get_ue_golomb(gb);
            s->source.aspect_ratio.den = dirac_get_ue_golomb(gb);
        } else {
            /* Use a pre-set aspect ratio.  */
            s->source.aspect_ratio = preset_aspect_ratios[idx - 1];
        }
    }

    /* Override clean area.  */
    if (get_bits(gb, 1)) {
        s->source.clean_width = dirac_get_ue_golomb(gb);
        s->source.clean_height = dirac_get_ue_golomb(gb);
        s->source.clean_left_offset = dirac_get_ue_golomb(gb);
        s->source.clean_right_offset = dirac_get_ue_golomb(gb);
    }

    /* Override signal range.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_get_ue_golomb(gb);
        if (! idx) {
            s->source.luma_offset = dirac_get_ue_golomb(gb);
            s->source.luma_excursion = dirac_get_ue_golomb(gb);
            s->source.chroma_offset = dirac_get_ue_golomb(gb);
            s->source.chroma_excursion = dirac_get_ue_golomb(gb);
        } else {
            /* Use a pre-set signal range.  */
            s->source.luma_offset = preset_luma_offset[idx - 1];
            s->source.luma_excursion = preset_luma_excursion[idx - 1];
            s->source.chroma_offset = preset_chroma_offset[idx - 1];
            s->source.chroma_excursion = preset_chroma_excursion[idx - 1];
        }
    }

    /* Color spec.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_get_ue_golomb(gb);

        s->source.color_primaries = preset_primaries[idx];
        s->source.k_r = preset_kr[preset_matrix[idx]];
        s->source.k_b = preset_kb[preset_matrix[idx]];
        s->source.transfer_function = preset_transfer_func[idx];

        /* XXX: color_spec?  */

        if (! idx) {
            /* Color primaries.  */
            if (get_bits(gb, 1)) {
                int primaries_idx = dirac_get_ue_golomb(gb);
                s->source.color_primaries = preset_primaries[primaries_idx];
            }

            /* Override matrix.  */
            if (get_bits(gb, 1)) {
                int matrix_idx = dirac_get_ue_golomb(gb);

                s->source.k_r = preset_kr[preset_matrix[matrix_idx]];
                s->source.k_b = preset_kb[preset_matrix[matrix_idx]];
            }

            /* Transfer function.  */
            if (get_bits(gb, 1)) {
                int tf_idx = dirac_get_ue_golomb(gb);
                s->source.transfer_function = preset_transfer_func[tf_idx];
            }
        } else {
            /* XXX: Use the index.  */
        }
    }

}

/**
 * Parse the access unit header
 */
static int parse_access_unit_header(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;
    unsigned int version_major;
    unsigned int version_minor;
    unsigned int video_format;

    /* Parse parameters.  */
    version_major = dirac_get_ue_golomb(gb);
    version_minor = dirac_get_ue_golomb(gb);
    /* XXX: Don't check the version yet, existing encoders do not yet
       set this to a sane value (0.6 at the moment).  */

    /* XXX: Not yet documented in the spec.  This is actually the main
       thing that is missing.  */
    s->profile = dirac_get_ue_golomb(gb);
    s->level = dirac_get_ue_golomb(gb);

    dprintf(avctx, "Access unit header: Version %d.%d\n",
            version_major, version_minor);
    dprintf(avctx, "Profile: %d, Level: %d\n", s->profile, s->level);

    video_format = dirac_get_ue_golomb(gb);
    dprintf(avctx, "Video format: %d\n", video_format);

    /* Fill in defaults for the sequence parameters.  */
    memcpy(&s->sequence, &sequence_parameters_defaults[video_format],
           sizeof(s->sequence));
    /* Override the defaults.  */
    parse_sequence_parameters(avctx);

    /* Fill in defaults for the source parameters.  */
    memcpy(&s->source, &source_parameters_defaults[video_format],
           sizeof(s->source));
    /* Override the defaults.  */
    parse_source_parameters(avctx);

    /* Fill in defaults for the decoding parameters.  */
    memcpy(&s->decoding, &decoding_parameters_defaults[video_format],
           sizeof(s->decoding));

    return 0;
}

static struct dirac_arith_context_set context_set_split =
    {
        .follow = { ARITH_CONTEXT_SB_F1, ARITH_CONTEXT_SB_F2 },
        .follow_length = 2,
        .data = ARITH_CONTEXT_SB_DATA
    };

static struct dirac_arith_context_set context_set_mv =
    {
        .follow = { ARITH_CONTEXT_VECTOR_F1, ARITH_CONTEXT_VECTOR_F2,
                    ARITH_CONTEXT_VECTOR_F3, ARITH_CONTEXT_VECTOR_F4,
                    ARITH_CONTEXT_VECTOR_F5 },
        .follow_length = 5,
        .data = ARITH_CONTEXT_VECTOR_DATA,
        .sign = ARITH_CONTEXT_VECTOR_SIGN
    };
static struct dirac_arith_context_set context_set_dc =
    {
        .follow = { ARITH_CONTEXT_DC_F1, ARITH_CONTEXT_DC_F2 },
        .follow_length = 2,
        .data = ARITH_CONTEXT_DC_DATA,
        .sign = ARITH_CONTEXT_DC_SIGN
    };

static struct dirac_arith_context_set context_sets_waveletcoeff[12] = {
    {
        /* Parent = 0, Zero neighbourhood, sign predict 0 */
        .follow = { ARITH_CONTEXT_ZPZN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_ZERO,
    }, {
        /* Parent = 0, Zero neighbourhood, sign predict < 0 */
        .follow = { ARITH_CONTEXT_ZPZN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_NEG
    }, {
        /* Parent = 0, Zero neighbourhood, sign predict > 0 */
        .follow = { ARITH_CONTEXT_ZPZN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_POS
    },

    {
        /* Parent = 0, No Zero neighbourhood, sign predict  0 */
        .follow = { ARITH_CONTEXT_ZPNN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_ZERO
    }, {
        /* Parent = 0, No Zero neighbourhood, sign predict < 0 */
        .follow = { ARITH_CONTEXT_ZPNN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_NEG
    }, {
        /* Parent = 0, No Zero neighbourhood, sign predict > 0 */
        .follow = { ARITH_CONTEXT_ZPNN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_POS
    },

    {
        /* Parent != 0, Zero neighbourhood, sign predict 0 */
        .follow = { ARITH_CONTEXT_NPZN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_ZERO
    }, {
        /* Parent != 0, Zero neighbourhood, sign predict < 0 */
        .follow = { ARITH_CONTEXT_NPZN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_NEG
    }, {
        /* Parent != 0, Zero neighbourhood, sign predict > 0 */
        .follow = { ARITH_CONTEXT_NPZN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_POS
    },


    {
        /* Parent != 0, No Zero neighbourhood, sign predict 0 */
        .follow = { ARITH_CONTEXT_NPNN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_ZERO
    }, {
        /* Parent != 0, No Zero neighbourhood, sign predict < 0 */
        .follow = { ARITH_CONTEXT_NPNN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_NEG
    }, {
        /* Parent != 0, No Zero neighbourhood, sign predict > 0 */
        .follow = { ARITH_CONTEXT_NPNN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .follow_length = 6,
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_POS
    }
};

/**
 * Calculate the width of a subband on a given level
 *
 * @param level the level of the subband
 * @return width of the subband
 */
static int inline subband_width(AVCodecContext *avctx, int level) {
    DiracContext *s = avctx->priv_data;
    if (level == 0)
        return s->padded_width >> s->frame_decoding.wavelet_depth;
    return s->padded_width >> (s->frame_decoding.wavelet_depth - level + 1);
}

/**
 * Calculate the height of a subband on a given level
 *
 * @param level the level of the subband
 * @return height of the subband
 */
static int inline subband_height(AVCodecContext *avctx, int level) {
    DiracContext *s = avctx->priv_data;
    if (level == 0)
        return s->padded_height >> s->frame_decoding.wavelet_depth;
    return s->padded_height >> (s->frame_decoding.wavelet_depth - level + 1);
}

static int inline coeff_quant_factor(int idx) {
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

static int inline coeff_quant_offset(AVCodecContext *avctx, int idx) {
    DiracContext *s = avctx->priv_data;

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
 * Dequantize a coefficient
 *
 * @param coeff coefficient to dequantize
 * @param qoffset quantizer offset
 * @param qfactor quantizer factor
 * @return dequantized coefficient
 */
static int inline coeff_dequant(AVCodecContext *avctx, int coeff,
                                int qoffset, int qfactor) {
    int64_t magnitude = abs(coeff) * qfactor;

    if (! magnitude)
        return 0;

    magnitude += qoffset;
    magnitude >>= 2;

    /* Reintroduce the sign.  */
    if (coeff < 0)
        magnitude = -magnitude;
    return magnitude;
}

/**
 * Calculate the horizontal position of a coefficient given a level,
 * orientation and horizontal position within the subband.
 *
 * @param level level of the subband
 * @param orientation orientation of the subband within the level
 * @param x position within the subband
 * @return horizontal position within the coefficient array
 */
static int inline coeff_posx(AVCodecContext *avctx, int level,
                             subband_t orientation, int x) {
    if (orientation == subband_hl || orientation == subband_hh)
        return subband_width(avctx, level) + x;

    return x;
}

/**
 * Calculate the vertical position of a coefficient given a level,
 * orientation and vertical position within the subband.
 *
 * @param level level of the subband
 * @param orientation orientation of the subband within the level
 * @param y position within the subband
 * @return vertical position within the coefficient array
 */
static int inline coeff_posy(AVCodecContext *avctx, int level,
                             subband_t orientation, int y) {
    if (orientation == subband_lh || orientation == subband_hh)
        return subband_height(avctx, level) + y;

    return y;
}

/**
 * Returns if the pixel has a zero neighbourhood (the coefficient at
 * the left, top and left top of this coefficient are all zero)
 *
 * @param data coefficients
 * @param level level of the current subband
 * @param orientation the orientation of the current subband
 * @param v vertical position of the coefficient
 * @param h horizontal position of the coefficient
 * @return 1 if zero neighbourhood, otherwise 0
 */
static int zero_neighbourhood(AVCodecContext *avctx, int *data, int level,
                              subband_t orientation, int v, int h) {
    int x = coeff_posx(avctx, level, orientation, h);
    int y = coeff_posy(avctx, level, orientation, v);
    DiracContext *s = avctx->priv_data;

    /* Check if there is a zero to the left and top left of this
       coefficient.  */
    if (v > 0 && ((data[x + (y - 1) * s->padded_width])
                  || ( h > 0 && data[x + (y - 1) * s->padded_width - 1])))
        return 0;
    else if (h > 0 && data[x + y * s->padded_width - 1])
        return 0;

    return 1;
}

/**
 * Determine the most efficient context to use for arithmetic decoding
 * of this coefficient (given by a position in a subband).
 *
 * @param data coefficients
 * @param level level of subband
 * @param v vertical position of the coefficient
 * @param h horizontal position of the coefficient
 * @return prediction for the sign: -1 when negative, 1 when positive, 0 when 0
 */
static int sign_predict(AVCodecContext *avctx, int *data, int level,
                        subband_t orientation, int v, int h) {
    int x = coeff_posx(avctx, level, orientation, h);
    int y = coeff_posy(avctx, level, orientation, v);
    DiracContext *s = avctx->priv_data;

    if (orientation == subband_hl && v > 0)
        return DIRAC_SIGN(data[x + (y - 1) * s->padded_width]);
    else if (orientation == subband_lh && h > 0)
        return DIRAC_SIGN(data[x + y * s->padded_width - 1]);
    else
        return 0;
}

/**
 * Unpack a single coefficient
 *
 * @param data coefficients
 * @param level level of the current subband
 * @param orientation orientation of the subband
 * @param v vertical position of the to be decoded coefficient in the subband
 * @param h horizontal position of the to be decoded coefficient in the subband
 * @param qoffset quantizer offset
 * @param qfact quantizer factor
 */
static void coeff_unpack(AVCodecContext *avctx, int *data, int level,
                         subband_t orientation, int v, int h,
                         int qoffset, int qfactor) {
    int parent = 0;
    int nhood;
    int sign_pred;
    int idx;
    int coeff;
    struct dirac_arith_context_set *context;
    DiracContext *s = avctx->priv_data;
    int vdata, hdata;

    /* The value of the pixel belonging to the lower level.  */
    if (level >= 2) {
        int x = coeff_posx(avctx, level - 1, orientation, h >> 1);
        int y = coeff_posy(avctx, level - 1, orientation, v >> 1);
        parent = data[s->padded_width * y + x] != 0;
    }

    /* Determine if the pixel has only zeros in its neighbourhood.  */
    nhood = zero_neighbourhood(avctx, data, level, orientation, v, h);

    sign_pred = sign_predict(avctx, data, level, orientation, v, h);

    /* Calculate an index into context_sets_waveletcoeff.  */
    idx = parent * 6 + (!nhood) * 3;
    if (sign_pred == -1)
        idx += 1;
    else if (sign_pred == 1)
        idx += 2;

    context = &context_sets_waveletcoeff[idx];

    coeff = dirac_arith_read_int(&s->arith, context);
    vdata = coeff_posy(avctx, level, orientation, v);
    hdata = coeff_posx(avctx, level, orientation, h);
    coeff = coeff_dequant(avctx, coeff, qoffset, qfactor);

    data[hdata + vdata * s->padded_width] = coeff;
}

/**
 * Decode a codeblock
 *
 * @param data coefficients
 * @param level level of the current subband
 * @param orientation orientation of the current subband
 * @param x position of the codeblock within the subband in units of codeblocks
 * @param y position of the codeblock within the subband in units of codeblocks
 * @param quant quantizer offset
 * @param quant quantizer factor
 */
static void codeblock(AVCodecContext *avctx, int *data, int level,
                      subband_t orientation, int x, int y,
                      int qoffset, int qfactor) {
    DiracContext *s = avctx->priv_data;
    int blockcnt_one = (s->codeblocksh[level] + s->codeblocksv[level]) == 2;
    int zero = 0;
    int left, right, top, bottom;
    int v, h;

    left   = (subband_width(avctx, level)  *  x     ) / s->codeblocksh[level];
    right  = (subband_width(avctx, level)  * (x + 1)) / s->codeblocksh[level];
    top    = (subband_height(avctx, level) *  y     ) / s->codeblocksv[level];
    bottom = (subband_height(avctx, level) * (y + 1)) / s->codeblocksv[level];

    if (!blockcnt_one) {
        /* Determine if this codeblock is a zero block.  */
        zero = dirac_arith_get_bit(&s->arith, ARITH_CONTEXT_ZERO_BLOCK);
    }

    if (zero)
        return; /* All coefficients remain 0.  */

    /* XXX: This matches the reference implementation, check the
       spec.  */
    for (v = top; v < bottom; v++)
        for (h = left; h < right; h++)
            coeff_unpack(avctx, data, level, orientation, v, h,
                         qoffset, qfactor);
}

/**
 * Intra DC Prediction
 *
 * @param data coefficients
 */
static void intra_dc_prediction(AVCodecContext *avctx, int *data) {
    DiracContext *s = avctx->priv_data;
    int pred;
    int x, y;
    int *line = data;

    for (y = 0; y < subband_height(avctx, 0); y++) {
        for (x = 0; x < subband_width(avctx, 0); x++) {
            if (x > 0 && y > 0) {
                /* Use 3 coefficients for prediction.  XXX: check
                   why mid_pred can't be used.  */
                pred = (line[x - 1]
                        + line[x - s->padded_width]
                        + line[x - s->padded_width - 1]);
                if (pred > 0)
                    pred = (pred + 1) / 3;
                else /* XXX: For now just do what the reference
                        implementation does.  Check this.  */
                    pred = -((-pred)+1)/3;
            } else if (x > 0) {
                /* Just use the coefficient left of this one.  */
                pred = data[x - 1];
            } else if (y > 0)
                pred = line[-s->padded_width];
            else
                pred = 0;

            line[x] += pred;
        }
        line += s->padded_width;
    }
}

/**
 * Decode a subband
 *
 * @param data coefficients
 * @param level level of the subband
 * @param orientation orientation of the subband
 */
static int subband(AVCodecContext *avctx, int *data, int level,
                   subband_t orientation) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;
    int length;
    int quant, qoffset, qfactor;
    int x, y;

    length = dirac_get_ue_golomb(gb);
    if (! length) {
        align_get_bits(gb);
    } else {
        quant = dirac_get_ue_golomb(gb);
        qfactor = coeff_quant_factor(quant);
        qoffset = coeff_quant_offset(avctx, quant) + 2;

        dirac_arith_init(&s->arith, gb, length);

        for (y = 0; y < s->codeblocksv[level]; y++)
            for (x = 0; x < s->codeblocksh[level]; x++)
                codeblock(avctx, data, level, orientation, x, y,
                          qoffset, qfactor);
        dirac_arith_flush(&s->arith);
    }

    if (level == 0 && s->refs == 0)
        intra_dc_prediction(avctx, data);

    return 0;
}


struct block_params {
    int xblen;
    int yblen;
    int xbsep;
    int ybsep;
};

static const struct block_params block_param_defaults[] = {
    {  8,  8,  4,  4 },
    { 12, 12,  8,  8 },
    { 16, 16, 12, 12 },
    { 24, 24, 16, 16 }
};

/**
 * Unpack the motion compensation parameters
 */
static void dirac_unpack_prediction_parameters(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;

    /* Override block parameters.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_get_ue_golomb(gb);
        if (idx == 0) {
            s->frame_decoding.luma_xblen = dirac_get_ue_golomb(gb);
            s->frame_decoding.luma_yblen = dirac_get_ue_golomb(gb);
            s->frame_decoding.luma_xbsep = dirac_get_ue_golomb(gb);
            s->frame_decoding.luma_ybsep = dirac_get_ue_golomb(gb);
        } else {
            s->frame_decoding.luma_xblen = block_param_defaults[idx].xblen;
            s->frame_decoding.luma_yblen = block_param_defaults[idx].yblen;
            s->frame_decoding.luma_xbsep = block_param_defaults[idx].xbsep;
            s->frame_decoding.luma_ybsep = block_param_defaults[idx].ybsep;
        }
    }

    /* Setup the blen and bsep parameters for the chroma
       component.  */
    s->frame_decoding.chroma_xblen = s->frame_decoding.luma_xblen / s->chroma_hratio;
    s->frame_decoding.chroma_yblen = s->frame_decoding.luma_yblen / s->chroma_vratio;
    s->frame_decoding.chroma_xbsep = s->frame_decoding.luma_xbsep / s->chroma_hratio;
    s->frame_decoding.chroma_ybsep = s->frame_decoding.luma_ybsep / s->chroma_vratio;

    /* Override motion vector precision.  */
    if (get_bits(gb, 1))
        s->frame_decoding.mv_precision = dirac_get_ue_golomb(gb);

    /* Read the global motion compensation parameters.  */
    s->globalmc_flag = get_bits(gb, 1);
    if (s->globalmc_flag) {
        int ref;
        for (ref = 0; ref < s->refs; ref++) {
            /* Pan/til parameters.  */
            if (get_bits(gb, 1)) {
                s->globalmc.b[0] = dirac_get_se_golomb(gb);
                s->globalmc.b[1] = dirac_get_se_golomb(gb);
            } else {
                s->globalmc.b[0] = 0;
                s->globalmc.b[1] = 0;
            }

            /* Rotation/shear parameters.  */
            if (get_bits(gb, 1)) {
                s->globalmc.zrs_exp = dirac_get_ue_golomb(gb);
                s->globalmc.A[0][0] = dirac_get_se_golomb(gb);
                s->globalmc.A[0][1] = dirac_get_se_golomb(gb);
                s->globalmc.A[1][0] = dirac_get_se_golomb(gb);
                s->globalmc.A[1][1] = dirac_get_se_golomb(gb);
            } else {
                s->globalmc.zrs_exp = 0;
                s->globalmc.A[0][0] = 0;
                s->globalmc.A[0][1] = 0;
                s->globalmc.A[1][0] = 0;
                s->globalmc.A[1][1] = 0;
            }

            /* Perspective parameters.  */
            if (get_bits(gb, 1)) {
                s->globalmc.perspective_exp = dirac_get_ue_golomb(gb);
                s->globalmc.c[0]            = dirac_get_se_golomb(gb);
                s->globalmc.c[1]            = dirac_get_se_golomb(gb);
            } else {
                s->globalmc.perspective_exp = 0;
                s->globalmc.c[0]            = 0;
                s->globalmc.c[1]            = 0;
            }
        }
    }

    /* Picture prediction mode.  XXX: Not used yet in the specification.  */
    if (get_bits(gb, 1)) {
        /* XXX: Just ignore it, it should and will be zero.  */
        dirac_get_ue_golomb(gb);
    }

    /* Override reference picture weights.  */
    if (get_bits(gb, 1)) {
        s->frame_decoding.picture_weight_precision = dirac_get_ue_golomb(gb);
        s->frame_decoding.picture_weight_ref1 = dirac_get_se_golomb(gb);
        if (s->refs == 2)
            s->frame_decoding.picture_weight_ref2 = dirac_get_se_golomb(gb);
    }
}

static inline int split_prediction(AVCodecContext *avctx, int x, int y) {
    DiracContext *s = avctx->priv_data;

    if (x == 0 && y == 0)
        return 0;
    else if (y == 0)
        return s->sbsplit[ y      * s->sbwidth + x - 1];
    else if (x == 0)
        return s->sbsplit[(y - 1) * s->sbwidth + x    ];

    /* XXX: Is not precisely the same as mean() in the spec.  */
    return (  s->sbsplit[(y - 1) * s->sbwidth + x    ]
            + s->sbsplit[ y      * s->sbwidth + x - 1]
            + s->sbsplit[(y - 1) * s->sbwidth + x - 1] + 1) / 3;
}

static inline int mode_prediction(AVCodecContext *avctx,
                                  int x, int y, int ref) {
    DiracContext *s = avctx->priv_data;
    int cnt;

    if (x == 0 && y == 0)
        return 0;
    else if (y == 0)
        return s->blmotion[ y      * s->blwidth + x - 1].use_ref[ref];
    else if (x == 0)
        return s->blmotion[(y - 1) * s->blwidth + x    ].use_ref[ref];

    /* Return the majority.  */
    cnt = s->blmotion[ y      * s->blwidth + x - 1].use_ref[ref]
        + s->blmotion[(y - 1) * s->blwidth + x    ].use_ref[ref]
        + s->blmotion[(y - 1) * s->blwidth + x - 1].use_ref[ref];

    if (cnt >= 2)
        return 1;
    else
        return 0;
}

static inline int global_mode_prediction(AVCodecContext *avctx,
                                         int x, int y) {
    DiracContext *s = avctx->priv_data;
    int cnt;

    if (x == 0 && y == 0)
        return 0;
    else if (y == 0)
        return s->blmotion[ y      * s->blwidth + x - 1].use_global;
    else if (x == 0)
        return s->blmotion[(y - 1) * s->blwidth + x    ].use_global;

    /* Return the majority.  */
    cnt = s->blmotion[ y      * s->blwidth + x - 1].use_global
        + s->blmotion[(y - 1) * s->blwidth + x    ].use_global
        + s->blmotion[(y - 1) * s->blwidth + x - 1].use_global;
    if (cnt >= 2)
        return 1;
    else
        return 0;
}

static void blockmode_prediction(AVCodecContext *avctx, int x, int y) {
    DiracContext *s = avctx->priv_data;
    int res = dirac_arith_get_bit(&s->arith, ARITH_CONTEXT_PMODE_REF1);

    res ^= mode_prediction(avctx, x, y, 0);
    s->blmotion[y * s->blwidth + x].use_ref[0] = res;
    if (s->refs == 2) {
        res = dirac_arith_get_bit(&s->arith, ARITH_CONTEXT_PMODE_REF2);
        res ^= mode_prediction(avctx, x, y, 1);
        s->blmotion[y * s->blwidth + x].use_ref[1] = res;
    } else {
        s->blmotion[y * s->blwidth + x].use_ref[1] = 0;
    }
}

static void blockglob_prediction(AVCodecContext *avctx, int x, int y) {
    DiracContext *s = avctx->priv_data;

    s->blmotion[y * s->blwidth + x].use_global = 0;

    /* Global motion compensation is not used at all.  */
    if (!s->globalmc_flag)
        return;

    /* Global motion compensation is not used for this block.  */
    if (s->blmotion[y * s->blwidth + x].use_ref[0] == 0
        || s->blmotion[y * s->blwidth + x].use_ref[0] == 0) {
        int res = dirac_arith_get_bit(&s->arith, ARITH_CONTEXT_GLOBAL_BLOCK);
        res ^= global_mode_prediction(avctx, x, y);
        s->blmotion[y * s->blwidth + x].use_global = res;
    }
}

static void propagate_block_data(AVCodecContext *avctx, int step,
                                 int x, int y) {
    DiracContext *s = avctx->priv_data;
    int i, j;

    /* XXX: For now this is rather inefficient, because everything is
       copied.  This function is called quite often.  */
    for (j = y; j < y + step; j++)
        for (i = x; i < x + step; i++)
            s->blmotion[j * s->blwidth + i] = s->blmotion[y * s->blwidth + x];
}

static int motion_vector_prediction(AVCodecContext *avctx, int x, int y,
                                    int ref, int dir) {
    DiracContext *s = avctx->priv_data;
    int cnt = 0;
    int left = 0, top = 0, lefttop = 0;

    if (x > 0) {
        /* Test if the block to the left has a motion vector for this
           reference frame.  */
        if (!s->blmotion[y * s->blwidth + x - 1].use_global
            && s->blmotion[y * s->blwidth + x - 1].use_ref[ref]) {
            if (ref == 0) /* XXX */
                left = s->blmotion[y * s->blwidth + x - 1].ref1[dir];
            else
                left = s->blmotion[y * s->blwidth + x - 1].ref2[dir];

            cnt++;
        }

        /* This is the only reference, return it.  */
        if (y == 0)
            return left;
    }

    if (y > 0) {
        /* Test if the block above the current one has a motion vector
           for this reference frame.  */
        if (!s->blmotion[(y - 1) * s->blwidth + x].use_global
            && s->blmotion[(y - 1) * s->blwidth + x].use_ref[ref])
            {
                if (ref == 0) /* XXX */
                    top = s->blmotion[(y - 1) * s->blwidth + x].ref1[dir];
                else
                    top = s->blmotion[(y - 1) * s->blwidth + x].ref2[dir];

                cnt++;
            }

        /* This is the only reference, return it.  */
        if (x == 0)
            return top;
    }

    if (x > 0 && y > 0) {
        /* Test if the block above the current one has a motion vector
           for this reference frame.  */
        if (!s->blmotion[(y - 1) * s->blwidth + x - 1].use_global
            && s->blmotion[(y - 1) * s->blwidth + x - 1].use_ref[ref]) {
            if (ref == 0) /* XXX */
                lefttop = s->blmotion[(y - 1) * s->blwidth + x - 1].ref1[dir];
            else
                lefttop = s->blmotion[(y - 1) * s->blwidth + x - 1].ref2[dir];

            cnt++;
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

static int block_dc_prediction(AVCodecContext *avctx,
                               int x, int y, int comp) {
    DiracContext *s = avctx->priv_data;
    int total = 0;
    int cnt = 0;

    if (x > 0) {
        if (   !s->blmotion[y * s->blwidth + x - 1].use_ref[0]
            && !s->blmotion[y * s->blwidth + x - 1].use_ref[1]) {
            total += s->blmotion[y * s->blwidth + x - 1].dc[comp];
            cnt++;
        }
    }

    if (y > 0) {
        if (   !s->blmotion[(y - 1) * s->blwidth + x].use_ref[0]
            && !s->blmotion[(y - 1) * s->blwidth + x].use_ref[1]) {
            total += s->blmotion[(y - 1) * s->blwidth + x].dc[comp];
            cnt++;
        }
    }

    if (x > 0 && y > 0) {
        if (   !s->blmotion[(y - 1) * s->blwidth + x - 1].use_ref[0]
            && !s->blmotion[(y - 1) * s->blwidth + x - 1].use_ref[1]) {
            total += s->blmotion[(y - 1) * s->blwidth + x - 1].dc[comp];
            cnt++;
        }
    }

    if (cnt == 0)
        return 1 << (s->sequence.video_depth - 1);

    /* Return the average of all DC values that were counted.  */
    return (total + (cnt >> 1)) / cnt;
}

static void unpack_block_dc(AVCodecContext *avctx, int x, int y, int comp) {
    DiracContext *s = avctx->priv_data;
    int res;

    s->blmotion[y * s->blwidth + x].dc[comp] = 0; /* XXX */
    if (   s->blmotion[y * s->blwidth + x].use_ref[0]
        || s->blmotion[y * s->blwidth + x].use_ref[1])
        return;

    res = dirac_arith_read_int(&s->arith, &context_set_dc);
    res += block_dc_prediction(avctx, x, y, comp);

    s->blmotion[y * s->blwidth + x].dc[comp] = res;
}

/**
 * Unpack a single motion vector
 *
 * @param ref reference frame
 * @param dir direction horizontal=0, vertical=1
 */
static void dirac_unpack_motion_vector(AVCodecContext *avctx,
                                       int ref, int dir,
                                       int x, int y) {
    DiracContext *s = avctx->priv_data;
    int res;

    /* First determine if for this block in the specific reference
       frame a motion vector is required.  */
    if (!s->blmotion[y * s->blwidth + x].use_ref[ref]
        || s->blmotion[y * s->blwidth + x].use_global)
        return;

    res = dirac_arith_read_int(&s->arith, &context_set_mv);
    res += motion_vector_prediction(avctx, x, y, ref, dir);
    if (ref == 0) /* XXX */
        s->blmotion[y * s->blwidth + x].ref1[dir] = res;
    else
        s->blmotion[y * s->blwidth + x].ref2[dir] = res;
}

/**
 * Unpack motion vectors
 *
 * @param ref reference frame
 * @param dir direction horizontal=0, vertical=1
 */
static void dirac_unpack_motion_vectors(AVCodecContext *avctx,
                                        int ref, int dir) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;
    int length;
    int x, y;

    length = dirac_get_ue_golomb(gb);
    dirac_arith_init(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
                        int q, p;
            int blkcnt = 1 << s->sbsplit[y * s->sbwidth + x];
            int step = 4 >> s->sbsplit[y * s->sbwidth + x];

            for (q = 0; q < blkcnt; q++)
                for (p = 0; p < blkcnt; p++) {
                    dirac_unpack_motion_vector(avctx, ref, dir,
                                         4 * x + p * step,
                                         4 * y + q * step);
                    propagate_block_data(avctx, step,
                                         4 * x + p * step,
                                         4 * y + q * step);
                }
        }
    dirac_arith_flush(&s->arith);
}

/**
 * Unpack the motion compensation parameters
 */
static void dirac_unpack_prediction_data(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;
    int length;
    int comp;
    int x, y;

#define DIVRNDUP(a, b) ((a + b - 1) / b)

    s->sbwidth  = DIVRNDUP(s->sequence.luma_width,
                           (s->frame_decoding.luma_xbsep << 2));
    s->sbheight = DIVRNDUP(s->sequence.luma_height,
                           (s->frame_decoding.luma_ybsep << 2));
    s->blwidth  = s->sbwidth  << 2;
    s->blheight = s->sbheight << 2;

    s->sbsplit  = av_mallocz(s->sbwidth * s->sbheight * sizeof(int));
    s->blmotion = av_mallocz(s->blwidth * s->blheight * sizeof(*s->blmotion));

    /* Superblock splitmodes.  */
    length = dirac_get_ue_golomb(gb);
    dirac_arith_init(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
            int res = dirac_arith_read_uint(&s->arith, &context_set_split);
            s->sbsplit[y * s->sbwidth + x] = (res +
                                              split_prediction(avctx, x, y));
            s->sbsplit[y * s->sbwidth + x] %= 3;
        }
    dirac_arith_flush(&s->arith);

    /* Prediction modes.  */
    length = dirac_get_ue_golomb(gb);
    dirac_arith_init(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
            int q, p;
            int blkcnt = 1 << s->sbsplit[y * s->sbwidth + x];
            int step   = 4 >> s->sbsplit[y * s->sbwidth + x];

            for (q = 0; q < blkcnt; q++)
                for (p = 0; p < blkcnt; p++) {
                    blockmode_prediction(avctx,
                                         4 * x + p * step,
                                         4 * y + q * step);
                    blockglob_prediction(avctx,
                                         4 * x + p * step,
                                         4 * y + q * step);
                    propagate_block_data(avctx, step,
                                         4 * x + p * step,
                                         4 * y + q * step);
                }
        }
    dirac_arith_flush(&s->arith);

    /* Unpack the motion vectors.  */
    dirac_unpack_motion_vectors(avctx, 0, 0);
    dirac_unpack_motion_vectors(avctx, 0, 1);
    if (s->refs == 2) {
        dirac_unpack_motion_vectors(avctx, 1, 0);
        dirac_unpack_motion_vectors(avctx, 1, 1);
    }

    /* Unpack the DC values for all the three components (YUV).  */
    for (comp = 0; comp < 3; comp++) {
        /* Unpack the DC values.  */
        length = dirac_get_ue_golomb(gb);
        dirac_arith_init(&s->arith, gb, length);
        for (y = 0; y < s->sbheight; y++)
            for (x = 0; x < s->sbwidth; x++) {
                int q, p;
                int blkcnt = 1 << s->sbsplit[y * s->sbwidth + x];
                int step   = 4 >> s->sbsplit[y * s->sbwidth + x];

                for (q = 0; q < blkcnt; q++)
                    for (p = 0; p < blkcnt; p++) {
                        unpack_block_dc(avctx,
                                        4 * x + p * step,
                                        4 * y + q * step,
                                        comp);
                        propagate_block_data(avctx, step,
                                             4 * x + p * step,
                                             4 * y + q * step);
                    }
            }
        dirac_arith_flush(&s->arith);
    }
}

/**
 * Decode a single component
 *
 * @param coeffs coefficients for this component
 */
static void decode_component(AVCodecContext *avctx, int *coeffs) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;
    int level;
    subband_t orientation;

   /* Align for coefficient bitstream.  */
    align_get_bits(gb);

     /* Unpack LL, level 0.  */
    subband(avctx, coeffs, 0, subband_ll);

    /* Unpack all other subbands at all levels.  */
    for (level = 1; level <= s->frame_decoding.wavelet_depth; level++) {
        for (orientation = 1; orientation <= subband_hh; orientation++)
            subband(avctx, coeffs, level, orientation);
    }
 }

static void dirac_subband_idwt_reorder(AVCodecContext *avctx, int *data,
                                       int *synth, int level) {
    DiracContext *s = avctx->priv_data;
    int x, y;
    int width       = subband_width(avctx, level);
    int height      = subband_height(avctx, level);
    int synth_width = width  << 1;
    int *synth_line;
    int *line_ll;
    int *line_lh;
    int *line_hl;
    int *line_hh;

    line_ll    = data;
    line_hl    = data + width;
    line_lh    = data + height * s->padded_width;
    line_hh    = data + height * s->padded_width + width;
    synth_line = synth;

    /* Reorder the coefficients.  */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            synth_line[(x << 1)                  ] = line_ll[x];
            synth_line[(x << 1)               + 1] = line_hl[x];
            synth_line[(x << 1) + synth_width    ] = line_lh[x];
            synth_line[(x << 1) + synth_width + 1] = line_hh[x];
        }

        synth_line += synth_width << 1;
        line_ll    += s->padded_width;
        line_lh    += s->padded_width;
        line_hl    += s->padded_width;
        line_hh    += s->padded_width;
    }
}

/**
 * IDWT transform (5,3) for a specific subband
 *
 * @param data coefficients to transform
 * @param level level of the current transform
 * @return 0 when successful, otherwise -1 is returned
 */
static int dirac_subband_idwt_53(AVCodecContext *avctx,
                                 int *data, int level) {
    DiracContext *s = avctx->priv_data;
    int *synth, *synthline;
    int x, y;
    int width = subband_width(avctx, level);
    int height = subband_height(avctx, level);
    int synth_width = width  << 1;
    int synth_height = height << 1;

START_TIMER

    synth = av_malloc(synth_width * synth_height * sizeof(int));
    if (!synth) {
        av_log(avctx, AV_LOG_ERROR, "av_malloc() failed\n");
        return -1;
    }

    dirac_subband_idwt_reorder(avctx, data, synth, level);

    /* LeGall(5,3)
       First lifting step)
       Even, predict, s=5, t_{-1}=-1, t_0=9, t_1=9, t_2=-1:
         A[2*n]   -= (-A[2*n-1] + A[2*n+1] + 2) >> 2

       Second lifting step)
       Odd, update, s=1, t_0=1, t_1=1:
         A[2*n+1] += (A[2*n] + A[2*n+2] + 1) >> 1
    */

    /* Vertical synthesis: Lifting stage 1.  */
    synthline = synth;
    for (x = 0; x < synth_width; x++) {
        synthline[x] -= (synthline[synth_width + x]
                       + synthline[synth_width + x]
                       + 2) >> 2;
    }
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x] -= (synthline[x - synth_width]
                           + synthline[x + synth_width]
                           + 2) >> 2;
        }
        synthline += (synth_width << 1);
    }
    synthline = synth + (synth_height - 2) * synth_width;
    for (x = 0; x < synth_width; x++) {
        synthline[x] -= (synthline[x - synth_width]
                       + synthline[x + synth_width]
                       + 2) >> 2;
    }

    /* Vertical synthesis: Lifting stage 2.  */
    synthline = synth + synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] += (synthline[x]
                       + synthline[x + synth_width]
                       + 1) >> 1;
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x + synth_width] += (synthline[x]
                                         + synthline[x + synth_width * 2]
                                         + 1) >> 1;
        }
        synthline += (synth_width << 1);
    }
    synthline = synth + (synth_height - 1) * synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] += (synthline[x - synth_width]
                       + synthline[x - synth_width]
                       + 1) >> 1;


    /* Horizontal synthesis.  */
    synthline = synth;
    for (y = 0; y < synth_height; y++) {

        /* Lifting stage 1.  */
        for (x = 0; x < width - 1; x++) {
            synthline[2*x] -= (synthline[2*x - 1]
                             + synthline[2*x + 1]
                             + 2) >> 2;
        }
        synthline[synth_width - 2] -= (synthline[synth_width - 3]
                                     + synthline[synth_width - 1]
                                     + 2) >> 2;

        /* Lifting stage 2.  */
        for (x = 0; x < width - 1; x++) {
            synthline[2*x + 1] += (synthline[2*x]
                                 + synthline[2*x + 2]
                                 + 1) >> 1;
        }
        synthline[synth_width - 1] += (synthline[synth_width - 2]
                                     + synthline[synth_width - 2]
                                     + 1) >> 1;
        synthline += synth_width;
    }

    /* Shift away one bit that was use for additional precision and
       copy back to the coefficients buffer.  */
    synthline = synth;
    for (y = 0; y < synth_height; y++) {
        for (x = 0; x < synth_width; x++)
            data[x] = (synthline[x] + 1) >> 1;
        synthline += synth_width;
        data      += s->padded_width;
    }

STOP_TIMER("idwt53")

    av_free(synth);

    return 0;
}

/**
 * IDWT transform (9,7) for a specific subband
 *
 * @param data coefficients to transform
 * @param level level of the current transform
 * @return 0 when successful, otherwise -1 is returned
 */
static int dirac_subband_idwt_97(AVCodecContext *avctx,
                                 int *data, int level) {
    DiracContext *s = avctx->priv_data;
    int *synth, *synthline;
    int x, y;
    int width = subband_width(avctx, level);
    int height = subband_height(avctx, level);
    int synth_width = width  << 1;
    int synth_height = height << 1;

START_TIMER

    /* XXX: This should be removed, the reordering should be done in
       place.  */
    synth = av_malloc(synth_width * synth_height * sizeof(int));
    if (!synth) {
        av_log(avctx, AV_LOG_ERROR, "av_malloc() failed\n");
        return -1;
    }

    dirac_subband_idwt_reorder(avctx, data, synth, level);

    /* Deslauriers(9,7)
       First lifting step)
       Even, predict, s=5, t_{-1}=-1, t_0=9, t_1=9, t_2=-1:
         A[2*n]   -= (-A[2*n-1] + A[2*n+1] + 2) >> 2

       Second lifting step)
       Odd, update, s=4, t_{-1}=-1, t_0=9, t_1=9, t_2=-1:
         A[2*n+1] += (-A[2*n-2] + 9*A[2*n] + 9*A[2*n+2] + A[2*n+4] + 8) >> 4
    */

    /* Vertical synthesis: Lifting stage 1.  */
    synthline = synth;
    for (x = 0; x < synth_width; x++)
        synthline[x] -= (synthline[synth_width]
                       + synthline[synth_width]
                       + 2) >> 2;
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x] -= (synthline[x - synth_width]
                           + synthline[x + synth_width]
                           + 2) >> 2;
        }
        synthline += synth_width << 1;
    }
    synthline = synth + (synth_height - 2) * synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] -= (synthline[x - synth_width]
                       + synthline[x + synth_width]
                       + 2) >> 2;

    /* Vertical synthesis: Lifting stage 2.  */
    synthline = synth + synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] += (-     synthline[x -     synth_width]
                         + 9 * synthline[x -     synth_width]
                         + 9 * synthline[x +     synth_width]
                         -     synthline[x + 3 * synth_width]
                                   + 8) >> 4;
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 2; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x + synth_width] += (-     synthline[x - 2 * synth_width]
                                           + 9 * synthline[x                  ]
                                           + 9 * synthline[x + 2 * synth_width]
                                           -     synthline[x + 4 * synth_width]
                                           + 8) >> 4;
        }
        synthline += synth_width << 1;
    }
    synthline = synth + (synth_height - 1) * synth_width;
    for (x = 0; x < synth_width; x++) {
        synthline[x] += (-     synthline[x - 3 * synth_width]
                         + 9 * synthline[x -     synth_width]
                         + 9 * synthline[x -     synth_width]
                         -     synthline[x -     synth_width]
                                   + 8) >> 4;
        synthline[x - synth_width * 2] += (-     synthline[x - 5* synth_width]
                                           + 9 * synthline[x - 3* synth_width]
                                           + 9 * synthline[x -    synth_width]
                                           -     synthline[x -    synth_width]
                                           + 8) >> 4;
    }


    /* Horizontal synthesis.  */
    synthline = synth;
    for (y = 0; y < synth_height; y++) {
        /* Lifting stage 1.  */
        synthline[0] -= (synthline[1]
                       + synthline[1]
                       + 2) >> 2;
        for (x = 1; x < width - 1; x++) {
            synthline[2 * x] -= (synthline[2 * x - 1]
                               + synthline[2 * x + 1]
                               + 2) >> 2;
        }
        synthline[synth_width - 2] -= ( synthline[synth_width - 3]
                                      + synthline[synth_width - 1]
                                      + 2) >> 2;

        /* Lifting stage 2.  */
        synthline[1] += (-     synthline[0]
                         + 9 * synthline[0]
                         + 9 * synthline[2]
                         -     synthline[4]
                         + 8) >> 4;
        for (x = 1; x < width - 2; x++) {
            synthline[2*x + 1] += (-     synthline[2 * x - 2]
                                   + 9 * synthline[2 * x    ]
                                   + 9 * synthline[2 * x + 2]
                                   -     synthline[2 * x + 4]
                                   + 8) >> 4;
        }
        synthline[synth_width - 1] += (-     synthline[synth_width - 4]
                                       + 9 * synthline[synth_width - 2]
                                       + 9 * synthline[synth_width - 2]
                                       -     synthline[synth_width - 2]
                                       + 8) >> 4;
        synthline[synth_width - 3] += (-     synthline[synth_width - 6]
                                       + 9 * synthline[synth_width - 4]
                                       + 9 * synthline[synth_width - 2]
                                       -     synthline[synth_width - 2]
                                       + 8) >> 4;
        synthline += synth_width;
    }

    /* Shift away one bit that was use for additional precision and
       copy back to the coefficients buffer.  */
    synthline = synth;
    for (y = 0; y < synth_height; y++) {
        for (x = 0; x < synth_width; x++)
            data[x] = (synthline[x] + 1) >> 1;
        synthline += synth_width;
        data      += s->padded_width;
    }

STOP_TIMER("idwt97")

    av_free(synth);

    return 0;
}


static int dirac_idwt(AVCodecContext *avctx, int *coeffs) {
    int level;
    int wavelet_idx;
    DiracContext *s = avctx->priv_data;

    /* XXX: The spec starts with level 0.  Most likely a bug in the
       spec.  */
    for (level = 1; level <= s->frame_decoding.wavelet_depth; level++) {
        if (s->refs == 0)
            wavelet_idx = s->frame_decoding.wavelet_idx_intra;
        else
            wavelet_idx = s->frame_decoding.wavelet_idx_inter;

        switch(wavelet_idx) {
        case 0:
            dirac_subband_idwt_97(avctx, coeffs, level);
            break;
        case 1:
            dirac_subband_idwt_53(avctx, coeffs, level);
            break;
        default:
            av_log(avctx, AV_LOG_INFO, "unknown IDWT index: %d\n",
                   wavelet_idx);
        }
    }

    return 0;
}

static int reference_frame_idx(AVCodecContext *avctx, int framenr) {
    DiracContext *s = avctx->priv_data;
    int i;

    for (i = 0; i < s->refcnt; i++) {
        AVFrame *f = &s->refframes[i];
        if (f->display_picture_number == framenr)
            return i;
    }

    return -1;
}

static void interpolate_frame_halfpel(AVFrame *refframe, int width, int height,
                                      uint8_t *pixels, int comp) {
    uint8_t *lineout;
    uint8_t *refdata;
    uint8_t *linein;
    int outwidth = width * 2;
    int x, y;
    const int t[5] = { 167, -56, 25, -11, 3 };

    refdata = refframe->data[comp];

    /* Copy even lines.  */
    lineout = pixels;
    linein = refdata;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++)
            lineout[x * 2] = linein[x];

        /* Skip one line, we are copying to even lines.  */
        lineout += outwidth * 2;

        linein += refframe->linesize[comp];
    }

    /* Interpolate odd lines.  */
    lineout = pixels + outwidth;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            int i;
            int val = 0;

            for (i = 0; i <= 4; i++) {
                int ypos;
                ypos = ((y * 2 - 1) >> 1) - i;

                /* XXX: Instead of clipping, it would be better to
                   break up the loop and handle the last lines as a
                   special case.  */
                val += t[i] * refdata[av_clip(ypos, 0, height - 1)
                                     * refframe->linesize[comp] + x];
                ypos = ((y * 2 + 1) >> 1) + i;
                val += t[i] * refdata[av_clip(ypos, 0, height - 1)
                                     * refframe->linesize[comp] + x];
            }

            val += 128;
            val >>= 8;
            lineout[x * 2] = av_clip_uint8(val);
        }

        /* Skip one line, we are interpolating to odd lines.  */
        lineout += outwidth * 2;
    }

    /* At this place the even rows of pixels are in place, no copying
       is required..  */

    /* Interpolate the odd rows of pixels.  */
    lineout = pixels;
    linein  = pixels;
    for (y = 0; y < height * 2; y++) {
        for (x = 1; x < width; x++) {
            int i;
            int val = 0;

            for (i = 0; i <= 4; i++) {
                int xpos;
                xpos = (((x << 1) - 1) >> 1) - i;
                /* The data that is called `ref2' in the specification
                   is stored in the even rows.  */
                xpos <<= 1;
                val += t[i] * linein[av_clip(xpos, 0, outwidth - 1)];

                xpos = (((x << 1) + 1) >> 1) + i;
                /* The data that is called `ref2' in the specification
                   is stored in the even rows.  */
                xpos <<= 1;
                val += t[i] * linein[av_clip(xpos, 0, outwidth - 1)];
            }

            val += 128;
            val >>= 8;
            lineout[x * 2 + 1] = av_clip_uint8(val);
        }
        lineout += outwidth;
        linein  += outwidth;
    }
}

static inline int get_halfpel(uint8_t *refframe, int width, int height,
                              int x, int y) {
    int xpos;
    int ypos;

    xpos = av_clip(x, 0, width  - 1);
    ypos = av_clip(y, 0, height - 1);

    return refframe[xpos + ypos * width];
}

static int upconvert(AVCodecContext *avctx, uint8_t *refframe,
                     int width, int height, int x, int y, int comp) {
    DiracContext *s = avctx->priv_data;
    int hx, hy;
    int rx, ry;
    int w00, w01, w10, w11;
    int val = 0;

    if (s->frame_decoding.mv_precision == 0
        || s->frame_decoding.mv_precision == 1)
        return get_halfpel(refframe, width, height, x, y);

    hx = x >> (s->frame_decoding.mv_precision - 1);
    hy = y >> (s->frame_decoding.mv_precision - 1);
    rx = x - (hx << (s->frame_decoding.mv_precision - 1));
    ry = y - (hy << (s->frame_decoding.mv_precision - 1));

    /* Calculate weights.  */
    w00 = ((1 << (s->frame_decoding.mv_precision - 1)) - ry)
        * ((1 << (s->frame_decoding.mv_precision - 1)) - rx);
    w01 = ((1 << (s->frame_decoding.mv_precision - 1)) - ry) * rx;
    w10 = ((1 << (s->frame_decoding.mv_precision - 1)) - rx) * ry;
    w11 = ry * rx;

    val += w00 * get_halfpel(refframe, width, height, hx    , hy    );
    val += w01 * get_halfpel(refframe, width, height, hx + 1, hy    );
    val += w10 * get_halfpel(refframe, width, height, hx    , hy + 1);
    val += w11 * get_halfpel(refframe, width, height, hx + 1, hy + 1);
    val += 1 << (s->frame_decoding.mv_precision - 1);

    return val >> s->frame_decoding.mv_precision;
}

static int motion_comp_blockpred(AVCodecContext *avctx, uint8_t *refframe,
                                 int ref, struct dirac_blockmotion *currblock,
                                 int x, int y, int width, int height,
                                 int comp) {
    DiracContext *s = avctx->priv_data;
    int vect[2];
    int px, py;

    if (!currblock->use_global) {
        /* XXX */
        if (ref == 0) {
            vect[0] = currblock->ref1[0];
            vect[1] = currblock->ref1[1];
        } else {
            vect[0] = currblock->ref2[0];
            vect[1] = currblock->ref2[1];
        }
    } else {
        dprintf(avctx, "global motion compensation has not been implemented yet\n");
        /* XXX */
        vect[0] = 0;
        vect[1] = 0;
    }

    if (comp != 0) {
        if (s->chroma_hratio)
            vect[0] >>= 1;
        if (s->chroma_vratio)
            vect[1] >>= 1;
    }

    if (s->frame_decoding.mv_precision > 0) {
        px = (x << s->frame_decoding.mv_precision) + vect[0];
        py = (y << s->frame_decoding.mv_precision) + vect[1];
    } else {
        px = (x + vect[0]) << 1;
        py = (y + vect[1]) << 1;
    }

    /* Upconversion.  */
    return upconvert(avctx, refframe, width, height, px, py, comp);
}

static inline int spatial_wt(int i, int x, int bsep, int blen,
                             int offset, int blocks) {
    int pos = x - (i * bsep - offset);
    int max;

    max = 2 * (blen - bsep);
    if (i == 0 && pos < (blen >> 1))
        return max;
    else if (i == blocks - 1 && pos >= (blen >> 1))
        return max;
    else
        return av_clip(blen - 2*FFABS(pos - (blen - 1) / 2), 0, max);
}

static int motion_comp(AVCodecContext *avctx, int x, int y,
                       AVFrame *ref1, AVFrame *ref2, int *coeffs, int comp) {
    DiracContext *s = avctx->priv_data;
    int width, height;
    int xblen, yblen;
    int xbsep, ybsep;
    int xoffset, yoffset;
    int p = 0, val = 0;
    int i, j;
    int hbits, vbits;
    int total_wt_bits;

    int istart, istop;
    int jstart, jstop;

    if (comp == 0) {
        width  = s->sequence.luma_width;
        height = s->sequence.luma_height;
        xblen  = s->frame_decoding.luma_xblen;
        yblen  = s->frame_decoding.luma_yblen;
        xbsep  = s->frame_decoding.luma_xbsep;
        ybsep  = s->frame_decoding.luma_ybsep;
    } else {
        width  = s->sequence.chroma_width;
        height = s->sequence.chroma_height;
        xblen  = s->frame_decoding.chroma_xblen;
        yblen  = s->frame_decoding.chroma_yblen;
        xbsep  = s->frame_decoding.chroma_xbsep;
        ybsep  = s->frame_decoding.chroma_ybsep;
    }

    xoffset = (xblen - xbsep) / 2;
    yoffset = (yblen - ybsep) / 2;

    hbits = av_log2(xoffset) + 2;
    vbits = av_log2(yoffset) + 2;
    total_wt_bits = hbits + vbits
                    + s->frame_decoding.picture_weight_precision;

    /* XXX: Check if these values are right.  */
    istart = FFMAX(0,           (x - xoffset) / xbsep - 1);
    jstart = FFMAX(0,           (y - yoffset) / ybsep - 1);
    istop  = FFMIN(s->blwidth,  (x + xoffset) / xbsep + 1);
    jstop  = FFMIN(s->blheight, (y + yoffset) / ybsep + 1);

    for (j = jstart; j < jstop; j++)
        for (i = istart; i < istop; i++) {
            struct dirac_blockmotion *currblock;
            int xstart = FFMAX(0, i * xbsep - xoffset);
            int ystart = FFMAX(0, j * ybsep - yoffset);
            int xstop  = FFMIN(xstart + xblen, width);
            int ystop  = FFMIN(ystart + yblen, height);

            if (x < xstart || x > xstop)
                continue;
            if (y < ystart || y > ystop)
                continue;

            currblock = &s->blmotion[i + j * s->blwidth];

            if (currblock->use_ref[0] == 0 && currblock->use_ref[1] == 0) {
                /* Intra */
                val  =  currblock->dc[comp];
                val  <<= s->frame_decoding.picture_weight_precision;
            } else if (currblock->use_ref[1] == 0) {
                /* Reference frame 1 only.  */
                val  =  motion_comp_blockpred(avctx, s->ref1data, 0,
                                              currblock, x, y, s->ref1width,
                                              s->ref1height, comp);
                val  *= (s->frame_decoding.picture_weight_ref1
                        + s->frame_decoding.picture_weight_ref2);
            } else if (currblock->use_ref[0] == 0) {
                /* Reference frame 2 only.  */
                val  =  motion_comp_blockpred(avctx, s->ref2data, 1,
                                              currblock, x, y, s->ref2width,
                                              s->ref2height, comp);
                val  *= (s->frame_decoding.picture_weight_ref1
                        + s->frame_decoding.picture_weight_ref2);
            } else {
                /* Reference frame 1 and 2.  */
                int val1, val2;
                val1 =  motion_comp_blockpred(avctx, s->ref1data, 0,
                                              currblock, x, y, s->ref1width,
                                              s->ref1height, comp);
                val1 *= s->frame_decoding.picture_weight_ref1;
                val2 =  motion_comp_blockpred(avctx, s->ref2data, 1,
                                              currblock, x, y, s->ref2width,
                                              s->ref2height, comp);
                val2 *= s->frame_decoding.picture_weight_ref2;
                val = val1 + val2;
            }

            p += val
                * spatial_wt(i, x, xbsep, xblen, xoffset, s->blwidth)
                * spatial_wt(j, y, ybsep, yblen, yoffset, s->blheight);
        }

    p = (p + (1 << (total_wt_bits - 1))) >> total_wt_bits;
    return p;
}

static int dirac_motion_compensation(AVCodecContext *avctx, int *coeffs,
                                     int comp) {
    DiracContext *s = avctx->priv_data;
    int width, height;
    int x, y;
    int refidx1, refidx2 = 0;
    AVFrame *ref1 = 0, *ref2 = 0;

    if (comp == 0) {
        width  = s->sequence.luma_width;
        height = s->sequence.luma_height;
    } else {
        width  = s->sequence.chroma_width;
        height = s->sequence.chroma_height;

    }

    refidx1 = reference_frame_idx(avctx, s->ref1);
    ref1 = &s->refframes[refidx1];
    s->ref1width = width << 1;
    s->ref1height = height << 1;
    s->ref1data = av_malloc(s->ref1width * s->ref1height);
    if (!s->ref1data) {
        av_log(avctx, AV_LOG_ERROR, "av_malloc() failed\n");
        return -1;
    }
    interpolate_frame_halfpel(ref1, width, height, s->ref1data, comp);

    /* XXX: somehow merge with the code above.  */
    if (s->refs == 2) {
        refidx2 = reference_frame_idx(avctx, s->ref2);
        ref2 = &s->refframes[refidx2];

        s->ref2width = width << 1;
        s->ref2height = height << 1;
        s->ref2data = av_malloc(s->ref2width * s->ref2height);
        if (!s->ref2data) {
            av_log(avctx, AV_LOG_ERROR, "av_malloc() failed\n");
            return -1;
        }
        interpolate_frame_halfpel(ref2, width, height, s->ref2data, comp);
    }
    else
        s->ref2data = NULL;

    /* XXX: It might be more efficient to loop over the blocks,
       because for every blocks all the parameters are the same.  For
       now this code just copies the behavior as described in the
       specification.  */
    for (y = 0; y < height; y++)
        for (x = 0; x < width; x++) {
            coeffs[y * s->padded_width + x] += motion_comp(avctx, x, y,
                                                           ref1, ref2,
                                                           coeffs, comp);
        }

    av_free(s->ref1data);
    av_free(s->ref2data);

    return 0;
}

/**
 * Decode an intra frame.
 *
 * @return 0 when successful, otherwise -1 is returned
 */
static int dirac_decode_frame(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    int comp;
    int x,y;

START_TIMER

    for (comp = 0; comp < 3; comp++) {
        int *coeffs;
        uint8_t *frame = s->picture.data[comp];
        int width, height;

        if (comp == 0) {
            width = s->sequence.luma_width;
            height = s->sequence.luma_height;
            s->padded_width = s->padded_luma_width;
            s->padded_height = s->padded_luma_height;
        } else {
            width = s->sequence.chroma_width;
            height = s->sequence.chroma_height;
            s->padded_width = s->padded_chroma_width;
            s->padded_height = s->padded_chroma_height;
        }

        coeffs = av_malloc(s->padded_width * s->padded_height * sizeof(int));
        if (! coeffs) {
            av_log(avctx, AV_LOG_ERROR, "av_malloc() failed\n");
            return -1;
        }

        memset(coeffs, 0, s->padded_width * s->padded_height * sizeof(int));

        if (!s->zero_res)
            decode_component(avctx, coeffs);

        dirac_idwt(avctx, coeffs);

        if (s->refs) {
            if (dirac_motion_compensation(avctx, coeffs, comp))
                return -1;
        }

        /* Copy the decoded coefficients into the frame.  */
        for (x = 0; x < width; x++)
            for (y = 0; y < height; y++)
                frame[x + y * s->picture.linesize[comp]]
                    = av_clip_uint8(coeffs[x + y * s->padded_width]);
        av_free(coeffs);
    }

STOP_TIMER("dirac_frame_decode");

    return 0;
}

/**
 * Parse a frame and setup DiracContext to decode it
 *
 * @return 0 when successful, otherwise -1 is returned
 */
static int parse_frame(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    int retire;
    int filter;
    int i;
    GetBitContext *gb = s->gb;

    /* Setup decoding parameter defaults for this frame.  */
    memcpy(&s->frame_decoding, &s->decoding, sizeof(s->frame_decoding));

    s->picture.pict_type = FF_I_TYPE;
    s->picture.key_frame = 1;

    s->picnum = get_bits_long(gb, 32);

    if (s->refs) {
        s->ref1 = dirac_get_se_golomb(gb) + s->picnum;
        if (s->refs == 2)
            s->ref2 = dirac_get_se_golomb(gb) + s->picnum;
    }

    /* Retire the reference frames that are not used anymore.  */
    retire = dirac_get_ue_golomb(gb);
    s->retirecnt = retire;
    for (i = 0; i < retire; i++) {
        uint32_t retire_num;

        retire_num = dirac_get_se_golomb(gb) + s->picnum;
        s->retireframe[i] = retire_num;
    }

    if (s->refs) {
        align_get_bits(gb);
        dirac_unpack_prediction_parameters(avctx);
        align_get_bits(gb);
        dirac_unpack_prediction_data(avctx);
    }

    align_get_bits(gb);

    /* Wavelet transform data.  */
    if (s->refs == 0)
        s->zero_res = 0;
    else
        s->zero_res = get_bits(gb, 1);

    if (!s->zero_res) {
        /* Override wavelet transform parameters.  */
        if (get_bits(gb, 1)) {
            dprintf(avctx, "Non default filter\n");
            filter = dirac_get_ue_golomb(gb); /* XXX */
        } else {
            dprintf(avctx, "Default filter\n");
            filter = s->frame_decoding.wavelet_idx_intra;
        }

        /* Overrid wavelet depth.  */
        if (get_bits(gb, 1)) {
            dprintf(avctx, "Non default depth\n");
            s->frame_decoding.wavelet_depth = dirac_get_ue_golomb(gb);
        }
        dprintf(avctx, "Depth: %d\n", s->frame_decoding.wavelet_depth);

        /* Spatial partitioning.  */
        if (get_bits(gb, 1)) {
            int idx;

            dprintf(avctx, "Spatial partitioning\n");

            /* Override the default partitioning.  */
            if (get_bits(gb, 1)) {
                for (i = 0; i <= s->frame_decoding.wavelet_depth; i++) {
                    s->codeblocksh[i] = dirac_get_ue_golomb(gb);
                    s->codeblocksv[i] = dirac_get_ue_golomb(gb);
                }

                dprintf(avctx, "Non-default partitioning\n");

            } else {
                /* Set defaults for the codeblocks.  */
                for (i = 0; i <= s->frame_decoding.wavelet_depth; i++) {
                    if (s->refs == 0) {
                        s->codeblocksh[i] = i <= 2 ? 1 : 4;
                        s->codeblocksv[i] = i <= 2 ? 1 : 3;
                    } else {
                        if (i <= 1) {
                            s->codeblocksh[i] = 1;
                            s->codeblocksv[i] = 1;
                        } else if (i == 2) {
                            s->codeblocksh[i] = 8;
                            s->codeblocksv[i] = 6;
                        } else {
                            s->codeblocksh[i] = 12;
                            s->codeblocksv[i] = 8;
                        }
                    }
                }
            }

            idx = dirac_get_ue_golomb(gb);
            dprintf(avctx, "Codeblock mode idx: %d\n", idx);
            /* XXX: Here 0, so single quant.  */
        }
    }

#define CALC_PADDING(size, depth) \
         (((size + (1 << depth) - 1) >> depth) << depth)

    /* Round up to a multiple of 2^depth.  */
    s->padded_luma_width    = CALC_PADDING(s->sequence.luma_width,
                                           s->frame_decoding.wavelet_depth);
    s->padded_luma_height   = CALC_PADDING(s->sequence.luma_height,
                                           s->frame_decoding.wavelet_depth);
    s->padded_chroma_width  = CALC_PADDING(s->sequence.chroma_width,
                                           s->frame_decoding.wavelet_depth);
    s->padded_chroma_height = CALC_PADDING(s->sequence.chroma_height,
                                           s->frame_decoding.wavelet_depth);

    return 0;
}


static int decode_frame(AVCodecContext *avctx, void *data, int *data_size,
                        uint8_t *buf, int buf_size){
    DiracContext *s = avctx->priv_data;
    GetBitContext gb;
    AVFrame *picture = data;
    int i;
    int parse_code = buf[4];

    dprintf(avctx, "Decoding frame: size=%d head=%c%c%c%c parse=%02x\n",
            buf_size, buf[0], buf[1], buf[2], buf[3], buf[4]);

    init_get_bits(&gb, &buf[13], (buf_size - 13) * 8);
    s->gb = &gb;

    if (parse_code ==  pc_access_unit_header) {
        parse_access_unit_header(avctx);

        /* Dump the header.  */
#if 1
        dump_sequence_parameters(avctx);
        dump_source_parameters(avctx);
#endif

        return 0;
    }

    /* If this is not a picture, return.  */
    if ((parse_code & 0x08) != 0x08)
        return 0;

    s->refs = parse_code & 0x03;

    parse_frame(avctx);

    avctx->pix_fmt = PIX_FMT_YUVJ420P; /* XXX */

    if (avcodec_check_dimensions(avctx, s->sequence.luma_width,
                                 s->sequence.luma_height)) {
        av_log(avctx, AV_LOG_ERROR,
               "avcodec_check_dimensions() failed\n");
        return -1;
    }

    avcodec_set_dimensions(avctx, s->sequence.luma_width,
                           s->sequence.luma_height);

    if (s->picture.data[0] != NULL)
        avctx->release_buffer(avctx, &s->picture);

    s->picture.reference = (parse_code & 0x04) == 0x04;

    if (avctx->get_buffer(avctx, &s->picture) < 0) {
        av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
        return -1;
    }

#if 1
    for (i = 0; i < s->refcnt; i++)
        dprintf(avctx, "Reference frame #%d\n",
                s->refframes[i].display_picture_number);

    if (s->refs)
        dprintf(avctx, "First reference frame: #%d\n", s->ref1);
    if (s->refs == 2)
        dprintf(avctx, "Second reference frame: #%d\n", s->ref2);
#endif

    if (dirac_decode_frame(avctx))
        return -1;

    s->picture.display_picture_number = s->picnum;

    if (s->picture.reference
        || s->picture.display_picture_number != avctx->frame_number) {
        if (s->refcnt + 1 == REFFRAME_CNT) {
            av_log(avctx, AV_LOG_ERROR, "reference picture buffer overrun\n");
            return -1;
        }

        s->refframes[s->refcnt++] = s->picture;
    }

    /* Retire frames that were reordered and displayed if they are no
       reference frames either.  */
    for (i = 0; i < s->refcnt; i++) {
        AVFrame *f = &s->refframes[i];

        if (f->reference == 0
            && f->display_picture_number < avctx->frame_number) {
            s->retireframe[s->retirecnt++] = f->display_picture_number;
        }
    }

    for (i = 0; i < s->retirecnt; i++) {
        AVFrame *f;
        int idx, j;

        idx = reference_frame_idx(avctx, s->retireframe[i]);
        if (idx == -1) {
            av_log(avctx, AV_LOG_WARNING, "frame to retire #%d not found\n",
                   s->retireframe[i]);
            continue;
        }

        f = &s->refframes[idx];
        /* Do not retire frames that were not displayed yet.  */
        if (f->display_picture_number >= avctx->frame_number) {
            f->reference = 0;
            continue;
        }

        if (f->data[0] != NULL)
            avctx->release_buffer(avctx, f);
        s->refcnt--;

        for (j = idx; j < idx + s->refcnt; j++) {
            s->refframes[j] = s->refframes[j + 1];
        }
    }

    if (s->picture.display_picture_number > avctx->frame_number) {
        int idx;

        if (!s->picture.reference) {
            /* This picture needs to be shown at a later time.  */
            s->refframes[s->refcnt++] = s->picture;
        }

        idx = reference_frame_idx(avctx, avctx->frame_number);
        if (idx == -1) {
            /* The frame is not yet decoded.  */
            *data_size = 0;
        } else {
            *data_size = sizeof(AVFrame);
            *picture = s->refframes[idx];
        }
    } else {
        /* The right frame at the right time :-) */
        *data_size = sizeof(AVFrame);
        *picture = s->picture;
    }

    if (s->picture.reference
        || s->picture.display_picture_number < avctx->frame_number)
        avcodec_get_frame_defaults(&s->picture);

    return buf_size;
}


AVCodec dirac_decoder = {
    "dirac",
    CODEC_TYPE_VIDEO,
    CODEC_ID_DIRAC,
    sizeof(DiracContext),
    decode_init,
    NULL,
    decode_end,
    decode_frame,
    0,
    NULL
};

/* #ifdef CONFIG_ENCODERS */
/* AVCodec dirac_encoder = { */
/*     "dirac", */
/*     CODEC_TYPE_VIDEO, */
/*     CODEC_ID_DIRAC, */
/*     sizeof(DiracContext), */
/*     NULL, */
/*     NULL, */
/*     NULL, */
/* }; */
/* #endif */
