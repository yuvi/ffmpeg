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
#include "mpeg12data.h"

typedef enum {
    TRANSFER_FUNC_TV,
    TRANSFER_FUNC_EXTENDED_GAMUT,
    TRANSFER_FUNC_LINEAR,
    TRANSFER_FUNC_DCI_GAMMA
} transfer_func_t;

#define DIRAC_SIGN(x) ((x) > 0 ? 2 : ((x) < 0 ? 1 : 0))
#define DIRAC_PARSE_INFO_PREFIX 0x42424344

struct source_parameters
{
    /* Interlacing.  */
    char interlaced;                     ///< flag for interlacing
    char top_field_first;
    char sequential_fields;

    AVRational frame_rate;             ///< frame rate

    AVRational aspect_ratio;           ///< aspect ratio

    /* Clean area.  */
    uint16_t clean_width;
    uint16_t clean_height;
    uint16_t clean_left_offset;
    uint16_t clean_right_offset;

    /* Luma and chroma offsets.  */
    uint16_t luma_offset;
    uint16_t luma_excursion;
    uint16_t chroma_offset;
    uint16_t chroma_excursion;

    uint16_t color_spec;
    uint16_t color_primaries; /* XXX: ??? */

    float k_r;
    float k_b; /* XXX: ??? */

    transfer_func_t transfer_function;
};

struct sequence_parameters
{
    /* Information about the frames.  */
    int luma_width;                    ///< the luma component width
    int luma_height;                   ///< the luma component height
    /** Choma format: 0: 4:4:4, 1: 4:2:2, 2: 4:2:0 */
    int chroma_format;
    char video_depth;                  ///< depth in bits

    /* Calculated:  */
    int chroma_width;                  ///< the chroma component width
    int chroma_height;                 ///< the chroma component height
};

struct decoding_parameters
{
    uint8_t wavelet_depth;                 ///< depth of the IDWT
    uint8_t wavelet_idx_intra;             ///< wavelet transform for intra frames
    uint8_t wavelet_idx_inter;             ///< wavelet transform for inter frames

    uint8_t luma_xbsep;
    uint8_t luma_xblen;
    uint8_t luma_ybsep;
    uint8_t luma_yblen;

    uint8_t mv_precision;

    uint16_t picture_weight_ref1;
    uint16_t picture_weight_ref2;
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
    uint8_t chroma_xbsep;
    uint8_t chroma_xblen;
    uint8_t chroma_ybsep;
    uint8_t chroma_yblen;
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

static const AVRational preset_aspect_ratios[3] =
{
    {1, 1}, {10, 11}, {12, 11}
};

static const uint8_t preset_luma_offset[3] = { 0, 16, 64 };
static const uint16_t preset_luma_excursion[3] = { 255, 235, 876 };
static const uint16_t preset_chroma_offset[3] = { 128, 128, 512 };
static const uint16_t preset_chroma_excursion[3] = { 255, 224, 896 };

static const uint8_t preset_primaries[4] = { 0, 1, 2, 3 };
static const uint8_t preset_matrix[4] = {0, 1, 1, 2 };
static const transfer_func_t preset_transfer_func[3] =
{
    TRANSFER_FUNC_TV, TRANSFER_FUNC_TV, TRANSFER_FUNC_DCI_GAMMA
};
static const float preset_kr[3] = { 0.2126, 0.299, 0 /* XXX */ };
static const float preset_kb[3] = {0.0722, 0.114, 0 /* XXX */ };

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
    uint8_t *halfpel[3];
};

typedef struct DiracContext {
    unsigned int profile;
    unsigned int level;

    AVCodecContext *avctx;
    GetBitContext gb;

    PutBitContext pb;
    int last_parse_code;

    AVFrame picture;

    uint32_t picnum;
    int refcnt;
    struct reference_frame refframes[REFFRAME_CNT]; /* XXX */

    int retirecnt;
    uint32_t retireframe[REFFRAME_CNT];

    struct source_parameters source;
    struct sequence_parameters sequence;
    struct decoding_parameters decoding;

    struct decoding_parameters frame_decoding;

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

    uint8_t *refdata[2];
    int refwidth;
    int refheight;

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

static int encode_init(AVCodecContext *avctx){
    av_log_set_level(AV_LOG_DEBUG);
    return 0;
}

static int encode_end(AVCodecContext *avctx)
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
static void parse_sequence_parameters(DiracContext *s) {
    GetBitContext *gb = &s->gb;

    /* Override the luma dimensions.  */
    if (get_bits1(gb)) {
        s->sequence.luma_width  = svq3_get_ue_golomb(gb);
        s->sequence.luma_height = svq3_get_ue_golomb(gb);
    }

    /* Override the chroma format.  */
    if (get_bits1(gb))
        s->sequence.chroma_format = svq3_get_ue_golomb(gb);

    /* Calculate the chroma dimensions.  */
    s->chroma_hshift = (s->sequence.chroma_format == 0 ? 0 : 1);
    s->chroma_vshift = (s->sequence.chroma_format <= 1 ? 0 : 1);
    s->sequence.chroma_width  = s->sequence.luma_width  >> s->chroma_hshift;
    s->sequence.chroma_height = s->sequence.luma_height >> s->chroma_vshift;

    /* Override the video depth.  */
    if (get_bits1(gb))
        s->sequence.video_depth = svq3_get_ue_golomb(gb);
}

/**
 * Parse the source parameters in the access unit header
 */
static void parse_source_parameters(DiracContext *s) {
    GetBitContext *gb = &s->gb;

    /* Access Unit Source parameters.  */
    if (get_bits1(gb)) {
        /* Interlace.  */
        s->source.interlaced = get_bits1(gb);

        if (s->source.interlaced) {
            if (get_bits1(gb))
                s->source.top_field_first = get_bits1(gb);

            if (get_bits1(gb))
                s->source.sequential_fields = get_bits1(gb);
        }
    }

    /* Framerate.  */
    if (get_bits1(gb)) {
        int idx = svq3_get_ue_golomb(gb);
        if (! idx) {
            s->source.frame_rate.num = svq3_get_ue_golomb(gb);
            s->source.frame_rate.den = svq3_get_ue_golomb(gb);
        } else {
            /* Use a pre-set framerate.  */
            s->source.frame_rate = ff_frame_rate_tab[idx];
        }
    }

    /* Override aspect ratio.  */
    if (get_bits1(gb)) {
        int idx = svq3_get_ue_golomb(gb);
        if (! idx) {
            s->source.aspect_ratio.num = svq3_get_ue_golomb(gb);
            s->source.aspect_ratio.den = svq3_get_ue_golomb(gb);
        } else {
            /* Use a pre-set aspect ratio.  */
            s->source.aspect_ratio = preset_aspect_ratios[idx - 1];
        }
    }

    /* Override clean area.  */
    if (get_bits1(gb)) {
        s->source.clean_width        = svq3_get_ue_golomb(gb);
        s->source.clean_height       = svq3_get_ue_golomb(gb);
        s->source.clean_left_offset  = svq3_get_ue_golomb(gb);
        s->source.clean_right_offset = svq3_get_ue_golomb(gb);
    }

    /* Override signal range.  */
    if (get_bits1(gb)) {
        int idx = svq3_get_ue_golomb(gb);
        if (! idx) {
            s->source.luma_offset      = svq3_get_ue_golomb(gb);
            s->source.luma_excursion   = svq3_get_ue_golomb(gb);
            s->source.chroma_offset    = svq3_get_ue_golomb(gb);
            s->source.chroma_excursion = svq3_get_ue_golomb(gb);
        } else {
            /* Use a pre-set signal range.  */
            s->source.luma_offset = preset_luma_offset[idx - 1];
            s->source.luma_excursion = preset_luma_excursion[idx - 1];
            s->source.chroma_offset = preset_chroma_offset[idx - 1];
            s->source.chroma_excursion = preset_chroma_excursion[idx - 1];
        }
    }

    /* Color spec.  */
    if (get_bits1(gb)) {
        int idx = svq3_get_ue_golomb(gb);

        s->source.color_primaries = preset_primaries[idx];
        s->source.k_r = preset_kr[preset_matrix[idx]];
        s->source.k_b = preset_kb[preset_matrix[idx]];
        s->source.transfer_function = preset_transfer_func[idx];

        /* XXX: color_spec?  */

        if (! idx) {
            /* Color primaries.  */
            if (get_bits1(gb)) {
                int primaries_idx = svq3_get_ue_golomb(gb);
                s->source.color_primaries = preset_primaries[primaries_idx];
            }

            /* Override matrix.  */
            if (get_bits1(gb)) {
                int matrix_idx = svq3_get_ue_golomb(gb);

                s->source.k_r = preset_kr[preset_matrix[matrix_idx]];
                s->source.k_b = preset_kb[preset_matrix[matrix_idx]];
            }

            /* Transfer function.  */
            if (get_bits1(gb)) {
                int tf_idx = svq3_get_ue_golomb(gb);
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
static int parse_access_unit_header(DiracContext *s) {
    GetBitContext *gb = &s->gb;
    unsigned int version_major;
    unsigned int version_minor;
    unsigned int video_format;

    /* Parse parameters.  */
    version_major = svq3_get_ue_golomb(gb);
    version_minor = svq3_get_ue_golomb(gb);
    /* XXX: Don't check the version yet, existing encoders do not yet
       set this to a sane value (0.6 at the moment).  */

    /* XXX: Not yet documented in the spec.  This is actually the main
       thing that is missing.  */
    s->profile = svq3_get_ue_golomb(gb);
    s->level = svq3_get_ue_golomb(gb);
    dprintf(s->avctx, "Access unit header: Version %d.%d\n",
            version_major, version_minor);
    dprintf(s->avctx, "Profile: %d, Level: %d\n", s->profile, s->level);

    video_format = svq3_get_ue_golomb(gb);
    dprintf(s->avctx, "Video format: %d\n", video_format);

    /* Fill in defaults for the sequence parameters.  */
    memcpy(&s->sequence, &sequence_parameters_defaults[video_format],
           sizeof(s->sequence));
    /* Override the defaults.  */
    parse_sequence_parameters(s);

    /* Fill in defaults for the source parameters.  */
    memcpy(&s->source, &source_parameters_defaults[video_format],
           sizeof(s->source));
    /* Override the defaults.  */
    parse_source_parameters(s);

    /* Fill in defaults for the decoding parameters.  */
    memcpy(&s->decoding, &decoding_parameters_defaults[video_format],
           sizeof(s->decoding));

    return 0;
}

static struct dirac_arith_context_set context_set_split =
    {
        .follow = { ARITH_CONTEXT_SB_F1, ARITH_CONTEXT_SB_F2,
                    ARITH_CONTEXT_SB_F2, ARITH_CONTEXT_SB_F2,
                    ARITH_CONTEXT_SB_F2, ARITH_CONTEXT_SB_F2 },
        .data = ARITH_CONTEXT_SB_DATA
    };

static struct dirac_arith_context_set context_set_mv =
    {
        .follow = { ARITH_CONTEXT_VECTOR_F1, ARITH_CONTEXT_VECTOR_F2,
                    ARITH_CONTEXT_VECTOR_F3, ARITH_CONTEXT_VECTOR_F4,
                    ARITH_CONTEXT_VECTOR_F5, ARITH_CONTEXT_VECTOR_F5 },
        .data = ARITH_CONTEXT_VECTOR_DATA,
        .sign = ARITH_CONTEXT_VECTOR_SIGN
    };
static struct dirac_arith_context_set context_set_dc =
    {
        .follow = { ARITH_CONTEXT_DC_F1, ARITH_CONTEXT_DC_F2,
                    ARITH_CONTEXT_DC_F2, ARITH_CONTEXT_DC_F2,
                    ARITH_CONTEXT_DC_F2, ARITH_CONTEXT_DC_F2 },
        .data = ARITH_CONTEXT_DC_DATA,
        .sign = ARITH_CONTEXT_DC_SIGN
    };

static struct dirac_arith_context_set context_sets_waveletcoeff[12] = {
    {
        /* Parent = 0, Zero neighbourhood, sign predict 0 */
        .follow = { ARITH_CONTEXT_ZPZN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_ZERO,
    }, {
        /* Parent = 0, Zero neighbourhood, sign predict < 0 */
        .follow = { ARITH_CONTEXT_ZPZN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_NEG
    }, {
        /* Parent = 0, Zero neighbourhood, sign predict > 0 */
        .follow = { ARITH_CONTEXT_ZPZN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_POS
    },

    {
        /* Parent = 0, No Zero neighbourhood, sign predict  0 */
        .follow = { ARITH_CONTEXT_ZPNN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_ZERO
    }, {
        /* Parent = 0, No Zero neighbourhood, sign predict < 0 */
        .follow = { ARITH_CONTEXT_ZPNN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_NEG
    }, {
        /* Parent = 0, No Zero neighbourhood, sign predict > 0 */
        .follow = { ARITH_CONTEXT_ZPNN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_POS
    },

    {
        /* Parent != 0, Zero neighbourhood, sign predict 0 */
        .follow = { ARITH_CONTEXT_NPZN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_ZERO
    }, {
        /* Parent != 0, Zero neighbourhood, sign predict < 0 */
        .follow = { ARITH_CONTEXT_NPZN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_NEG
    }, {
        /* Parent != 0, Zero neighbourhood, sign predict > 0 */
        .follow = { ARITH_CONTEXT_NPZN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_POS
    },


    {
        /* Parent != 0, No Zero neighbourhood, sign predict 0 */
        .follow = { ARITH_CONTEXT_NPNN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_ZERO
    }, {
        /* Parent != 0, No Zero neighbourhood, sign predict < 0 */
        .follow = { ARITH_CONTEXT_NPNN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_NEG
    }, {
        /* Parent != 0, No Zero neighbourhood, sign predict > 0 */
        .follow = { ARITH_CONTEXT_NPNN_F1, ARITH_CONTEXT_NP_F2,
                    ARITH_CONTEXT_NP_F3, ARITH_CONTEXT_NP_F4,
                    ARITH_CONTEXT_NP_F5, ARITH_CONTEXT_NP_F6 },
        .data = ARITH_CONTEXT_COEFF_DATA,
        .sign = ARITH_CONTEXT_SIGN_POS
    }
};

/**
 * Calculate the width of a subband on a given level
 *
 * @param level subband level
 * @return subband width
 */
static int inline subband_width(DiracContext *s, int level) {
    if (level == 0)
        return s->padded_width >> s->frame_decoding.wavelet_depth;
    return s->padded_width >> (s->frame_decoding.wavelet_depth - level + 1);
}

/**
 * Calculate the height of a subband on a given level
 *
 * @param level subband level
 * @return height of the subband
 */
static int inline subband_height(DiracContext *s, int level) {
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

static int inline coeff_quant_offset(DiracContext *s, int idx) {
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
static int inline coeff_dequant(int coeff,
                                int qoffset, int qfactor) {
    int magnitude = coeff * qfactor;

    if (! magnitude)
        return 0;

    magnitude += qoffset;
    magnitude >>= 2;

    return magnitude;
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
                             subband_t orientation, int x) {
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
static int inline coeff_posy(DiracContext *s, int level,
                             subband_t orientation, int y) {
    if (orientation == subband_lh || orientation == subband_hh)
        return subband_height(s, level) + y;

    return y;
}

/**
 * Returns if the pixel has a zero neighbourhood (the coefficient at
 * the left, top and left top of this coefficient are all zero)
 *
 * @param data current coefficient
 * @param v vertical position of the coefficient
 * @param h horizontal position of the coefficient
 * @return 1 if zero neighbourhood, otherwise 0
 */
static int zero_neighbourhood(DiracContext *s, int16_t *data, int v, int h) {
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
static int sign_predict(DiracContext *s, int16_t *data,
                        subband_t orientation, int v, int h) {
    if (orientation == subband_hl && v > 0)
        return DIRAC_SIGN(data[-s->padded_width]);
    else if (orientation == subband_lh && h > 0)
        return DIRAC_SIGN(data[-1]);
    else
        return 0;
}

/**
 * Unpack a single coefficient
 *
 * @param data coefficients
 * @param level subband level
 * @param orientation orientation of the subband
 * @param v vertical position of the to be decoded coefficient in the subband
 * @param h horizontal position of the to be decoded coefficient in the subband
 * @param qoffset quantizer offset
 * @param qfact quantizer factor
 */
static void coeff_unpack(DiracContext *s, int16_t *data, int level,
                         subband_t orientation, int v, int h,
                         int qoffset, int qfactor) {
    int parent = 0;
    int nhood;
    int idx;
    int coeff;
    int read_sign;
    struct dirac_arith_context_set *context;
    uint16_t *coeffp;
    int vdata, hdata;

    vdata = coeff_posy(s, level, orientation, v);
    hdata = coeff_posx(s, level, orientation, h);

    coeffp = &data[hdata + vdata * s->padded_width];

    /* The value of the pixel belonging to the lower level.  */
    if (level >= 2) {
        int x = coeff_posx(s, level - 1, orientation, h >> 1);
        int y = coeff_posy(s, level - 1, orientation, v >> 1);
        parent = data[s->padded_width * y + x] != 0;
    }

    /* Determine if the pixel has only zeros in its neighbourhood.  */
    nhood = zero_neighbourhood(s, coeffp, v, h);

    /* Calculate an index into context_sets_waveletcoeff.  */
    idx = parent * 6 + (!nhood) * 3;
    idx += sign_predict(s, coeffp, orientation, v, h);

    context = &context_sets_waveletcoeff[idx];

    coeff = dirac_arith_read_uint(&s->arith, context);

    read_sign = coeff;
    coeff = coeff_dequant(coeff, qoffset, qfactor);
    if (read_sign) {
        if (dirac_arith_get_bit(&s->arith, context->sign))
            coeff = -coeff;
    }

    *coeffp = coeff;
}

/**
 * Decode a codeblock
 *
 * @param data coefficients
 * @param level subband level
 * @param orientation orientation of the current subband
 * @param x position of the codeblock within the subband in units of codeblocks
 * @param y position of the codeblock within the subband in units of codeblocks
 * @param quant quantizer offset
 * @param quant quantizer factor
 */
static void codeblock(DiracContext *s, int16_t *data, int level,
                      subband_t orientation, int x, int y,
                      int qoffset, int qfactor) {
    int blockcnt_one = (s->codeblocksh[level] + s->codeblocksv[level]) == 2;
    int left, right, top, bottom;
    int v, h;

    left   = (subband_width(s, level)  *  x     ) / s->codeblocksh[level];
    right  = (subband_width(s, level)  * (x + 1)) / s->codeblocksh[level];
    top    = (subband_height(s, level) *  y     ) / s->codeblocksv[level];
    bottom = (subband_height(s, level) * (y + 1)) / s->codeblocksv[level];

    if (!blockcnt_one) {
        /* Determine if this codeblock is a zero block.  */
        if (dirac_arith_get_bit(&s->arith, ARITH_CONTEXT_ZERO_BLOCK))
            return;
    }

    for (v = top; v < bottom; v++)
        for (h = left; h < right; h++)
            coeff_unpack(s, data, level, orientation, v, h,
                         qoffset, qfactor);
}

/**
 * Intra DC Prediction
 *
 * @param data coefficients
 */
static void intra_dc_prediction(DiracContext *s, int16_t *data) {
    int pred;
    int x, y;
    int16_t *line = data;

    for (y = 0; y < subband_height(s, 0); y++) {
        for (x = 0; x < subband_width(s, 0); x++) {
            if (x > 0 && y > 0) {
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
 * @param level subband level
 * @param orientation orientation of the subband
 */
static int subband(DiracContext *s, int16_t *data, int level,
                   subband_t orientation) {
    GetBitContext *gb = &s->gb;
    int length;
    int quant, qoffset, qfactor;
    int x, y;

    length = svq3_get_ue_golomb(gb);
    if (! length) {
        align_get_bits(gb);
    } else {
        quant = svq3_get_ue_golomb(gb);
        qfactor = coeff_quant_factor(quant);
        qoffset = coeff_quant_offset(s, quant) + 2;

        dirac_arith_init(&s->arith, gb, length);

        for (y = 0; y < s->codeblocksv[level]; y++)
            for (x = 0; x < s->codeblocksh[level]; x++)
                codeblock(s, data, level, orientation, x, y,
                          qoffset, qfactor);
        dirac_arith_flush(&s->arith);
    }

    return 0;
}

/**
 * Decode the DC subband
 *
 * @param data coefficients
 * @param level subband level
 * @param orientation orientation of the subband
 */
static int subband_dc(DiracContext *s, int16_t *data) {
    GetBitContext *gb = &s->gb;
    int length;
    int quant, qoffset, qfactor;
    int width, height;
    int x, y;

    width  = subband_width(s, 0);
    height = subband_height(s, 0);

    length = svq3_get_ue_golomb(gb);
    if (! length) {
        align_get_bits(gb);
    } else {
        quant = svq3_get_ue_golomb(gb);
        qfactor = coeff_quant_factor(quant);
        qoffset = coeff_quant_offset(s, quant) + 2;

        dirac_arith_init(&s->arith, gb, length);

        for (y = 0; y < height; y++)
            for (x = 0; x < width; x++)
                coeff_unpack(s, data, 0, subband_ll, y, x,
                         qoffset, qfactor);

        dirac_arith_flush(&s->arith);
    }

    if (s->refs == 0)
        intra_dc_prediction(s, data);

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
static void dirac_unpack_prediction_parameters(DiracContext *s) {
    GetBitContext *gb = &s->gb;

    /* Override block parameters.  */
    if (get_bits1(gb)) {
        int idx = svq3_get_ue_golomb(gb);
        if (idx == 0) {
            s->frame_decoding.luma_xblen = svq3_get_ue_golomb(gb);
            s->frame_decoding.luma_yblen = svq3_get_ue_golomb(gb);
            s->frame_decoding.luma_xbsep = svq3_get_ue_golomb(gb);
            s->frame_decoding.luma_ybsep = svq3_get_ue_golomb(gb);
        } else {
            s->frame_decoding.luma_xblen = block_param_defaults[idx].xblen;
            s->frame_decoding.luma_yblen = block_param_defaults[idx].yblen;
            s->frame_decoding.luma_xbsep = block_param_defaults[idx].xbsep;
            s->frame_decoding.luma_ybsep = block_param_defaults[idx].ybsep;
        }
    }

    /* Setup the blen and bsep parameters for the chroma
       component.  */
    s->frame_decoding.chroma_xblen = (s->frame_decoding.luma_xblen
                                      >> s->chroma_hshift);
    s->frame_decoding.chroma_yblen = (s->frame_decoding.luma_yblen
                                      >> s->chroma_vshift);
    s->frame_decoding.chroma_xbsep = (s->frame_decoding.luma_xbsep
                                      >> s->chroma_hshift);
    s->frame_decoding.chroma_ybsep = (s->frame_decoding.luma_ybsep
                                      >> s->chroma_vshift);

    /* Override motion vector precision.  */
    if (get_bits1(gb))
        s->frame_decoding.mv_precision = svq3_get_ue_golomb(gb);

    /* Read the global motion compensation parameters.  */
    s->globalmc_flag = get_bits1(gb);
    if (s->globalmc_flag) {
        int ref;
        for (ref = 0; ref < s->refs; ref++) {
            memset(&s->globalmc, 0, sizeof(s->globalmc));

            /* Pan/til parameters.  */
            if (get_bits1(gb)) {
                s->globalmc.b[0] = dirac_get_se_golomb(gb);
                s->globalmc.b[1] = dirac_get_se_golomb(gb);
            }

            /* Rotation/shear parameters.  */
            if (get_bits1(gb)) {
                s->globalmc.zrs_exp = svq3_get_ue_golomb(gb);
                s->globalmc.A[0][0] = dirac_get_se_golomb(gb);
                s->globalmc.A[0][1] = dirac_get_se_golomb(gb);
                s->globalmc.A[1][0] = dirac_get_se_golomb(gb);
                s->globalmc.A[1][1] = dirac_get_se_golomb(gb);
            }

            /* Perspective parameters.  */
            if (get_bits1(gb)) {
                s->globalmc.perspective_exp = svq3_get_ue_golomb(gb);
                s->globalmc.c[0]            = dirac_get_se_golomb(gb);
                s->globalmc.c[1]            = dirac_get_se_golomb(gb);
            }
        }
    }

    /* Picture prediction mode.  Not used yet in the specification.  */
    if (get_bits1(gb)) {
        /* Just ignore it, it should and will be zero.  */
        svq3_get_ue_golomb(gb);
    }

    /* XXX: For now set the weights here, I can't find this in the
       specification.  */
    s->frame_decoding.picture_weight_ref1 = 1;
    if (s->refs == 2) {
        s->frame_decoding.picture_weight_precision = 1;
        s->frame_decoding.picture_weight_ref2      = 1;
    } else {
        s->frame_decoding.picture_weight_precision = 0;
        s->frame_decoding.picture_weight_ref2      = 0;
    }

    /* Override reference picture weights.  */
    if (get_bits1(gb)) {
        s->frame_decoding.picture_weight_precision = svq3_get_ue_golomb(gb);
        s->frame_decoding.picture_weight_ref1 = dirac_get_se_golomb(gb);
        if (s->refs == 2)
            s->frame_decoding.picture_weight_ref2 = dirac_get_se_golomb(gb);
    }
}

static const int avgsplit[7] = { 0, 0, 1, 1, 1, 2, 2 };

static inline int split_prediction(DiracContext *s, int x, int y) {
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
static inline int mode_prediction(DiracContext *s,
                                  int x, int y, int refmask, int refshift) {
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
 * Blockmode prediction
 *
 * @param x    horizontal position of the MC block
 * @param y    vertical position of the MC block
 */
static void blockmode_prediction(DiracContext *s, int x, int y) {
    int res = dirac_arith_get_bit(&s->arith, ARITH_CONTEXT_PMODE_REF1);

    res ^= mode_prediction(s, x, y, DIRAC_REF_MASK_REF1, 0);
    s->blmotion[y * s->blwidth + x].use_ref |= res;
    if (s->refs == 2) {
        res = dirac_arith_get_bit(&s->arith, ARITH_CONTEXT_PMODE_REF2);
        res ^= mode_prediction(s, x, y, DIRAC_REF_MASK_REF2, 1);
        s->blmotion[y * s->blwidth + x].use_ref |= res << 1;
    }
}

/**
 * Prediction for global motion compensation
 *
 * @param x    horizontal position of the MC block
 * @param y    vertical position of the MC block
 */
static void blockglob_prediction(DiracContext *s, int x, int y) {
    /* Global motion compensation is not used at all.  */
    if (!s->globalmc_flag)
        return;

    /* Global motion compensation is not used for this block.  */
    if (s->blmotion[y * s->blwidth + x].use_ref & 3) {
        int res = dirac_arith_get_bit(&s->arith, ARITH_CONTEXT_GLOBAL_BLOCK);
        res ^= mode_prediction(s, x, y, DIRAC_REF_MASK_GLOBAL, 2);
        s->blmotion[y * s->blwidth + x].use_ref |= res << 2;
    }
}

/**
 * copy the block data to other MC blocks
 *
 * @param step superblock step size, so the number of MC blocks to copy
 * @param x    horizontal position of the MC block
 * @param y    vertical position of the MC block
 */
static void propagate_block_data(DiracContext *s, int step,
                                 int x, int y) {
    int i, j;

    /* XXX: For now this is rather inefficient, because everything is
       copied.  This function is called quite often.  */
    for (j = y; j < y + step; j++)
        for (i = x; i < x + step; i++)
            s->blmotion[j * s->blwidth + i] = s->blmotion[y * s->blwidth + x];
}

/**
 * Predict the motion vector
 *
 * @param x    horizontal position of the MC block
 * @param y    vertical position of the MC block
 * @param ref reference frame
 * @param dir direction horizontal=0, vertical=1
 */
static int motion_vector_prediction(DiracContext *s, int x, int y,
                                    int ref, int dir) {
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

static int block_dc_prediction(DiracContext *s,
                               int x, int y, int comp) {
    int total = 0;
    int cnt = 0;

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
        return 1 << (s->sequence.video_depth - 1);

    /* Return the average of all DC values that were counted.  */
    return (total + (cnt >> 1)) / cnt;
}

static void unpack_block_dc(DiracContext *s, int x, int y, int comp) {
    int res;

    if (s->blmotion[y * s->blwidth + x].use_ref & 3) {
        s->blmotion[y * s->blwidth + x].dc[comp] = 0;
        return;
    }

    res = dirac_arith_read_int(&s->arith, &context_set_dc);
    res += block_dc_prediction(s, x, y, comp);

    s->blmotion[y * s->blwidth + x].dc[comp] = res;
}

/**
 * Unpack a single motion vector
 *
 * @param ref reference frame
 * @param dir direction horizontal=0, vertical=1
 */
static void dirac_unpack_motion_vector(DiracContext *s,
                                       int ref, int dir,
                                       int x, int y) {
    int res;
    const int refmask = (ref + 1) | DIRAC_REF_MASK_GLOBAL;

    /* First determine if for this block in the specific reference
       frame a motion vector is required.  */
    if ((s->blmotion[y * s->blwidth + x].use_ref & refmask) != ref + 1)
        return;

    res = dirac_arith_read_int(&s->arith, &context_set_mv);
    res += motion_vector_prediction(s, x, y, ref, dir);
    s->blmotion[y * s->blwidth + x].vect[ref][dir] = res;
}

/**
 * Unpack motion vectors
 *
 * @param ref reference frame
 * @param dir direction horizontal=0, vertical=1
 */
static void dirac_unpack_motion_vectors(DiracContext *s,
                                        int ref, int dir) {
    GetBitContext *gb = &s->gb;
    int length;
    int x, y;

    length = svq3_get_ue_golomb(gb);
    dirac_arith_init(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
                        int q, p;
            int blkcnt = 1 << s->sbsplit[y * s->sbwidth + x];
            int step = 4 >> s->sbsplit[y * s->sbwidth + x];

            for (q = 0; q < blkcnt; q++)
                for (p = 0; p < blkcnt; p++) {
                    dirac_unpack_motion_vector(s, ref, dir,
                                         4 * x + p * step,
                                         4 * y + q * step);
                    propagate_block_data(s, step,
                                         4 * x + p * step,
                                         4 * y + q * step);
                }
        }
    dirac_arith_flush(&s->arith);
}

/**
 * Unpack the motion compensation parameters
 */
static void dirac_unpack_prediction_data(DiracContext *s) {
    GetBitContext *gb = &s->gb;
    int i;
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
    length = svq3_get_ue_golomb(gb);
    dirac_arith_init(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
            int res = dirac_arith_read_uint(&s->arith, &context_set_split);
            s->sbsplit[y * s->sbwidth + x] = (res +
                                              split_prediction(s, x, y));
            s->sbsplit[y * s->sbwidth + x] %= 3;
        }
    dirac_arith_flush(&s->arith);

    /* Prediction modes.  */
    length = svq3_get_ue_golomb(gb);
    dirac_arith_init(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
            int q, p;
            int blkcnt = 1 << s->sbsplit[y * s->sbwidth + x];
            int step   = 4 >> s->sbsplit[y * s->sbwidth + x];

            for (q = 0; q < blkcnt; q++)
                for (p = 0; p < blkcnt; p++) {
                    blockmode_prediction(s,
                                         4 * x + p * step,
                                         4 * y + q * step);
                    blockglob_prediction(s,
                                         4 * x + p * step,
                                         4 * y + q * step);
                    propagate_block_data(s, step,
                                         4 * x + p * step,
                                         4 * y + q * step);
                }
        }
    dirac_arith_flush(&s->arith);

    /* Unpack the motion vectors.  */
    for (i = 0; i < s->refs; i++) {
        dirac_unpack_motion_vectors(s, i, 0);
        dirac_unpack_motion_vectors(s, i, 1);
    }

    /* Unpack the DC values for all the three components (YUV).  */
    for (comp = 0; comp < 3; comp++) {
        /* Unpack the DC values.  */
        length = svq3_get_ue_golomb(gb);
        dirac_arith_init(&s->arith, gb, length);
        for (y = 0; y < s->sbheight; y++)
            for (x = 0; x < s->sbwidth; x++) {
                int q, p;
                int blkcnt = 1 << s->sbsplit[y * s->sbwidth + x];
                int step   = 4 >> s->sbsplit[y * s->sbwidth + x];

                for (q = 0; q < blkcnt; q++)
                    for (p = 0; p < blkcnt; p++) {
                        unpack_block_dc(s,
                                        4 * x + p * step,
                                        4 * y + q * step,
                                        comp);
                        propagate_block_data(s, step,
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
static void decode_component(DiracContext *s, int16_t *coeffs) {
    GetBitContext *gb = &s->gb;
    int level;
    subband_t orientation;

    /* Align for coefficient bitstream.  */
    align_get_bits(gb);

    /* Unpack LL, level 0.  */
    subband_dc(s, coeffs);

    /* Unpack all other subbands at all levels.  */
    for (level = 1; level <= s->frame_decoding.wavelet_depth; level++) {
        for (orientation = 1; orientation <= subband_hh; orientation++)
            subband(s, coeffs, level, orientation);
    }
 }

/**
 * Reorder coefficients so the IDWT synthesis can run in place
 *
 * @param data   coefficients
 * @param synth  output buffer
 * @param level  subband level
 */
static void dirac_subband_idwt_reorder(DiracContext *s, int16_t *data,
                                       int16_t *synth, int level) {
    int x, y;
    int width           = subband_width(s, level);
    int height          = subband_height(s, level);
    int synth_width     = width << 1;
    int16_t *synth_line = synth;
    int16_t *line_ll    = data;
    int16_t *line_lh    = data + height * s->padded_width;
    int16_t *line_hl    = data                            + width;
    int16_t *line_hh    = data + height * s->padded_width + width;

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
static int dirac_subband_idwt_53(DiracContext *s,
                                 int16_t *data, int level) {
    int16_t *synth, *synthline;
    int x, y;
    int width = subband_width(s, level);
    int height = subband_height(s, level);
    int synth_width = width  << 1;
    int synth_height = height << 1;

START_TIMER

    if (avcodec_check_dimensions(s->avctx, synth_width, synth_height)) {
        av_log(s->avctx, AV_LOG_ERROR, "avcodec_check_dimensions() failed\n");
        return -1;
    }

    synth = av_malloc(synth_width * synth_height * sizeof(int16_t));
    if (!synth) {
        av_log(s->avctx, AV_LOG_ERROR, "av_malloc() failed\n");
        return -1;
    }

    dirac_subband_idwt_reorder(s, data, synth, level);

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
        synthline[x] += (synthline[x - synth_width]
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
        synthline[0] -= (synthline[1]
                       + synthline[1]
                       + 2) >> 2;
        for (x = 1; x < width - 1; x++) {
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
static int dirac_subband_idwt_97(DiracContext *s,
                                 int16_t *data, int level) {
    int16_t *synth, *synthline;
    int x, y;
    int width = subband_width(s, level);
    int height = subband_height(s, level);
    int synth_width = width  << 1;
    int synth_height = height << 1;

START_TIMER

    if (avcodec_check_dimensions(s->avctx, synth_width, synth_height)) {
        av_log(s->avctx, AV_LOG_ERROR, "avcodec_check_dimensions() failed\n");
        return -1;
    }

    synth = av_malloc(synth_width * synth_height * sizeof(int16_t));
    if (!synth) {
        av_log(s->avctx, AV_LOG_ERROR, "av_malloc() failed\n");
        return -1;
    }

    dirac_subband_idwt_reorder(s, data, synth, level);

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
        synthline[x] -= (synthline[x + synth_width]
                       + synthline[x + synth_width]
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

/**
 * IDWT
 *
 * @param coeffs coefficients to transform
 * @return returns 0 on succes, otherwise -1
 */
static int dirac_idwt(DiracContext *s, int16_t *coeffs) {
    int level;
    int wavelet_idx;

    for (level = 1; level <= s->frame_decoding.wavelet_depth; level++) {
        if (s->refs == 0)
            wavelet_idx = s->frame_decoding.wavelet_idx_intra;
        else
            wavelet_idx = s->frame_decoding.wavelet_idx_inter;

        switch(wavelet_idx) {
        case 0:
            dirac_subband_idwt_97(s, coeffs, level);
            break;
        case 1:
            dirac_subband_idwt_53(s, coeffs, level);
            break;
        default:
            av_log(s->avctx, AV_LOG_INFO, "unknown IDWT index: %d\n",
                   wavelet_idx);
        }
    }

    return 0;
}

/**
 * Search a frame in the buffer of reference frames
 *
 * @param  frameno  frame number in display order
 * @return index of the reference frame in the reference frame buffer
 */
static int reference_frame_idx(DiracContext *s, int frameno) {
    int i;

    for (i = 0; i < s->refcnt; i++) {
        AVFrame *f = &s->refframes[i].frame;
        if (f->display_picture_number == frameno)
            return i;
    }

    return -1;
}

/**
 * Interpolate a frame
 *
 * @param refframe frame to grab the upconverted pixel from
 * @param width    frame width
 * @param height   frame height
 * @param pixels   buffer to write the interpolated pixels to
 * @param comp     component
 */
static inline void interpolate_frame_halfpel(AVFrame *refframe,
                                      int width, int height,
                                      uint8_t *pixels, int comp) {
    uint8_t *lineout;
    uint8_t *lineoutodd;
    uint8_t *refdata;
    uint8_t *linein;
    int outwidth = width * 2;
    int x, y;
    const int t[5] = { 167, -56, 25, -11, 3 };

START_TIMER

    refdata    = refframe->data[comp];

    lineout    = pixels;
    lineoutodd = pixels + outwidth;
    linein     = refdata;
    /* Top 4 lines.  */
    for (y = 0; y < 5; y++) {
        for (x = 0; x < width; x++) {
            int val = 128;
            uint8_t *li1 = linein;
            uint8_t *li2 = linein + refframe->linesize[comp];

            val += t[0] * 2 * linein[x];
            if (y > 1)
                li1 -= refframe->linesize[comp];

            val += t[1] * (li1[x] + li2[x]);
            if (y > 2)
                li1 -= refframe->linesize[comp];
            li2 += refframe->linesize[comp];

            val += t[2] * (li1[x] + li2[x]);
            if (y > 3)
                li1 -= refframe->linesize[comp];
            li2 += refframe->linesize[comp];

            val += t[3] * (li1[x] + li2[x]);
            if (y > 4)
                li1 -= refframe->linesize[comp];
            li2 += refframe->linesize[comp];

            val += t[4] * (li1[x] + li2[x]);

            val >>= 8;

            lineout[x * 2] = linein[x];
            lineoutodd[x * 2] = av_clip_uint8(val);
        }

        linein += refframe->linesize[comp];

        /* Skip one line, we are interpolating to odd lines.  */
        lineout    += outwidth * 2;
        lineoutodd += outwidth * 2;
    }

    /* Middle part.  */
    linein = refdata + refframe->linesize[comp] * 5;
    for (y = 5; y < height - 5; y++) {
        for (x = 0; x < width; x++) {
            int val = 128;
            uint8_t *li1 = linein - refframe->linesize[comp];
            uint8_t *li2 = linein + refframe->linesize[comp];

            val += t[0] * 2 * linein[x];

            val += t[1] * (li1[x] + li2[x]);
            li1 -= refframe->linesize[comp];
            li2 += refframe->linesize[comp];

            val += t[2] * (li1[x] + li2[x]);
            li1 -= refframe->linesize[comp];
            li2 += refframe->linesize[comp];

            val += t[3] * (li1[x] + li2[x]);
            li1 -= refframe->linesize[comp];
            li2 += refframe->linesize[comp];

            val += t[4] * (li1[x] + li2[x]);

            val >>= 8;

            lineout[x * 2] = linein[x];
            lineoutodd[x * 2] = av_clip_uint8(val);
        }

        linein += refframe->linesize[comp];

        /* Skip one line, we are interpolating to odd lines.  */
        lineout    += outwidth * 2;
        lineoutodd += outwidth * 2;
    }

    /* Bottom.  */
    linein = refdata + refframe->linesize[comp] * (height - 5);
    for (y = height - 5; y < height; y++) {
        for (x = 0; x < width; x++) {
            int val = 128;
            uint8_t *li1 = linein - refframe->linesize[comp];
            uint8_t *li2 = linein;

            val += t[0] * 2 * linein[x];
            if (y < height - 2)
                li2 += refframe->linesize[comp];

            val += t[1] * (li1[x] + li2[x]);
            li1 -= refframe->linesize[comp];
            if (y < height - 3)
                li2 += refframe->linesize[comp];

            val += t[2] * (li1[x] + li2[x]);
            li1 -= refframe->linesize[comp];
            if (y < height - 4)
                li2 += refframe->linesize[comp];

            val += t[3] * (li1[x] + li2[x]);
            li1 -= refframe->linesize[comp];
            if (y < height - 5)
                li2 += refframe->linesize[comp];

            val += t[4] * (li1[x] + li2[x]);

            val >>= 8;

            lineout[x * 2]    = linein[x];
            lineoutodd[x * 2] = av_clip_uint8(val);
        }

        linein += refframe->linesize[comp];

        /* Skip one line, we are interpolating to odd lines.  */
        lineout    += outwidth * 2;
        lineoutodd += outwidth * 2;
    }

    /* At this place the even rows of pixels are in place, no copying
       is required..  */

    /* Interpolate the odd rows of pixels: Left.  */
    lineout = pixels + 1;
    linein  = pixels;
    for (y = 0; y < height * 2; y++) {
        for (x = 0; x < 10; x += 2) {
            uint8_t *li1 = &linein[x];
            uint8_t *li2 = &linein[x];
            int val = 128;

            val += t[0] * (*li1 + *li2);

            if (x > 1)
                li1 -= 2;
            li2 += 2;
            val += t[1] * (*li1 + *li2);

            if (x > 2)
                li1 -= 2;
            li2 += 2;
            val += t[2] * (*li1 + *li2);

            if (x > 4)
                li1 -= 2;
            li2 += 2;
            val += t[3] * (*li1 + *li2);

            if (x > 6)
                li1 -= 2;
            li2 += 2;
            val += t[4] * (*li1 + *li2);

            val >>= 8;
            lineout[x] = av_clip_uint8(val);
        }
        lineout += outwidth;
        linein  += outwidth;
    }

    /* Middle.  */
    lineout = pixels + 1;
    linein  = pixels;
    for (y = 0; y < height * 2; y++) {
        for (x = 10; x < outwidth - 10; x += 2) {
            uint8_t *li1 = &linein[x];
            uint8_t *li2 = &linein[x];
            int val = 128;

            val += t[0] * (*li1 + *li2);
            li1 -= 2;
            li2 += 2;
            val += t[1] * (*li1 + *li2);
            li1 -= 2;
            li2 += 2;
            val += t[2] * (*li1 + *li2);
            li1 -= 2;
            li2 += 2;
            val += t[3] * (*li1 + *li2);
            li1 -= 2;
            li2 += 2;
            val += t[4] * (*li1 + *li2);

            val >>= 8;
            lineout[x] = av_clip_uint8(val);
        }
        lineout += outwidth;
        linein  += outwidth;
    }

    /* Right.  */
    lineout = pixels + 1;
    linein  = pixels;
    for (y = 0; y < height * 2; y++) {
        for (x = outwidth - 10; x < outwidth; x += 2) {
            uint8_t *li1 = &linein[x];
            uint8_t *li2 = &linein[x];
            int val = 128;

            val += t[0] * (*li1 + *li2);

            li1 -= 2;
            if (x < width - 4)
                li2 += 2;
            val += t[1] * (*li1 + *li2);

            li1 -= 2;
            if (x < width - 6)
                li2 += 2;
            val += t[2] * (*li1 + *li2);

            li1 -= 2;
            if (x < width - 8)
                li2 += 2;
            val += t[3] * (*li1 + *li2);

            li1 -= 2;
            if (x < width - 10)
                li2 += 2;
            val += t[4] * (*li1 + *li2);

            val >>= 8;
            lineout[x] = av_clip_uint8(val);
        }
        lineout += outwidth;
        linein  += outwidth;
    }

STOP_TIMER("halfpel");
}

/**
 * Get a pixel from the halfpel interpolated frame
 *
 * @param refframe frame to grab the upconverted pixel from
 * @param width    frame width
 * @param height   frame height
 * @param x        horizontal pixel position
 * @param y        vertical pixel position
 */
static inline int get_halfpel(uint8_t *refframe, int width, int height,
                              int x, int y) {
    int xpos;
    int ypos;

    xpos = av_clip(x, 0, width  - 1);
    ypos = av_clip(y, 0, height - 1);

    return refframe[xpos + ypos * width];
}

/**
 * Upconvert pixel (qpel/eighth-pel)
 *
 * @param refframe frame to grab the upconverted pixel from
 * @param width    frame width
 * @param height   frame height
 * @param x        horizontal pixel position
 * @param y        vertical pixel position
 * @param comp     component
 */
static int upconvert(DiracContext *s, uint8_t *refframe,
                     int width, int height, int x, int y, int comp) {
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

/**
 * Calculate WH or WV of the spatial weighting matrix
 *
 * @param i       block position
 * @param x       current pixel
 * @param bsep    block spacing
 * @param blen    block length
 * @param offset  xoffset/yoffset
 * @param blocks  number of blocks
 */
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
        return av_clip(blen - FFABS(2*pos - (blen - 1)), 0, max);
}

/**
 * Motion Compensation with two reference frames
 *
 * @param coeffs     coefficients to add the DC to
 * @param i          horizontal position of the MC block
 * @param j          vertical position of the MC block
 * @param xstart     top left coordinate of the MC block
 * @param ystop      top left coordinate of the MC block
 * @param xstop      bottom right coordinate of the MC block
 * @param ystop      bottom right coordinate of the MC block
 * @param ref1       first reference frame
 * @param ref2       second reference frame
 * @param currblock  MC block to use
 * @param comp       component
 */
static void motion_comp_block2refs(DiracContext *s, int16_t *coeffs,
                                   int i, int j, int xstart, int xstop,
                                   int ystart, int ystop, uint8_t *ref1,
                                   uint8_t *ref2,
                                   struct dirac_blockmotion *currblock,
                                   int comp) {
    int x, y;
    int16_t *line;
    int px1, py1;
    int px2, py2;
    int vect1[2];
    int vect2[2];

    vect1[0] = currblock->vect[0][0];
    vect1[1] = currblock->vect[0][1];
    vect2[0] = currblock->vect[1][0];
    vect2[1] = currblock->vect[1][1];

    if (comp != 0) {
        vect1[0] >>= s->chroma_hshift;
        vect2[0] >>= s->chroma_hshift;
        vect1[1] >>= s->chroma_vshift;
        vect2[1] >>= s->chroma_vshift;
    }

    line = &coeffs[s->width * ystart];
    for (y = ystart; y < ystop; y++) {
        for (x = xstart; x < xstop; x++) {
            int val1 = 0;
            int val2 = 0;
            int val = 0;

            if (s->frame_decoding.mv_precision > 0) {
                px1 = (x << s->frame_decoding.mv_precision) + vect1[0];
                py1 = (y << s->frame_decoding.mv_precision) + vect1[1];
                px2 = (x << s->frame_decoding.mv_precision) + vect2[0];
                py2 = (y << s->frame_decoding.mv_precision) + vect2[1];
            } else {
                px1 = (x + vect1[0]) << 1;
                py1 = (y + vect1[1]) << 1;
                px2 = (x + vect2[0]) << 1;
                py2 = (y + vect2[1]) << 1;
            }

            val1 = upconvert(s, ref1, s->refwidth, s->refheight,
                             px1, py1, comp);
            val1 *= s->frame_decoding.picture_weight_ref1;

            val2 = upconvert(s, ref2, s->refwidth, s->refheight,
                             px2, py2, comp);
            val2 *= s->frame_decoding.picture_weight_ref2;

            val = val1 + val2;
            val = (val
                   * spatial_wt(i, x, s->xbsep, s->xblen,
                                s->xoffset, s->current_blwidth)
                   * spatial_wt(j, y, s->ybsep, s->yblen,
                                s->yoffset, s->current_blheight));

            line[x] += val;
        }
        line += s->width;
    }
}

/**
 * Motion Compensation with one reference frame
 *
 * @param coeffs     coefficients to add the DC to
 * @param i          horizontal position of the MC block
 * @param j          vertical position of the MC block
 * @param xstart     top left coordinate of the MC block
 * @param ystop      top left coordinate of the MC block
 * @param xstop      bottom right coordinate of the MC block
 * @param ystop      bottom right coordinate of the MC block
 * @param refframe   reference frame
 * @param ref        0=first refframe 1=second refframe
 * @param currblock  MC block to use
 * @param comp       component
 */
static void motion_comp_block1ref(DiracContext *s, int16_t *coeffs,
                                  int i, int j, int xstart, int xstop,
                                  int ystart, int ystop, uint8_t *refframe,
                                  int ref,
                                  struct dirac_blockmotion *currblock,
                                  int comp) {
    int x, y;
    int16_t *line;
    int px, py;
    int vect[2];

        vect[0] = currblock->vect[ref][0];
        vect[1] = currblock->vect[ref][1];

    if (comp != 0) {
        vect[0] >>= s->chroma_hshift;
        vect[1] >>= s->chroma_vshift;
    }

    line = &coeffs[s->width * ystart];
    for (y = ystart; y < ystop; y++) {
        for (x = xstart; x < xstop; x++) {
            int val = 0;

            if (s->frame_decoding.mv_precision > 0) {
                px = (x << s->frame_decoding.mv_precision) + vect[0];
                py = (y << s->frame_decoding.mv_precision) + vect[1];
            } else {
                px = (x + vect[0]) << 1;
                py = (y + vect[1]) << 1;
            }

            val = upconvert(s, refframe, s->refwidth, s->refheight,
                            px, py, comp);
            val *= s->frame_decoding.picture_weight_ref1
                 + s->frame_decoding.picture_weight_ref2;

            val = (val
                   * spatial_wt(i, x, s->xbsep, s->xblen,
                                s->xoffset, s->current_blwidth)
                   * spatial_wt(j, y, s->ybsep, s->yblen,
                                s->yoffset, s->current_blheight));

            line[x] += val;
        }
        line += s->width;
    }
}

/**
 * Motion Compensation DC values (no reference frame)
 *
 * @param coeffs coefficients to add the DC to
 * @param i      horizontal position of the MC block
 * @param j      vertical position of the MC block
 * @param xstart top left coordinate of the MC block
 * @param ystop  top left coordinate of the MC block
 * @param xstop  bottom right coordinate of the MC block
 * @param ystop  bottom right coordinate of the MC block
 * @param dcval  DC value to apply to all coefficients in the MC block
 */
static inline void motion_comp_dc_block(DiracContext *s,
                                        int16_t *coeffs, int i, int j,
                                        int xstart, int xstop, int ystart,
                                        int ystop, int dcval) {
    int x, y;
    int16_t *line;

    dcval <<= s->frame_decoding.picture_weight_precision;

    line = &coeffs[s->width * ystart];
    for (y = ystart; y < ystop; y++) {
        for (x = xstart; x < xstop; x++) {
            int val;

            val = dcval
                   * spatial_wt(i, x, s->xbsep, s->xblen,
                                s->xoffset, s->current_blwidth)
                   * spatial_wt(j, y, s->ybsep, s->yblen,
                                s->yoffset, s->current_blheight);

            line[x] += val;
        }
        line += s->width;
    }
}

/**
 * Motion compensation
 *
 * @param coeffs coefficients to which the MC results will be added
 * @param comp component
 * @return returns 0 on succes, otherwise -1
 */
static int dirac_motion_compensation(DiracContext *s, int16_t *coeffs,
                                     int comp) {
    int i, j;
    int x, y;
    int refidx[2] = { 0 };
    int cacheframe[2] = {1, 1};
    AVFrame *ref[2] = { 0 };
    struct dirac_blockmotion *currblock;
    int16_t *mcpic;
    int16_t *mcline;
    int16_t *coeffline;
    int xstart, ystart;
    int xstop, ystop;
    int hbits, vbits;
    int total_wt_bits;

    if (comp == 0) {
        s->width  = s->sequence.luma_width;
        s->height = s->sequence.luma_height;
        s->xblen  = s->frame_decoding.luma_xblen;
        s->yblen  = s->frame_decoding.luma_yblen;
        s->xbsep  = s->frame_decoding.luma_xbsep;
        s->ybsep  = s->frame_decoding.luma_ybsep;
    } else {
        s->width  = s->sequence.chroma_width;
        s->height = s->sequence.chroma_height;
        s->xblen  = s->frame_decoding.chroma_xblen;
        s->yblen  = s->frame_decoding.chroma_yblen;
        s->xbsep  = s->frame_decoding.chroma_xbsep;
        s->ybsep  = s->frame_decoding.chroma_ybsep;
    }

    s->xoffset = (s->xblen - s->xbsep) / 2;
    s->yoffset = (s->yblen - s->ybsep) / 2;
    hbits      = av_log2(s->xoffset) + 2;
    vbits      = av_log2(s->yoffset) + 2;

    total_wt_bits = hbits + vbits
                       + s->frame_decoding.picture_weight_precision;

    s->refwidth = s->width << 1;
    s->refheight = s->height << 1;

    if (avcodec_check_dimensions(s->avctx, s->refwidth, s->refheight)) {
        av_log(s->avctx, AV_LOG_ERROR, "avcodec_check_dimensions() failed\n");
        return -1;
    }

    for (i = 0; i < s->refs; i++) {
        refidx[i] = reference_frame_idx(s, s->ref[i]);
        ref[i] = &s->refframes[refidx[i]].frame;

        if (s->refframes[refidx[i]].halfpel[comp] == NULL) {
            s->refdata[i] = av_malloc(s->refwidth * s->refheight);
            if (!s->refdata[i]) {
                if (i == 1)
                    av_free(s->refdata[0]);
                av_log(s->avctx, AV_LOG_ERROR, "av_malloc() failed\n");
                return -1;
            }
            interpolate_frame_halfpel(ref[i], s->width, s->height,
                                      s->refdata[i], comp);
        } else {
            s->refdata[i] = s->refframes[refidx[i]].halfpel[comp];
            cacheframe[i] = 2;
        }
    }

    if (avcodec_check_dimensions(s->avctx, s->width, s->height)) {
        for (i = 0; i < s->refs; i++)
            av_free(s->refdata[i]);

        av_log(s->avctx, AV_LOG_ERROR, "avcodec_check_dimensions() failed\n");
        return -1;
    }

    mcpic = av_malloc(s->width * s->height * sizeof(int16_t));
    if (!mcpic) {
        for (i = 0; i < s->refs; i++)
            av_free(s->refdata[i]);

        av_log(s->avctx, AV_LOG_ERROR, "av_malloc() failed\n");
        return -1;
    }
    memset(mcpic, 0, s->width * s->height * sizeof(int16_t));

    {
        START_TIMER;

        s->current_blwidth  = (s->width  - s->xoffset) / s->xbsep + 1;
        s->current_blheight = (s->height - s->yoffset) / s->ybsep + 1;

        currblock = s->blmotion;
        for (j = 0; j < s->current_blheight; j++) {
            for (i = 0; i < s->current_blwidth; i++) {
                struct dirac_blockmotion *block = &currblock[i];

                /* XXX: These calculations do not match those in the
                   Dirac specification, but are correct.  */
                xstart  = i * s->xbsep - s->xoffset;
                ystart  = j * s->ybsep - s->yoffset;
                xstop   = FFMIN(xstart + s->xblen, s->width);
                ystop   = FFMIN(ystart + s->yblen, s->height);
                xstart  = FFMAX(0, xstart);
                ystart  = FFMAX(0, ystart);

                /* Intra */
                if ((block->use_ref & 3) == 0)
                    motion_comp_dc_block(s, mcpic, i, j,
                                         xstart, xstop, ystart, ystop,
                                         block->dc[comp]);
                /* Reference frame 1 only.  */
                else if ((block->use_ref & 3) == DIRAC_REF_MASK_REF1)
                    motion_comp_block1ref(s, mcpic, i, j,
                                          xstart, xstop, ystart,
                                          ystop,s->refdata[0], 0, block, comp);
                /* Reference frame 2 only.  */
                else if ((block->use_ref & 3) == DIRAC_REF_MASK_REF2)
                    motion_comp_block1ref(s, mcpic, i, j,
                                          xstart, xstop, ystart, ystop,
                                          s->refdata[1], 1, block, comp);
                /* Both reference frames.  */
                else
                    motion_comp_block2refs(s, mcpic, i, j,
                                           xstart, xstop, ystart, ystop,
                                           s->refdata[0], s->refdata[1],
                                           block, comp);
            }
            currblock += s->blwidth;
        }

        coeffline = coeffs;
        mcline    = mcpic;
        /* XXX: It might be more efficient (for cache) to merge this
           with the loop above somehow.  */
        for (y = 0; y < s->height; y++) {
            for (x = 0; x < s->width; x++) {
                int16_t coeff = mcline[x] + (1 << (total_wt_bits - 1));
                coeffline[x] += coeff >> total_wt_bits;
            }
            coeffline += s->padded_width;
            mcline    += s->width;
        }

        av_free(mcpic);

        STOP_TIMER("motioncomp");
    }

    for (i = 0; i < s->retirecnt; i++) {
        if (cacheframe[0] == 1 && i == refidx[0])
            cacheframe[0] = 0;
        if (cacheframe[1] == 1 && i == refidx[1])
            cacheframe[1] = 0;
    }

    for (i = 0; i < s->refs; i++) {
        if (cacheframe[i])
            s->refframes[refidx[i]].halfpel[comp] = s->refdata[i];
        else
            av_freep(&s->refdata[i]);
    }

    return 0;
}

/**
 * Decode a frame.
 *
 * @return 0 when successful, otherwise -1 is returned
 */
static int dirac_decode_frame(DiracContext *s) {
    int16_t *coeffs;
    int16_t *line;
    int comp;
    int x,y;

START_TIMER

    if (avcodec_check_dimensions(s->avctx, s->padded_luma_width,
                                 s->padded_luma_height)) {
        av_log(s->avctx, AV_LOG_ERROR, "avcodec_check_dimensions() failed\n");
        return -1;
    }

    coeffs = av_malloc(s->padded_luma_width
                       * s->padded_luma_height
                       * sizeof(int16_t));
    if (! coeffs) {
        av_log(s->avctx, AV_LOG_ERROR, "av_malloc() failed\n");
        return -1;
    }

    for (comp = 0; comp < 3; comp++) {
        uint8_t *frame = s->picture.data[comp];
        int width, height;

        if (comp == 0) {
            width            = s->sequence.luma_width;
            height           = s->sequence.luma_height;
            s->padded_width  = s->padded_luma_width;
            s->padded_height = s->padded_luma_height;
        } else {
            width            = s->sequence.chroma_width;
            height           = s->sequence.chroma_height;
            s->padded_width  = s->padded_chroma_width;
            s->padded_height = s->padded_chroma_height;
        }

        memset(coeffs, 0,
               s->padded_width * s->padded_height * sizeof(int16_t));

        if (!s->zero_res)
            decode_component(s, coeffs);

        dirac_idwt(s, coeffs);

        if (s->refs) {
            if (dirac_motion_compensation(s, coeffs, comp))
                return -1;
        }

        /* Copy the decoded coefficients into the frame.  */
        line = coeffs;
        for (y = 0; y < height; y++) {
            for (x = 0; x < width; x++)
                frame[x]= av_clip_uint8(line[x]);

            line  += s->padded_width;
            frame += s->picture.linesize[comp];
        }
    }

    av_free(coeffs);

STOP_TIMER("dirac_frame_decode");

    return 0;
}

/**
 * Parse a frame and setup DiracContext to decode it
 *
 * @return 0 when successful, otherwise -1 is returned
 */
static int parse_frame(DiracContext *s) {
    int retire;
    int filter;
    int i;
    GetBitContext *gb = &s->gb;

    /* Setup decoding parameter defaults for this frame.  */
    memcpy(&s->frame_decoding, &s->decoding, sizeof(s->frame_decoding));

    s->picture.pict_type = FF_I_TYPE;
    s->picture.key_frame = 1;

    s->picnum = get_bits_long(gb, 32);

    for (i = 0; i < s->refs; i++)
        s->ref[i] = dirac_get_se_golomb(gb) + s->picnum;

    /* Retire the reference frames that are not used anymore.  */
    retire = svq3_get_ue_golomb(gb);
    s->retirecnt = retire;
    for (i = 0; i < retire; i++) {
        uint32_t retire_num;

        retire_num = dirac_get_se_golomb(gb) + s->picnum;
        s->retireframe[i] = retire_num;
    }

    if (s->refs) {
        align_get_bits(gb);
        dirac_unpack_prediction_parameters(s);
        align_get_bits(gb);
        dirac_unpack_prediction_data(s);
    }

    align_get_bits(gb);

    /* Wavelet transform data.  */
    if (s->refs == 0)
        s->zero_res = 0;
    else
        s->zero_res = get_bits1(gb);

    if (!s->zero_res) {
        /* Override wavelet transform parameters.  */
        if (get_bits1(gb)) {
            dprintf(s->avctx, "Non default filter\n");
            filter = svq3_get_ue_golomb(gb); /* XXX */
        } else {
            dprintf(s->avctx, "Default filter\n");
            filter = s->frame_decoding.wavelet_idx_intra;
        }

        /* Overrid wavelet depth.  */
        if (get_bits1(gb)) {
            dprintf(s->avctx, "Non default depth\n");
            s->frame_decoding.wavelet_depth = svq3_get_ue_golomb(gb);
        }
        dprintf(s->avctx, "Depth: %d\n", s->frame_decoding.wavelet_depth);

        /* Spatial partitioning.  */
        if (get_bits1(gb)) {
            int idx;

            dprintf(s->avctx, "Spatial partitioning\n");

            /* Override the default partitioning.  */
            if (get_bits1(gb)) {
                for (i = 0; i <= s->frame_decoding.wavelet_depth; i++) {
                    s->codeblocksh[i] = svq3_get_ue_golomb(gb);
                    s->codeblocksv[i] = svq3_get_ue_golomb(gb);
                }

                dprintf(s->avctx, "Non-default partitioning\n");

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

            idx = svq3_get_ue_golomb(gb);
            dprintf(s->avctx, "Codeblock mode idx: %d\n", idx);
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
    AVFrame *picture = data;
    int i;
    int parse_code;

    if (buf_size == 0) {
        int idx = reference_frame_idx(s, avctx->frame_number);
        if (idx == -1) {
            /* The frame was not found.  */
            *data_size = 0;
        } else {
            *data_size = sizeof(AVFrame);
            *picture = s->refframes[idx].frame;
        }
        return 0;
    }

    parse_code = buf[4];

    dprintf(avctx, "Decoding frame: size=%d head=%c%c%c%c parse=%02x\n",
            buf_size, buf[0], buf[1], buf[2], buf[3], buf[4]);

    init_get_bits(&s->gb, &buf[13], (buf_size - 13) * 8);
    s->avctx = avctx;

    if (parse_code ==  pc_access_unit_header) {
        parse_access_unit_header(s);

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

    parse_frame(s);

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
                s->refframes[i].frame.display_picture_number);

    for (i = 0; i < s->refs; i++)
        dprintf(avctx, "Reference frame %d: #%d\n", i, s->ref[i]);
#endif

    if (dirac_decode_frame(s))
        return -1;

    s->picture.display_picture_number = s->picnum;

    if (s->picture.reference
        || s->picture.display_picture_number != avctx->frame_number) {
        if (s->refcnt + 1 == REFFRAME_CNT) {
            av_log(avctx, AV_LOG_ERROR, "reference picture buffer overrun\n");
            return -1;
        }

        s->refframes[s->refcnt].halfpel[0] = 0;
        s->refframes[s->refcnt].halfpel[1] = 0;
        s->refframes[s->refcnt].halfpel[2] = 0;
        s->refframes[s->refcnt++].frame = s->picture;
    }

    /* Retire frames that were reordered and displayed if they are no
       reference frames either.  */
    for (i = 0; i < s->refcnt; i++) {
        AVFrame *f = &s->refframes[i].frame;

        if (f->reference == 0
            && f->display_picture_number < avctx->frame_number) {
            s->retireframe[s->retirecnt++] = f->display_picture_number;
        }
    }

    for (i = 0; i < s->retirecnt; i++) {
        AVFrame *f;
        int idx, j;

        idx = reference_frame_idx(s, s->retireframe[i]);
        if (idx == -1) {
            av_log(avctx, AV_LOG_WARNING, "frame to retire #%d not found\n",
                   s->retireframe[i]);
            continue;
        }

        f = &s->refframes[idx].frame;
        /* Do not retire frames that were not displayed yet.  */
        if (f->display_picture_number >= avctx->frame_number) {
            f->reference = 0;
            continue;
        }

        if (f->data[0] != NULL)
            avctx->release_buffer(avctx, f);

        av_free(s->refframes[idx].halfpel[0]);
        av_free(s->refframes[idx].halfpel[1]);
        av_free(s->refframes[idx].halfpel[2]);

        s->refcnt--;

        for (j = idx; j < idx + s->refcnt; j++) {
            s->refframes[j] = s->refframes[j + 1];
        }
    }

    if (s->picture.display_picture_number > avctx->frame_number) {
        int idx;

        if (!s->picture.reference) {
            /* This picture needs to be shown at a later time.  */

            s->refframes[s->refcnt].halfpel[0] = 0;
            s->refframes[s->refcnt].halfpel[1] = 0;
            s->refframes[s->refcnt].halfpel[2] = 0;
            s->refframes[s->refcnt++].frame = s->picture;
        }

        idx = reference_frame_idx(s, avctx->frame_number);
        if (idx == -1) {
            /* The frame is not yet decoded.  */
            *data_size = 0;
        } else {
            *data_size = sizeof(AVFrame);
            *picture = s->refframes[idx].frame;
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

static void dirac_encode_parse_info(DiracContext *s, int parsecode) {
    put_bits(&s->pb, 32, DIRAC_PARSE_INFO_PREFIX);
    put_bits(&s->pb, 8,  parsecode);
    /* XXX: These will be filled in after encoding.  */
    put_bits(&s->pb, 32, 0);
    put_bits(&s->pb, 32, 0);
}

static void dirac_encode_sequence_parameters(DiracContext *s) {
    AVCodecContext *avctx = s->avctx;
    struct sequence_parameters *seq = &s->sequence;
    const struct sequence_parameters *seqdef;
    int video_format = 0;

    seqdef = &sequence_parameters_defaults[video_format];

    /* Fill in defaults for the sequence parameters.  */
    memcpy(&s->sequence, seqdef, sizeof(s->sequence));

    /* Fill in the sequence parameters using the information set by
       the user. XXX: Only support YUV420P for now.  */
    seq->luma_width    = avctx->width;
    seq->luma_height   = avctx->height;
    seq->chroma_width  = avctx->width  / 2;
    seq->chroma_height = avctx->height / 2;
    seq->video_depth   = 8;
    seq->chroma_format = 2;

    /* Set video format to 0.  In the future a best match is perhaps
       better.  */
    dirac_set_ue_golomb(&s->pb, video_format);


    /* Override image dimensions.  */
    if (seq->luma_width != seqdef->luma_width
        || seq->luma_height != seqdef->luma_height) {
        put_bits(&s->pb, 1, 1);

        dirac_set_ue_golomb(&s->pb, seq->luma_width);
        dirac_set_ue_golomb(&s->pb, seq->luma_height);
    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override chroma format.  */
    if (seq->chroma_format != seqdef->chroma_format) {
        put_bits(&s->pb, 1, 1);

        /* XXX: Hardcoded to 4:2:0.  */
        dirac_set_ue_golomb(&s->pb, 2);
    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override video depth.  */
    if (seq->video_depth != seqdef->video_depth) {
        put_bits(&s->pb, 1, 1);

        dirac_set_ue_golomb(&s->pb, seq->video_depth);
    } else {
        put_bits(&s->pb, 1, 0);
    }
}

static void dirac_encode_source_parameters(DiracContext *s) {
    AVCodecContext *avctx = s->avctx;
    struct source_parameters *source = &s->source;
    const struct source_parameters *sourcedef;
    int video_format = 0;

    sourcedef = &source_parameters_defaults[video_format];

    /* Fill in defaults for the source parameters.  */
    memcpy(&s->source, sourcedef, sizeof(s->source));

    /* Fill in the source parameters using the information set by the
       user. XXX: No support for interlacing.  */
    source->interlaced         = 0;
    source->frame_rate.num     = avctx->time_base.den;
    source->frame_rate.den     = avctx->time_base.num;
    source->clean_width        = avctx->width;
    source->clean_height       = avctx->height;

    if (avctx->sample_aspect_ratio.num != 0)
        source->aspect_ratio = avctx->sample_aspect_ratio;

    /* Override interlacing options.  */
    if (source->interlaced != sourcedef->interlaced) {
        put_bits(&s->pb, 1, 1);

        put_bits(&s->pb, 1, source->interlaced);

        /* Override top field first flag.  */
        if (source->top_field_first != sourcedef->top_field_first) {
            put_bits(&s->pb, 1, 1);

            put_bits(&s->pb, 1, source->top_field_first);

        } else {
            put_bits(&s->pb, 1, 0);
        }

        /* Override sequential fields flag.  */
        if (source->sequential_fields != sourcedef->sequential_fields) {
            put_bits(&s->pb, 1, 1);

            put_bits(&s->pb, 1, source->sequential_fields);

        } else {
            put_bits(&s->pb, 1, 0);
        }

    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override frame rate.  */
    if (av_cmp_q(source->frame_rate, sourcedef->frame_rate) != 0) {
        put_bits(&s->pb, 1, 1);

        /* XXX: Some default frame rates can be used.  For now just
           set the index to 0 and write the frame rate.  */
        dirac_set_ue_golomb(&s->pb, 0);

        dirac_set_ue_golomb(&s->pb, source->frame_rate.num);
        dirac_set_ue_golomb(&s->pb, source->frame_rate.den);
    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override aspect ratio.  */
    if (av_cmp_q(source->aspect_ratio, sourcedef->aspect_ratio) != 0) {
        put_bits(&s->pb, 1, 1);

        /* XXX: Some default aspect ratios can be used.  For now just
           set the index to 0 and write the aspect ratio.  */
        dirac_set_ue_golomb(&s->pb, 0);

        dirac_set_ue_golomb(&s->pb, source->aspect_ratio.num);
        dirac_set_ue_golomb(&s->pb, source->aspect_ratio.den);
    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override clean area.  */
    if (source->clean_width != sourcedef->clean_width
        || source->clean_height != sourcedef->clean_height
        || source->clean_left_offset != sourcedef->clean_left_offset
        || source->clean_right_offset != sourcedef->clean_right_offset) {
        put_bits(&s->pb, 1, 1);

        dirac_set_ue_golomb(&s->pb, source->clean_width);
        dirac_set_ue_golomb(&s->pb, source->clean_height);
        dirac_set_ue_golomb(&s->pb, source->clean_left_offset);
        dirac_set_ue_golomb(&s->pb, source->clean_right_offset);
    } else {
        put_bits(&s->pb, 1, 1);
    }

    /* Override signal range.  */
    if (source->luma_offset != sourcedef->luma_offset
        || source->luma_excursion != sourcedef->luma_excursion
        || source->chroma_offset != sourcedef->chroma_offset
        || source->chroma_excursion != sourcedef->chroma_excursion) {
        put_bits(&s->pb, 1, 1);

        /* XXX: Some default signal ranges can be used.  For now just
           set the index to 0 and write the signal range.  */
        dirac_set_ue_golomb(&s->pb, 0);

        dirac_set_ue_golomb(&s->pb, source->luma_offset);
        dirac_set_ue_golomb(&s->pb, source->luma_excursion);
        dirac_set_ue_golomb(&s->pb, source->chroma_offset);
        dirac_set_ue_golomb(&s->pb, source->chroma_excursion);
    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override color spec.  */
    /* XXX: For now this won't be overridden at all.  Just set this to
       defaults.  */
    put_bits(&s->pb, 1, 0);
}

static void dirac_encode_access_unit_header(DiracContext *s) {
    /* First write the Access Unit Parse Parameters.  */

    dirac_set_ue_golomb(&s->pb, 0); /* version major */
    dirac_set_ue_golomb(&s->pb, 1); /* version minor */
    dirac_set_ue_golomb(&s->pb, 0); /* profile */
    dirac_set_ue_golomb(&s->pb, 0); /* level */

    dirac_encode_sequence_parameters(s);
    dirac_encode_source_parameters(s);
}

static int encode_frame(AVCodecContext *avctx, unsigned char *buf,
                        int buf_size, void *data) {
    DiracContext *s = avctx->priv_data;
    AVFrame *picture = data;

    dprintf(avctx, "Encoding frame size=%d\n", buf_size);

    init_put_bits(&s->pb, buf, buf_size * 8);
    s->avctx = avctx;
    s->picture = *picture;

    if (s->last_parse_code == 0) {
        dirac_encode_parse_info(s, pc_access_unit_header);
        dirac_encode_access_unit_header(s);
    }

    flush_put_bits(&s->pb);


    return put_bits_count(&s->pb) * 8;
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
    CODEC_CAP_DELAY,
    NULL
};

#ifdef CONFIG_ENCODERS
AVCodec dirac_encoder = {
    "dirac",
    CODEC_TYPE_VIDEO,
    CODEC_ID_DIRAC,
    sizeof(DiracContext),
    encode_init,
    encode_frame,
    encode_end,
    .pix_fmts = (enum PixelFormat[]) {PIX_FMT_YUV420P, -1}
};
#endif
