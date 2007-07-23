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

struct source_parameters
{
    /* Interlacing.  */
    int interlaced;
    int top_field_first;
    int sequential_fields;

    AVRational frame_rate;

    AVRational aspect_ratio;

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
    int luma_width;
    int luma_height;
    /* 0: 4:4:4, 1: 4:2:2, 2: 4:2:0 */
    int chroma_format;
    int video_depth;

    /* Calculated:  */
    int chroma_width;
    int chroma_height;
};

struct decoding_parameters
{
    int wavelet_depth;
    int wavelet_idx_intra;
    int wavelet_idx_inter;

    int luma_xbsep;
    int luma_xblen;
    int luma_ybsep;
    int luma_yblen;

    int mv_precision;

    int picture_weight_ref1;
    int picture_weight_ref2;
    int picture_weight_bits;

    /* Codeblocks h*v.  */
    int intra_hlevel_012, intra_vlevel_012;
    int intra_hlevel_other, intra_vlevel_other;
    int inter_hlevel_01, inter_vlevel_01;
    int inter_hlevel_2, inter_vlevel_2;
    int inter_hlevel_other, inter_vlevel_other;

    int slice_width;
    int slide_height;
    int slice_bits;
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

typedef struct DiracContext {
    int next_picture;
    int access_unit;
    unsigned int profile;
    unsigned int level;

    GetBitContext *gb;

    AVFrame picture;

    struct source_parameters source;
    struct sequence_parameters sequence;
    struct decoding_parameters decoding;

    struct decoding_parameters frame_decoding;

    int codeblocksh[7]; /* XXX: 7 levels.  */
    int codeblocksv[7]; /* XXX: 7 levels.  */

    int padded_luma_width;
    int padded_luma_height;
    int padded_chroma_width;
    int padded_chroma_height;

    /* Current component.  */
    int padded_width;
    int padded_height;

    /* State of arithmetic decoding.  */
    struct dirac_arith_state arith;
} DiracContext;

static int decode_init(AVCodecContext *avctx){
    av_log_set_level (AV_LOG_DEBUG);
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

    dprintf (avctx, "Frame rate: %d/%d = %f\n",
             source->frame_rate.num, source->frame_rate.den,
             (double) source->frame_rate.num / source->frame_rate.den);
    dprintf (avctx, "Aspect ratio: %d/%d = %f\n",
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
        break;

    case 1:
        /* 4:2:2 */
        s->sequence.chroma_width = s->sequence.luma_width >> 1;
        s->sequence.chroma_height = s->sequence.luma_height;
        break;

    case 2:
        /* 4:2:0 */
        s->sequence.chroma_width = s->sequence.luma_width >> 1;
        s->sequence.chroma_height = s->sequence.luma_height >> 1;
        break;
    }

    /* Override the video depth.  */
    if (get_bits(gb, 1))
        s->sequence.video_depth = dirac_get_ue_golomb(gb);
}

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
                int transfer_idx = dirac_get_ue_golomb(gb);
                s->source.transfer_function = preset_transfer_func[transfer_idx];
            }
        } else {
            /* XXX: Use the index.  */
        }
    }

}

static int parse_access_unit_header(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;
    unsigned int version_major;
    unsigned int version_minor;
    unsigned int video_format;

    /* Parse parameters.  */
    s->next_picture = get_bits_long(gb, 32);

    version_major = dirac_get_ue_golomb(gb);
    version_minor = dirac_get_ue_golomb(gb);
    /* XXX: Don't check the version yet, existing encoders do not yet
       set this to a sane value (0.6 at the moment).  */

    /* XXX: Not yet documented in the spec.  This is actually the main
       thing that is missing.  */
    s->profile = dirac_get_ue_golomb(gb);
    s->level = dirac_get_ue_golomb(gb);

    dprintf (avctx, "Access unit header: Version %d.%d\n",
             version_major, version_minor);
    dprintf (avctx, "Profile: %d, Level: %d\n", s->profile, s->level);

    video_format = dirac_get_ue_golomb(gb);
    dprintf (avctx, "Video format: %d\n", video_format);

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


static int inline subband_width(AVCodecContext *avctx, int level) {
    DiracContext *s = avctx->priv_data;
    if (level == 0)
        return s->padded_width >> s->frame_decoding.wavelet_depth;
    return s->padded_width >> (s->frame_decoding.wavelet_depth - level + 1);
}

static int inline subband_height(AVCodecContext *avctx, int level) {
    DiracContext *s = avctx->priv_data;
    if (level == 0)
        return s->padded_height >> s->frame_decoding.wavelet_depth;
    return s->padded_height >> (s->frame_decoding.wavelet_depth - level + 1);
}

static int inline coeff_quant_factor(uint64_t idx) {
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

static int inline coeff_quant_offset(int idx) {
    if (idx == 0)
        return 1;
    /* XXX: Hardcode for intra frames.  */
    if (idx == 1)
        return 2;
    return (coeff_quant_factor(idx) + 1) >> 1;
}

static int inline coeff_dequant(int coeff, int idx) {
    int64_t magnitude = abs(coeff) * coeff_quant_factor(idx);

    if (! magnitude)
        return 0;

    magnitude += coeff_quant_offset(idx) + 2;
    magnitude >>= 2;

    /* Reintroduce the sign.  */
    if (coeff < 0)
        magnitude = -magnitude;
    return magnitude;
}

static int inline coeff_posx(AVCodecContext *avctx, int level,
                      subband_t orientation, int x) {
    int right = 0;
    if (orientation == subband_hl || orientation == subband_hh)
        right = 1;

    return right * subband_width(avctx, level) + x;
}

static int inline coeff_posy(AVCodecContext *avctx, int level,
                      subband_t orientation, int y) {
    int bottom = 0;
    if (orientation == subband_lh || orientation == subband_hh)
        bottom = 1;

    return bottom * subband_height(avctx, level) + y;
}

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
    else if  (h > 0 && data[x + y * s->padded_width - 1])
        return 0;

    return 1;
}

static int sign_predict(AVCodecContext *avctx, int *data, int level,
                        subband_t orientation, int v, int h) {
    int x = coeff_posx(avctx, level, orientation, h);
    int y = coeff_posy(avctx, level, orientation, v);
    DiracContext *s = avctx->priv_data;

    switch (orientation) {
    case subband_ll:
    case subband_hh:
        return 0;
    case subband_hl:
        if (v == 0)
            return 0;
        else {
            if (data[x + (y - 1) * s->padded_width] == 0) return 0;
            return FFSIGN(data[x + (y - 1) * s->padded_width]);
        }
    case subband_lh:
        if (h == 0)
            return 0;
        else {
            if (data[x + y * s->padded_width - 1] == 0) return 0;
            return FFSIGN(data[x + y * s->padded_width - 1]);
        }
    }

    return 0;
}

static void coeff_unpack(AVCodecContext *avctx, int *data, int level,
                         subband_t orientation, int v, int h, int quant) {
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

    /* XXX: This was recently changed in the reference specification
       to function exactly like in the specification.  For now, I'll
       match this behavior.  */
#if 0
    /* XXX: this is what the reference implementation effectively
       does, although this does not seem to comply with the spec.  I
       have asked the Dirac BBC why this seems to be required.  */
    if (level < 2)
        parent = 1;
#endif

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
    coeff = coeff_dequant(coeff, quant);

    data[hdata + vdata * s->padded_width] = coeff;
}

static void codeblock(AVCodecContext *avctx, int *data, int level,
                      subband_t orientation, int x, int y, int quant) {
    DiracContext *s = avctx->priv_data;
    int blockcnt = s->codeblocksh[level] * s->codeblocksv[level];
    int zero = 0;

    int left = (subband_width(avctx, level) * x) / s->codeblocksh[level];
    int right = (subband_width(avctx, level) * (x + 1)) / s->codeblocksh[level];
    int top = (subband_height(avctx, level) * y) / s->codeblocksv[level];
    int bottom = (subband_height(avctx, level) * (y + 1)) / s->codeblocksv[level];

    int v, h;

    if (blockcnt != 1 && orientation != subband_ll) {
        /* Determine if this codeblock is a zero block.  */
        zero = dirac_arith_get_bit(&s->arith, ARITH_CONTEXT_ZERO_BLOCK);
    }

    if (zero)
        return; /* All coefficients remain 0.  */

    /* XXX: This matches the reference implementation, check the
       spec.  */
    for (v = top; v < bottom; v++)
        for (h = left; h < right; h++)
            coeff_unpack(avctx, data, level, orientation, v, h, quant);

    /* XXX: Quantization.  */
}

static void intra_dc_prediction(AVCodecContext *avctx, int *data, int level,
                                subband_t orientation) {
    DiracContext *s = avctx->priv_data;
    int pred;
    int h, v;

    for (v = 0; v < subband_height(avctx, 0); v++)
        for (h = 0; h < subband_width(avctx, 0); h++) {
            int x = coeff_posx(avctx, level, orientation, h);
            int y = coeff_posy(avctx, level, orientation, v);

            if (h > 0) {
                if (v > 0) {
                    /* Use 3 coefficients for prediction.  */
                    pred = (data[x + y * s->padded_width - 1]
                            + data[x + (y - 1) * s->padded_width]
                            + data[x + (y - 1) * s->padded_width - 1]);
                    if (pred > 0)
                        pred = (pred + 1) / 3;
                    else /* XXX: For now just do what the reference
                            implementation does.  Check this.  */
                        pred = -((-pred)+1)/3;

                } else {
                    /* Just use the coefficient left of this one.  */
                    pred = data[x - 1];
                }
            } else {
                if (v > 0)
                    pred = data[(y - 1) * s->padded_width];
                else
                    pred = 0;
            }

            data[x + y * s->padded_width] += pred;
        }
}

static int subband(AVCodecContext *avctx, int *data, int level,
                   subband_t orientation) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;
    int length;
    int quant;
    int x, y;

    length = dirac_get_ue_golomb(gb);
    if (! length)
        {
            align_get_bits(gb);
        } else {
            quant = dirac_get_ue_golomb(gb);

            dirac_arith_init(&s->arith, gb, length);

            for (y = 0; y < s->codeblocksv[level]; y++)
                for (x = 0; x < s->codeblocksh[level]; x++)
                    codeblock(avctx, data, level, orientation, x, y, quant);
            dirac_arith_flush(&s->arith);
        }

    /* XXX: This should be done for intra frames only.  */
    if (level == 0)
        intra_dc_prediction(avctx, data, level, orientation);

    return 0;
}

static void decode_component(AVCodecContext *avctx, int *coeffs) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;
    int level;

   /* Align for coefficient bitstream.  */
    align_get_bits(gb);

     /* Unpack LL, level 0.  */
    subband(avctx, coeffs, 0, subband_ll);

    /* Unpack all other subbands.  */
    for (level = 1; level <= s->frame_decoding.wavelet_depth; level++) {
        /* Unpack HL, level i.  */
        subband(avctx, coeffs, level, subband_hl);

        /* Unpack LH, level i.  */
        subband(avctx, coeffs, level, subband_lh);

        /* Unpack HH, level i.  */
        subband(avctx, coeffs, level, subband_hh);
    }
 }

static int decode_intra_frame(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    int comp;
    int x,y;

    for (comp = 0; comp < 3; comp++) {
        int *coeffs;
        uint8_t *frame = s->picture.data[comp];

        if (comp == 0) {
            s->padded_width = s->padded_luma_width;
            s->padded_height = s->padded_luma_height;
        } else {
            s->padded_width = s->padded_chroma_width;
            s->padded_height = s->padded_chroma_height;
        }

        coeffs = av_malloc(s->padded_width * s->padded_height * sizeof(int));
        if (! coeffs) {
            av_log(avctx, AV_LOG_ERROR, "av_malloc() failed\n");
            return -1;
        }

        memset(coeffs, 0, s->padded_width * s->padded_height);

        decode_component(avctx, coeffs);

        /* XXX: Show the coefficients in a frame.  */
        for (x = 0; x < s->padded_width; x++)
            for (y = 0; y < s->padded_height; y++)
                frame[x + y * s->picture.linesize[comp]] = coeffs[x + y * s->padded_width];
        av_free(coeffs);
    }

    return 0;
}

static int parse_frame(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    int picnum;
    int retire;
    int filter;
    int i;
    GetBitContext *gb = s->gb;

    /* Setup decoding parameter defaults for this frame.  */
    memcpy(&s->frame_decoding, &s->decoding, sizeof(s->frame_decoding));

    s->picture.pict_type= FF_I_TYPE;
    s->picture.key_frame= 1;
    s->picture.reference = 0;

    picnum = get_bits_long(gb, 32);
    retire = dirac_get_ue_golomb(gb);

    for (i = 0; i < retire; i++)
        dirac_get_se_golomb(gb); /* XXX */

    dprintf (avctx, "Picture #%d, retire: %d\n", picnum, retire);

    align_get_bits(gb);

    /* Wavelet transform data.  */
    /* XXX: Skip all interframe stuff for now.  */

    /* Override wavelet transform parameters.  */
    if (get_bits(gb, 1)) {
        dprintf (avctx, "Non default filter\n");
        filter = dirac_get_ue_golomb(gb);
    } else {
        dprintf (avctx, "Default filter\n");
        filter = s->frame_decoding.wavelet_idx_intra;
    }

    dprintf (avctx, "Wavelet filter: %d\n", filter);

    if (filter == 0)
        dprintf(avctx, "Wavelet filter: Deslauriers-Debuc (9,3)\n");
    else
        dprintf(avctx, "Unsupported filter\n");

    /* Overrid wavelet depth.  */
    if (get_bits(gb, 1)) {
        dprintf (avctx, "Non default depth\n");
        s->frame_decoding.wavelet_depth = dirac_get_ue_golomb(gb);
    }
    dprintf(avctx, "Depth: %d\n", s->frame_decoding.wavelet_depth);

    /* Spatial partitioning.  */
    if (get_bits(gb, 1)) {
        int idx;

        dprintf (avctx, "Spatial partitioning\n");

        /* Override the default partitioning.  */
        if (get_bits(gb, 1)) {
            for (i = 0; i <= s->frame_decoding.wavelet_depth; i++) {
                s->codeblocksh[i] = dirac_get_ue_golomb(gb);
                s->codeblocksv[i] = dirac_get_ue_golomb(gb);
            }

            dprintf (avctx, "Non-default partitioning\n");

        } else {
            /* Set defaults for the codeblocks.  */
            /* XXX: Hardcoded for intra frames.  */
            for (i = 0; i <= s->frame_decoding.wavelet_depth; i++) {
                s->codeblocksh[i] = i <= 2 ? 1 : 4;
                s->codeblocksv[i] = i <= 2 ? 1 : 3;
                dprintf(avctx, "codeblock size level=%d, v=%d, h=%d\n", i,
                        s->codeblocksv[i], s->codeblocksh[i]);

            }
        }

        idx = dirac_get_ue_golomb(gb);
        dprintf(avctx, "Codeblock mode idx: %d\n", idx);
        /* XXX: Here 0, so single quant.  */
    }

    /* Rounded up to a multiple of 2^depth.  */
    s->padded_luma_width = ((s->sequence.luma_width + (1 << s->frame_decoding.wavelet_depth) - 1)
                            >> s->frame_decoding.wavelet_depth) << s->frame_decoding.wavelet_depth;
    s->padded_luma_height = ((s->sequence.luma_height + (1 << s->frame_decoding.wavelet_depth) - 1)
                             >> s->frame_decoding.wavelet_depth) << s->frame_decoding.wavelet_depth;
    s->padded_chroma_width = ((s->sequence.chroma_width + (1 << s->frame_decoding.wavelet_depth) - 1)
                              >> s->frame_decoding.wavelet_depth) << s->frame_decoding.wavelet_depth;
    s->padded_chroma_height = ((s->sequence.chroma_height + (1 << s->frame_decoding.wavelet_depth) - 1)
                               >> s->frame_decoding.wavelet_depth) << s->frame_decoding.wavelet_depth;

    return 0;
}


static int decode_frame(AVCodecContext *avctx, void *data, int *data_size, uint8_t *buf, int buf_size){
    DiracContext *s = avctx->priv_data;
    GetBitContext gb;
    AVFrame *picture = data;

    int parse_code = buf[4];
    dprintf (avctx, "Decoding frame: size=%d head=%c%c%c%c parse=%02x\n", buf_size, buf[0], buf[1], buf[2], buf[3], buf[4]);

    init_get_bits(&gb, &buf[13], (buf_size - 13) * 8);
    s->gb = &gb;

    switch (parse_code) {
    case pc_access_unit_header:
        parse_access_unit_header (avctx);

        /* Dump the header.  */
#if 1
        dump_sequence_parameters(avctx);
        dump_source_parameters(avctx);
#endif

        return 0;
    case pc_intra_ref:
        parse_frame(avctx);

        avctx->pix_fmt = PIX_FMT_YUV420P; /* XXX */

        if (avcodec_check_dimensions(avctx, s->padded_luma_width, s->padded_luma_height)) {
            av_log(avctx, AV_LOG_ERROR, "avcodec_check_dimensions() failed\n");
            return -1;
        }

        avcodec_set_dimensions(avctx, s->padded_luma_width, s->padded_luma_height);

        if (s->picture.data[0] != NULL)
            avctx->release_buffer(avctx, &s->picture);

        if (avctx->get_buffer(avctx, &s->picture) < 0) {
            av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
            return -1;
        }

        if (decode_intra_frame(avctx))
            return -1;
    }

    *data_size = sizeof(AVFrame);
    *picture = s->picture;

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
