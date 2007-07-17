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

/* TODO:

- Compare svq3 golomb to Dirac spec.
- Compare the CABAC implementation to the Dirac spec
- Clean things up!!!
- Make sure the coding style is correct
- Fill in the missing bits

*/

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

    int padded_width;
    int padded_height;
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

static int dirac_golomb(GetBitContext *gb) {
    int val = 1;
    while (! get_bits (gb, 1)) {
        val <<= 1;
        if (get_bits (gb, 1))
            val++;
    }
    val--;
    return val;
}


static int dirac_golomb_sign(GetBitContext *gb) {
    int val = dirac_golomb(gb);
    if (val)
        if (get_bits(gb, 1))
            val = -val;
    return val;
}

static void parse_sequence_parameters(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;

    /* Override the luma dimensions.  */
    if (get_bits(gb, 1)) {
        s->sequence.luma_width = dirac_golomb(gb);
        s->sequence.luma_height = dirac_golomb(gb);
    }

    /* Override the chroma format.  */
    if (get_bits(gb, 1)) {
        s->sequence.chroma_format = dirac_golomb(gb);
        dprintf (avctx, "Chroma index: %d\n", s->sequence.chroma_format);
    }

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
    if (get_bits(gb, 1)) {
        s->sequence.video_depth = dirac_golomb(gb);
        dprintf (avctx, "override depth: %d\n", s->sequence.video_depth);
    }

    dprintf(avctx, "Video mode: %dx%d@%d\n", s->sequence.luma_width, s->sequence.luma_height, s->sequence.video_depth);
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

        dprintf(avctx, "Interlace!\n");
    }

    /* Framerate.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_golomb(gb);
        if (! idx) {
            s->source.frame_rate.num = dirac_golomb(gb);
            s->source.frame_rate.den = dirac_golomb(gb);
            dprintf (avctx, "Framerate index: %d/%d = %f\n",
                     s->source.frame_rate.num, s->source.frame_rate.den,
                     (double) s->source.frame_rate.num / s->source.frame_rate.den);

        } else {
            /* Use a pre-set framerate.  */
            s->source.frame_rate = preset_frame_rates[idx - 1];
        }
    }

    /* Override aspect ratio.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_golomb(gb);
        if (! idx) {
            s->source.aspect_ratio.num = dirac_golomb(gb);
            s->source.aspect_ratio.den = dirac_golomb(gb);
        } else {
            /* Use a pre-set aspect ratio.  */
            s->source.aspect_ratio = preset_aspect_ratios[idx - 1];
        }
    }

    /* Override clean area.  */
    if (get_bits(gb, 1)) {
        s->source.clean_width = dirac_golomb(gb);
        s->source.clean_height = dirac_golomb(gb);
        s->source.clean_left_offset = dirac_golomb(gb);
        s->source.clean_right_offset = dirac_golomb(gb);
        dprintf (avctx, "Clean area %dx%d %d:%d\n", s->source.clean_width, s->source.clean_height,
                 s->source.clean_left_offset, s->source.clean_right_offset);
    }

    /* Override signal range.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_golomb(gb);
        if (! idx) {
            s->source.luma_offset = dirac_golomb(gb);
            s->source.luma_excursion = dirac_golomb(gb);
            s->source.chroma_offset = dirac_golomb(gb);
            s->source.chroma_excursion = dirac_golomb(gb);
        } else {
            /* Use a pre-set signal range.  */
            s->source.luma_offset = preset_luma_offset[idx - 1];
            s->source.luma_excursion = preset_luma_excursion[idx - 1];
            s->source.chroma_offset = preset_chroma_offset[idx - 1];
            s->source.chroma_excursion = preset_chroma_excursion[idx - 1];
        }
        dprintf(avctx, "Signal range flag: %d\n", idx);
    }

    /* Color spec.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_golomb(gb);

        s->source.color_primaries = preset_primaries[idx];
        s->source.k_r = preset_kr[preset_matrix[idx]];
        s->source.k_b = preset_kb[preset_matrix[idx]];
        s->source.transfer_function = preset_transfer_func[idx];

        /* XXX: color_spec?  */

        if (! idx) {
            /* Color primaries.  */
            if (get_bits(gb, 1)) {
                int primaries_idx = dirac_golomb(gb);
                s->source.color_primaries = preset_primaries[primaries_idx];

                dprintf(avctx, "Color primaries flag\n");
            }

            /* Override matrix.  */
            if (get_bits(gb, 1)) {
                int matrix_idx = dirac_golomb(gb);

                s->source.k_r = preset_kr[preset_matrix[matrix_idx]];
                s->source.k_b = preset_kb[preset_matrix[matrix_idx]];

                dprintf(avctx, "matrix flag\n");

            }

            /* Transfer function.  */
            if (get_bits(gb, 1)) {
                int transfer_idx = dirac_golomb(gb);
                s->source.transfer_function = preset_transfer_func[transfer_idx];

                dprintf(avctx, "Transfer function flag\n");
            }
        } else {
        }

        dprintf(avctx, "Color specification flag\n");
        dprintf (avctx, "Color spec idx: %d\n", idx);
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

    version_major = dirac_golomb(gb);
    version_minor = dirac_golomb(gb);
    /* XXX: Don't check the version yet, existing encoders do not yet
       set this to a sane value (0.6 at the moment).  */

    /* XXX: Not yet documented in the spec.  This is actually the main
       thing that is missing.  */
    s->profile = dirac_golomb(gb);
    s->level = dirac_golomb(gb);

    dprintf (avctx, "Access unit header: Version %d.%d\n",
             version_major, version_minor);
    dprintf (avctx, "Profile: %d, Level: %d\n", s->profile, s->level);

    video_format = dirac_golomb(gb);
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

    dprintf (avctx, "Header read!\n");

    return 0;
}

/* Arithmetic decoding.  XXX: Based on the pseudocode from the spec,
   use ffmpeg code or integrate this properly into ffmpeg if nothing
   is there.  */

/* XXX: Ugly and will be cleaned up.  Move into DiracContext.  */
static unsigned int arith_low;
static unsigned int arith_range;
static unsigned int arith_code;
static unsigned int arith_bits_left;

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

#define ARITH_CONTEXT_COUNT (ARITH_CONTEXT_DC_SIGN + 1)

static unsigned int arith_contexts[ARITH_CONTEXT_COUNT];

static void arith_init (AVCodecContext *avctx, GetBitContext *gb, int length) {
    int i;

    align_get_bits(gb);
    arith_bits_left = 8 * length - 16;
    arith_low = 0;
    arith_range = 0x10000;
    arith_code = get_bits_long(gb, 16);

    /* Initialize contexts.  */
    for (i = 0; i < ARITH_CONTEXT_COUNT; i++) {
        arith_contexts[i] = 0x8000;
    }
}

static unsigned int arith_lookup[256] = {
    0,    2,    5,    8,    11,   15,   20,   24,
    29,   35,   41,   47,   53,   60,   67,   74,
    82,   89,   97,   106,  114,  123,  132,  141,
    150,  160,  170,  180,  190,  201,  211,  222,
    233,  244,  256,  267,  279,  291,  303,  315,
    327,  340,  353,  366,  379,  392,  405,  419,
    433,  447,  461,  475,  489,  504,  518,  533,
    548,  563,  578,  593,  609,  624,  640,  656,
    672,  688,  705,  721,  738,  754,  771,  788,
    805,  822,  840,  857,  875,  892,  910,  928,
    946,  964,  983,  1001, 1020, 1038, 1057, 1076,
    1095, 1114, 1133, 1153, 1172, 1192, 1211, 1231,
    1251, 1271, 1291, 1311, 1332, 1352, 1373, 1393,
    1414, 1435, 1456, 1477, 1498, 1520, 1541, 1562,
    1584, 1606, 1628, 1649, 1671, 1694, 1716, 1738,
    1760, 1783, 1806, 1828, 1851, 1874, 1897, 1920,
    1935, 1942, 1949, 1955, 1961, 1968, 1974, 1980,
    1985, 1991, 1996, 2001, 2006, 2011, 2016, 2021,
    2025, 2029, 2033, 2037, 2040, 2044, 2047, 2050,
    2053, 2056, 2058, 2061, 2063, 2065, 2066, 2068,
    2069, 2070, 2071, 2072, 2072, 2072, 2072, 2072,
    2072, 2071, 2070, 2069, 2068, 2066, 2065, 2063,
    2060, 2058, 2055, 2052, 2049, 2045, 2042, 2038,
    2033, 2029, 2024, 2019, 2013, 2008, 2002, 1996,
    1989, 1982, 1975, 1968, 1960, 1952, 1943, 1934,
    1925, 1916, 1906, 1896, 1885, 1874, 1863, 1851,
    1839, 1827, 1814, 1800, 1786, 1772, 1757, 1742,
    1727, 1710, 1694, 1676, 1659, 1640, 1622, 1602,
    1582, 1561, 1540, 1518, 1495, 1471, 1447, 1422,
    1396, 1369, 1341, 1312, 1282, 1251, 1219, 1186,
    1151, 1114, 1077, 1037, 995,  952,  906,  857,
    805, 750,   690,  625,  553,  471,  376,  255
};

static int arith_get_bit (GetBitContext *gb, int context) {
    unsigned int prob_zero = arith_contexts[context];
    unsigned int count;
    unsigned int range_times_prob;
    unsigned int ret;

    count = arith_code - arith_low;
    range_times_prob = (arith_range * prob_zero) >> 16;
    if (count >= range_times_prob) {
        ret = 1;
        arith_low += range_times_prob;
        arith_range -= range_times_prob;
    } else {
        ret = 0;
        arith_range = range_times_prob;
    }

    /* Update contexts. */
    if (ret)
        arith_contexts[context] -= arith_lookup[arith_contexts[context] >> 8];
    else
        arith_contexts[context] += arith_lookup[255 - (arith_contexts[context] >> 8)];

    while (arith_range <= 0x4000) {
        if (((arith_low + arith_range - 1)^arith_low) >= 0x8000) {
            arith_code ^= 0x4000;
            arith_low ^= 0x4000;
        }
        arith_low <<= 1;
        arith_range <<= 1;
        arith_low &= 0xFFFF;
        arith_code <<= 1;
        if (arith_bits_left > 0) {
            arith_code |= get_bits (gb, 1);
            arith_bits_left--;
        }
        else {
            /* Get default: */
            arith_code |= 1;
        }
        arith_code &= 0xffff;
    }

    return ret;
}

struct context_set {
    unsigned int follow[6];
    unsigned int follow_length;
    unsigned int data;
    unsigned int sign;
};

struct context_set context_sets_waveletcoeff[12] = {
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

static unsigned int follow_context (int index, struct context_set *context_set) {
    int pos;
    pos = (index < context_set->follow_length ? index
           : context_set->follow_length - 1);
    return context_set->follow[pos];
}

static unsigned int arith_read_uint (GetBitContext *gb, struct context_set *context_set) {
    int ret = 1;
    int index = 0;

    while (arith_get_bit (gb, follow_context(index, context_set)) == 0) {
        ret <<= 1;
        if (arith_get_bit (gb, context_set->data))
            ret++;
        index++;
    }
    ret--;
    return ret;
}

static int arith_read_int (GetBitContext *gb, struct context_set *context_set) {
    int ret = arith_read_uint (gb, context_set);
    if (ret != 0 && arith_get_bit(gb, context_set->sign))
        ret = -ret;
    return ret;
}

static void arith_flush(GetBitContext *gb) {
    skip_bits_long(gb, arith_bits_left);
    arith_bits_left = 0;
}

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
            return (data[x + (y - 1) * s->padded_width] < 0) ? -1 : 1;
        }
    case subband_lh:
        if (h == 0)
            return 0;
        else {
            if (data[x + y * s->padded_width - 1] == 0) return 0;
            return (data[x + y * s->padded_width - 1] < 0) ? -1 : 1;
        }
    }

    return 0;
}

static void coeff_unpack(AVCodecContext *avctx, int *data, int level,
                         subband_t orientation, int v, int h) {
    int parent = 0;
    int nhood;
    int sign_pred;
    int idx;
    int coeff;
    struct context_set *context;
    DiracContext *s = avctx->priv_data;
    int vdata, hdata;

    /* The value of the pixel belonging to the lower level.  */
    if (level >= 2) {
        int x = coeff_posx(avctx, level - 1, orientation, h >> 1);
        int y = coeff_posy(avctx, level - 1, orientation, v >> 1);
        parent = data[s->padded_width * y + x] != 0;
    }

    /* XXX: this is what the reference implementation effectively
       does, although this does not seem to comply with the spec.  I
       have asked the Dirac BBC why this seems to be required.  */
    if (level < 2)
        parent = 1;

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

    coeff = arith_read_int(s->gb, context);
    vdata = coeff_posy(avctx, level, orientation, v);
    hdata = coeff_posx(avctx, level, orientation, h);
    data[hdata + vdata * s->padded_width] = coeff;
}

static void codeblock(AVCodecContext *avctx, int *data, int level,
                      subband_t orientation, int width, int height, int x, int y) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;
    int blockcnt = s->codeblocksh[level] * s->codeblocksv[level];
    int zero = 0;

    int left = (subband_width(avctx, level) * x) / s->codeblocksh[level];
    int right = (subband_width(avctx, level) * (x + 1)) / s->codeblocksh[level];
    int top = (subband_height(avctx, level) * y) / s->codeblocksv[level];
    int bottom = (subband_height(avctx, level) * (y + 1)) / s->codeblocksv[level];

    int v, h;

    if (blockcnt != 1 && orientation != subband_ll) {
        /* Determine if this codeblock is a zero block.  */
        zero = arith_get_bit(gb, ARITH_CONTEXT_ZERO_BLOCK);
    }

    if (zero)
        return; /* All coefficients remain 0.  */

    /* XXX: This matches the reference implementation, check the
       spec.  */
    for (v = top; v < bottom; v++)
        for (h = left; h < right; h++)
            coeff_unpack(avctx, data, level, orientation, v, h);

    /* XXX: Quantization.  */
}

static void intra_dc_prediction(AVCodecContext *avctx, int *data, int level,
                                int width, int height, subband_t orientation) {
    int pred;
    int h, v;

    for (v = 0; v < subband_width(avctx, 0); v++)
        for (h = 0; h < subband_height(avctx, 0); h++) {
            int x = coeff_posx(avctx, level, orientation, h);
            int y = coeff_posy(avctx, level, orientation, v);

            if (h > 0) {
                if (v > 0) {
                    /* Use 3 coefficients for prediction.  */
                    pred = (data[x + y * width - 1]
                            + data[x + (y - 1) * width]
                            + data[x + (y - 1) * width - 1]) / 3;
                } else {
                    /* Just use the coefficient left of this one.  */
                    pred = data[x + y * width - 1];
                }
            } else {
                if (v > 0)
                    pred = data[x + (y - 1) * width];
                else
                    pred = 0;
            }

            data[x + y * width] += pred;
        }
}

static int subband(AVCodecContext *avctx, int *data, int level,
                   int width, int height, subband_t orientation) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;
    int length;
    int quant;
    int x, y;

    length = dirac_golomb(gb);
    if (! length)
        {
            align_get_bits(gb);
        } else {
            quant = dirac_golomb(gb);

            arith_init(avctx, gb, length);

            for (y = 0; y < s->codeblocksv[level]; y++)
                for (x = 0; x < s->codeblocksh[level]; x++)
                    codeblock(avctx, data, level, orientation, width, height, x, y);
            arith_flush(gb);
        }

    /* XXX: This should be done for intra frames only.  */
    if (level == 0)
        intra_dc_prediction(avctx, data, level, width, height, orientation);

    return 0;
}

static int inline coeff_quant_factor(int idx) {
    int base;
    if (idx < 0)
        idx = 0;
    base = 1 << (idx / 4);
    switch(idx & 3) {
    case 0:
        return base << 2;
    case 1:
        return (503829 * base + 52958) / 105917;
    case 2:
        return (665857 * idx + 58854) / 117708;
    case 3:
        return (440253 * base + 32722) / 65444;
    }
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

static int decode_intra_frame(AVCodecContext *avctx) {
    DiracContext *s = avctx->priv_data;
    GetBitContext *gb = s->gb;
    int width = s->sequence.luma_width;
    int height = s->sequence.luma_height;
    int coeffs[s->padded_width * s->padded_height];
    uint8_t *frame = s->picture.data[0];
    int level;
    int x,y;

    /* Coefficient unpacking.  */

    memset(coeffs, 0, sizeof(coeffs));

    dprintf(avctx, "width: %d, height: %d, padded width: %d, padded height: %d\n",
            width, height, s->padded_width, s->padded_height);

   /* Align for coefficient bitstream.  */
    align_get_bits(gb);

     /* Unpack LL, level 0.  */
    subband(avctx, coeffs, 0, width, height, subband_ll);

    /* Unpack all other subbands.  */
    for (level = 1; level <= s->frame_decoding.wavelet_depth; level++) {
        /* Unpack HL, level i.  */
        subband(avctx, coeffs, level, width, height, subband_hl);

        /* Unpack LH, level i.  */
        subband(avctx, coeffs, level, width, height, subband_lh);

        /* Unpack HH, level i.  */
        subband(avctx, coeffs, level, width, height, subband_hh);
    }

    /* XXX: Show the coefficients in a frame.  */
    for (x = 0; x < s->padded_width; x++)
        for (y = 0; y < s->padded_height; y++)
            frame[x + y * s->picture.linesize[0]] = coeffs[x + y * s->padded_width];

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
    retire = dirac_golomb(gb);

    for (i = 0; i < retire; i++)
        dirac_golomb_sign(gb); /* XXX */

    dprintf (avctx, "Picture #%d, retire: %d\n", picnum, retire);

    align_get_bits(gb);

    /* Wavelet transform data.  */
    /* XXX: Skip all interframe stuff for now.  */

    /* Override wavelet transform parameters.  */
    if (get_bits(gb, 1)) {
        dprintf (avctx, "Non default filter\n");
        filter = dirac_golomb(gb);
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
        s->frame_decoding.wavelet_depth = dirac_golomb(gb);
    }
    dprintf(avctx, "Depth: %d\n", s->frame_decoding.wavelet_depth);

    /* Spatial partitioning.  */
    if (get_bits(gb, 1)) {
        int idx;

        dprintf (avctx, "Spatial partitioning\n");

        /* Override the default partitioning.  */
        if (get_bits(gb, 1)) {
            for (i = 0; i <= s->frame_decoding.wavelet_depth; i++) {
                s->codeblocksh[i] = dirac_golomb(gb);
                s->codeblocksv[i] = dirac_golomb(gb);
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

        idx = dirac_golomb(gb);
        dprintf(avctx, "Codeblock mode idx: %d\n", idx);
        /* XXX: Here 0, so single quant.  */
    }

    /* Rounded up to a multiple of 2^depth.  */
    s->padded_width = ((s->sequence.luma_width + (1 << s->frame_decoding.wavelet_depth) - 1)
                       >> s->frame_decoding.wavelet_depth) << s->frame_decoding.wavelet_depth;
    s->padded_height = ((s->sequence.luma_height + (1 << s->frame_decoding.wavelet_depth) - 1)
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

        return 0;
    case pc_intra_ref:
        parse_frame(avctx);

        avctx->pix_fmt = PIX_FMT_YUV444P; /* XXX */

        if (avcodec_check_dimensions(avctx, s->padded_width, s->padded_height)) {
            av_log(avctx, AV_LOG_ERROR, "avcodec_check_dimensions() failed\n");
            return -1;
        }

        avcodec_set_dimensions(avctx, s->padded_width, s->padded_height);

        if (s->picture.data[0] != NULL)
            avctx->release_buffer(avctx, &s->picture);

        if (avctx->get_buffer(avctx, &s->picture) < 0) {
            av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
            return -1;
        }

        decode_intra_frame(avctx);
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
