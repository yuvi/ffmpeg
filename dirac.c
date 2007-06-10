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



typedef struct DiracContext {
    int access_unit;
    unsigned int profile;
    unsigned int level;

    unsigned int luma_width;
    unsigned int luma_height;
    unsigned int depth;

    int framerate_numer;
    int framerate_denom;

    unsigned int clean_width;
    unsigned int clean_height;
    unsigned int clean_left_offset;
    unsigned int clean_right_offset;

    int codeblocksh[7]; /* XXX: 7 levels.  */
    int codeblocksv[7]; /* XXX: 7 levels.  */
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
    pc_eos = 0x10,
    pc_aux_data = 0x20,
    pc_padding = 0x60,
    pc_intra_ref = 0x0c
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

static int parse_access_unit_header (AVCodecContext *avctx, GetBitContext *gb) {
    DiracContext *s = avctx->priv_data;
    uint32_t version_major;
    uint32_t version_minor;
    uint32_t video_format;

    /* Parse parameters.  */

    /* XXX: Picture number of next frame.  */
    skip_bits_long(gb, 32);
    version_major = dirac_golomb(gb);
    version_minor = dirac_golomb(gb);
    s->profile = dirac_golomb(gb);
    s->level = dirac_golomb(gb);
    /* XXX: Check the version (0.6).  The current Dirac encoders don't
       set this value properly anyways.  */
    dprintf (avctx, "Access unit header: Version %d.%d\n", version_major, version_minor);
    dprintf (avctx, "Profile: %d, Level: %d\n", s->profile, s->level);

    /* Sequence parameters.  */
    video_format = dirac_golomb(gb);
    dprintf (avctx, "Video format: %d\n", video_format);

    /* XXX: Fill in defaults.  */

    /* Set custom dimensions.  */
    if (get_bits(gb, 1)) {
        s->luma_width = dirac_golomb(gb);
        s->luma_height = dirac_golomb(gb);
    }

    /* Set chroma format.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_golomb(gb);
        dprintf (avctx, "Chroma index: %d\n", idx);
        /* XXX: Set chroma dimensions by scaling luma dimensions.  */

    }

    if (get_bits(gb, 1)) {
        s->depth = dirac_golomb(gb);
        dprintf (avctx, "override depth: %d\n", s->depth);
    }

    dprintf(avctx, "Video mode: %dx%d@%d\n", s->luma_width, s->luma_height, s->depth);

    /* Access Unit Source parameters.  */

    if (get_bits(gb, 1)) {
        /* Interlace.  */
        dprintf(avctx, "Interlace!\n");
        /* XXX: Currently not supported */
    }

    /* Framerate.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_golomb(gb);
        if (! idx) {
            s->framerate_numer = dirac_golomb(gb);//svq3_get_ue_golomb(gb);
            s->framerate_denom = dirac_golomb(gb);//svq3_get_ue_golomb(gb);
            dprintf (avctx, "Framerate index: %d/%d = %f\n",
                     s->framerate_numer, s->framerate_denom,
                     (double) s->framerate_numer / s->framerate_denom);

        }
        /* XXX: else use presets from Appendix E.  */
    }
    /* Clean area.  */
    if (get_bits(gb, 1)) {
        s->clean_width = dirac_golomb(gb);
        s->clean_height = dirac_golomb(gb);
        s->clean_left_offset = dirac_golomb(gb);
        s->clean_right_offset = dirac_golomb(gb);
        dprintf (avctx, "Clean area %dx%d %d:%d\n", s->clean_width, s->clean_height,
                 s->clean_left_offset, s->clean_right_offset);
    }

    /* Signal range.  */
    if (get_bits(gb, 1)) {
        dprintf(avctx, "Signal range flag\n");
    }

    /* Color spec.  */
    if (get_bits(gb, 1)) {
        int idx = dirac_golomb(gb);

        dprintf(avctx, "Color specification flag\n");
        dprintf (avctx, "Color spec idx: %d\n", idx);
        /* XXX: preset */

        if (idx == 0) {
            /* Color primaries.  */
            if (get_bits(gb, 1)) {
                dprintf(avctx, "Color primaries flag\n");
            }

            /* Transfer function.  */
            if (get_bits(gb, 1)) {
                dprintf(avctx, "Transfer function flag\n");
            }
        }
    }

    dprintf (avctx, "Header read!\n");

    return 0;
}

/* Arithmetic decoding.  XXX: Based on the pseudocode from the spec,
   use ffmpeg code or integrate this properly into ffmpeg if nothing
   is there.  */

/* XXX: Ugly and will be cleaned up.  Move into DiracContext.  */
static int arith_low;
static int arith_range;
static int arith_code;
static int arith_bits_left;

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

/* XXX: Check the spec again on this.  */
typedef int arith_context_t[1];
static arith_context_t arith_contexts[ARITH_CONTEXT_COUNT];

static void arith_init (GetBitContext *gb, int length) {
    int i;

    arith_bits_left = 8 * length;
    arith_low = 0;
    arith_range = 0x10000;
    arith_code = get_bits_long(gb, 15);

    /* Initialize contexts.  */
    for (i = 0; i < ARITH_CONTEXT_COUNT; i++) {
        arith_contexts[i][1] = 0x8000;
    }
}

static void arith_renormalize (GetBitContext *gb) {
    if ((arith_low + arith_range - 1) && arith_low >= 0x8000) {
        arith_code &= 0x4000;
        arith_low &= 0x4000;
    }
    arith_low <<= 1;
    arith_range <<= 1;
    arith_low &= 0xFFFF;
    arith_code <<= 1;
    if (arith_bits_left > 0) {
        arith_code += get_bits (gb, 1);
        arith_bits_left--;
    }
    arith_code &= 0xffff;
}

static int arith_lookup[256] = {
    0, 2, 5, 8, 11, 15, 20, 24,
    29, 35, 41, 47, 53, 60, 67, 74,
    82, 89, 97, 106, 114, 123, 132, 141,
    150, 160, 170, 180, 190, 201, 211, 222,
    233, 244, 256, 267, 279, 291, 303, 315,
    327, 340, 353, 366, 379, 392, 405, 419,
    433, 447, 461, 475, 489, 504, 518, 533,
    548, 563, 578, 593, 609, 624, 640, 656,
    672, 688, 705, 721, 738, 754, 771, 788,
    805, 822, 840, 857, 875, 892, 910, 928,
    946, 964, 983, 1001, 1020, 1038, 1057, 1076,
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
    1151, 1114, 1077, 1037,  995, 952, 906, 857,
    805, 750,   690,  625,  553, 471, 376, 255
};

static int arith_get_bit (GetBitContext *gb, int context) {
    int prob_zero = arith_contexts[context][1];
    int count;
    int range_times_prob;
    int ret;

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
        arith_contexts[context][1] -= arith_lookup[arith_contexts[context][1] >> 8];
    else
        arith_contexts[context][1] -= arith_lookup[255 - (arith_contexts[context][1] >> 8)];

    while (arith_range <= 0x4000)
        arith_renormalize (gb);
    return ret;
}

struct context_set {
    int follow[16]; /* XXX */
    int follow_length;
    int data;
    int sign;
};

struct context_set context_sets_waveletcoeff[12] = {
    {
        /* Parent = 0, Zero neighbourhood, sign predict 0 */
        .follow = { ARITH_CONTEXT_ZPZN_F1, ARITH_CONTEXT_ZP_F2,
                    ARITH_CONTEXT_ZP_F3, ARITH_CONTEXT_ZP_F4,
                    ARITH_CONTEXT_ZP_F5, ARITH_CONTEXT_ZP_F6, 0 },
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

static int follow_context (int index, struct context_set *context_set) {
    int pos;
    pos = (index < context_set->follow_length ? index
           : context_set->follow_length) - 1;
    return context_set->follow[pos];
}

static int arith_read_uint (GetBitContext *gb, struct context_set *context_set) {
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

static void codeblock (AVCodecContext *avctx, GetBitContext *gb, int level, int x, int y) {
    DiracContext *s = avctx->priv_data;
    int blockcnt = s->codeblocksh[level] * s->codeblocksv[level];
    int zero = 0;

    if (blockcnt != 1) {
        /* Determine if this codeblock is a zero block.  */
        zero = arith_get_bit(gb, ARITH_CONTEXT_ZERO_BLOCK);
    }

    if (zero)
        return; /* All coefficients remain 0.  */

    /* XXX: Quantization.  */
}

static int subband (AVCodecContext *avctx, GetBitContext *gb, int level, subband_t band) {
    DiracContext *s = avctx->priv_data;
    int length;
    int quant;
    int x, y;

    length = dirac_golomb(gb);
    if (! length)
        {
            align_get_bits(gb);
            dprintf (avctx, "Zero subband\n");

            return 0;
        } else {
            quant = dirac_golomb(gb);
            dprintf (avctx, "Length: %d, quant: %d\n", length, quant);

            arith_init(gb, length);

            for (y = 0; y < s->codeblocksv[level]; y++)
                for (x = 0; x < s->codeblocksh[level]; x++)
                    codeblock(avctx, gb, level, x, y);
            arith_flush(gb);
        }

    return 0;
}


static int decode_intra_frame(AVCodecContext *avctx, GetBitContext *gb,
                              void *data, int *data_size) {
    DiracContext *s = avctx->priv_data;
    int picnum;
    int retire;
    int i;

    picnum = get_bits_long(gb, 32);
    retire = dirac_golomb(gb);

    dprintf (avctx, "Picture #%d, retire: %d\n", picnum, retire);

    align_get_bits(gb);

    /* Wavelet transform data.  */
    /* XXX: Skip all interframe stuff for now.  */

    /* Wavelet transform parameters.  */
    if (get_bits(gb, 1)) {
        dprintf (avctx, "Non default filter\n");
    } else {
        dprintf (avctx, "Default filter, select (9, 3) for intra frame\n");
    }

    /* Wavelet depth.  */
    if (get_bits(gb, 1)) {
        dprintf (avctx, "Non default depth\n");
    }
    /* XXX: What's the default depth?  reference implementation: 4 */

    /* Spatial partitioning.  */
    if (get_bits(gb, 1)) {
        int idx;

        dprintf (avctx, "Spatial partitioning\n");

        if (get_bits(gb, 1)) {
            dprintf (avctx, "Non-default partitioning\n");
        } else {
            /* Set defaults for the codeblocks.  */
            for (i = 0; i <= s->level; i++) {
                s->codeblocksv[i] = i <= 2 ? 1 : 4;
                s->codeblocksh[i] = i <= 2 ? 1 : 3;
            }
        }

        idx = dirac_golomb(gb);
        dprintf(avctx, "Codeblock mode idx: %d\n", idx);
        /* XXX: Here 0, so single quant.  */
    }

    /* Coefficient unpacking.  */

    /* Unpack LL, level 0.  */
    subband (avctx, gb, 0, subband_ll);

    return 0;
}


static int decode_frame(AVCodecContext *avctx, void *data, int *data_size, uint8_t *buf, int buf_size){
    DiracContext *s = avctx->priv_data;
    GetBitContext gb;

    int parse_code = buf[4];
    dprintf (avctx, "Decoding frame: size=%d head=%c%c%c%c parse=%02x\n", buf_size, buf[0], buf[1], buf[2], buf[3], buf[4]);

    init_get_bits(&gb, &buf[13], (buf_size - 13) * 8);

    switch (parse_code) {
    case pc_access_unit_header:
        return parse_access_unit_header (avctx, &gb);
    case pc_intra_ref:
        return decode_intra_frame(avctx, &gb, data, data_size);
    }

    *data_size = 0;
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
