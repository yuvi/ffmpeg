/*
 * Copyright (C) 2003-2004 the ffmpeg project
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
 * @file libavcodec/vp3.c
 * On2 VP3 Video Decoder
 *
 * VP3 Video Decoder by Mike Melanson (mike at multimedia.cx)
 * For more information about the VP3 coding process, visit:
 *   http://wiki.multimedia.cx/index.php?title=On2_VP3
 *
 * Theora decoder by Alex Beregszaszi
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "avcodec.h"
#include "dsputil.h"
#include "get_bits.h"

#include "vp3data.h"
#include "xiph.h"

#undef printf

struct vp3_block {
    int16_t dc;
    uint8_t qpi;
    uint8_t modes;
};

typedef struct {
    AVCodecContext *avctx;
    int theora, theora_tables;
    int version;
    int width, height;
    AVFrame golden_frame;
    AVFrame last_frame;
    AVFrame current_frame;
    int keyframe;
    DSPContext dsp;
    int flipped_image;

    int         data_offset[3];
    int         linesize[3];
    int         h_edge_pos;
    int         v_edge_pos;
    int         chroma_x_shift;
    int         chroma_y_shift;

    int         superblock_width[3];
    int         superblock_height[3];
    int         superblock_count[3];
    int         num_superblocks;        ///< sum total of superblock_count

    int         block_width[3];         ///< superblock_width*4  (includes nonexistant blocks)
    int         block_height[3];        ///< superblock_height*4 (includes nonexistant blocks)
    int         num_blocks;

    /**
     * maps each coded block in coding order to the corresponding index of block_dc
     */
    int         *coded_blocks[3];
    /**
     * maps every block in coding order to the corresponding index of block_dc
     * -1 means the block doesn't exist
     */
    int         *all_blocks[3];

    /**
     * block_width x block_height array, coding order
     * contains DC, QP index, and coding modes for each block
     */
    struct vp3_block *blocks[3];

    /**
     * number of blocks that contain DCT coefficients at the given level or higher
     */
    int         num_coded_blocks[3][64];

    /**
     * Each dct token is 5 bits, plus up to 12 bits of extradata. (17 bits...)
     * We store them raw rather than decode to coefficients because EOB and
     * zero runs mean that it's not easy to tell which block gets which coeff.
     * This requires that we do IDCT in hilbert order, which means the minimum
     * slice height be 64 for 4:2:0 and 32 otherwise.
     * Alternative approach (which this replaces) is to use a linked list, but
     * that has significant overhead in both reordering when reading the tokens
     * and when doing the IDCT.
     */
    int16_t     *dct_tokens[3][64];
    int16_t     *dct_tokens_base;




    uint8_t     *superblock_coding[3];  ///< chroma planes are contiguous to luma

    int         macroblock_count;
    /**
     * array in coded order, possible values are MODE_*
     * macroblocks that don't exist are set to MODE_NONEXISTANT
     * Only luma macroblocks have coding modes, so this array does not have chroma
     */
    uint8_t     *macroblock_coding;
#define MODE_INTER_NO_MV      0
#define MODE_INTRA            1
#define MODE_INTER_PLUS_MV    2
#define MODE_INTER_LAST_MV    3
#define MODE_INTER_PRIOR_LAST 4
#define MODE_USING_GOLDEN     5
#define MODE_GOLDEN_MV        6
#define MODE_INTER_FOURMV     7
#define CODING_MODE_COUNT     8

#define MODE_NONEXISTANT      8   ///< internal mode

    /**
     * block_width[plane] by block_height[plane] array, in coding order
     * the 3 planes must be contiguous in the same array
     *
     * bit 0-1: qp index     (& BLOCK_QP_INDEX)
     * bit 2: block is coded (& BLOCK_IS_CODED)
     * bit 3: block is intra (& BLOCK_IS_INTRA)
     * bit 4: block is a golden frame ref (& BLOCK_USES_GOLDEN)
     * bit 7: block doesn't really exist (it's here to make superblock <-> block
     *      mapping easier.) This is set during init and should never be changed
     *
     * If the block is not coded (but does exist) block_coding must have the value 0
     *
     * For luma, only true/false is checked; macroblock_coding is used
     *  to differentiate intra/inter
     * For chroma, 0/MODE_INTRA/whatever is needed to differentiate
     *  uncoded/intra/inter since macroblock_coding can't easily be used
     */
    uint8_t     *block_coding[3];
#define BLOCK_QP_INDEX 3
#define BLOCK_IS_CODED 4
#define BLOCK_IS_INTRA 8
#define BLOCK_USES_GOLDEN 16        // needed for DC prediction
#define BLOCK_NONEXISTANT 0x80

    /**
     * Array to map a macroblock index to a chroma block index, both in coding order.
     * This has one entry per macroblock in 4:2:0, two entries for 4:2:2,
     * and is unused in 4:4:4, as luma and chroma have the same coding order.
     */
    int         *mb_to_uvblk_i;

    int         mv_i;               ///< index into mvs array
    int8_t      *mvs;               ///< array of all motion vectors in coded order
    int         num_mvs;
    int8_t      last_mv[2];
    int8_t      prior_last_mv[2];

    /**
     * coeff_type represents the token used to decode coefficient data
     *  in hilbert order
     * 0 - one coefficient, in coeff_data for AC or dc_val for DC
     * 1 - EOB run, run length is stored in coeff_data
     * 2+- zero run, coeff_type is (run length)<<1 and the next coefficient
     *     is stored in coeff_data
     */
    uint8_t     *coeff_type[3][64];
    int16_t     *coeff_data[3][64];
    int16_t     *dc_val[3];

    int16_t     *dc_val_base;
    uint8_t     *coeff_tokens_base;
    int16_t     *coeff_data_base;

    uint8_t     *edge_emu_buffer;

    // precalculated offset in the frame from the start of the macroblock
    // [macroblock][block]
    int         luma_offset[4][4];

    // precalculated offset in the frame from the start of the superblock
    // [macroblock][block]
    int         chroma_offset[4][4];

    // precalculated change in the frame from the last macroblock
    // [plane][macroblock]
    int         hilbert_mb_delta[2][4];

    // [plane][inter][qpi][coeff]
    int16_t     qmat[3][2][3][64];


    DECLARE_ALIGNED_16(DCTELEM, block[64]);
    DECLARE_ALIGNED_8(int, bounding_values_array[256+2]);

    ScanTable scantable;

    /* tables */
    uint16_t coded_dc_scale_factor[64];
    uint32_t coded_ac_scale_factor[64];
    uint8_t base_matrix[384][64];
    uint8_t qr_count[2][3];
    uint8_t qr_size [2][3][64];
    uint16_t qr_base[2][3][64];
    int pixel_addresses_initialized;

    VLC dc_vlc[16];
    VLC ac_vlc_1[16];
    VLC ac_vlc_2[16];
    VLC ac_vlc_3[16];
    VLC ac_vlc_4[16];

    VLC superblock_run_length_vlc;
    VLC fragment_run_length_vlc;
    VLC mode_code_vlc;
    VLC motion_vector_vlc;

    int qps[3];
    int nqps;
    int last_qps[3];

    /* Huffman decode */
    int hti;
    unsigned int hbits;
    int entries;
    int huff_code_size;
    uint16_t huffman_table[80][32][2];

    uint8_t filter_limit_values[64];
} Vp3DecodeContext;

#define SB_NOT_CODED        0
#define SB_PARTIALLY_CODED  1
#define SB_FULLY_CODED      2

/* There are 6 preset schemes, plus a free-form scheme */
static const uint8_t ModeAlphabet[6][CODING_MODE_COUNT] =
{
    /* scheme 1: Last motion vector dominates */
    {    MODE_INTER_LAST_MV,    MODE_INTER_PRIOR_LAST,
         MODE_INTER_PLUS_MV,    MODE_INTER_NO_MV,
         MODE_INTRA,            MODE_USING_GOLDEN,
         MODE_GOLDEN_MV,        MODE_INTER_FOURMV },

    /* scheme 2 */
    {    MODE_INTER_LAST_MV,    MODE_INTER_PRIOR_LAST,
         MODE_INTER_NO_MV,      MODE_INTER_PLUS_MV,
         MODE_INTRA,            MODE_USING_GOLDEN,
         MODE_GOLDEN_MV,        MODE_INTER_FOURMV },

    /* scheme 3 */
    {    MODE_INTER_LAST_MV,    MODE_INTER_PLUS_MV,
         MODE_INTER_PRIOR_LAST, MODE_INTER_NO_MV,
         MODE_INTRA,            MODE_USING_GOLDEN,
         MODE_GOLDEN_MV,        MODE_INTER_FOURMV },

    /* scheme 4 */
    {    MODE_INTER_LAST_MV,    MODE_INTER_PLUS_MV,
         MODE_INTER_NO_MV,      MODE_INTER_PRIOR_LAST,
         MODE_INTRA,            MODE_USING_GOLDEN,
         MODE_GOLDEN_MV,        MODE_INTER_FOURMV },

    /* scheme 5: No motion vector dominates */
    {    MODE_INTER_NO_MV,      MODE_INTER_LAST_MV,
         MODE_INTER_PRIOR_LAST, MODE_INTER_PLUS_MV,
         MODE_INTRA,            MODE_USING_GOLDEN,
         MODE_GOLDEN_MV,        MODE_INTER_FOURMV },

    /* scheme 6 */
    {    MODE_INTER_NO_MV,      MODE_USING_GOLDEN,
         MODE_INTER_LAST_MV,    MODE_INTER_PRIOR_LAST,
         MODE_INTER_PLUS_MV,    MODE_INTRA,
         MODE_GOLDEN_MV,        MODE_INTER_FOURMV },

};


static void init_hilbert_walk(Vp3DecodeContext *s, int y_stride, int uv_stride)
{
    int i, mb, plane;

    int hilbert_luma[4][4] = {
        {0,            8,          8*y_stride+8, 8*y_stride},
        {0,            8*y_stride, 8*y_stride+8, 8},
        {0,            8*y_stride, 8*y_stride+8, 8},
        {8*y_stride+8, 8*y_stride, 0,            8}
    };
    int hilbert_chroma[4][4] = {
        {             0,               8,   8*uv_stride+8,   8*uv_stride},
        {16*uv_stride,    24*uv_stride,    24*uv_stride+8,  16*uv_stride+8},
        {16*uv_stride+16, 24*uv_stride+16, 24*uv_stride+24, 16*uv_stride+24},
        { 8*uv_stride+24,  8*uv_stride+16,              16,              24}
    };
    int hilbert_mb_delta[2][4] = {
        {16*y_stride,  16, -16*y_stride,  16},
        {16*uv_stride, 16, -16*uv_stride, 16}
    };

    for (mb = 0; mb < 4; mb++)
        for (i = 0; i < 4; i++) {
            s->luma_offset[mb][i]   = hilbert_luma[mb][i];
            s->chroma_offset[mb][i] = hilbert_chroma[mb][i];
        }

    for (plane = 0; plane < 2; plane++)
        for (mb = 0; mb < 4; mb++)
            s->hilbert_mb_delta[plane][mb] = hilbert_mb_delta[plane][mb];
}

/*
 * This function sets up the dequantization tables used for a particular
 * frame.
 */
static void init_dequantizer(Vp3DecodeContext *s, int qpi)
{
    int ac_scale_factor = s->coded_ac_scale_factor[s->qps[qpi]];
    int dc_scale_factor = s->coded_dc_scale_factor[s->qps[qpi]];
    int i, plane, inter, qri, bmi, bmj, qistart;

    for(inter=0; inter<2; inter++){
        for(plane=0; plane<3; plane++){
            int sum=0;
            for(qri=0; qri<s->qr_count[inter][plane]; qri++){
                sum+= s->qr_size[inter][plane][qri];
                if(s->qps[qpi] <= sum)
                    break;
            }
            qistart= sum - s->qr_size[inter][plane][qri];
            bmi= s->qr_base[inter][plane][qri  ];
            bmj= s->qr_base[inter][plane][qri+1];
            for(i=0; i<64; i++){
                int coeff= (  2*(sum    -s->qps[qpi])*s->base_matrix[bmi][i]
                            - 2*(qistart-s->qps[qpi])*s->base_matrix[bmj][i]
                            + s->qr_size[inter][plane][qri])
                           / (2*s->qr_size[inter][plane][qri]);

                int qmin= 8<<(inter + !i);
                int qscale= i ? ac_scale_factor : dc_scale_factor;

                s->qmat[plane][inter][qpi][s->dsp.idct_permutation[i]]= av_clip((qscale * coeff)/100 * 4, qmin, 4096);
            }
            // all DC coefficients use the same quant so as not to interfere with DC prediction
            s->qmat[plane][inter][qpi][0] = s->qmat[plane][inter][0][0];
        }
    }
}

/*
 * This function initializes the loop filter boundary limits if the frame's
 * quality index is different from the previous frame's.
 *
 * The filter_limit_values may not be larger than 127.
 */
static void init_loop_filter(Vp3DecodeContext *s)
{
    int *bounding_values= s->bounding_values_array+127;
    int filter_limit;
    int x;
    static int value;

    filter_limit = s->filter_limit_values[s->qps[0]];

    /* set up the bounding values */
    memset(s->bounding_values_array, 0, 256 * sizeof(int));
    for (x = 0; x < filter_limit; x++) {
        bounding_values[-x] = -x;
        bounding_values[x] = x;
    }
    for (x = value = filter_limit; x < 128 && value; x++, value--) {
        bounding_values[ x] =  value;
        bounding_values[-x] = -value;
    }
    if (value)
        bounding_values[128] = value;
    bounding_values[129] = bounding_values[130] = filter_limit * 0x02020202;
}

/*
 * This function unpacks all of the block coding information from the bitstream
 */
static int unpack_block_coding(Vp3DecodeContext *s, GetBitContext *gb)
{
    int num_superblocks = s->superblock_count[0] + s->superblock_count[1]*2;
    int i, j, blk_i, bit, run_length, plane;
    int superblocks_decoded = 0;
    int num_partially_coded = 0;

    /* unpack the list of partially-coded superblocks */
    bit = get_bits1(gb);
    do {
        run_length = get_vlc2(gb, s->superblock_run_length_vlc.table, 6, 2) + 1;
        if (run_length == 34)
            run_length += get_bits(gb, 12);

        if (superblocks_decoded + run_length > num_superblocks)
            return -1;

        memset(s->superblock_coding[0] + superblocks_decoded, 
               SB_PARTIALLY_CODED*bit, run_length);

        superblocks_decoded += run_length;
        if (bit)
            num_partially_coded += run_length;

        if (run_length == 4129)
            bit = get_bits1(gb);
        else
            bit ^= 1;
    } while (superblocks_decoded < num_superblocks);

    /* unpack the list of fully coded superblocks if any of the blocks were
     * not marked as partially coded in the previous step */
    if (num_superblocks > num_partially_coded) {
        superblocks_decoded = i = 0;

        bit = get_bits1(gb);
        do {
            run_length = get_vlc2(gb, s->superblock_run_length_vlc.table, 6, 2) + 1;
            if (run_length == 34)
                run_length += get_bits(gb, 12);

            for (j = 0; j < run_length; i++) {
                if (i > num_superblocks)
                    return -1;

                if (s->superblock_coding[0][i] == SB_NOT_CODED) {
                    s->superblock_coding[0][i] = SB_FULLY_CODED*bit;
                    j++;
                }
            }
            superblocks_decoded += run_length;

            if (run_length == 4129)
                bit = get_bits1(gb);
            else
                bit ^= 1;
        } while (superblocks_decoded < num_superblocks - num_partially_coded);
    }

    for (plane = 0; plane < 3; plane++) {
        uint8_t *block_coding = s->block_coding[plane];
        int num_coded_blocks = 0;

        for (i = 0; i < s->superblock_count[plane]; i++) {
            if (s->superblock_coding[plane][i] == SB_PARTIALLY_CODED) {
                int num_blocks = 16;
                blk_i = 0;

                bit = get_bits1(gb);
                do {
                    run_length = get_vlc2(gb, s->fragment_run_length_vlc.table, 5, 2) + 1;
                    num_blocks -= run_length;

                    do {
                        if (!(block_coding[i*16 + blk_i] & BLOCK_NONEXISTANT)) {
                            block_coding[i*16 + blk_i] = bit << 2;
                            num_coded_blocks += bit;
                            run_length--;
                        } else
                            num_blocks--;

                    } while (++blk_i < 16 && run_length > 0);
                } while (blk_i < 16 && num_blocks > 0);
            } else {
                // SB_NOT_CODED and SB_FULLY_CODED set all blocks to the same coding
                int sb_coded = s->superblock_coding[plane][i] == SB_FULLY_CODED;
                for (blk_i = i*16; blk_i < i*16 + 16; blk_i++)
                    // only set the coding if the block exists
                    if (!(block_coding[blk_i] & BLOCK_NONEXISTANT)) {
                        block_coding[blk_i] = sb_coded << 2;
                        num_coded_blocks += sb_coded;
                    }
            }
        }

        // initialize the number of coded coefficients
        for (i = 0; i < 64; i++)
            s->num_coded_blocks[plane][i] = num_coded_blocks;
    }

    return 0;
}

static void set_macroblock_flag(Vp3DecodeContext *s, int mb_i, int flag)
{
    int plane, j;
    uint8_t *block_coding[3] = {s->block_coding[0], s->block_coding[1], s->block_coding[2]};
    int *mb_to_uvblk_i = s->mb_to_uvblk_i;

#define SET_IF_CODED(plane, index)\
    if (block_coding[plane][index] & BLOCK_IS_CODED)\
        block_coding[plane][index] |= flag;

    // set luma blocks
    for (j = 0; j < 4; j++)
        SET_IF_CODED(0, 4*mb_i + j);

    for (plane = 1; plane < 3; plane++) {
        if (s->chroma_y_shift) {
            SET_IF_CODED(plane, mb_to_uvblk_i[mb_i]);
        } else if (s->chroma_x_shift) {
            SET_IF_CODED(plane, mb_to_uvblk_i[2*mb_i  ]);
            SET_IF_CODED(plane, mb_to_uvblk_i[2*mb_i+1]);
        } else
            for (j = 0; j < 4; j++)
                SET_IF_CODED(plane, 4*mb_i + j);
    }
#undef SET_IF_CODED
}

/*
 * This function unpacks all the coding mode data for individual macroblocks
 * from the bitstream.
 */
static void unpack_modes(Vp3DecodeContext *s, GetBitContext *gb)
{
    int i, mb_i, coding_mode;
    uint8_t custom_mode_alphabet[CODING_MODE_COUNT] = {0};
    const uint8_t *mode_tbl;
    uint8_t *block_coding = s->block_coding[0];
    uint8_t *macroblock_coding = s->macroblock_coding;
    int num_mvs = 0;
    int scheme = get_bits(gb, 3);

    if (!scheme) {
        for (i = 0; i < 8; i++)
            custom_mode_alphabet[get_bits(gb, 3)] = i;
        mode_tbl = custom_mode_alphabet;
    } else
        mode_tbl = ModeAlphabet[scheme-1];

    for (mb_i = 0; mb_i < s->macroblock_count; mb_i++) {
        if (macroblock_coding[mb_i] == MODE_NONEXISTANT)
            continue;

        /* coding modes are only stored if the macroblock has at least one
         * luma block coded, otherwise it must be INTER_NO_MV */
        for (i = 0; i < 4; i++)
            if (block_coding[4*mb_i + i] & BLOCK_IS_CODED)
                break;
        if (i == 4) {
            macroblock_coding[mb_i] = MODE_INTER_NO_MV;
            continue;
        }

        if (scheme == 7)
            coding_mode = get_bits(gb, 3);
        else
            coding_mode = mode_tbl[get_vlc2(gb, s->mode_code_vlc.table, 3, 3)];
        macroblock_coding[mb_i] = coding_mode;

        if (coding_mode == MODE_INTER_PLUS_MV || coding_mode == MODE_GOLDEN_MV)
            num_mvs++;
        else if (coding_mode == MODE_INTER_FOURMV)
            for (i = 0; i < 4; i++)
                if (block_coding[4*mb_i + i] & BLOCK_IS_CODED)
                    num_mvs++;
        else if (coding_mode == MODE_INTRA)
            set_macroblock_flag(s, mb_i, BLOCK_IS_INTRA);

        if (coding_mode == MODE_GOLDEN_MV || coding_mode == MODE_USING_GOLDEN)
            set_macroblock_flag(s, mb_i, BLOCK_USES_GOLDEN);
    }
    s->num_mvs = num_mvs;
}

/*
 * This function unpacks all the motion vectors for the individual
 * macroblocks from the bitstream.
 */
static void unpack_vectors(Vp3DecodeContext *s, GetBitContext *gb)
{
    int i;
    int8_t *mvs = s->mvs;

    if (get_bits1(gb))
        for (i = 0; i < s->num_mvs; i++) {
            mvs[i*2]   = fixed_motion_vector_table[get_bits(gb, 6)];
            mvs[i*2+1] = fixed_motion_vector_table[get_bits(gb, 6)];
        }
    else
        for (i = 0; i < s->num_mvs; i++) {
            mvs[i*2]   = motion_vector_table[get_vlc2(gb, s->motion_vector_vlc.table, 6, 2)];
            mvs[i*2+1] = motion_vector_table[get_vlc2(gb, s->motion_vector_vlc.table, 6, 2)];
        }
}

static int unpack_block_qpis(Vp3DecodeContext *s, GetBitContext *gb)
{
    int qpi, i, j, bit, run_length, blocks_decoded, num_blocks_at_qpi;
    int num_coded_blocks = s->num_coded_blocks[0][0] + s->num_coded_blocks[1][0] + s->num_coded_blocks[2][0];
    int num_blocks = num_coded_blocks;
    struct vp3_block *blocks = s->blocks[0];
    int *coded_blocks = s->coded_blocks[0];

    for (qpi = 0; qpi < s->nqps-1 && num_blocks > 0; qpi++) {
        i = blocks_decoded = num_blocks_at_qpi = 0;

        bit = get_bits1(gb);

        do {
            run_length = get_vlc2(gb, s->superblock_run_length_vlc.table, 6, 2) + 1;
            if (run_length == 34)
                run_length += get_bits(gb, 12);
            blocks_decoded += run_length;

            if (!bit)
                num_blocks_at_qpi += run_length;

            for (j = 0; j < run_length; i++) {
                if (i > num_coded_blocks)
                    return -1;

                if (blocks[coded_blocks[i]].qpi == qpi) {
                    blocks[coded_blocks[i]].qpi += bit;
                    j++;
                }
            }

            if (run_length == 4129)
                bit = get_bits1(gb);
            else
                bit ^= 1;
        } while (blocks_decoded < num_blocks);

        num_blocks -= num_blocks_at_qpi;
    }

    return 0;
}

/*
 * This function is called by unpack_dct_coeffs() to extract the VLCs from
 * the bitstream. The VLCs encode tokens which are used to unpack DCT
 * data. This function unpacks all the VLCs for either the Y plane or both
 * C planes, and is called for DC coefficients or different AC coefficient
 * levels (since different coefficient types require different VLC tables.
 *
 * This function returns a residual eob run. E.g, if a particular token gave
 * instructions to EOB the next 5 fragments and there were only 2 fragments
 * left in the current fragment range, 3 would be returned so that it could
 * be passed into the next call to this same function.
 */
static int unpack_vlcs(Vp3DecodeContext *s, GetBitContext *gb,
                       VLC *table, int zzi, int plane, int eob_run)
{
    int i, j = 0;
    int token, token_type;
    int zero_run;
    DCTELEM coeff;
    int bits_to_get;
    int blocks_ended;
    int coeff_i = 0;
    int num_coeffs = s->num_coded_blocks[plane][zzi];
    int16_t *dct_tokens  = s->dct_tokens[plane][zzi];

    // TODO: Mike's dereference optimizations, it's less readable so later

    // also, if we collapse EOB tokens into one, we can stuff token + extrabits
    // into one uint16_t like so:
    // token = x>>11
    // token == 0 || 1 -> EOB, extrabits = x & 0xfff   (lsb of token overlaps)
    // token >= 7    -> coeff, extrabits = x & 0x3ff

    // or collapsing everything: 2 bits for token type, 12 bits for extradata
    //   extradata stored in upper 14 bits for simplicity with sign extension
    // 0 -> EOB run  (12 bits)
    // 1 -> zero run (7 bits)
    // 2 -> 1 coeff  (11 bits)
    // 3 -> n zeros  (7 bits) followed by coeff (3 bits)
    //  really 5 bits for n zeros for 3, but 7 means 1 and 3 can be combined

    static uint8_t token_to_type[32] = {
        0,0,0,0,0,0,0,                  // EOB
        1,1,                            // pure zero run
        2,2,2,2,2,2,2,2,2,2,2,2,2,2,    // one coeff
        3,3,3,3,3,3,3,3,3               // zero run followed by coeff
    };

    if (num_coeffs < 0)
        av_log(s->avctx, AV_LOG_ERROR, "Invalid number of coefficents at zzi %d\n", zzi);

    if (eob_run > num_coeffs) {
        coeff_i = blocks_ended = num_coeffs;
        eob_run -= num_coeffs;
    } else {
        coeff_i = blocks_ended = eob_run;
        eob_run = 0;
    }

    // insert fake EOB token, is this needed?
    if (blocks_ended)
        dct_tokens[j++] = blocks_ended << 2;

    int first = plane;

    while (coeff_i < num_coeffs) {
        /* decode a VLC into a token */
        token = get_vlc2(gb, table->table, 5, 3);
        token_type = token_to_type[token];

        /* use the token to get a zero run, a coefficient, and an eob run */
        switch (token_type) {
        case 0:
            eob_run = eob_run_base[token];
            if (eob_run_get_bits[token])
                eob_run += get_bits(gb, eob_run_get_bits[token]);

            // if (eob_run == 0)
                // printf("EOB run 0\n");

            // if (!first)
                // printf("EOB run %d\n", eob_run);

            // only recort the number of blocks ended in this plane,
            // the spill will be recorded in the next plane.
            if (eob_run > num_coeffs - coeff_i) {
                dct_tokens[j++] =(num_coeffs - coeff_i) << 2;
                blocks_ended   += num_coeffs - coeff_i;
                eob_run        -= num_coeffs - coeff_i;
                coeff_i         = num_coeffs;
            } else {
                dct_tokens[j++] = eob_run << 2;
                blocks_ended   += eob_run;
                coeff_i        += eob_run;
                eob_run = 0;
            }
            break;
        case 1:
            zero_run = get_bits(gb, zero_run_get_bits[token]);
            dct_tokens[j++] = (zero_run << 2) + 1;
            // if (!first)
                // printf("zero run %d, coeff 0\n", zero_run);
            break;
        case 2:
            bits_to_get = coeff_get_bits[token];
            if (!bits_to_get)
                coeff = coeff_tables[token][0];
            else
                coeff = coeff_tables[token][get_bits(gb, bits_to_get)];
            zero_run = 0;

            // save DC (into raster order)
            if (!zzi)
                s->blocks[0][s->coded_blocks[plane][coeff_i]].dc = coeff;

            dct_tokens[j++] = (coeff << 2) + 2;
            // if (!first)
                // printf("coeff %d\n", coeff);
            break;
        case 3:
            // todo: recombine cases or whatever
            bits_to_get = coeff_get_bits[token];
            if (!bits_to_get)
                coeff = coeff_tables[token][0];
            else
                coeff = coeff_tables[token][get_bits(gb, bits_to_get)];

            zero_run = zero_run_base[token];
            if (zero_run_get_bits[token])
                zero_run += get_bits(gb, zero_run_get_bits[token]);

            dct_tokens[j++] = (coeff << 9) + (zero_run << 2) + 1;
            // if (!first)
                // printf("zero run %d, coeff %d\n", zero_run, coeff);
            break;
        }

        if (token_type) {
            if (zzi + zero_run > 64) {
                av_log(s->avctx, AV_LOG_ERROR, "Invalid zero run of %d with"
                       " %d coeffs left\n", zero_run, 64-zzi);
                zero_run = 64 - zzi;
            }

            // zero runs code multiple coefficients,
            // so don't try to decode coeffs for those higher levels
            for (i = zzi+1; i <= zzi+zero_run; i++)
                s->num_coded_blocks[plane][i]--;
            coeff_i++;
        }
        first = 1;
    }

    if (blocks_ended > s->num_coded_blocks[plane][zzi])
        av_log(s->avctx, AV_LOG_ERROR, "More blocks ended than coded!\n");

    // decrement the number of blocks that have higher coeffecients for each
    // EOB run at this level
    if (blocks_ended)
        for (i = zzi+1; i < 64; i++)
            s->num_coded_blocks[plane][i] -= blocks_ended;

    // setup the next buffer
    if (plane < 2)
        s->dct_tokens[plane+1][zzi] = dct_tokens + j;
    else if (zzi < 63)
        s->dct_tokens[0][zzi+1] = dct_tokens + j;

    return eob_run;
}

static void reverse_dc_prediction(Vp3DecodeContext *s, int plane);

/*
 * This function unpacks all of the DCT coefficient data from the
 * bitstream.
 */
static int unpack_dct_coeffs(Vp3DecodeContext *s, GetBitContext *gb)
{
    int i, plane, vlc_table_i;
    int dc_y_table;
    int dc_c_table;
    int ac_y_table;
    int ac_c_table;
    int residual_eob_run = 0;

    s->dct_tokens[0][0] = s->dct_tokens_base;

    /* fetch the DC table indexes */
    dc_y_table = get_bits(gb, 4);
    dc_c_table = get_bits(gb, 4);

    /* unpack the DC coefficients */
    for (plane = 0; plane < 3; plane++) {
        vlc_table_i = plane ? dc_c_table : dc_y_table;
        residual_eob_run = unpack_vlcs(s, gb, &s->dc_vlc[vlc_table_i],
                                       0, plane, residual_eob_run);
        reverse_dc_prediction(s, plane);
    }
 
    /* fetch the AC table indexes */
    ac_y_table = get_bits(gb, 4);
    ac_c_table = get_bits(gb, 4);

#define UNPACK_AC(GROUP, START, END)\
    for (i = START; i <= END; i++) {\
        for (plane = 0; plane < 3; plane++) {\
            vlc_table_i = plane ? ac_c_table : ac_y_table;\
            residual_eob_run = unpack_vlcs(s, gb, &s->GROUP[vlc_table_i],\
                                           i, plane, residual_eob_run);\
        }\
    }

    UNPACK_AC(ac_vlc_1, 1,  5)
    UNPACK_AC(ac_vlc_2, 6,  14)
    UNPACK_AC(ac_vlc_3, 15, 27)
    UNPACK_AC(ac_vlc_4, 28, 63)
#undef UNPACK_AC

    return 0;
}

/*
 * This function reverses the DC prediction for each coded fragment in
 * the frame. Much of this function is adapted directly from the original
 * VP3 source code.
 */
// FIXME keyframe coding modes
#define COMPATIBLE_FRAME(x) (s->keyframe || (s->blocks[plane][u].modes&0x3) == current_bin)
#define FRAME_CODED(x) (s->keyframe || s->blocks[plane][u].modes)
#define DC_COEFF(u) (s->blocks[plane][u].dc)

static void reverse_dc_prediction(Vp3DecodeContext *s, int plane)
{

#define PUL 8
#define PU 4
#define PUR 2
#define PL 1

    int width = s->block_width[plane];
    int height = s->block_height[plane];
    int x, y;

    int predicted_dc;

    /* DC values for the left, up-left, up, and up-right fragments */
    int vl, vul, vu, vur;

    /* indexes for the left, up-left, up, and up-right fragments */
    int l, ul, u, ur;

    /*
     * The 6 fields mean:
     *   0: up-left multiplier
     *   1: up multiplier
     *   2: up-right multiplier
     *   3: left multiplier
     */
    static const int predictor_transform[16][4] = {
        {  0,  0,  0,  0},
        {  0,  0,  0,128},        // PL
        {  0,  0,128,  0},        // PUR
        {  0,  0, 53, 75},        // PUR|PL
        {  0,128,  0,  0},        // PU
        {  0, 64,  0, 64},        // PU|PL
        {  0,128,  0,  0},        // PU|PUR
        {  0,  0, 53, 75},        // PU|PUR|PL
        {128,  0,  0,  0},        // PUL
        {  0,  0,  0,128},        // PUL|PL
        { 64,  0, 64,  0},        // PUL|PUR
        {  0,  0, 53, 75},        // PUL|PUR|PL
        {  0,128,  0,  0},        // PUL|PU
       {-104,116,  0,116},        // PUL|PU|PL
        { 24, 80, 24,  0},        // PUL|PU|PUR
       {-104,116,  0,116}         // PUL|PU|PUR|PL
    };

    int current_bin;

    /* there is a last DC predictor for each of the 3 frame types
     * intra+golden is invalid, but save space here just in case */
    short last_dc[4] = {0};

    int transform = 0;

    vul = vu = vur = vl = 0;

    /* for each fragment row... */
    for (y = 0; y < height; y++) {

        /* for each fragment in a row... */
        for (x = 0; x < width; x++) {

            /* reverse prediction if this block was coded */
            if (!s->keyframe && !s->blocks[plane][y*width + x].modes)
                continue;

            current_bin = s->blocks[plane][y*width + x].modes & 0x3;

                transform= 0;
                if(x){
                    l= y*width + x-1;
                    vl = DC_COEFF(l);
                    if(FRAME_CODED(l) && COMPATIBLE_FRAME(l))
                        transform |= PL;
                }
                if(y){
                    u= (y-1)*width + x;
                    vu = DC_COEFF(u);
                    if(FRAME_CODED(u) && COMPATIBLE_FRAME(u))
                        transform |= PU;
                    if(x){
                        ul= (y-1)*width + x-1;
                        vul = DC_COEFF(ul);
                        if(FRAME_CODED(ul) && COMPATIBLE_FRAME(ul))
                            transform |= PUL;
                    }
                    if(x + 1 < s->block_width[plane]){
                        ur= (y-1)*width + x+1;
                        vur = DC_COEFF(ur);
                        if(FRAME_CODED(ur) && COMPATIBLE_FRAME(ur))
                            transform |= PUR;
                    }
                }

                if (transform == 0) {

                    /* if there were no fragments to predict from, use last
                     * DC saved */
                    predicted_dc = last_dc[current_bin];
                } else {

                    /* apply the appropriate predictor transform */
                    predicted_dc =
                        (predictor_transform[transform][0] * vul) +
                        (predictor_transform[transform][1] * vu) +
                        (predictor_transform[transform][2] * vur) +
                        (predictor_transform[transform][3] * vl);

                    predicted_dc /= 128;

                    /* check for outranging on the [ul u l] and
                     * [ul u ur l] predictors */
                    if ((transform == 13) || (transform == 15)) {
                        if (FFABS(predicted_dc - vu) > 128)
                            predicted_dc = vu;
                        else if (FFABS(predicted_dc - vl) > 128)
                            predicted_dc = vl;
                        else if (FFABS(predicted_dc - vul) > 128)
                            predicted_dc = vul;
                    }
                }
                if (0 && !y)
                    printf("dc %d -> %d\n", s->blocks[plane][y*width + x].dc, s->blocks[plane][y*width + x].dc + predicted_dc);
                /* at long last, apply the predictor */
                s->blocks[plane][y*width + x].dc += predicted_dc;
                /* save the DC */
                last_dc[current_bin] = s->blocks[plane][y*width + x].dc;
        }
    }
}


static void dequant(Vp3DecodeContext *s, int plane, int inter, struct vp3_block *block)
{
    uint8_t *perm = s->scantable.permutated;
    int16_t *dequantizer = s->qmat[plane][inter][block->qpi];
    int i = 0;

    s->dsp.clear_block(s->block);
    do {
        int token = *s->dct_tokens[plane][i];
        // printf(" %d: ", i);
        switch (token & 3) {
        case 0: // EOB
            // printf("EOB run %d\n", token >> 2);
            // 0-3 are token types, so the EOB run must be 0
            if (--token < 4)
                s->dct_tokens[plane][i]++;
            else
                *s->dct_tokens[plane][i] = token & ~3;
            goto end;
        case 1: // zero run
            s->dct_tokens[plane][i]++;
            i += (token >> 2) & 0x7f;
            // printf("zero run %d, coeff %d * %d\n", (token >> 2) & 0x7f, token >> 9, dequantizer[perm[i]]);
            // TODO: set qmat so [perm[i]] isn't needed
            s->block[perm[i]] = (token >> 9) * dequantizer[perm[i]];
            i++;
            break;
        case 2: // coeff
            // printf("coeff %d * %d\n", token>>2, dequantizer[perm[i]]);
            s->block[perm[i]] = (token >> 2) * dequantizer[perm[i]];
            s->dct_tokens[plane][i++]++;
            break;
        default:
        // printf("error %x\n", token);
            av_log(s->avctx, AV_LOG_ERROR, "internal: invalid token type\n");
            return;
        }
    } while (i < 64);
end:
    // TODO: worth munging up the function to avoid doing this twice?
    s->block[perm[0]] = block->dc * s->qmat[plane][inter][0][0];
    // printf("DC %d -> %d\n", (int)block->dc, (int)s->block[perm[0]]);
#if 0
    int x, y;

    printf("QPI::%d\n", (int)block->qpi);
    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++)
            printf("  %5d", dequantizer[y*8+x]);
        printf("\n");
    }
    printf("DEQUANT\n");
    for (y = 0; y < 8; y++) {
        for (x = 0; x < 8; x++)
            printf(" %5d", s->block[y*8+x]);
        printf("\n");
    }
#endif
}

static const uint8_t hilbert_offset[16][2] = {
    {0,0}, {1,0}, {1,1}, {0,1},
    {0,2}, {0,3}, {1,3}, {1,2},
    {2,2}, {2,3}, {3,3}, {3,2},
    {3,1}, {2,1}, {2,0}, {3,0}
};

/**
 * Perform the final rendering for a particular slice of data.
 * The slice number ranges from 0..(superblock_height - 1).
 */
static void render_slice(Vp3DecodeContext *s, int sb_y)
{
    int sb_x, mb_i, i;
    uint8_t *sb_dst, *dst;
    int plane = 0;
    int block_i = sb_y * 4*s->block_width[plane];

    static const uint8_t mb_offset[4][2] = {
        {0,0}, {0,1}, {1,1}, {1,0}
    };

    for (sb_x = 0; sb_x < s->superblock_width[plane]; sb_x++) {
        sb_dst = s->current_frame.data[0] + s->data_offset[0]
                + 32*sb_y * s->linesize[0] + 32*sb_x;

        for (mb_i = 0; mb_i < 4; mb_i++) {
            // bound check
            if (4*sb_x + 2*mb_offset[mb_i][0] > s->block_width[0] ||
                4*sb_y + 2*mb_offset[mb_i][1] > s->block_height[0])
                continue;

            for (i = 0; i < 4; i++) {
                dst = 8*hilbert_offset[4*mb_i + i][1] * s->linesize[0] +
                      8*hilbert_offset[4*mb_i + i][0] + sb_dst;

                dequant(s, plane, 0, &s->blocks[0][s->coded_blocks[plane][block_i++]]);
                s->dsp.idct_put(dst, s->linesize[0], s->block);
#undef exit
                // if (block_i > 1)
                    // exit(0);
            }
        }
    }
}

static av_cold void init_block_mapping(AVCodecContext *avctx)
{
    Vp3DecodeContext *s = avctx->priv_data;
    int sb_x, sb_y, plane, i, start = 0;
    int *all_blocks = s->all_blocks[0];
    int j = 0;

    for (plane = 0; plane < 3; plane++) {
        for (sb_y = 0; sb_y < s->superblock_height[plane]; sb_y++)
            for (sb_x = 0; sb_x < s->superblock_width[plane]; sb_x++)
                for (i = 0; i < 16; i++) {
                    int x = 4*sb_x + hilbert_offset[i][0];
                    int y = 4*sb_y + hilbert_offset[i][1];

                    if (x < s->block_width[plane] && y < s->block_height[plane])
                        all_blocks[j++] = start + y*s->block_width[plane] + x;
                    else
                        all_blocks[j++] = -1;
                }
        // assign the chroma lists to be contiguous from luma
        if (plane < 2)
            s->all_blocks[plane+1] = all_blocks + j;
        // all indicies are offset from the start of luma
        start += s->block_width[plane] * s->block_height[plane];
    }
}

/*
 * This is the ffmpeg/libavcodec API init function.
 */
static av_cold int vp3_decode_init(AVCodecContext *avctx)
{
    Vp3DecodeContext *s = avctx->priv_data;
    int i, inter, plane;

    if (avctx->codec_tag == MKTAG('V','P','3','0'))
        s->version = 0;
    else
        s->version = 1;

    s->avctx = avctx;
    s->width = FFALIGN(avctx->width, 16);
    s->height = FFALIGN(avctx->height, 16);
    avctx->pix_fmt = PIX_FMT_YUV420P;
    avctx->chroma_sample_location = AVCHROMA_LOC_CENTER;
    if(avctx->idct_algo==FF_IDCT_AUTO)
        avctx->idct_algo=FF_IDCT_VP3;
    dsputil_init(&s->dsp, avctx);

    ff_init_scantable(s->dsp.idct_permutation, &s->scantable, ff_zigzag_direct);

    /* initialize to an impossible value which will force a recalculation
     * in the first frame decode */
    for (i = 0; i < 3; i++)
        s->qps[i] = -1;

    s->num_superblocks = 0;
    s->num_blocks = 0;
    for (i = 0; i < 3; i++) { // 420
        s->superblock_width[i]  = FFALIGN(s->width  >> !!i, 32) / 32;
        s->superblock_height[i] = FFALIGN(s->height >> !!i, 32) / 32;
        s->superblock_count[i]  = s->superblock_width[i] * s->superblock_height[i];
        s->num_superblocks     += s->superblock_count[i];
        s->block_width[i]  = s->width  >> (3 + !!i);
        s->block_height[i] = s->height >> (3 + !!i);
        s->num_blocks     += s->block_width[i] * s->block_height[i];
    }

    s->blocks[0]            = av_malloc(s->num_blocks * sizeof(*s->blocks[0]));
    s->coded_blocks[0]      = av_malloc(s->num_blocks * sizeof(*s->coded_blocks[0]));
    s->all_blocks[0]        = av_malloc(16*s->num_superblocks * sizeof(*s->all_blocks[0]));
    s->dct_tokens_base      = av_malloc(64*s->num_blocks * sizeof(*s->dct_tokens_base));
    s->superblock_coding[0] = av_malloc(s->num_superblocks);

    for (i = 1; i < 3; i++) {
        s->blocks[i] = s->blocks[i-1] + s->block_width[i-1] * s->block_height[i-1];
        s->superblock_coding[i] = s->superblock_coding[i-1] + s->superblock_count[i-1];
    }

    init_block_mapping(avctx);

    if (!s->theora_tables)
    {
        for (i = 0; i < 64; i++) {
            s->coded_dc_scale_factor[i] = vp31_dc_scale_factor[i];
            s->coded_ac_scale_factor[i] = vp31_ac_scale_factor[i];
            s->base_matrix[0][i] = vp31_intra_y_dequant[i];
            s->base_matrix[1][i] = vp31_intra_c_dequant[i];
            s->base_matrix[2][i] = vp31_inter_dequant[i];
            s->filter_limit_values[i] = vp31_filter_limit_values[i];
        }

        for(inter=0; inter<2; inter++){
            for(plane=0; plane<3; plane++){
                s->qr_count[inter][plane]= 1;
                s->qr_size [inter][plane][0]= 63;
                s->qr_base [inter][plane][0]=
                s->qr_base [inter][plane][1]= 2*inter + (!!plane)*!inter;
            }
        }

        /* init VLC tables */
        for (i = 0; i < 16; i++) {

            /* DC histograms */
            init_vlc(&s->dc_vlc[i], 5, 32,
                &dc_bias[i][0][1], 4, 2,
                &dc_bias[i][0][0], 4, 2, 0);

            /* group 1 AC histograms */
            init_vlc(&s->ac_vlc_1[i], 5, 32,
                &ac_bias_0[i][0][1], 4, 2,
                &ac_bias_0[i][0][0], 4, 2, 0);

            /* group 2 AC histograms */
            init_vlc(&s->ac_vlc_2[i], 5, 32,
                &ac_bias_1[i][0][1], 4, 2,
                &ac_bias_1[i][0][0], 4, 2, 0);

            /* group 3 AC histograms */
            init_vlc(&s->ac_vlc_3[i], 5, 32,
                &ac_bias_2[i][0][1], 4, 2,
                &ac_bias_2[i][0][0], 4, 2, 0);

            /* group 4 AC histograms */
            init_vlc(&s->ac_vlc_4[i], 5, 32,
                &ac_bias_3[i][0][1], 4, 2,
                &ac_bias_3[i][0][0], 4, 2, 0);
        }
    } else {
        for (i = 0; i < 16; i++) {

            /* DC histograms */
            if (init_vlc(&s->dc_vlc[i], 5, 32,
                &s->huffman_table[i][0][1], 4, 2,
                &s->huffman_table[i][0][0], 4, 2, 0) < 0)
                goto vlc_fail;

            /* group 1 AC histograms */
            if (init_vlc(&s->ac_vlc_1[i], 5, 32,
                &s->huffman_table[i+16][0][1], 4, 2,
                &s->huffman_table[i+16][0][0], 4, 2, 0) < 0)
                goto vlc_fail;

            /* group 2 AC histograms */
            if (init_vlc(&s->ac_vlc_2[i], 5, 32,
                &s->huffman_table[i+16*2][0][1], 4, 2,
                &s->huffman_table[i+16*2][0][0], 4, 2, 0) < 0)
                goto vlc_fail;

            /* group 3 AC histograms */
            if (init_vlc(&s->ac_vlc_3[i], 5, 32,
                &s->huffman_table[i+16*3][0][1], 4, 2,
                &s->huffman_table[i+16*3][0][0], 4, 2, 0) < 0)
                goto vlc_fail;

            /* group 4 AC histograms */
            if (init_vlc(&s->ac_vlc_4[i], 5, 32,
                &s->huffman_table[i+16*4][0][1], 4, 2,
                &s->huffman_table[i+16*4][0][0], 4, 2, 0) < 0)
                goto vlc_fail;
        }
    }

    init_vlc(&s->superblock_run_length_vlc, 6, 34,
        &superblock_run_length_vlc_table[0][1], 4, 2,
        &superblock_run_length_vlc_table[0][0], 4, 2, 0);

    init_vlc(&s->fragment_run_length_vlc, 5, 30,
        &fragment_run_length_vlc_table[0][1], 4, 2,
        &fragment_run_length_vlc_table[0][0], 4, 2, 0);

    init_vlc(&s->mode_code_vlc, 3, 8,
        &mode_code_vlc_table[0][1], 2, 1,
        &mode_code_vlc_table[0][0], 2, 1, 0);

    init_vlc(&s->motion_vector_vlc, 6, 63,
        &motion_vector_vlc_table[0][1], 2, 1,
        &motion_vector_vlc_table[0][0], 2, 1, 0);

    for (i = 0; i < 3; i++) {
        s->current_frame.data[i] = NULL;
        s->last_frame.data[i] = NULL;
        s->golden_frame.data[i] = NULL;
    }

    return 0;

vlc_fail:
    av_log(avctx, AV_LOG_FATAL, "Invalid huffman table\n");
    return -1;
}

/*
 * This is the ffmpeg/libavcodec API frame decode function.
 */
static int vp3_decode_frame(AVCodecContext *avctx,
                            void *data, int *data_size,
                            AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    Vp3DecodeContext *s = avctx->priv_data;
    GetBitContext gb;
    int i, j, plane;

    init_get_bits(&gb, buf, buf_size * 8);

    if (s->theora && get_bits1(&gb))
    {
        av_log(avctx, AV_LOG_ERROR, "Header packet passed to frame decoder, skipping\n");
        return -1;
    }

    s->keyframe = !get_bits1(&gb);
    if (!s->theora)
        skip_bits(&gb, 1);
    for (i = 0; i < 3; i++)
        s->last_qps[i] = s->qps[i];

    s->nqps=0;
    do{
        s->qps[s->nqps++]= get_bits(&gb, 6);
    } while(s->theora >= 0x030200 && s->nqps<3 && get_bits1(&gb));
    for (i = s->nqps; i < 3; i++)
        s->qps[i] = -1;

    if (s->avctx->debug & FF_DEBUG_PICT_INFO)
        av_log(s->avctx, AV_LOG_INFO, " VP3 %sframe #%d: Q index = %d\n",
            s->keyframe?"key":"", avctx->frame_number+1, s->qps[0]);

    if (s->qps[0] != s->last_qps[0])
        init_loop_filter(s);

    for (i = 0; i < s->nqps; i++)
        // reinit all dequantizers if the first one changed, because
        // the DC of the first quantizer must be used for all matrices
        if (s->qps[i] != s->last_qps[i] || s->qps[0] != s->last_qps[0])
            init_dequantizer(s, i);

    if (avctx->skip_frame >= AVDISCARD_NONKEY && !s->keyframe)
        return buf_size;

    if (s->keyframe) {
        if (!s->theora)
        {
            skip_bits(&gb, 4); /* width code */
            skip_bits(&gb, 4); /* height code */
            if (s->version)
            {
                s->version = get_bits(&gb, 5);
                if (avctx->frame_number == 1)
                    av_log(s->avctx, AV_LOG_DEBUG, "VP version: %d\n", s->version);
            }
        }
        if (s->version || s->theora)
        {
                if (get_bits1(&gb))
                    av_log(s->avctx, AV_LOG_ERROR, "Warning, unsupported keyframe coding type?!\n");
            skip_bits(&gb, 2); /* reserved? */
        }

        if (s->last_frame.data[0] == s->golden_frame.data[0]) {
            if (s->golden_frame.data[0])
                avctx->release_buffer(avctx, &s->golden_frame);
            s->last_frame= s->golden_frame; /* ensure that we catch any access to this released frame */
        } else {
            if (s->golden_frame.data[0])
                avctx->release_buffer(avctx, &s->golden_frame);
            if (s->last_frame.data[0])
                avctx->release_buffer(avctx, &s->last_frame);
        }

        s->golden_frame.reference = 3;
        if(avctx->get_buffer(avctx, &s->golden_frame) < 0) {
            av_log(s->avctx, AV_LOG_ERROR, "vp3: get_buffer() failed\n");
            return -1;
        }

        /* golden frame is also the current frame */
        s->current_frame= s->golden_frame;

        // copy full block list, skipping nonexistant blocks
        for (plane = 0; plane < 3; plane++) {
            for (i = 0, j = 0; i < 16*s->superblock_count[plane]; i++)
                if (s->all_blocks[plane][i] >= 0)
                    s->coded_blocks[plane][j++] = s->all_blocks[plane][i];

            if (plane < 2)
                s->coded_blocks[plane+1] = s->coded_blocks[plane] + j;

            s->num_coded_blocks[plane][0] = j;
            for (j = 1; j < 64; j++)
                s->num_coded_blocks[plane][j] = s->num_coded_blocks[plane][j-1];
        }

        // fixme: only memset the number of coded blocks if possible
        // though doesn't matter much for intra
        memset(s->blocks[0], 0, s->num_blocks * sizeof(*s->blocks[0]));

        /* time to figure out pixel addresses? */
    }

    if (unpack_block_qpis(s, &gb)){
        av_log(s->avctx, AV_LOG_ERROR, "error in unpack_block_qpis\n");
        return -1;
    }
    if (unpack_dct_coeffs(s, &gb)){
        av_log(s->avctx, AV_LOG_ERROR, "error in unpack_dct_coeffs\n");
        return -1;
    }

    for (i = 0; i < 3; i++) {
        s->data_offset[i] = 0;
        s->linesize[i] = s->current_frame.linesize[i];
        if (!s->flipped_image) {
            s->data_offset[i] = (s->height-1) * s->linesize[i];
            s->linesize[i] *= -1;
        }
    }

    i = 0;
    for (i = 0; i < s->superblock_height[0]; i++)
        render_slice(s, i);

    *data_size=sizeof(AVFrame);
    *(AVFrame*)data= s->current_frame;

    /* release the last frame, if it is allocated and if it is not the
     * golden frame */
    if ((s->last_frame.data[0]) &&
        (s->last_frame.data[0] != s->golden_frame.data[0]))
        avctx->release_buffer(avctx, &s->last_frame);

    /* shuffle frames (last = current) */
    s->last_frame= s->current_frame;
    s->current_frame.data[0]= NULL; /* ensure that we catch any access to this released frame */
#undef exit
    // exit(0);
    return buf_size;
}

/*
 * This is the ffmpeg/libavcodec API module cleanup function.
 */
static av_cold int vp3_decode_end(AVCodecContext *avctx)
{
    Vp3DecodeContext *s = avctx->priv_data;
    int i;

    for (i = 0; i < 16; i++) {
        free_vlc(&s->dc_vlc[i]);
        free_vlc(&s->ac_vlc_1[i]);
        free_vlc(&s->ac_vlc_2[i]);
        free_vlc(&s->ac_vlc_3[i]);
        free_vlc(&s->ac_vlc_4[i]);
    }

    free_vlc(&s->superblock_run_length_vlc);
    free_vlc(&s->fragment_run_length_vlc);
    free_vlc(&s->mode_code_vlc);
    free_vlc(&s->motion_vector_vlc);

    /* release all frames */
    if (s->golden_frame.data[0] && s->golden_frame.data[0] != s->last_frame.data[0])
        avctx->release_buffer(avctx, &s->golden_frame);
    if (s->last_frame.data[0])
        avctx->release_buffer(avctx, &s->last_frame);
    /* no need to release the current_frame since it will always be pointing
     * to the same frame as either the golden or last frame */

    return 0;
}

#if CONFIG_THEORA_DECODER
static int read_huffman_tree(AVCodecContext *avctx, GetBitContext *gb)
{
    Vp3DecodeContext *s = avctx->priv_data;

    if (get_bits1(gb)) {
        int token;
        if (s->entries >= 32) { /* overflow */
            av_log(avctx, AV_LOG_ERROR, "huffman tree overflow\n");
            return -1;
        }
        token = get_bits(gb, 5);
        //av_log(avctx, AV_LOG_DEBUG, "hti %d hbits %x token %d entry : %d size %d\n", s->hti, s->hbits, token, s->entries, s->huff_code_size);
        s->huffman_table[s->hti][token][0] = s->hbits;
        s->huffman_table[s->hti][token][1] = s->huff_code_size;
        s->entries++;
    }
    else {
        if (s->huff_code_size >= 32) {/* overflow */
            av_log(avctx, AV_LOG_ERROR, "huffman tree overflow\n");
            return -1;
        }
        s->huff_code_size++;
        s->hbits <<= 1;
        if (read_huffman_tree(avctx, gb))
            return -1;
        s->hbits |= 1;
        if (read_huffman_tree(avctx, gb))
            return -1;
        s->hbits >>= 1;
        s->huff_code_size--;
    }
    return 0;
}

static int theora_decode_header(AVCodecContext *avctx, GetBitContext *gb)
{
    Vp3DecodeContext *s = avctx->priv_data;
    int visible_width, visible_height;

    s->theora = get_bits_long(gb, 24);
    av_log(avctx, AV_LOG_DEBUG, "Theora bitstream version %X\n", s->theora);

    /* 3.2.0 aka alpha3 has the same frame orientation as original vp3 */
    /* but previous versions have the image flipped relative to vp3 */
    if (s->theora < 0x030200)
    {
        s->flipped_image = 1;
        av_log(avctx, AV_LOG_DEBUG, "Old (<alpha3) Theora bitstream, flipped image\n");
    }

    visible_width  = s->width  = get_bits(gb, 16) << 4;
    visible_height = s->height = get_bits(gb, 16) << 4;

    if(avcodec_check_dimensions(avctx, s->width, s->height)){
        av_log(avctx, AV_LOG_ERROR, "Invalid dimensions (%dx%d)\n", s->width, s->height);
        s->width= s->height= 0;
        return -1;
    }

    if (s->theora >= 0x030200) {
        visible_width  = get_bits_long(gb, 24);
        visible_height = get_bits_long(gb, 24);

        skip_bits(gb, 8); /* offset x */
        skip_bits(gb, 8); /* offset y */
    }

    skip_bits(gb, 32); /* fps numerator */
    skip_bits(gb, 32); /* fps denumerator */
    skip_bits(gb, 24); /* aspect numerator */
    skip_bits(gb, 24); /* aspect denumerator */

    if (s->theora < 0x030200)
        skip_bits(gb, 5); /* keyframe frequency force */
    skip_bits(gb, 8); /* colorspace */
    skip_bits(gb, 24); /* bitrate */

    skip_bits(gb, 6); /* quality hint */

    if (s->theora >= 0x030200)
    {
        skip_bits(gb, 5); /* keyframe frequency force */
        skip_bits(gb, 2); /* pixel format: 420,res,422,444 */
        skip_bits(gb, 3); /* reserved */
    }

//    align_get_bits(gb);

    if (   visible_width  <= s->width  && visible_width  > s->width-16
        && visible_height <= s->height && visible_height > s->height-16)
        avcodec_set_dimensions(avctx, visible_width, visible_height);
    else
        avcodec_set_dimensions(avctx, s->width, s->height);

    return 0;
}

static int theora_decode_tables(AVCodecContext *avctx, GetBitContext *gb)
{
    Vp3DecodeContext *s = avctx->priv_data;
    int i, n, matrices, inter, plane;

    if (s->theora >= 0x030200) {
        n = get_bits(gb, 3);
        /* loop filter limit values table */
        for (i = 0; i < 64; i++) {
            s->filter_limit_values[i] = get_bits(gb, n);
            if (s->filter_limit_values[i] > 127) {
                av_log(avctx, AV_LOG_ERROR, "filter limit value too large (%i > 127), clamping\n", s->filter_limit_values[i]);
                s->filter_limit_values[i] = 127;
            }
        }
    }

    if (s->theora >= 0x030200)
        n = get_bits(gb, 4) + 1;
    else
        n = 16;
    /* quality threshold table */
    for (i = 0; i < 64; i++)
        s->coded_ac_scale_factor[i] = get_bits(gb, n);

    if (s->theora >= 0x030200)
        n = get_bits(gb, 4) + 1;
    else
        n = 16;
    /* dc scale factor table */
    for (i = 0; i < 64; i++)
        s->coded_dc_scale_factor[i] = get_bits(gb, n);

    if (s->theora >= 0x030200)
        matrices = get_bits(gb, 9) + 1;
    else
        matrices = 3;

    if(matrices > 384){
        av_log(avctx, AV_LOG_ERROR, "invalid number of base matrixes\n");
        return -1;
    }

    for(n=0; n<matrices; n++){
        for (i = 0; i < 64; i++)
            s->base_matrix[n][i]= get_bits(gb, 8);
    }

    for (inter = 0; inter <= 1; inter++) {
        for (plane = 0; plane <= 2; plane++) {
            int newqr= 1;
            if (inter || plane > 0)
                newqr = get_bits1(gb);
            if (!newqr) {
                int qtj, plj;
                if(inter && get_bits1(gb)){
                    qtj = 0;
                    plj = plane;
                }else{
                    qtj= (3*inter + plane - 1) / 3;
                    plj= (plane + 2) % 3;
                }
                s->qr_count[inter][plane]= s->qr_count[qtj][plj];
                memcpy(s->qr_size[inter][plane], s->qr_size[qtj][plj], sizeof(s->qr_size[0][0]));
                memcpy(s->qr_base[inter][plane], s->qr_base[qtj][plj], sizeof(s->qr_base[0][0]));
            } else {
                int qri= 0;
                int qi = 0;

                for(;;){
                    i= get_bits(gb, av_log2(matrices-1)+1);
                    if(i>= matrices){
                        av_log(avctx, AV_LOG_ERROR, "invalid base matrix index\n");
                        return -1;
                    }
                    s->qr_base[inter][plane][qri]= i;
                    if(qi >= 63)
                        break;
                    i = get_bits(gb, av_log2(63-qi)+1) + 1;
                    s->qr_size[inter][plane][qri++]= i;
                    qi += i;
                }

                if (qi > 63) {
                    av_log(avctx, AV_LOG_ERROR, "invalid qi %d > 63\n", qi);
                    return -1;
                }
                s->qr_count[inter][plane]= qri;
            }
        }
    }

    /* Huffman tables */
    for (s->hti = 0; s->hti < 80; s->hti++) {
        s->entries = 0;
        s->huff_code_size = 1;
        if (!get_bits1(gb)) {
            s->hbits = 0;
            if(read_huffman_tree(avctx, gb))
                return -1;
            s->hbits = 1;
            if(read_huffman_tree(avctx, gb))
                return -1;
        }
    }

    s->theora_tables = 1;

    return 0;
}

static av_cold int theora_decode_init(AVCodecContext *avctx)
{
    Vp3DecodeContext *s = avctx->priv_data;
    GetBitContext gb;
    int ptype;
    uint8_t *header_start[3];
    int header_len[3];
    int i;

    s->theora = 1;

    if (!avctx->extradata_size)
    {
        av_log(avctx, AV_LOG_ERROR, "Missing extradata!\n");
        return -1;
    }

    if (ff_split_xiph_headers(avctx->extradata, avctx->extradata_size,
                              42, header_start, header_len) < 0) {
        av_log(avctx, AV_LOG_ERROR, "Corrupt extradata\n");
        return -1;
    }

  for(i=0;i<3;i++) {
    init_get_bits(&gb, header_start[i], header_len[i]);

    ptype = get_bits(&gb, 8);

     if (!(ptype & 0x80))
     {
        av_log(avctx, AV_LOG_ERROR, "Invalid extradata!\n");
//        return -1;
     }

    // FIXME: Check for this as well.
    skip_bits_long(&gb, 6*8); /* "theora" */

    switch(ptype)
    {
        case 0x80:
            theora_decode_header(avctx, &gb);
                break;
        case 0x81:
// FIXME: is this needed? it breaks sometimes
//            theora_decode_comments(avctx, gb);
            break;
        case 0x82:
            if (theora_decode_tables(avctx, &gb))
                return -1;
            break;
        default:
            av_log(avctx, AV_LOG_ERROR, "Unknown Theora config packet: %d\n", ptype&~0x80);
            break;
    }
    if(ptype != 0x81 && 8*header_len[i] != get_bits_count(&gb))
        av_log(avctx, AV_LOG_WARNING, "%d bits left in packet %X\n", 8*header_len[i] - get_bits_count(&gb), ptype);
    if (s->theora < 0x030200)
        break;
  }

    return vp3_decode_init(avctx);
}

AVCodec theora_decoder = {
    "theora",
    CODEC_TYPE_VIDEO,
    CODEC_ID_THEORA,
    sizeof(Vp3DecodeContext),
    theora_decode_init,
    NULL,
    vp3_decode_end,
    vp3_decode_frame,
    CODEC_CAP_DR1,
    NULL,
    .long_name = NULL_IF_CONFIG_SMALL("Theora"),
};
#endif

AVCodec vp3_decoder = {
    "vp3",
    CODEC_TYPE_VIDEO,
    CODEC_ID_VP3,
    sizeof(Vp3DecodeContext),
    vp3_decode_init,
    NULL,
    vp3_decode_end,
    vp3_decode_frame,
    CODEC_CAP_DR1,
    NULL,
    .long_name = NULL_IF_CONFIG_SMALL("On2 VP3"),
};
