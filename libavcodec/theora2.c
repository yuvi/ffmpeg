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
#undef exit

// TODO: splitting this could provide a small cache/memory bandwidth benefit
//  at the cost of increased register pressure
// Maybe try packing mb_mode, qpi, and coded flags into one uint8_t array
struct vp3_block {
    int16_t dc;
    uint8_t mb_mode;    // 0-7, MODE_*
    uint8_t qpi:2;      // 0-2
    uint8_t coded:2;    // 0-2, SB_*
    uint8_t luma_coded:3;
    // quick note: bitfields obviously have to be read before writing
    //  if the other members are untouched
    // qpi and coded can be initialized together in unpack_block_coding,
    //  so just 1 write is needed
    // unpack_block_qpis is simplified to always read the qpi beforehand anyway
    // mb_mode is written after reading coded, but does not need to be initialized
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

    int         data_offset;
    int         uvdata_offset;
    int         linesize;
    int         uvlinesize;
    int         chroma_x_shift;
    int         chroma_y_shift;

    int         superblock_width[3];
    int         superblock_height[3];
    int         superblock_count[3];
    int         num_superblocks;

    // uv_block_width rather than [3]?
    int         block_width[3];     // does not include nonexistant blocks
    int         block_height[3];
    int         num_blocks;

#define SB_NOT_CODED        0
#define SB_PARTIALLY_CODED  1
#define SB_FULLY_CODED      2
    uint8_t     *superblock_coding[3];  ///< chroma planes are contiguous to luma

    /**
     * maps each coded block in coding order to the raster index of blocks[0]
     */
    int         *coded_blocks[3];
    /**
     * maps every block in coding order to the raster index of blocks[0]
     * -1 means the block doesn't exist
     */
    int         *all_blocks[3];

    /**
     * block_width x block_height array, raster order
     * contains DC, QP index, and coding modes for each block
     */
    struct vp3_block *blocks[3];

    /**
     * number of blocks that contain DCT coefficients at the given level or higher
     */
    int         num_coded_blocks[3][64];

    /**
     * This is a list of all tokens in bitstream order. Reordering takes place
     * by pulling from each level during IDCT. As a consequence, IDCT must be
     * in Hilbert order, making the minimum slice height 64 for 4:2:0 and 32
     * otherwise. The 32 different tokens with up to 12 bits of extradata are
     * collapsed into 3 types, packed as follows:
     *   (from the low to high bits)
     *
     * 2 bits: type (0,1,2)
     *   0: EOB run, 14 bits for run length (12 needed)
     *   1: zero run, 7 bits for run length
     *                7 bits for the next coefficient (3 needed)
     *   2: coefficient, 14 bits (11 needed)
     * 
     * Coefficients are signed, so are packed in the highest bits for automatic
     * sign extension.
     */
    int16_t     *dct_tokens[3][64];
    int16_t     *dct_tokens_base;


    int         mv_i;               ///< index into mvs array
    int8_t      *mvs;               ///< array of all motion vectors in coded order
    int         num_mvs;
    int8_t      last_mv[2];
    int8_t      prior_last_mv[2];

    uint8_t     *edge_emu_buffer;
    uint8_t     *edge_emu_buffer_base;

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

#define VLC_TOKEN_BITS 8
#define VLC_LONG_RUN_BITS 6
#define VLC_SHORT_RUN_BITS 5
#define VLC_MB_MODE_BITS 3
#define VLC_MV_BITS 6

#define MODE_INTER_NO_MV      0
#define MODE_INTRA            1
#define MODE_INTER_PLUS_MV    2
#define MODE_INTER_LAST_MV    3
#define MODE_INTER_PRIOR_LAST 4
#define MODE_USING_GOLDEN     5
#define MODE_GOLDEN_MV        6
#define MODE_INTER_FOURMV     7
#define CODING_MODE_COUNT     8

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

static const uint8_t hilbert_offset[16][2] = {
    {0,0}, {1,0}, {1,1}, {0,1},
    {0,2}, {0,3}, {1,3}, {1,2},
    {2,2}, {2,3}, {3,3}, {3,2},
    {3,1}, {2,1}, {2,0}, {3,0}
};

// TODO: I think bitmath can get rid of this
static const uint8_t mb_offset[4][2] = {
    {0,0}, {0,1}, {1,1}, {1,0}
};

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
    int value;

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
    int i, j, block_i, bit, run_length, plane, coded;
    int superblocks_decoded = 0;
    int num_partially_coded = 0;
    struct vp3_block *blocks = s->blocks[0];

    /* unpack the list of partially-coded superblocks */
    bit = get_bits1(gb);
    do {
        run_length = get_vlc2(gb, s->superblock_run_length_vlc.table, VLC_LONG_RUN_BITS, 2) + 1;
        if (run_length == 34)
            run_length += get_bits(gb, 12);

        if (superblocks_decoded + run_length > s->num_superblocks)
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
    } while (superblocks_decoded < s->num_superblocks);

    /* unpack the list of fully coded superblocks if any of the blocks were
     * not marked as partially coded in the previous step */
    if (s->num_superblocks > num_partially_coded) {
        superblocks_decoded = i = 0;

        bit = get_bits1(gb);
        do {
            run_length = get_vlc2(gb, s->superblock_run_length_vlc.table, VLC_LONG_RUN_BITS, 2) + 1;
            if (run_length == 34)
                run_length += get_bits(gb, 12);

            for (j = 0; j < run_length; i++) {
                if (i > s->num_superblocks)
                    return -1;

                if (!s->superblock_coding[0][i]) {
                    s->superblock_coding[0][i] = SB_FULLY_CODED*bit;
                    j++;
                }
            }
            superblocks_decoded += run_length;

            if (run_length == 4129)
                bit = get_bits1(gb);
            else
                bit ^= 1;
        } while (superblocks_decoded < s->num_superblocks - num_partially_coded);
    }

    if (num_partially_coded) {
        bit = get_bits1(gb) ^ 1;
        run_length = 0;
    }

    // decode block coded flag
    // coded blocks are one list; runs are allowed between superblocks
    for (plane = 0; plane < 3; plane++) {
        int num_coded_blocks = 0;
        int *all_blocks = s->all_blocks[plane];
        int *coded_blocks = s->coded_blocks[plane];

        for (i = 0; i < s->superblock_count[plane]; i++) {
            int sb_coded = s->superblock_coding[plane][i];
            for (j = 0; j < 16; j++) {
                block_i = all_blocks[16*i + j];
                if (block_i < 0)
                    continue;

                if (sb_coded == SB_PARTIALLY_CODED) {
                    if (run_length-- == 0) {
                        run_length = get_vlc2(gb, s->fragment_run_length_vlc.table, VLC_SHORT_RUN_BITS, 2);
                        bit ^= 1;
                    }
                    coded = bit;
                } else
                    coded = sb_coded;

                // this initializes the other elements to 0
                blocks[block_i] = (struct vp3_block){ .coded = coded, .mb_mode = MODE_INTER_NO_MV };

                if (coded)
                    coded_blocks[num_coded_blocks++] = block_i;
            }
        }
        // initialize the number of coded coefficients
        for (i = 0; i < 64; i++)
            s->num_coded_blocks[plane][i] = num_coded_blocks;            
        if (plane < 2)
            s->coded_blocks[plane+1] = s->coded_blocks[plane] + num_coded_blocks;
    }
    return 0;
}

/*
 * This function unpacks all the coding mode data for individual macroblocks
 * from the bitstream.
 */
static void unpack_modes(Vp3DecodeContext *s, GetBitContext *gb)
{
    int x, y, i, sb_x, sb_y, mb_i, coding_mode, plane;
    uint8_t custom_mode_alphabet[CODING_MODE_COUNT] = {0};
    const uint8_t *mode_tbl;
    int num_mvs = 0;
    int scheme = get_bits(gb, 3);

    if (!scheme) {
        for (i = 0; i < 8; i++)
            custom_mode_alphabet[get_bits(gb, 3)] = i;
        mode_tbl = custom_mode_alphabet;
    } else
        mode_tbl = ModeAlphabet[scheme-1];

    for (sb_y = 0; sb_y < s->superblock_height[0]; sb_y++)
        for (sb_x = 0; sb_x < s->superblock_width[0]; sb_x++)
            for (mb_i = 0; mb_i < 4; mb_i++) {
                int mb_x = 2*sb_x + mb_offset[mb_i][0];
                int mb_y = 2*sb_y + mb_offset[mb_i][1];
                int luma_coded = 0;

                // bound check
                if (2*mb_x >= s->block_width[0] || 2*mb_y >= s->block_height[0])
                    continue;

                /* coding modes are only stored if the macroblock has at least one
                 * luma block coded, otherwise it must be INTER_NO_MV */
                for (y = 2*mb_y; y < 2*mb_y+2; y++)
                    for (x = 2*mb_x; x < 2*mb_x+2; x++)
                        if (s->blocks[0][y*s->block_width[0] + x].coded)
                            luma_coded++;
                if (!luma_coded)
                    continue;

                if (scheme == 7)
                    coding_mode = get_bits(gb, 3);
                else
                    coding_mode = mode_tbl[get_vlc2(gb, s->mode_code_vlc.table, VLC_MB_MODE_BITS, 3)];

                if (coding_mode == MODE_INTER_PLUS_MV || coding_mode == MODE_GOLDEN_MV)
                    num_mvs++;
                else if (coding_mode == MODE_INTER_FOURMV)
                    num_mvs += luma_coded;

                for (y = 2*mb_y; y < 2*mb_y+2; y++)
                    for (x = 2*mb_x; x < 2*mb_x+2; x++) {
                        s->blocks[0][y*s->block_width[0] + x].mb_mode = coding_mode;
                        s->blocks[0][y*s->block_width[0] + x].luma_coded = luma_coded;
                    }

                // 420 chroma
                for (plane = 1; plane < 3; plane++)
                    s->blocks[plane][mb_y*s->block_width[1] + mb_x].mb_mode = coding_mode;
            }

    s->num_mvs = num_mvs;
}

/*
 * This function unpacks all the motion vectors for the individual
 * macroblocks from the bitstream.
 */
static void unpack_vectors(Vp3DecodeContext *s, GetBitContext *gb)
{
    int i, num_mvs = s->num_mvs;
    int8_t *mvs = s->mvs;

    if (get_bits1(gb))
        for (i = 0; i < num_mvs; i++) {
            mvs[i*2]   = fixed_motion_vector_table[get_bits(gb, 6)];
            mvs[i*2+1] = fixed_motion_vector_table[get_bits(gb, 6)];
        }
    else
        for (i = 0; i < num_mvs; i++) {
            mvs[i*2]   = motion_vector_table[get_vlc2(gb, s->motion_vector_vlc.table, VLC_MV_BITS, 2)];
            mvs[i*2+1] = motion_vector_table[get_vlc2(gb, s->motion_vector_vlc.table, VLC_MV_BITS, 2)];
        }
    s->mv_i = 0;
}

static int unpack_block_qpis(Vp3DecodeContext *s, GetBitContext *gb)
{
    int qpi, i, j, bit, run_length, blocks_decoded, num_blocks_at_qpi;
    int num_coded_blocks = s->num_coded_blocks[0][0] + s->num_coded_blocks[1][0] + s->num_coded_blocks[2][0];
    int num_blocks = num_coded_blocks;

    for (qpi = 0; qpi < s->nqps-1 && num_blocks > 0; qpi++) {
        i = blocks_decoded = num_blocks_at_qpi = 0;

        bit = get_bits1(gb);

        do {
            run_length = get_vlc2(gb, s->superblock_run_length_vlc.table, VLC_LONG_RUN_BITS, 2) + 1;
            if (run_length == 34)
                run_length += get_bits(gb, 12);
            blocks_decoded += run_length;

            if (!bit)
                num_blocks_at_qpi += run_length;

            for (j = 0; j < run_length; i++) {
                if (i >= num_coded_blocks)
                    return -1;

                if (s->blocks[0][s->coded_blocks[0][i]].qpi == qpi) {
                    s->blocks[0][s->coded_blocks[0][i]].qpi += bit;
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
 * This function reverses the DC prediction for each coded fragment in
 * the frame. Much of this function is adapted directly from the original
 * VP3 source code.
 */

#define BLOCK_CODED(x) (keyframe || blocks[x].coded)
// probably not worth the obfuscation
//#define MODE_BIN(mode) ((((1<<14)|(2<<12)|(2<<10)|(1<<8)|(1<<6)|(1<<4)|(1))>>(mode))&3)
#define MODE_BIN(mode) mode_bin[mode]
#define BLOCK_MODE(x) (BLOCK_CODED(x) ? MODE_BIN(blocks[x].mb_mode) : 3)

static av_always_inline void reverse_dc_prediction(Vp3DecodeContext *s, int plane, int keyframe)
{

#define PUL 8
#define PU 4
#define PUR 2
#define PL 1

    int width = s->block_width[plane];
    int height = s->block_height[plane];
    struct vp3_block *blocks = s->blocks[plane];
    int x, y, predicted_dc, current_bin, transform = 0;

    /* This table shows which types of blocks can use other blocks for
     * prediction. For example, INTRA is the only mode in this table to
     * have a frame number of 0. That means INTRA blocks can only predict
     * from other INTRA blocks. There are 2 golden frame coding types;
     * blocks encoding in these modes can only predict from other blocks
     * that were encoded with these 1 of these 2 modes. */
    static const unsigned char mode_bin[8] = {
        1,    /* MODE_INTER_NO_MV */
        0,    /* MODE_INTRA */
        1,    /* MODE_INTER_PLUS_MV */
        1,    /* MODE_INTER_LAST_MV */
        1,    /* MODE_INTER_PRIOR_MV */
        2,    /* MODE_USING_GOLDEN */
        2,    /* MODE_GOLDEN_MV */
        1     /* MODE_INTER_FOUR_MV */
    };

    /* there is a last DC predictor for each of the 3 frame types 
       3 is special meaning no predictor from there */
    short last_dc[4] = {0};

    int bin_l, bin_t, bin_tl, bin_tr;

    /* DC values for the left, up-left, up, and up-right fragments */
    int vl, vul, vu, vur;
    vul = vu = vur = vl = 0;

    // special case first row for the moment
    for (x = 0; x < width; x++) {
        if (!BLOCK_CODED(x))
            continue;
        current_bin = BLOCK_MODE(x);
        last_dc[current_bin] = blocks[x].dc += last_dc[current_bin];
    }

    for (y = 1; y < height; y++) {
        bin_l = 3;
        bin_tl = 3;
        bin_t = BLOCK_MODE(0);
        blocks += width;

        for (x = 0; x < width; x++) {
            if (x+1 < width)
                bin_tr = BLOCK_MODE(x+1-width);
            else
                bin_tr = 3;

            current_bin = BLOCK_MODE(x);
            if (current_bin == 3) {
                bin_l = 3;
                bin_tl = bin_t;
                bin_t = bin_tr;
                continue;
            }

            transform = 0;
            if (bin_l == current_bin)
                transform |= PL;
            if (bin_t == current_bin)
                transform |= PU;
            if (bin_tl == current_bin)
                transform |= PUL;
            if (bin_tr == current_bin)
                transform |= PUR;

            bin_l = current_bin;
            bin_tl = bin_t;
            bin_t = bin_tr;

#define DC_L  blocks[x      -1].dc
#define DC_U  blocks[x-width  ].dc
#define DC_UL blocks[x-width-1].dc
#define DC_UR blocks[x-width+1].dc

            switch (transform) {
            case PL:
            case PUL|PL:
                predicted_dc = DC_L; break;
            case PU:
            case PU|PUR:
            case PUL|PU:
                predicted_dc = DC_U; break;
            case PUL:
                predicted_dc = DC_UL; break;
            case PUR:
                predicted_dc = DC_UR; break;
            case PU|PL:
                predicted_dc = (DC_U + DC_L) / 2; break;
            case PUL|PUR:
                predicted_dc = (DC_UL + DC_UR) / 2; break;
            case PUR|PL:
            case PU|PUR|PL:
            case PUL|PUR|PL:
                predicted_dc = (53*DC_UR + 75*DC_L) / 128; break;
            case PUL|PU|PUR:
                predicted_dc = (3*DC_UL + 10*DC_U + 3*DC_UR) / 16; break;
            case PUL|PU|PL:
            case PUL|PU|PUR|PL:
                vl  = DC_L;
                vu  = DC_U;
                vul = DC_UL;
                predicted_dc = (29*vl + 29*vu - 26*vul) / 32;
                /* check for outranging on the [ul u l] and
                 * [ul u ur l] predictors */
                if (FFABS(predicted_dc - vu) > 128)
                    predicted_dc = vu;
                else if (FFABS(predicted_dc - vl) > 128)
                    predicted_dc = vl;
                else if (FFABS(predicted_dc - vul) > 128)
                    predicted_dc = vul;
                break;
            default:
                predicted_dc = last_dc[current_bin]; break;
            }

            /* at long last, apply the predictor */
            blocks[x].dc += predicted_dc;
            /* save the DC */
            last_dc[current_bin] = blocks[x].dc;
        }
    }
}

static av_noinline void reverse_dc_prediction_intra(Vp3DecodeContext *s, int plane)
{
    reverse_dc_prediction(s, plane, 1);
}

static av_noinline void reverse_dc_prediction_inter(Vp3DecodeContext *s, int plane)
{
    reverse_dc_prediction(s, plane, 0);
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
    int zero_run = 0;
    DCTELEM coeff;
    int bits_to_get;
    int blocks_ended;
    int coeff_i = 0;
    int num_coeffs = s->num_coded_blocks[plane][zzi];
    int16_t *dct_tokens  = s->dct_tokens[plane][zzi];
    VLC_TYPE (*vlc_table)[2] = table->table;

    static const uint8_t token_to_type[32] = {
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

    // insert fake EOB token to cover the split between planes or zzi
    if (blocks_ended)
        dct_tokens[j++] = blocks_ended << 2;

    while (coeff_i < num_coeffs) {
        /* decode a VLC into a token */
        token = get_vlc2(gb, vlc_table, VLC_TOKEN_BITS, 2);
        token_type = token_to_type[token];

        /* use the token to get a zero run, a coefficient, and an eob run */
        switch (token_type) {
        case 0:
            eob_run = eob_run_base[token];
            if (eob_run_get_bits[token])
                eob_run += get_bits(gb, eob_run_get_bits[token]);

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
        if (s->keyframe)
            reverse_dc_prediction_intra(s, plane);
        else
            reverse_dc_prediction_inter(s, plane);
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

#if 0
    // these are probably harmless in that I don't get the virtual zzi or
    // whatever libtheora was going on about here
    for (plane = 0; plane < 3; plane++)
        if (s->num_coded_blocks[plane][63])
            av_log(s->avctx, AV_LOG_WARNING, "%d blocks not ended in plane %d\n",
                   s->num_coded_blocks[plane][63], plane);
#endif
    return 0;
}


static inline void prefetch_motion(Vp3DecodeContext *s, uint8_t **pix, int mb_x, int mb_y, 
                                   int motion_x, int motion_y)
{
    /* fetch pixels for estimated mv 4 macroblocks ahead
     * optimized for 64byte cache lines */
    const int mx= (motion_x>>1) + 16*mb_x + 8;
    const int my= (motion_y>>1) + 16*mb_y;
    int off= s->data_offset + mx + (my + (mb_x&3)*4)*s->linesize + 64;
    s->dsp.prefetch(pix[0]+off, s->linesize, 4);
    off= s->uvdata_offset + (mx>>1) + ((my>>1) + (mb_x&7))*s->uvlinesize + 64;
    s->dsp.prefetch(pix[1]+off, pix[2]-pix[1], 2);
}

static inline void vp3_skip_mb(Vp3DecodeContext *s, int mb_x, int mb_y, uint8_t *dst[3])
{
    // 420
    uint8_t *ptr_y  = s->last_frame.data[0] + s->data_offset  + 16*mb_y*s->linesize  + 16*mb_x;
    uint8_t *ptr_cr = s->last_frame.data[1] + s->uvdata_offset + 8*mb_y*s->uvlinesize + 8*mb_x;
    uint8_t *ptr_cb = s->last_frame.data[2] + s->uvdata_offset + 8*mb_y*s->uvlinesize + 8*mb_x;

    prefetch_motion(s, s->last_frame.data, mb_x, mb_y, 0, 0);

    s->dsp.put_no_rnd_pixels_tab[0][0](dst[0], ptr_y,  s->linesize,  16);
    s->dsp.put_no_rnd_pixels_tab[1][0](dst[1], ptr_cr, s->uvlinesize, 8);
    s->dsp.put_no_rnd_pixels_tab[1][0](dst[2], ptr_cb, s->uvlinesize, 8);
}

static inline void vp3_motion(Vp3DecodeContext *s, int mb_x, int mb_y,
                              uint8_t *dst[3], int mb_mode, int luma_coded)
{
    int motion_x, motion_y, mx, my, x, y;
    int dxy, uvdxy, src_x, src_y, uvsrc_x, uvsrc_y;
    uint8_t *ptr_y, *ptr_cb, *ptr_cr;
    AVFrame *ref;

    if (mb_mode == MODE_INTER_PLUS_MV || mb_mode == MODE_GOLDEN_MV) {
        motion_x = s->mvs[s->mv_i++];
        motion_y = s->mvs[s->mv_i++];
    } else if (mb_mode == MODE_INTER_LAST_MV) {
        motion_x = s->last_mv[0];
        motion_y = s->last_mv[1];
    } else if (mb_mode == MODE_INTER_PRIOR_LAST) {
        motion_x = s->prior_last_mv[0];
        motion_y = s->prior_last_mv[1];
    } else {
        motion_x = 0;
        motion_y = 0;
    }

    if (mb_mode == MODE_INTER_PLUS_MV || mb_mode == MODE_INTER_PRIOR_LAST) {
        s->prior_last_mv[0] = s->last_mv[0];
        s->prior_last_mv[1] = s->last_mv[1];
        s->last_mv[0] = motion_x;
        s->last_mv[1] = motion_y;
    }

    if (mb_mode == MODE_GOLDEN_MV || mb_mode == MODE_USING_GOLDEN)
        ref = &s->golden_frame;
    else
        ref = &s->last_frame;

    prefetch_motion(s, ref->data, mb_x, mb_y, motion_x, motion_y);

    dxy = ((motion_y & 1) << 1) | (motion_x & 1);
    src_x = 16*mb_x + (motion_x >> 1);
    src_y = 16*mb_y + (motion_y >> 1);

    if (s->chroma_y_shift) { // 420
        mx = (motion_x >> 1) | (motion_x & 1);
        my = (motion_y >> 1) | (motion_y & 1);
        uvdxy = ((my & 1) << 1) | (mx & 1);
        uvsrc_x = 8*mb_x + (mx >> 1);
        uvsrc_y = 8*mb_y + (my >> 1);
    } else if (s->chroma_x_shift) { // 422
        mx = (motion_x >> 1) | (motion_x & 1);
        uvdxy = ((motion_y & 1) << 1) | (mx & 1);
        uvsrc_x = 8*mb_x + (mx >> 1);
        uvsrc_y = src_y;
    } else { // 444
        uvdxy = dxy;
        uvsrc_x = src_x;
        uvsrc_y = src_y;
    }

    ptr_y  = ref->data[0] + s->data_offset   +   src_y * s->linesize   +   src_x;
    ptr_cb = ref->data[1] + s->uvdata_offset + uvsrc_y * s->uvlinesize + uvsrc_x;
    ptr_cr = ref->data[2] + s->uvdata_offset + uvsrc_y * s->uvlinesize + uvsrc_x;

    // 420
    if (s->avctx->flags&CODEC_FLAG_EMU_EDGE) {
        if (   (unsigned)src_x > s->width  - (motion_x&1) - 16
            || (unsigned)src_y > s->height - (motion_y&1) - 16){
            ff_emulated_edge_mc(s->edge_emu_buffer, ptr_y, s->linesize,
                                17, 17, src_x, src_y,
                                s->width, s->height);
            ptr_y = s->edge_emu_buffer;
            if (!CONFIG_GRAY || !(s->avctx->flags&CODEC_FLAG_GRAY)) {
                uint8_t *uvbuf= s->edge_emu_buffer+18*s->linesize;
                ff_emulated_edge_mc(uvbuf, ptr_cb, s->uvlinesize,
                                    9, 9, uvsrc_x, uvsrc_y,
                                    s->width>>1, s->height>>1);
                ff_emulated_edge_mc(uvbuf+16, ptr_cr, s->uvlinesize,
                                    9, 9, uvsrc_x, uvsrc_y,
                                    s->width>>1, s->height>>1);
                ptr_cb = uvbuf;
                ptr_cr = uvbuf+16;
            }
        }
    }


    if (luma_coded == 4) {
        if (dxy != 3)
            s->dsp.put_no_rnd_pixels_tab[0][dxy](dst[0], ptr_y, s->linesize, 16);
        else {
            // d is 0 if motion_x and _y have the same sign, else -1
            int d= (motion_x ^ motion_y)>>31;
            s->dsp.put_no_rnd_pixels_l2[0](dst[0], ptr_y-d, ptr_y+d+1+s->linesize, s->linesize, 16);
        }
    } else {
        uint8_t *skip_y = s->last_frame.data[0] + s->data_offset + 16*mb_y*s->linesize + 16*mb_x;
        for (y = 0; y < 2; y++)
            for (x = 0; x < 2; x++) {
                int offset = 8*y*s->linesize + 8*x;
                if (s->blocks[0][(2*mb_y+y)*s->block_width[0] + 2*mb_x+x].coded) {
                    if (dxy != 3)
                        s->dsp.put_no_rnd_pixels_tab[1][dxy](dst[0] + offset, ptr_y + offset, s->linesize, 8);
                    else {
                        // d is 0 if motion_x and _y have the same sign, else -1
                        int d= (motion_x ^ motion_y)>>31;
                        s->dsp.put_no_rnd_pixels_l2[1](dst[0] + offset, ptr_y-d + offset,
                                            ptr_y+d+1+s->linesize + offset, s->linesize, 8);
                    }
                } else
                    s->dsp.put_no_rnd_pixels_tab[1][0](dst[0] + offset, skip_y + offset, s->linesize, 8);
            }
    }

    if (CONFIG_GRAY && s->avctx->flags&CODEC_FLAG_GRAY)
        return;

    if (uvdxy != 3) {
        s->dsp.put_no_rnd_pixels_tab[s->chroma_x_shift][uvdxy]
            (dst[1], ptr_cb, s->uvlinesize, 16 >> s->chroma_y_shift);
        s->dsp.put_no_rnd_pixels_tab[s->chroma_x_shift][uvdxy]
            (dst[2], ptr_cr, s->uvlinesize, 16 >> s->chroma_y_shift);
    } else {
        int d= (motion_x ^ motion_y)>>31;
        s->dsp.put_no_rnd_pixels_l2[s->chroma_x_shift](dst[1], ptr_cb-d,
             ptr_cb+d+1+s->uvlinesize, s->uvlinesize, 16 >> s->chroma_y_shift);
        s->dsp.put_no_rnd_pixels_l2[s->chroma_x_shift](dst[2], ptr_cr-d,
             ptr_cr+d+1+s->uvlinesize, s->uvlinesize, 16 >> s->chroma_y_shift);
    }
}


static inline void vp3_hpel_motion(Vp3DecodeContext *s,
                                   uint8_t *dst, uint8_t *src,
                                   int src_x, int src_y, int stride,
                                   int motion_x, int motion_y)
{
    int dxy;

    dxy = ((motion_y & 1) << 1) | (motion_x & 1);
    src_x += motion_x >> 1;
    src_y += motion_y >> 1;
    src += src_y*stride + src_x;

    if (s->avctx->flags&CODEC_FLAG_EMU_EDGE)
        if (   (unsigned)src_x > s->width  - (motion_x&1) - 8
            || (unsigned)src_y > s->height - (motion_y&1) - 8){
            ff_emulated_edge_mc(s->edge_emu_buffer, src, stride, 9, 9,
                                src_x, src_y, s->width, s->height);
            src = s->edge_emu_buffer;
        }

    if (dxy != 3)
        s->dsp.put_no_rnd_pixels_tab[1][dxy](dst, src, stride, 8);
    else {
        int d= (motion_x ^ motion_y)>>31;
        s->dsp.put_no_rnd_pixels_l2[1](dst, src-d, src+d+1+stride, stride, 8);
    }
}

#define BLOCK_I(x,y) ((y)*s->block_width[plane] + (x))
#define BLOCK_CODED_XY(x,y) (s->keyframe || s->blocks[plane][BLOCK_I(x,y)].coded)

static inline void vp3_4mv_motion(Vp3DecodeContext *s, int mb_x, int mb_y,
                                  uint8_t *dst[3])
{
    int i, blk_x, blk_y, mx, my;
    int plane = 0;
    int motion_x[4], motion_y[4];
    uint8_t *src[3] = {
        s->last_frame.data[0] + s->data_offset,
        s->last_frame.data[1] + s->uvdata_offset,
        s->last_frame.data[2] + s->uvdata_offset
    };

    s->prior_last_mv[0] = s->last_mv[0];
    s->prior_last_mv[1] = s->last_mv[1];

    for (i = 0; i < 4; i++) {
        blk_x = i &1;
        blk_y = i>>1;

        if (BLOCK_CODED_XY(2*mb_x + blk_x, 2*mb_y + blk_y)) {
            s->last_mv[0] = motion_x[i] = s->mvs[s->mv_i++];
            s->last_mv[1] = motion_y[i] = s->mvs[s->mv_i++];
        } else {
            motion_x[i] = 0;
            motion_y[i] = 0;
        }
        vp3_hpel_motion(s, dst[0] + 8*blk_y*s->linesize + 8*blk_x,
                        src[0], 16*mb_x + 8*blk_x, 16*mb_y + 8*blk_y,
                        s->linesize, motion_x[i], motion_y[i]);
    }

    prefetch_motion(s, s->last_frame.data, mb_x, mb_y, motion_x[0], motion_y[0]);

    if (CONFIG_GRAY && s->avctx->flags&CODEC_FLAG_GRAY)
        return;

    if (s->chroma_y_shift) {
        mx = motion_x[0] + motion_x[1] + motion_x[2] + motion_x[3];
        my = motion_y[0] + motion_y[1] + motion_y[2] + motion_y[3];
        mx = RSHIFT(mx, 2);
        my = RSHIFT(my, 2);
        mx = (mx >> 1) | (mx & 1);
        my = (my >> 1) | (my & 1);
        vp3_hpel_motion(s, dst[1], src[1], 8*mb_x, 8*mb_y, s->uvlinesize, mx, my);
        vp3_hpel_motion(s, dst[2], src[2], 8*mb_x, 8*mb_y, s->uvlinesize, mx, my);
    }
#if 0
    else if (s->chroma_x_shift)
        for (i = 0; i < 4; i+=2) {
            blk_y = 8*(i>>1);
            mx = RSHIFT(motion_x[i] + motion_x[i+1], 1);
            my = RSHIFT(motion_y[i] + motion_y[i+1], 1);
            mx = (mx >> 1) | (mx & 1);
            my = (my >> 1) | (my & 1);
            vp3_hpel_motion(s, dst_cb + blk_y*s->uvlinesize,
                            s->last_frame.data[1],
                            8*mb_x, 16*mb_y + blk_y,
                            s->uvlinesize, mx, my);
            vp3_hpel_motion(s, dst_cr + 8*i*s->uvlinesize,
                            s->last_frame.data[2],
                            8*mb_x, 16*mb_y + blk_y,
                            s->uvlinesize, mx, my);
        }
    else
        for (i = 0; i < 4; i++) {
            blk_x = 8*(i &1);
            blk_y = 8*(i>>1);
            mx = (motion_x[i] >> 1) | (motion_x[i] & 1);
            my = (motion_y[i] >> 1) | (motion_y[i] & 1);
            vp3_hpel_motion(s, dst_cb + blk_y*s->uvlinesize + blk_x,
                            s->last_frame.data[1],
                            16*mb_x + blk_x, 16*mb_y + blk_y,
                            s->uvlinesize, mx, my);
            vp3_hpel_motion(s, dst_cr + blk_y*s->uvlinesize + blk_x,
                            s->last_frame.data[2],
                            16*mb_x + blk_x, 16*mb_y + blk_y,
                            s->uvlinesize, mx, my);
        }
#endif
}


static inline int dequant(Vp3DecodeContext *s, struct vp3_block *block,
                          int plane, int inter)
{
    int16_t *dequantizer = s->qmat[plane][inter][block->qpi];
    uint8_t *perm = s->scantable.permutated;
    int i = 0;

    s->dsp.clear_block(s->block);

    do {
        int token = *s->dct_tokens[plane][i];
        switch (token & 3) {
        case 0: // EOB
            // 0-3 are token types, so the EOB run must be 0
            if (--token < 4)
                s->dct_tokens[plane][i]++;
            else
                *s->dct_tokens[plane][i] = token & ~3;
            goto end;
        case 1: // zero run
            s->dct_tokens[plane][i]++;
            i += (token >> 2) & 0x7f;
            // TODO: set qmat so [perm[i]] isn't needed
            s->block[perm[i]] = (token >> 9) * dequantizer[perm[i]];
            i++;
            break;
        case 2: // coeff
            s->block[perm[i]] = (token >> 2) * dequantizer[perm[i]];
            s->dct_tokens[plane][i++]++;
            break;
        default:
            av_log(s->avctx, AV_LOG_ERROR, "internal: invalid token type\n");
            return i;
        }
    } while (i < 64);
end:
    // TODO: worth munging up the function to avoid doing this twice?
    s->block[perm[0]] = block->dc * s->qmat[plane][inter][0][0];
    return i;
}

/**
 * Perform the final rendering for a particular slice of data.
 * The slice number ranges from 0..(superblock_height - 1).
 */
static void render_luma_sb_row(Vp3DecodeContext *s, int sb_y)
{
    int sb_x, mb_i, mb_x, mb_y, x, y, i, mb_mode, luma_coded;
    struct vp3_block *block;

    uint8_t *dst[3] = {
        s->current_frame.data[0] + s->data_offset,
        s->current_frame.data[1] + s->uvdata_offset,
        s->current_frame.data[2] + s->uvdata_offset
    };
    // int stride[3] = { s->linesize[0], s->linesize[1], s->linesize[2] };

    for (sb_x = 0; sb_x < s->superblock_width[0]; sb_x++)
        for (mb_i = 0; mb_i < 4; mb_i++) {
            mb_x = 2*sb_x + mb_offset[mb_i][0];
            mb_y = 2*sb_y + mb_offset[mb_i][1];

            if (2*mb_x >= s->block_width[0] || 2*mb_y >= s->block_height[0])
                continue;

            mb_mode = s->blocks[0][2*mb_y*s->block_width[0] + 2*mb_x].mb_mode;
            luma_coded = s->blocks[0][2*mb_y*s->block_width[0] + 2*mb_x].luma_coded;

            if (s->keyframe || mb_mode == MODE_INTRA)
                for (i = 0; i < 4; i++) {
                    x = 4*sb_x + hilbert_offset[4*mb_i + i][0];
                    y = 4*sb_y + hilbert_offset[4*mb_i + i][1];
                    block = &s->blocks[0][y*s->block_width[0] + x];

                    if (s->keyframe || block->coded) {
                        dequant(s, block, 0, 0);
                        s->dsp.idct_put(dst[0] + 8*y*s->linesize + 8*x, s->linesize, s->block);
                    } else {
                        uint8_t *src = s->last_frame.data[0] + s->data_offset + 8*y*s->linesize + 8*x;
                        s->dsp.put_pixels_tab[1][0](dst[0] + 8*y*s->linesize + 8*x, src, s->linesize, 8);
                    }
                }
            else {
                // 420
                uint8_t *dst_mb[3] = {
                    dst[0] + 16*mb_y*s->linesize +  16*mb_x,
                    dst[1] +  8*mb_y*s->uvlinesize + 8*mb_x,
                    dst[2] +  8*mb_y*s->uvlinesize + 8*mb_x
                };

                if (mb_mode == MODE_INTER_NO_MV) {
                    vp3_skip_mb(s, mb_x, mb_y, dst_mb);
                    if (!luma_coded)
                        continue;
                } else if (mb_mode != MODE_INTER_FOURMV)
                    vp3_motion(s, mb_x, mb_y, dst_mb, mb_mode, luma_coded);
                else
                    vp3_4mv_motion(s, mb_x, mb_y, dst_mb);

                for (i = 0; i < 4; i++) {
                    x = 4*sb_x + hilbert_offset[4*mb_i + i][0];
                    y = 4*sb_y + hilbert_offset[4*mb_i + i][1];
                    block = &s->blocks[0][y*s->block_width[0] + x];

                    if (block->coded) {
                        dequant(s, block, 0, 1);
                        s->dsp.idct_add(dst[0] + 8*y*s->linesize + 8*x, s->linesize, s->block);
                    }
                }
            }
        }
}

static void render_chroma_sb_row(Vp3DecodeContext *s, int sb_y)
{
    int sb_x, x, y, i;
    int intra, plane = 0;
    struct vp3_block *block;
    void (*idct_func)(uint8_t *dst, int stride, int16_t block[64]);

    uint8_t *dst[2] = {
        s->current_frame.data[1] + s->uvdata_offset,
        s->current_frame.data[2] + s->uvdata_offset
    };

    for (plane = 1; plane < 3; plane++)
        for (sb_x = 0; sb_x < s->superblock_width[plane]; sb_x++)
            for (i = 0; i < 16; i++) {
                x = 4*sb_x + hilbert_offset[i][0];
                y = 4*sb_y + hilbert_offset[i][1];
                block = &s->blocks[plane][y*s->block_width[plane] + x];

                if (x >= s->block_width[plane] || y >= s->block_height[plane])
                    continue;

                intra = s->keyframe | (block->mb_mode == MODE_INTRA);
                if (intra)
                    idct_func = s->dsp.idct_put;
                else
                    idct_func = s->dsp.idct_add;

                if (s->keyframe || block->coded) {
                    dequant(s, block, plane, !intra);
                    idct_func(dst[plane-1] + 8*y*s->uvlinesize + 8*x, s->uvlinesize, s->block);
                } else {
                    uint8_t *src = s->last_frame.data[plane] + s->uvdata_offset + 8*y*s->uvlinesize + 8*x;
                    s->dsp.put_pixels_tab[1][0](dst[plane-1] + 8*y*s->uvlinesize + 8*x, src, s->uvlinesize, 8);
                }
            }
}

static av_always_inline void apply_loop_filter(Vp3DecodeContext *s, int plane, int y, int yend, int keyframe)
{
    int x, stride = plane ? s->uvlinesize : s->linesize;
    int *lf_bounds = s->bounding_values_array+127;
    int b_width = s->block_width[plane];
    int b_height = s->block_height[plane];
    struct vp3_block *blocks = &s->blocks[plane][y*b_width];
    uint8_t *dst = s->current_frame.data[plane] + 8*y*stride;
    dst += plane ? s->uvdata_offset : s->data_offset;

    for (; y < yend; y++) {
        for (x = 0; x < b_width; x++)
            if (BLOCK_CODED(x)) {
                /* do not perform left edge filter for left columns frags */
                if (x > 0)
                    s->dsp.vp3_h_loop_filter(dst + 8*x, stride, lf_bounds);

                /* do not perform top edge filter for top row fragments */
                if (y > 0)
                    s->dsp.vp3_v_loop_filter(dst + 8*x, stride, lf_bounds);

                /* do not perform right edge filter for right column
                 * fragments or if right fragment neighbor is also coded
                 * in this frame (it will be filtered in next iteration) */
                if (x < b_width-1 && !BLOCK_CODED(x+1))
                    s->dsp.vp3_h_loop_filter(dst + 8*x+8, stride, lf_bounds);

                /* do not perform bottom edge filter for bottom row
                 * fragments or if bottom fragment neighbor is also coded
                 * in this frame (it will be filtered in the next row) */
                if (y < b_height-1 && !BLOCK_CODED(x+b_width))
                    s->dsp.vp3_v_loop_filter(dst + 8*x + 8*stride, stride, lf_bounds);
            }
        dst += 8*stride;
        blocks += b_width;
    }
}

static av_noinline void apply_loop_filter_intra(Vp3DecodeContext *s, int plane, int y, int yend)
{
    apply_loop_filter(s, plane, y, yend, 1);
}

static av_noinline void apply_loop_filter_inter(Vp3DecodeContext *s, int plane, int y, int yend)
{
    apply_loop_filter(s, plane, y, yend, 0);
}

static av_cold void init_block_mapping(AVCodecContext *avctx)
{
    Vp3DecodeContext *s = avctx->priv_data;
    int sb_x, sb_y, plane, start = 0;
    int x, y, i, j = 0;

    for (plane = 0; plane < 3; plane++) {
        for (sb_y = 0; sb_y < s->superblock_height[plane]; sb_y++)
            for (sb_x = 0; sb_x < s->superblock_width[plane]; sb_x++)
                for (i = 0; i < 16; i++) {
                    x = 4*sb_x + hilbert_offset[i][0];
                    y = 4*sb_y + hilbert_offset[i][1];

                    if (x < s->block_width[plane] && y < s->block_height[plane])
                        s->all_blocks[0][j++] = start + y*s->block_width[plane] + x;
                    else
                        s->all_blocks[0][j++] = -1;
                }

        // assign the chroma lists to be contiguous from luma
        if (plane < 2)
            s->all_blocks[plane+1] = s->all_blocks[0] + j;

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

    avcodec_get_chroma_sub_sample(avctx->pix_fmt, &s->chroma_x_shift, &s->chroma_y_shift);

    s->num_superblocks = 0;
    s->num_blocks = 0;
    for (i = 0; i < 3; i++) { // 420
        s->superblock_width[i]  = FFALIGN(s->width  >> !!i, 32) / 32;
        s->superblock_height[i] = FFALIGN(s->height >> !!i, 32) / 32;
        s->superblock_count[i]  = s->superblock_width[i] * s->superblock_height[i];
        s->num_superblocks     += s->superblock_count[i];
        s->block_width[i]  = FFALIGN(s->width,  16) >> (3 + !!i);
        s->block_height[i] = FFALIGN(s->height, 16) >> (3 + !!i);
        s->num_blocks     += s->block_width[i] * s->block_height[i];
    }

    s->blocks[0]            = av_malloc(s->num_blocks * sizeof(*s->blocks[0]));
    s->coded_blocks[0]      = av_malloc(s->num_blocks * sizeof(*s->coded_blocks[0]));
    s->all_blocks[0]        = av_malloc(16*s->num_superblocks * sizeof(*s->all_blocks[0]));
    s->superblock_coding[0] = av_malloc(s->num_superblocks);
    s->dct_tokens_base      = av_malloc(64*s->num_blocks * sizeof(*s->dct_tokens_base));
    s->mvs                  = av_malloc(2*s->num_blocks * sizeof(*s->mvs));
    s->edge_emu_buffer_base = av_malloc((s->width+64)*17*2);
    if (s->flipped_image)
        s->edge_emu_buffer  = s->edge_emu_buffer_base - (s->width+64)*17;
    else
        s->edge_emu_buffer  = s->edge_emu_buffer_base + (s->width+64)*17;

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
            init_vlc(&s->dc_vlc[i], VLC_TOKEN_BITS, 32,
                &dc_bias[i][0][1], 4, 2,
                &dc_bias[i][0][0], 4, 2, 0);

            /* group 1 AC histograms */
            init_vlc(&s->ac_vlc_1[i], VLC_TOKEN_BITS, 32,
                &ac_bias_0[i][0][1], 4, 2,
                &ac_bias_0[i][0][0], 4, 2, 0);

            /* group 2 AC histograms */
            init_vlc(&s->ac_vlc_2[i], VLC_TOKEN_BITS, 32,
                &ac_bias_1[i][0][1], 4, 2,
                &ac_bias_1[i][0][0], 4, 2, 0);

            /* group 3 AC histograms */
            init_vlc(&s->ac_vlc_3[i], VLC_TOKEN_BITS, 32,
                &ac_bias_2[i][0][1], 4, 2,
                &ac_bias_2[i][0][0], 4, 2, 0);

            /* group 4 AC histograms */
            init_vlc(&s->ac_vlc_4[i], VLC_TOKEN_BITS, 32,
                &ac_bias_3[i][0][1], 4, 2,
                &ac_bias_3[i][0][0], 4, 2, 0);
        }
    } else {
        for (i = 0; i < 16; i++) {

            /* DC histograms */
            if (init_vlc(&s->dc_vlc[i], VLC_TOKEN_BITS, 32,
                &s->huffman_table[i][0][1], 4, 2,
                &s->huffman_table[i][0][0], 4, 2, 0) < 0)
                goto vlc_fail;

            /* group 1 AC histograms */
            if (init_vlc(&s->ac_vlc_1[i], VLC_TOKEN_BITS, 32,
                &s->huffman_table[i+16][0][1], 4, 2,
                &s->huffman_table[i+16][0][0], 4, 2, 0) < 0)
                goto vlc_fail;

            /* group 2 AC histograms */
            if (init_vlc(&s->ac_vlc_2[i], VLC_TOKEN_BITS, 32,
                &s->huffman_table[i+16*2][0][1], 4, 2,
                &s->huffman_table[i+16*2][0][0], 4, 2, 0) < 0)
                goto vlc_fail;

            /* group 3 AC histograms */
            if (init_vlc(&s->ac_vlc_3[i], VLC_TOKEN_BITS, 32,
                &s->huffman_table[i+16*3][0][1], 4, 2,
                &s->huffman_table[i+16*3][0][0], 4, 2, 0) < 0)
                goto vlc_fail;

            /* group 4 AC histograms */
            if (init_vlc(&s->ac_vlc_4[i], VLC_TOKEN_BITS, 32,
                &s->huffman_table[i+16*4][0][1], 4, 2,
                &s->huffman_table[i+16*4][0][0], 4, 2, 0) < 0)
                goto vlc_fail;
        }
    }

    init_vlc(&s->superblock_run_length_vlc, VLC_LONG_RUN_BITS, 34,
        &superblock_run_length_vlc_table[0][1], 4, 2,
        &superblock_run_length_vlc_table[0][0], 4, 2, 0);

    init_vlc(&s->fragment_run_length_vlc, VLC_SHORT_RUN_BITS, 30,
        &fragment_run_length_vlc_table[0][1], 4, 2,
        &fragment_run_length_vlc_table[0][0], 4, 2, 0);

    init_vlc(&s->mode_code_vlc, VLC_MB_MODE_BITS, 8,
        &mode_code_vlc_table[0][1], 2, 1,
        &mode_code_vlc_table[0][0], 2, 1, 0);

    init_vlc(&s->motion_vector_vlc, VLC_MV_BITS, 63,
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

        // initialize blocks, for inter frames unpack_block_coding does this
        memset(s->blocks[0], 0, s->num_blocks * sizeof(*s->blocks[0]));

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
    } else {
        /* allocate a new current frame */
        s->current_frame.reference = 3;
        if(avctx->get_buffer(avctx, &s->current_frame) < 0) {
            av_log(s->avctx, AV_LOG_ERROR, "vp3: get_buffer() failed\n");
            return -1;
        }

        if (unpack_block_coding(s, &gb)){
            av_log(s->avctx, AV_LOG_ERROR, "error in unpack_block_coding\n");
            return -1;
        }
        unpack_modes(s, &gb);
        unpack_vectors(s, &gb);
    }

    if (unpack_block_qpis(s, &gb)){
        av_log(s->avctx, AV_LOG_ERROR, "error in unpack_block_qpis\n");
        return -1;
    }
    if (unpack_dct_coeffs(s, &gb)){
        av_log(s->avctx, AV_LOG_ERROR, "error in unpack_dct_coeffs\n");
        return -1;
    }

    s->linesize   = s->current_frame.linesize[0];
    s->uvlinesize = s->current_frame.linesize[1];
    s->data_offset = s->uvdata_offset = s->data_offset = 0;
    if (!s->flipped_image) {
        s->data_offset = (s->height-1) * s->linesize;
        s->uvdata_offset = ((s->height>>1)-1) * s->uvlinesize;
        s->linesize   *= -1;
        s->uvlinesize *= -1;
    }

    // 420
    for (i = 0; i < s->superblock_height[0]; i+=2) {
        render_luma_sb_row(s, i);
        if (s->keyframe)
            apply_loop_filter_intra(s, 0, 4*i - !!i, FFMIN(4*i+3, s->block_height[0]-1));
        else
            apply_loop_filter_inter(s, 0, 4*i - !!i, FFMIN(4*i+3, s->block_height[0]-1));
        if (i+1 < s->superblock_height[0]) {
            render_luma_sb_row(s, i+1);
            if (s->keyframe)
                apply_loop_filter_intra(s, 0, 4*i+4 - 1, FFMIN(4*i+7, s->block_height[0]-1));
            else
                apply_loop_filter_inter(s, 0, 4*i+4 - 1, FFMIN(4*i+7, s->block_height[0]-1));
        }
        render_chroma_sb_row(s, i>>1);
        if (s->keyframe) {
            apply_loop_filter_intra(s, 1, 2*i - !!i, FFMIN(2*i+3, s->block_height[1]-1));
            apply_loop_filter_intra(s, 2, 2*i - !!i, FFMIN(2*i+3, s->block_height[2]-1));
        } else {
            apply_loop_filter_inter(s, 1, 2*i - !!i, FFMIN(2*i+3, s->block_height[1]-1));
            apply_loop_filter_inter(s, 2, 2*i - !!i, FFMIN(2*i+3, s->block_height[2]-1));
        }
    }
    // apply the loop filter to the last row
    for (i = 0; i < 3; i++) {
        if (s->keyframe)
            apply_loop_filter_intra(s, i, s->block_height[i]-1, s->block_height[i]);
        else
            apply_loop_filter_inter(s, i, s->block_height[i]-1, s->block_height[i]);
    }

    // 420
    if (!(s->avctx->flags&CODEC_FLAG_EMU_EDGE)) {
        s->dsp.draw_edges(s->current_frame.data[0], s->current_frame.linesize[0], s->width   , s->height   , EDGE_WIDTH  );
        s->dsp.draw_edges(s->current_frame.data[1], s->current_frame.linesize[1], s->width>>1, s->height>>1, EDGE_WIDTH/2);
        s->dsp.draw_edges(s->current_frame.data[2], s->current_frame.linesize[2], s->width>>1, s->height>>1, EDGE_WIDTH/2);
    }

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
