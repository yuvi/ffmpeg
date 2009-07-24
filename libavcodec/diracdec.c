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
 * @file libavcodec/diracdec.c
 * Dirac Decoder
 * @author Marco Gerards <marco@gnu.org>
 */

#include "dirac.h"
#include "avcodec.h"
#include "dsputil.h"
#include "get_bits.h"
#include "bytestream.h"
#include "golomb.h"
#include "dirac_arith.h"
#include "mpeg12data.h"
#include "dwt.h"

/**
 * Value of Picture.reference when Picture is not a reference picture, but
 * is held for delayed output.
 */
#define DELAYED_PIC_REF 4

static AVFrame *get_frame(AVFrame *framelist[], int picnum)
{
    int i;
    for (i = 0; framelist[i]; i++)
        if (framelist[i]->display_picture_number == picnum)
            return framelist[i];
    return NULL;
}

static AVFrame *remove_frame(AVFrame *(*framelist)[], int picnum)
{
    AVFrame *remove_pic = NULL;
    int i, remove_idx = -1;

    for (i = 0; (*framelist)[i]; i++)
        if ((*framelist)[i]->display_picture_number == picnum) {
            remove_pic = (*framelist)[i];
            remove_idx = i;
        }

    if (remove_pic)
        for (i = remove_idx; (*framelist)[i]; i++)
            (*framelist)[i] = (*framelist)[i+1];

    return remove_pic;
}

static int add_frame(AVFrame *(*framelist)[], int maxframes, AVFrame *frame)
{
    int i;
    for (i = 0; i < maxframes; i++)
        if (!(*framelist)[i]) {
            (*framelist)[i] = frame;
            return 0;
        }
    return -1;
}

static int allocate_sequence_buffers(DiracContext *s)
{
    int w = CALC_PADDING(s->source.width,  MAX_DECOMPOSITIONS);
    int h = CALC_PADDING(s->source.height, MAX_DECOMPOSITIONS);
    int sbwidth  = DIVRNDUP(s->source.width, 4);
    int sbheight = DIVRNDUP(s->source.height, 4);
    int refwidth  = (s->source.width  + 2 * MAX_BLOCKSIZE) << 1;
    int refheight = (s->source.height + 2 * MAX_BLOCKSIZE) << 1;

    s->mcpic = av_malloc(s->source.width*s->source.height * 2);
    s->spatial_idwt_buffer = av_malloc(w*h * sizeof(IDWTELEM));

    s->sbsplit  = av_malloc(sbwidth * sbheight * sizeof(int));
    s->blmotion = av_malloc(((sbwidth * sbheight)<<2) * sizeof(*s->blmotion));

    s->refdata[0] = av_malloc(refwidth * refheight);
    s->refdata[1] = av_malloc(refwidth * refheight);

    if (!s->mcpic || !s->spatial_idwt_buffer || !s->sbsplit || !s->blmotion ||
        !s->refdata[0] || !s->refdata[1])
        return AVERROR(ENOMEM);
    return 0;
}

static void free_sequence_buffers(DiracContext *s)
{
    av_freep(&s->spatial_idwt_buffer);
    av_freep(&s->mcpic);
    av_freep(&s->sbsplit);
    av_freep(&s->blmotion);
    av_freep(&s->refdata[0]);
    av_freep(&s->refdata[1]);
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    DiracContext *s = avctx->priv_data;

    s->all_frames = av_mallocz(MAX_FRAMES * sizeof(AVFrame));
    if (!s->all_frames)
        return -1;

    return 0;
}

static av_cold int decode_end(AVCodecContext *avctx)
{
    DiracContext *s = avctx->priv_data;

    av_free(s->all_frames);
    free_sequence_buffers(s);

    return 0;
}

/**
 * Dequantize a coefficient
 *
 * @param coeff coefficient to dequantize
 * @param qoffset quantizer offset
 * @param qfactor quantizer factor
 * @return dequantized coefficient
 */
static inline int coeff_dequant(int coeff, int qoffset, int qfactor)
{
    if (! coeff)
        return 0;

    coeff *= qfactor;

    coeff += qoffset;
    coeff >>= 2;

    return coeff;
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
static inline void coeff_unpack_arith(DiracContext *s, SubBand *b,
                         IDWTELEM *coeffp, int x, int y,
                         int qoffset, int qfactor)
{
    int parent = 0;
    int nhood;
    int coeff;
    int read_sign;
    int sign_ctx;

    /* The value of the pixel belonging to the lower level. */
    if (b->parent)
        parent = b->parent->ibuf[b->parent->stride * (y>>1) + (x>>1)] != 0;

    /* Determine if the pixel has only zeros in its neighbourhood. */
    nhood = zero_neighbourhood(coeffp, x, y, b->stride);

    sign_ctx = sign_predict(coeffp, b->orientation, x, y, b->stride);

    coeff = dirac_get_arith_uint(&s->arith, (parent<<1) | nhood,
                                 ARITH_CONTEXT_COEFF_DATA);

    read_sign = coeff;
    coeff = coeff_dequant(coeff, qoffset, qfactor);
    if (read_sign && dirac_get_arith_bit(&s->arith, sign_ctx))
        coeff = -coeff;

    *coeffp = coeff;
}

static inline void coeff_unpack_vlc(DiracContext *s, SubBand *b, IDWTELEM *buf,
                                    int x, int y, int qoffset, int qfactor)
{
    int coeff, sign;
    coeff = sign = svq3_get_ue_golomb(&s->gb);
    coeff = coeff_dequant(coeff, qoffset, qfactor);
    if (sign && get_bits1(&s->gb))
        coeff = -coeff;

    *buf = coeff;
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
static inline void codeblock(DiracContext *s, SubBand *b,
                      int left, int right, int top, int bottom,
                      unsigned int *quant, int blockcnt_one, int is_arith)
{
    int x, y;
    unsigned int qoffset, qfactor;
    IDWTELEM *buf;

    if (!blockcnt_one) {
        int zero_block;
        /* Determine if this codeblock is a zero block. */
        if (is_arith)
            zero_block = dirac_get_arith_bit(&s->arith, ARITH_CONTEXT_ZERO_BLOCK);
        else
            zero_block = get_bits1(&s->gb);

        if (zero_block)
            return;

        if (s->codeblock_mode && is_arith)
            *quant += dirac_get_arith_int(&s->arith, ARITH_CONTEXT_Q_OFFSET_FOLLOW,
                                          ARITH_CONTEXT_Q_OFFSET_DATA,
                                          ARITH_CONTEXT_Q_OFFSET_SIGN);
        else if (s->codeblock_mode)
            *quant += dirac_get_se_golomb(&s->gb);
    }

    qfactor = coeff_quant_factor(*quant);
    qoffset = coeff_quant_offset(s->num_refs == 0, *quant) + 2;

    buf = b->ibuf + top*b->stride;
    for (y = top; y < bottom; y++) {
        for (x = left; x < right; x++) {
            if (is_arith)
                coeff_unpack_arith(s, b, buf+x, x, y, qoffset, qfactor);
            else
                coeff_unpack_vlc(s, b, buf+x, x, y, qoffset, qfactor);
        }
        buf += b->stride;
    }
}

/**
 * Intra DC Prediction
 *
 * @param data coefficients
 */
static inline void intra_dc_prediction(SubBand *b)
{
    int x, y;
    IDWTELEM *line = b->ibuf;

    for (y = 0; y < b->height; y++) {
        for (x = 0; x < b->width; x++) {
            line[x] += intra_dc_coeff_prediction(&line[x], x, y, b->stride);
        }
        line += b->stride;
    }
}

/**
 * Decode a subband
 *
 * @param data coefficients
 * @param level subband level
 * @param orientation orientation of the subband
 */
static av_always_inline void decode_subband_internal(DiracContext *s, SubBand *b, int is_arith)
{
    GetBitContext *gb = &s->gb;
    unsigned int length;
    unsigned int quant;
    int cb_x, cb_y;
    int cb_numx = s->codeblocksh[b->level + (b->orientation != subband_ll)];
    int cb_numy = s->codeblocksv[b->level + (b->orientation != subband_ll)];
    int blockcnt_one = (cb_numx + cb_numy) == 2;
    int left, right, top, bottom;

    length = svq3_get_ue_golomb(gb);
    if (!length)
        return;

    quant = svq3_get_ue_golomb(gb);
    align_get_bits(gb);

    if (is_arith)
        dirac_init_arith_decoder(&s->arith, gb, length);

    top = 0;
    for (cb_y = 0; cb_y < cb_numy; cb_y++) {
        bottom = (b->height * (cb_y+1)) / cb_numy;
        left = 0;
        for (cb_x = 0; cb_x < cb_numx; cb_x++) {
            right = (b->width * (cb_x+1)) / cb_numx;
            codeblock(s, b, left, right, top, bottom, &quant, blockcnt_one, is_arith);
            left = right;
        }
        top = bottom;
    }

    if (is_arith)
        dirac_get_arith_terminate(&s->arith);

    if (b->orientation == subband_ll && s->num_refs == 0)
        intra_dc_prediction(b);
}

static av_noinline void decode_subband_arith(DiracContext *s, SubBand *b)
{
    decode_subband_internal(s, b, 1);
}

static av_noinline void decode_subband_vlc(DiracContext *s, SubBand *b)
{
    decode_subband_internal(s, b, 0);
}

/**
 * Decode a single component
 *
 * @param coeffs coefficients for this component
 */
static void decode_component(DiracContext *s, int comp)
{
    GetBitContext *gb = &s->gb;
    int level;
    dirac_subband orientation;

    /* Unpack all subbands at all levels. */
    for (level = 0; level < s->wavelet_depth; level++) {
        for (orientation = (level ? 1 : 0); orientation < 4; orientation++) {
            SubBand *b = &s->plane[comp].band[level][orientation];
            align_get_bits(gb);
            if (s->is_arith)
                decode_subband_arith(s, b);
            else
                decode_subband_vlc(s, b);
        }
    }
}

static void init_planes(DiracContext *s)
{
    int i, w, h, level, orientation;

    for (i = 0; i < 3; i++) {
        Plane *p = &s->plane[i];

        p->width  = s->source.width  >> (i ? s->chroma_hshift : 0);
        p->height = s->source.height >> (i ? s->chroma_vshift : 0);
        p->padded_width  = w = CALC_PADDING(p->width , s->wavelet_depth);
        p->padded_height = h = CALC_PADDING(p->height, s->wavelet_depth);

        for (level = s->wavelet_depth-1; level >= 0; level--) {
            for (orientation = level ? 1 : 0; orientation < 4; orientation++) {
                SubBand *b = &p->band[level][orientation];

                b->ibuf   = s->spatial_idwt_buffer;
                b->level  = level;
                b->stride = p->padded_width << (s->wavelet_depth - level);
                b->width  = (w + !(orientation&1))>>1;
                b->height = (h + !(orientation>1))>>1;
                b->orientation = orientation;

                if (orientation & 1)
                    b->ibuf += (w+1)>>1;
                if (orientation > 1)
                    b->ibuf += b->stride>>1;

                if (level)
                    b->parent = &p->band[level-1][orientation];
            }
            w = (w+1)>>1;
            h = (h+1)>>1;
        }

        if (i > 0) {
            p->xblen = s->plane[0].xblen >> s->chroma_hshift;
            p->yblen = s->plane[0].yblen >> s->chroma_vshift;
            p->xbsep = s->plane[0].xbsep >> s->chroma_hshift;
            p->ybsep = s->plane[0].ybsep >> s->chroma_vshift;
        }

        p->xoffset = (p->xblen - p->xbsep) / 2;
        p->yoffset = (p->yblen - p->ybsep) / 2;

        p->total_wt_bits = s->picture_weight_precision +
            av_log2(p->xoffset) + av_log2(p->yoffset) + 4;

        if (s->num_refs) {
            p->current_blwidth  = (p->width  - p->xoffset) / p->xbsep + 1;
            p->current_blheight = (p->height - p->yoffset) / p->ybsep + 1;
        }
    }
}

static const struct {
    uint8_t xblen;
    uint8_t yblen;
    uint8_t xbsep;
    uint8_t ybsep;
} dirac_block_param_defaults[] = {
    {  8,  8,  4,  4 },
    { 12, 12,  8,  8 },
    { 16, 16, 12, 12 },
    { 24, 24, 16, 16 },
};

/**
 * Unpack the motion compensation parameters
 */
static int dirac_unpack_prediction_parameters(DiracContext *s)
{
    GetBitContext *gb = &s->gb;
    unsigned idx;

    align_get_bits(gb);
    idx = svq3_get_ue_golomb(gb);

    if (idx > 4)
        return -1;

    if (idx == 0) {
        s->plane[0].xblen = svq3_get_ue_golomb(gb);
        s->plane[0].yblen = svq3_get_ue_golomb(gb);
        s->plane[0].xbsep = svq3_get_ue_golomb(gb);
        s->plane[0].ybsep = svq3_get_ue_golomb(gb);
    } else {
        s->plane[0].xblen = dirac_block_param_defaults[idx - 1].xblen;
        s->plane[0].yblen = dirac_block_param_defaults[idx - 1].yblen;
        s->plane[0].xbsep = dirac_block_param_defaults[idx - 1].xbsep;
        s->plane[0].ybsep = dirac_block_param_defaults[idx - 1].ybsep;
    }

    if (!s->plane[0].xbsep || !s->plane[0].ybsep) {
        av_log(s->avctx, AV_LOG_ERROR, "Invalid block separation of 0\n");
        return -1;
    }
    if (s->plane[0].xbsep > s->plane[0].xblen || s->plane[0].ybsep > s->plane[0].yblen) {
        av_log(s->avctx, AV_LOG_ERROR, "Block seperation greater than size\n");
        return -1;
    }
    if (s->plane[0].xblen > MAX_BLOCKSIZE || s->plane[0].yblen > MAX_BLOCKSIZE) {
        av_log(s->avctx, AV_LOG_ERROR, "Block size larger than 64 unsupported\n");
        return -1;
    }

    /* Read motion vector precision. */
    s->mv_precision = svq3_get_ue_golomb(gb);
    if (s->mv_precision > 3) {
        av_log(s->avctx, AV_LOG_ERROR, "MV precision finer than eighth-pel\n");
        return -1;
    }

    /* Read the global motion compensation parameters. */
    s->globalmc_flag = get_bits1(gb);
    if (s->globalmc_flag) {
        int ref;
        av_log(s->avctx, AV_LOG_WARNING, "GMC not fully supported\n");
        for (ref = 0; ref < s->num_refs; ref++) {
            memset(&s->globalmc, 0, sizeof(s->globalmc));

            /* Pan/tilt parameters. */
            if (get_bits1(gb)) {
                s->globalmc.pan_tilt[0] = dirac_get_se_golomb(gb);
                s->globalmc.pan_tilt[1] = dirac_get_se_golomb(gb);
            }

            /* Rotation/shear parameters. */
            if (get_bits1(gb)) {
                s->globalmc.zrs_exp = svq3_get_ue_golomb(gb);
                s->globalmc.zrs[0][0] = dirac_get_se_golomb(gb);
                s->globalmc.zrs[0][1] = dirac_get_se_golomb(gb);
                s->globalmc.zrs[1][0] = dirac_get_se_golomb(gb);
                s->globalmc.zrs[1][1] = dirac_get_se_golomb(gb);
            } else {
                s->globalmc.zrs[0][0] = 1;
                s->globalmc.zrs[1][1] = 1;
            }

            /* Perspective parameters. */
            if (get_bits1(gb)) {
                s->globalmc.perspective_exp = svq3_get_ue_golomb(gb);
                s->globalmc.perspective[0] = dirac_get_se_golomb(gb);
                s->globalmc.perspective[1] = dirac_get_se_golomb(gb);
            }
        }
    }

    /* Picture prediction mode. May be used in the future. */
    if (svq3_get_ue_golomb(gb) != 0) {
        av_log(s->avctx, AV_LOG_ERROR, "Unknown picture prediction mode\n");
        return -1;
    }

    /* Default weights */
    s->picture_weight_precision = 1;
    s->picture_weight_ref1      = 1;
    s->picture_weight_ref2      = 1;

    /* Override reference picture weights. */
    if (get_bits1(gb)) {
        s->picture_weight_precision = svq3_get_ue_golomb(gb);
        s->picture_weight_ref1 = dirac_get_se_golomb(gb);
        if (s->num_refs == 2)
            s->picture_weight_ref2 = dirac_get_se_golomb(gb);
    }

    return 0;
}

/**
 * Blockmode prediction
 *
 * @param x    horizontal position of the MC block
 * @param y    vertical position of the MC block
 */
static void blockmode_prediction(DiracContext *s, int x, int y)
{
    int res = dirac_get_arith_bit(&s->arith, ARITH_CONTEXT_PMODE_REF1);

    res ^= mode_prediction(s, x, y, DIRAC_REF_MASK_REF1, 0);
    s->blmotion[y * s->blwidth + x].use_ref |= res;
    if (s->num_refs == 2) {
        res = dirac_get_arith_bit(&s->arith, ARITH_CONTEXT_PMODE_REF2);
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
static void blockglob_prediction(DiracContext *s, int x, int y)
{
    /* Global motion compensation is not used at all. */
    if (!s->globalmc_flag)
        return;

    /* Global motion compensation is not used for this block. */
    if (s->blmotion[y * s->blwidth + x].use_ref & 3) {
        int res = dirac_get_arith_bit(&s->arith, ARITH_CONTEXT_GLOBAL_BLOCK);
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
static void propagate_block_data(DiracContext *s, int step, int x, int y)
{
    int i, j;

    /* XXX: For now this is rather inefficient, because everything is
       copied.  This function is called quite often. */
    for (j = y; j < y + step; j++)
        for (i = x; i < x + step; i++)
            s->blmotion[j * s->blwidth + i] = s->blmotion[y * s->blwidth + x];
}

static void unpack_block_dc(DiracContext *s, int x, int y, int comp)
{
    int res;

    if (s->blmotion[y * s->blwidth + x].use_ref & 3) {
        s->blmotion[y * s->blwidth + x].dc[comp] = 0;
        return;
    }

    res = dirac_get_arith_int(&s->arith, ARITH_CONTEXT_DC_F1,
                              ARITH_CONTEXT_DC_DATA, ARITH_CONTEXT_DC_SIGN);
    res += block_dc_prediction(s, x, y, comp);

    s->blmotion[y * s->blwidth + x].dc[comp] = res;
}

/**
 * Unpack a single motion vector
 *
 * @param ref reference frame
 * @param dir direction horizontal=0, vertical=1
 */
static void dirac_unpack_motion_vector(DiracContext *s, int ref, int dir,
                                       int x, int y)
{
    int res;
    const int refmask = (ref + 1) | DIRAC_REF_MASK_GLOBAL;

    /* First determine if for this block in the specific reference
       frame a motion vector is required. */
    if ((s->blmotion[y * s->blwidth + x].use_ref & refmask) != ref + 1)
        return;

    res = dirac_get_arith_int(&s->arith, ARITH_CONTEXT_VECTOR_F1,
                        ARITH_CONTEXT_VECTOR_DATA, ARITH_CONTEXT_VECTOR_SIGN);
    res += motion_vector_prediction(s, x, y, ref, dir);
    s->blmotion[y * s->blwidth + x].vect[ref][dir] = res;
}

/**
 * Unpack motion vectors
 *
 * @param ref reference frame
 * @param dir direction horizontal=0, vertical=1
 */
static void dirac_unpack_motion_vectors(DiracContext *s, int ref, int dir)
{
    GetBitContext *gb = &s->gb;
    unsigned int length;
    int x, y;

    length = svq3_get_ue_golomb(gb);
    dirac_init_arith_decoder(&s->arith, gb, length);
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
    dirac_get_arith_terminate(&s->arith);
}

/**
 * Unpack the motion compensation parameters
 */
static int dirac_unpack_block_motion_data(DiracContext *s)
{
    GetBitContext *gb = &s->gb;
    int i;
    unsigned int length;
    int comp;
    int x, y;

    align_get_bits(gb);

    s->sbwidth  = DIVRNDUP(s->source.width,  (s->plane[0].xbsep << 2));
    s->sbheight = DIVRNDUP(s->source.height, (s->plane[0].ybsep << 2));
    s->blwidth  = s->sbwidth  << 2;
    s->blheight = s->sbheight << 2;

    memset(s->blmotion, 0, s->blwidth * s->blheight * sizeof(*s->blmotion));

    /* Superblock splitmodes. */
    length = svq3_get_ue_golomb(gb);
    dirac_init_arith_decoder(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
            int res = dirac_get_arith_uint(&s->arith, ARITH_CONTEXT_SB_F1,
                                           ARITH_CONTEXT_SB_DATA);
            s->sbsplit[y * s->sbwidth + x] = res + split_prediction(s, x, y);
            s->sbsplit[y * s->sbwidth + x] %= 3;
        }
    dirac_get_arith_terminate(&s->arith);

    /* Prediction modes. */
    length = svq3_get_ue_golomb(gb);
    dirac_init_arith_decoder(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
            int q, p;
            int blkcnt = 1 << s->sbsplit[y * s->sbwidth + x];
            int step   = 4 >> s->sbsplit[y * s->sbwidth + x];

            for (q = 0; q < blkcnt; q++)
                for (p = 0; p < blkcnt; p++) {
                    int xblk = 4*x + p*step;
                    int yblk = 4*y + q*step;
                    blockmode_prediction(s, xblk, yblk);
                    blockglob_prediction(s, xblk, yblk);
                    propagate_block_data(s, step, xblk, yblk);
                }
        }
    dirac_get_arith_terminate(&s->arith);

    /* Unpack the motion vectors. */
    for (i = 0; i < s->num_refs; i++) {
        dirac_unpack_motion_vectors(s, i, 0);
        dirac_unpack_motion_vectors(s, i, 1);
    }

    /* Unpack the DC values for all the three components (YUV). */
    for (comp = 0; comp < 3; comp++) {
        /* Unpack the DC values. */
        length = svq3_get_ue_golomb(gb);
        dirac_init_arith_decoder(&s->arith, gb, length);
        for (y = 0; y < s->sbheight; y++)
            for (x = 0; x < s->sbwidth; x++) {
                int q, p;
                int blkcnt = 1 << s->sbsplit[y * s->sbwidth + x];
                int step   = 4 >> s->sbsplit[y * s->sbwidth + x];

                for (q = 0; q < blkcnt; q++)
                    for (p = 0; p < blkcnt; p++) {
                        int xblk = 4*x + p*step;
                        int yblk = 4*y + q*step;
                        unpack_block_dc(s, xblk, yblk, comp);
                        propagate_block_data(s, step, xblk, yblk);
                    }
            }
        dirac_get_arith_terminate(&s->arith);
    }

    return 0;
}

/**
 * Decode a frame.
 *
 * @return 0 when successful, otherwise -1 is returned
 */
static int dirac_decode_frame_internal(DiracContext *s)
{
    IDWTELEM *line;
    int16_t *mcline;
    int comp;
    int x, y;

    for (comp = 0; comp < 3; comp++) {
        uint8_t *frame = s->current_picture->data[comp];
        int width, height;

        width  = s->source.width  >> (comp ? s->chroma_hshift : 0);
        height = s->source.height >> (comp ? s->chroma_vshift : 0);

        memset(s->spatial_idwt_buffer, 0,
               s->plane[comp].padded_width * s->plane[comp].padded_height * sizeof(IDWTELEM));

        if (!s->zero_res)
            decode_component(s, comp);

        ff_spatial_idwt2(s->spatial_idwt_buffer, s->plane[comp].padded_width, s->plane[comp].padded_height,
                  s->plane[comp].padded_width, s->wavelet_idx+2, s->wavelet_depth);

        if (s->num_refs) {
            if (dirac_motion_compensation(s, comp)) {
                return -1;
            }
        }

        /* Copy the decoded coefficients into the frame and also add
           the data calculated by MC. */
        line = s->spatial_idwt_buffer;
        if (s->num_refs) {
            int bias = 257 << (s->plane[comp].total_wt_bits - 1);
            mcline    = s->mcpic;
            for (y = 0; y < height; y++) {
                for (x = 0; x < width; x++) {
                    int coeff = mcline[x] + bias;
                    coeff = line[x] + (coeff >> s->plane[comp].total_wt_bits);
                    frame[x]= av_clip_uint8(coeff);
                }

                line  += s->plane[comp].padded_width;
                frame += s->current_picture->linesize[comp];
                mcline    += s->plane[comp].width;
            }
        } else {
            for (y = 0; y < height; y++) {
                for (x = 0; x < width; x++) {
                    frame[x]= av_clip_uint8(line[x] + 128);
                }

                line  += s->plane[comp].padded_width;
                frame += s->current_picture->linesize[comp];
            }
        }
    }

    return 0;
}

/**
 * Parse a frame and setup DiracContext to decode it
 *
 * @return 0 when successful, otherwise -1 is returned
 */
static int dirac_decode_picture_header(DiracContext *s)
{
    int retire, picnum;
    int i, level;
    GetBitContext *gb = &s->gb;

    picnum= s->current_picture->display_picture_number = get_bits_long(gb, 32);

    s->ref_pics[0] = s->ref_pics[1] = NULL;
    for (i = 0; i < s->num_refs; i++)
        s->ref_pics[i] = get_frame(s->ref_frames, picnum + dirac_get_se_golomb(gb));

    /* Retire the reference frames that are not used anymore. */
    if (s->current_picture->reference) {
        retire = picnum + dirac_get_se_golomb(gb);
        if (retire != picnum) {
            AVFrame *retire_pic = remove_frame(&s->ref_frames, retire);

            if (retire_pic) {
                if (retire_pic->reference & DELAYED_PIC_REF)
                    retire_pic->reference = DELAYED_PIC_REF;
                else
                    retire_pic->reference = 0;
            } else
                av_log(s->avctx, AV_LOG_DEBUG, "Frame to retire not found\n");
        }

        // if reference array is full, remove the oldest as per the spec
        while (add_frame(&s->ref_frames, MAX_REFERENCE_FRAMES, s->current_picture)) {
            av_log(s->avctx, AV_LOG_ERROR, "Reference frame overflow\n");
            remove_frame(&s->ref_frames, s->ref_frames[0]->display_picture_number);
        }
    }

    if (s->num_refs) {
        if (dirac_unpack_prediction_parameters(s))
            return -1;
        if (dirac_unpack_block_motion_data(s))
            return -1;
    }

    align_get_bits(gb);

    s->zero_res = s->num_refs ? get_bits1(gb) : 0;
    if (s->zero_res)
        return 0;

    s->wavelet_idx = svq3_get_ue_golomb(gb);

    if (s->wavelet_idx > 6)
        return -1;

    s->wavelet_depth = svq3_get_ue_golomb(gb);

    if (!s->low_delay) {
        /* Codeblock paramaters (core syntax only) */
        if (get_bits1(gb)) {
            for (i = 0; i <= s->wavelet_depth; i++) {
                s->codeblocksh[i] = svq3_get_ue_golomb(gb);
                s->codeblocksv[i] = svq3_get_ue_golomb(gb);
            }

            s->codeblock_mode = svq3_get_ue_golomb(gb);
        } else
            for (i = 0; i <= s->wavelet_depth; i++)
                s->codeblocksh[i] = s->codeblocksv[i] = 1;
    } else {
        s->x_slices        = svq3_get_ue_golomb(gb);
        s->y_slices        = svq3_get_ue_golomb(gb);
        s->slice_bytes.num = svq3_get_ue_golomb(gb);
        s->slice_bytes.den = svq3_get_ue_golomb(gb);

        if (get_bits1(gb)) {
            // custom quantization matrix
            s->quant_matrix[0][0] = svq3_get_ue_golomb(gb);
            for (level = 0; level < s->wavelet_depth; level++) {
                s->quant_matrix[level][1] = svq3_get_ue_golomb(gb);
                s->quant_matrix[level][2] = svq3_get_ue_golomb(gb);
                s->quant_matrix[level][3] = svq3_get_ue_golomb(gb);
            }
        } else {
            // default quantization matrix
            for (level = 0; level < s->wavelet_depth; level++)
                for (i = 0; i < 4; i++) {
                    s->quant_matrix[level][i] =
                        ff_dirac_default_qmat[s->wavelet_idx][level][i];

                    // haar with no shift differs for different depths
                    if (s->wavelet_idx == 3)
                        s->quant_matrix[level][i] +=
                            4*(s->wavelet_depth-1 - level);
                }
        }
    }
    init_planes(s);
    return 0;
}

static int get_delayed_pic(DiracContext *s, AVFrame *picture, int *data_size)
{
    AVFrame *out = s->delay_frames[0];
    int i, out_idx = 0;

    // find frame with lowest picture number
    for (i = 1; s->delay_frames[i]; i++)
        if (s->delay_frames[i]->display_picture_number < out->display_picture_number) {
            out = s->delay_frames[i];
            out_idx = i;
        }

    for (i = out_idx; s->delay_frames[i]; i++)
        s->delay_frames[i] = s->delay_frames[i+1];

    if (out) {
        out->reference ^= DELAYED_PIC_REF;
        *data_size = sizeof(AVFrame);
        *picture = *out;
    }

    return 0;
}

// 4 byte start code + byte parse code + 4 byte size + 4 byte previous size
#define DATA_UNIT_HEADER_SIZE 13

static int dirac_decode_data_unit(AVCodecContext *avctx, const uint8_t *buf, int size)
{
    DiracContext *s = avctx->priv_data;
    AVFrame *pic = NULL;
    int i, parse_code = buf[4];

    init_get_bits(&s->gb, &buf[13], (size - DATA_UNIT_HEADER_SIZE) * 8);

    if (parse_code == pc_seq_header) {
        if (s->seen_sequence_header)
            return 0;

        if (ff_dirac_parse_sequence_header(&s->gb, avctx, &s->source))
            return -1;

        avcodec_get_chroma_sub_sample(avctx->pix_fmt, &s->chroma_hshift,
                                      &s->chroma_vshift);

        if (allocate_sequence_buffers(s))
            return -1;
        s->seen_sequence_header = 1;
    } else if (parse_code == pc_eos) {
        free_sequence_buffers(s);
        s->seen_sequence_header = 0;
    } else if (parse_code & 0x8) {
        // picture data unit

        // find an unused frame
        for (i = 0; i < MAX_FRAMES; i++)
            if (s->all_frames[i].data[0] == NULL)
                pic = &s->all_frames[i];
        if (!pic) {
            av_log(avctx, AV_LOG_ERROR, "framelist full\n");
            return -1;
        }

        avcodec_get_frame_defaults(pic);
        s->num_refs    =  parse_code & 0x03;
        s->is_arith    = (parse_code & 0x48) == 0x08;
        s->low_delay   = (parse_code & 0x88) == 0x88;
        pic->reference = (parse_code & 0x0C) == 0x0C;
        pic->key_frame = s->num_refs == 0;
        pic->pict_type = s->num_refs + 1;

        if (avctx->get_buffer(avctx, pic) < 0) {
            av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
            return -1;
        }
        s->current_picture = pic;

        if (dirac_decode_picture_header(s))
            return -1;

        if (dirac_decode_frame_internal(s))
            return -1;
    }
    return 0;
}

static int dirac_decode_frame(AVCodecContext *avctx, void *data, int *data_size,
                              AVPacket *pkt)
{
    DiracContext *s = avctx->priv_data;
    AVFrame *picture = data;
    uint8_t *buf = pkt->data;
    int buf_size = pkt->size;
    int i, data_unit_size, buf_idx = 0;

    // release unused frames
    for (i = 0; i < MAX_FRAMES; i++)
        if (s->all_frames[i].data[0] && !s->all_frames[i].reference)
            avctx->release_buffer(avctx, &s->all_frames[i]);

    s->current_picture = NULL;
    *data_size = 0;

    // end of stream, so flush delayed pics
    if (buf_size == 0)
        return get_delayed_pic(s, picture, data_size);

    for (;;) {
        // BBCD start code search
        for (; buf_idx + DATA_UNIT_HEADER_SIZE < buf_size; buf_idx++) {
            if (buf[buf_idx  ] == 'B' && buf[buf_idx+1] == 'B' &&
                buf[buf_idx+2] == 'C' && buf[buf_idx+3] == 'D')
                break;
        }

        if (buf_idx + DATA_UNIT_HEADER_SIZE >= buf_size)
            break;

        data_unit_size = AV_RB32(buf+buf_idx+5);
        if (buf_idx + data_unit_size > buf_size) {
            av_log(s->avctx, AV_LOG_ERROR,
                "Data unit with size %d is larger than input buffer, discarding\n",
                data_unit_size);
            buf_idx += 4;
            continue;
        }

        if (dirac_decode_data_unit(avctx, buf+buf_idx, data_unit_size))
            return -1;
        buf_idx += data_unit_size;
    }

    if (!s->current_picture)
        return 0;

    if (s->current_picture->display_picture_number > avctx->frame_number) {
        AVFrame *delayed_frame = remove_frame(&s->delay_frames, avctx->frame_number);

        s->current_picture->reference |= DELAYED_PIC_REF;
        if (add_frame(&s->delay_frames, MAX_DELAYED_FRAMES, s->current_picture))
            av_log(avctx, AV_LOG_ERROR, "Delay frame overflow\n");

        if (delayed_frame) {
            delayed_frame->reference ^= DELAYED_PIC_REF;
            *data_size = sizeof(AVFrame);
            *picture = *delayed_frame;
        }
    } else {
        /* The right frame at the right time :-) */
        *data_size = sizeof(AVFrame);
        *picture = *s->current_picture;
    }

    return buf_idx;
}

AVCodec dirac_decoder = {
    "dirac",
    CODEC_TYPE_VIDEO,
    CODEC_ID_DIRAC,
    sizeof(DiracContext),
    decode_init,
    NULL,
    decode_end,
    dirac_decode_frame,
    CODEC_CAP_DR1 | CODEC_CAP_DELAY,
    .long_name = NULL_IF_CONFIG_SMALL("BBC Dirac"),
};
