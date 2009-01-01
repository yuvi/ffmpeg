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
 * @file diracdec.c
 * Dirac Decoder
 * @author Marco Gerards <marco@gnu.org>
 */

#include "dirac.h"
#include "avcodec.h"
#include "dsputil.h"
#include "bitstream.h"
#include "bytestream.h"
#include "golomb.h"
#include "dirac_arith.h"
#include "dirac_wavelet.h"
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

static int decode_init(AVCodecContext *avctx)
{
    DiracContext *s = avctx->priv_data;

    s->all_frames = av_mallocz(MAX_FRAMES * sizeof(AVFrame));
    if (!s->all_frames)
        return -1;

    return 0;
}

static int decode_end(AVCodecContext *avctx)
{
    DiracContext *s = avctx->priv_data;

    av_free(s->all_frames);
    av_free(s->spatial_idwt_buffer);

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
    int idx;
    int coeff;
    int read_sign;
    struct dirac_arith_context_set *context;

    /* The value of the pixel belonging to the lower level. */
    if (b->parent)
        parent = b->parent->ibuf[b->parent->stride * (y>>1) + (x>>1)] != 0;

    /* Determine if the pixel has only zeros in its neighbourhood. */
    nhood = zero_neighbourhood(coeffp, x, y, b->stride);

    /* Calculate an index into context_sets_waveletcoeff. */
    idx = parent * 6 + (!nhood) * 3;
    idx += sign_predict(coeffp, b->orientation, x, y, b->stride);

    context = &ff_dirac_context_sets_waveletcoeff[idx];

    coeff = dirac_arith_read_uint(&s->arith, context);

    read_sign = coeff;
    coeff = coeff_dequant(coeff, qoffset, qfactor);
    if (read_sign) {
        if (dirac_arith_get_bit(&s->arith, context->sign))
            coeff = -coeff;
    }

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
            zero_block = dirac_arith_get_bit(&s->arith, ARITH_CONTEXT_ZERO_BLOCK);
        else
            zero_block = get_bits1(&s->gb);

        if (zero_block)
            return;

        if (s->codeblock_mode && is_arith)
            *quant += dirac_arith_read_int(&s->arith, &ff_dirac_context_set_quant);
        else if (s->codeblock_mode)
            *quant += dirac_get_se_golomb(&s->gb);
    }

    qfactor = coeff_quant_factor(*quant);
    qoffset = coeff_quant_offset(s->refs == 0, *quant) + 2;

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
        dirac_arith_init(&s->arith, gb, length);

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
        dirac_arith_flush(&s->arith);

    if (b->orientation == subband_ll && s->refs == 0)
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

#define PAD(size, depth) \
    (((size + (1 << depth) - 1) >> depth) << depth)

    av_freep(&s->spatial_idwt_buffer);

    for (i = 0; i < 3; i++) {
        Plane *p = &s->plane[i];

        p->width  = s->source.width  >> (i ? s->chroma_hshift : 0);
        p->height = s->source.height >> (i ? s->chroma_vshift : 0);
        p->padded_width  = w = PAD(p->width , s->wavelet_depth);
        p->padded_height = h = PAD(p->height, s->wavelet_depth);

        if (i == 0)
            s->spatial_idwt_buffer =
                av_malloc(p->padded_width*p->padded_height * sizeof(IDWTELEM));

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

        if (s->refs) {
            p->current_blwidth  = (p->width  - p->xoffset) / p->xbsep + 1;
            p->current_blheight = (p->height - p->yoffset) / p->ybsep + 1;
        }
    }
#undef PAD
}

/**
 * Unpack the motion compensation parameters
 */
static int dirac_unpack_prediction_parameters(DiracContext *s)
{
    GetBitContext *gb = &s->gb;

    /* Read block parameters. */
    unsigned int idx = svq3_get_ue_golomb(gb);

    if (idx > 4)
        return -1;

    if (idx == 0) {
        s->plane[0].xblen = svq3_get_ue_golomb(gb);
        s->plane[0].yblen = svq3_get_ue_golomb(gb);
        s->plane[0].xbsep = svq3_get_ue_golomb(gb);
        s->plane[0].ybsep = svq3_get_ue_golomb(gb);
    } else {
        s->plane[0].xblen = ff_dirac_block_param_defaults[idx - 1].xblen;
        s->plane[0].yblen = ff_dirac_block_param_defaults[idx - 1].yblen;
        s->plane[0].xbsep = ff_dirac_block_param_defaults[idx - 1].xbsep;
        s->plane[0].ybsep = ff_dirac_block_param_defaults[idx - 1].ybsep;
    }

    /* Read motion vector precision. */
    s->mv_precision = svq3_get_ue_golomb(gb);

    /* Read the global motion compensation parameters. */
    s->globalmc_flag = get_bits1(gb);
    if (s->globalmc_flag) {
        int ref;
        av_log(s->avctx, AV_LOG_WARNING, "GMC not fully supported\n");
        for (ref = 0; ref < s->refs; ref++) {
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
        if (s->refs == 2)
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
static void blockglob_prediction(DiracContext *s, int x, int y)
{
    /* Global motion compensation is not used at all. */
    if (!s->globalmc_flag)
        return;

    /* Global motion compensation is not used for this block. */
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

    res = dirac_arith_read_int(&s->arith, &ff_dirac_context_set_dc);
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

    res = dirac_arith_read_int(&s->arith, &ff_dirac_context_set_mv);
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
static int dirac_unpack_block_motion_data(DiracContext *s)
{
    GetBitContext *gb = &s->gb;
    int i;
    unsigned int length;
    int comp;
    int x, y;

#define DIVRNDUP(a, b) ((a + b - 1) / b)

    s->sbwidth  = DIVRNDUP(s->source.width,  (s->plane[0].xbsep << 2));
    s->sbheight = DIVRNDUP(s->source.height, (s->plane[0].ybsep << 2));
    s->blwidth  = s->sbwidth  << 2;
    s->blheight = s->sbheight << 2;

    s->sbsplit  = av_mallocz(s->sbwidth * s->sbheight * sizeof(int));
    if (!s->sbsplit) {
        av_log(s->avctx, AV_LOG_ERROR, "av_mallocz() failed\n");
        return -1;
    }

    s->blmotion = av_mallocz(s->blwidth * s->blheight * sizeof(*s->blmotion));
    if (!s->blmotion) {
        av_freep(&s->sbsplit);
        av_log(s->avctx, AV_LOG_ERROR, "av_mallocz() failed\n");
        return -1;
    }

    /* Superblock splitmodes. */
    length = svq3_get_ue_golomb(gb);
    dirac_arith_init(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
            int res = dirac_arith_read_uint(&s->arith, &ff_dirac_context_set_split);
            s->sbsplit[y * s->sbwidth + x] = res + split_prediction(s, x, y);
            s->sbsplit[y * s->sbwidth + x] %= 3;
        }
    dirac_arith_flush(&s->arith);

    /* Prediction modes. */
    length = svq3_get_ue_golomb(gb);
    dirac_arith_init(&s->arith, gb, length);
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
    dirac_arith_flush(&s->arith);

    /* Unpack the motion vectors. */
    for (i = 0; i < s->refs; i++) {
        dirac_unpack_motion_vectors(s, i, 0);
        dirac_unpack_motion_vectors(s, i, 1);
    }

    /* Unpack the DC values for all the three components (YUV). */
    for (comp = 0; comp < 3; comp++) {
        /* Unpack the DC values. */
        length = svq3_get_ue_golomb(gb);
        dirac_arith_init(&s->arith, gb, length);
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
        dirac_arith_flush(&s->arith);
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

        if (s->refs) {
            if (dirac_motion_compensation(s, comp)) {
                av_freep(&s->sbsplit);
                av_freep(&s->blmotion);

                return -1;
            }
        }

        /* Copy the decoded coefficients into the frame and also add
           the data calculated by MC. */
        line = s->spatial_idwt_buffer;
        if (s->refs) {
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

        /* XXX: Just (de)allocate this once. */
        av_freep(&s->mcpic);
    }

    if (s->refs) {
        av_freep(&s->sbsplit);
        av_freep(&s->blmotion);
    }

    return 0;
}

/**
 * Parse a frame and setup DiracContext to decode it
 *
 * @return 0 when successful, otherwise -1 is returned
 */
static int parse_frame(DiracContext *s)
{
    int retire, picnum;
    int i, level;
    GetBitContext *gb = &s->gb;

    picnum= s->current_picture->display_picture_number = get_bits_long(gb, 32);

    s->ref_pics[0] = s->ref_pics[1] = NULL;
    for (i = 0; i < s->refs; i++)
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

    if (s->refs) {
        align_get_bits(gb);
        if (dirac_unpack_prediction_parameters(s))
            return -1;
        align_get_bits(gb);
        if (dirac_unpack_block_motion_data(s))
            return -1;
    }

    align_get_bits(gb);

    /* Wavelet transform data. */
    if (s->refs == 0)
        s->zero_res = 0;
    else
        s->zero_res = get_bits1(gb);

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

static int alloc_frame(AVCodecContext *avctx, int parse_code)
{
    static const uint8_t pict_type[3] = { FF_I_TYPE, FF_P_TYPE, FF_B_TYPE };
    DiracContext *s = avctx->priv_data;
    AVFrame *pic = NULL;
    int i;

    // find an unused frame
    for (i = 0; i < MAX_FRAMES; i++)
        if (s->all_frames[i].data[0] == NULL)
            pic = &s->all_frames[i];
    if (!pic) {
        av_log(avctx, AV_LOG_ERROR, "framelist full\n");
        return -1;
    }

    avcodec_get_frame_defaults(pic);

    s->refs = parse_code & 0x03;
    s->is_arith    = (parse_code & 0x48) == 0x08;
    s->low_delay   = (parse_code & 0x88) == 0x88;
    pic->reference = (parse_code & 0x0C) == 0x0C;
    pic->key_frame = s->refs == 0;
    pic->pict_type = pict_type[s->refs];

    if (avctx->get_buffer(avctx, pic) < 0) {
        av_log(avctx, AV_LOG_ERROR, "get_buffer() failed\n");
        return -1;
    }
    s->current_picture = pic;
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

int dirac_decode_frame(AVCodecContext *avctx, void *data, int *data_size,
                        const uint8_t *buf, int buf_size)
{
    DiracContext *s = avctx->priv_data;
    AVFrame *picture = data;
    int i;
    int parse_code = pc_padding;
    unsigned data_unit_size = buf_size, buf_read = 0;

    // release unused frames
    for (i = 0; i < MAX_FRAMES; i++)
        if (s->all_frames[i].data[0] && !s->all_frames[i].reference)
            avctx->release_buffer(s->avctx, &s->all_frames[i]);

    // end of stream, so flush delayed pics
    if (buf_size == 0)
        return get_delayed_pic(s, picture, data_size);

    // read through data units until we find a picture
    while (buf_read < buf_size) {
        if (buf[0] != 'B' || buf[1] != 'B' || buf[2] != 'C' || buf[3] != 'D') {
            buf++;
            buf_read++;
            continue;
        }
        parse_code = buf[4];
        data_unit_size = FFMAX(AV_RB32(buf+5), 13);
        if (data_unit_size > buf_size - buf_read)
            return -1;

        init_get_bits(&s->gb, &buf[13], (data_unit_size - 13) * 8);
        s->avctx = avctx;

        if (parse_code ==  pc_seq_header) {
            if (ff_dirac_parse_sequence_header(&s->gb, avctx, &s->source))
                return -1;

            avcodec_get_chroma_sub_sample(avctx->pix_fmt, &s->chroma_hshift,
                                          &s->chroma_vshift);

        } else if (parse_code & 0x8)
            // we found a picture
            break;

        buf += data_unit_size;
        buf_read += data_unit_size;
    }

    /* If this is not a picture, return. */
    if ((parse_code & 0x08) != 0x08)
        return 0;

    if (alloc_frame(avctx, parse_code) < 0)
        return -1;

    if (parse_frame(s) < 0)
        return -1;

    if (dirac_decode_frame_internal(s))
        return -1;

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

    return buf_read + data_unit_size;
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
    CODEC_CAP_DELAY,
    NULL
};
