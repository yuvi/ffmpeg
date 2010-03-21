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

#undef printf

#define CLEAR_ONCE

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

static AVFrame *remove_frame(AVFrame *framelist[], int picnum)
{
    AVFrame *remove_pic = NULL;
    int i, remove_idx = -1;

    for (i = 0; framelist[i]; i++)
        if (framelist[i]->display_picture_number == picnum) {
            remove_pic = framelist[i];
            remove_idx = i;
        }

    if (remove_pic)
        for (i = remove_idx; framelist[i]; i++)
            framelist[i] = framelist[i+1];

    return remove_pic;
}

static int add_frame(AVFrame *framelist[], int maxframes, AVFrame *frame)
{
    int i;
    for (i = 0; i < maxframes; i++)
        if (!framelist[i]) {
            framelist[i] = frame;
            return 0;
        }
    return -1;
}

static int alloc_sequence_buffers(DiracContext *s)
{
    int sbwidth   = DIVRNDUP(s->source.width,  4);
    int sbheight  = DIVRNDUP(s->source.height, 4);
    int refwidth  = (s->source.width  + 2 * MAX_BLOCKSIZE) << 1;
    int refheight = (s->source.height + 2 * MAX_BLOCKSIZE) << 1;
    int i, w, h, top_padding;

    for (i = 0; i < 3; i++) {
        w = s->source.width  >> (i ? s->chroma_x_shift : 0);
        h = s->source.height >> (i ? s->chroma_y_shift : 0);

        // we allocate the max we support here since num decompositions can
        // change from frame to frame. Stride is aligned to 16 for SIMD, and
        // we add padding on top to avoid if(y>0) checks
        top_padding = 1<<MAX_DECOMPOSITIONS;
        w = FFALIGN(CALC_PADDING(w, MAX_DECOMPOSITIONS), 16);
        h = top_padding + CALC_PADDING(h, MAX_DECOMPOSITIONS);

        s->plane[i].idwt_buf_base = av_mallocz(w*h * sizeof(IDWTELEM));
        s->plane[i].idwt_buf      = s->plane[i].idwt_buf_base + top_padding*w;
        if (!s->plane[i].idwt_buf_base)
            return AVERROR(ENOMEM);
    }

    s->mcpic       = av_malloc(s->source.width*s->source.height * 2);
    s->obmc_buffer = av_malloc((2*s->source.width + MAX_BLOCKSIZE) * 2*MAX_BLOCKSIZE);

    s->sbsplit  = av_malloc(sbwidth * sbheight);
    s->blmotion = av_malloc(sbwidth * sbheight * 4 * sizeof(*s->blmotion));

    s->refdata[0] = av_malloc(refwidth * refheight);
    s->refdata[1] = av_malloc(refwidth * refheight);

    if (!s->mcpic || !s->obmc_buffer || 
        !s->sbsplit || !s->blmotion || !s->refdata[0] || !s->refdata[1])
        return AVERROR(ENOMEM);
    return 0;
}

static void free_sequence_buffers(DiracContext *s)
{
    int i;
    for (i = 0; i < 3; i++)
        av_freep(&s->plane[i].idwt_buf_base);
    av_freep(&s->obmc_buffer);
    av_freep(&s->mcpic);
    av_freep(&s->sbsplit);
    av_freep(&s->blmotion);
    av_freep(&s->refdata[0]);
    av_freep(&s->refdata[1]);
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    DiracContext *s = avctx->priv_data;

    dsputil_init(&s->dsp, avctx);

    return 0;
}

static av_cold int decode_end(AVCodecContext *avctx)
{
    DiracContext *s = avctx->priv_data;

    free_sequence_buffers(s);

    return 0;
}

#define SIGN_CTX(x) (CTX_SIGN_ZERO + ((x) > 0) - ((x) < 0))

static void coeff_unpack_arith(dirac_arith *arith, int qfactor, int qoffset,
                               SubBand *b, IDWTELEM *buf, int x, int y)
{
    int nhood, coeff, sign;
    int sign_pred = 0;
    int parent = 0;

    // Check if the parent subband has a 0 in the corresponding position
    if (b->parent)
        parent = b->parent->ibuf[b->parent->stride * (y>>1) + (x>>1)] != 0;

    if (b->orientation == subband_hl)
        sign_pred = buf[-b->stride];

    // Determine if the pixel has only zeros in its neighbourhood
    if (x) {
        nhood = !(buf[-1] | buf[-b->stride] | buf[-1-b->stride]);
        if (b->orientation == subband_lh)
            sign_pred = buf[-1];
    } else {
        nhood = !buf[-b->stride];
    }

    coeff = dirac_get_arith_uint(arith, (parent<<1) | nhood, CTX_COEFF_DATA);
    if (coeff) {
        coeff = (coeff*qfactor + qoffset + 2)>>2;
        sign = dirac_get_arith_bit(arith, SIGN_CTX(sign_pred));
        coeff = (coeff ^ -sign) + sign;
    }
    *buf = coeff;
}

static int coeff_unpack_vlc(GetBitContext *gb, int qfactor, int qoffset)
{
    int sign, coeff;

    coeff = svq3_get_ue_golomb(gb);
    if (coeff) {
        coeff = (coeff*qfactor + qoffset + 2)>>2;
        sign = get_bits1(gb);
        coeff = (coeff ^ -sign) + sign;
    }
    return coeff;
}

/**
 * Decode the coeffs in the rectangle defined by left, right, top, bottom
 */
static inline void codeblock(DiracContext *s, SubBand *b,
                             int left, int right, int top, int bottom,
                             unsigned *quant, int blockcnt_one, int is_arith)
{
    GetBitContext *gb = &s->gb;
    int x, y, zero_block;
    int qoffset, qfactor;
    IDWTELEM *buf;

    // check for any coded coefficients in this codeblock
    if (!blockcnt_one) {
        if (is_arith)
            zero_block = dirac_get_arith_bit(&s->arith, CTX_ZERO_BLOCK);
        else
            zero_block = get_bits1(gb);

        if (zero_block) {
#ifndef CLEAR_ONCE
            buf = b->ibuf + y*b->stride + left;
            for (y = top; y < bottom; y++) {
                memset(buf, 0, (right - left)*sizeof(IDWTELEM));
                buf += b->stride;
            }
#endif
            return;
        }
    }

    if (s->codeblock_mode && (s->new_delta_quant || !blockcnt_one)) {
        if (is_arith)
            *quant += dirac_get_arith_int(&s->arith, CTX_DELTA_Q_F, CTX_DELTA_Q_DATA);
        else
            *quant += dirac_get_se_golomb(gb);
    }

    *quant = FFMIN(*quant, MAX_QUANT);

    qfactor = ff_dirac_qscale_tab[*quant];
    // TODO: context pointer?
    if (!s->num_refs)
        qoffset = ff_dirac_qoffset_intra_tab[*quant];
    else
        qoffset = ff_dirac_qoffset_inter_tab[*quant];

    buf = b->ibuf + top*b->stride;
    for (y = top; y < bottom; y++) {
        for (x = left; x < right; x++) {
            if (is_arith)
                coeff_unpack_arith(&s->arith, qfactor, qoffset, b, buf+x, x, y);
            else
                buf[x] = coeff_unpack_vlc(gb, qfactor, qoffset);
        }
        buf += b->stride;
    }
}

static inline void intra_dc_prediction(SubBand *b)
{
    IDWTELEM *buf = b->ibuf;
    int x, y;

    for (x = 1; x < b->width; x++)
        buf[x] += buf[x-1];
    buf += b->stride;

    for (y = 1; y < b->height; y++) {
        buf[0] += buf[-b->stride];

        for (x = 1; x < b->width; x++) {
            int pred = buf[x - 1] + buf[x - b->stride] + buf[x - b->stride-1];
            // magic constant division by 3
            buf[x] += ((pred+1)*21845 + 10922) >> 16;
        }
        buf += b->stride;
    }
}

static av_always_inline
void decode_subband_internal(DiracContext *s, SubBand *b, int is_arith)
{
    GetBitContext *gb = &s->gb;
    unsigned length, quant;
    int y, cb_x, cb_y, left, right, top, bottom;
    int cb_width  = s->codeblocksh[b->level + (b->orientation != subband_ll)];
    int cb_height = s->codeblocksv[b->level + (b->orientation != subband_ll)];
    int blockcnt_one = (cb_width + cb_height) == 2;

    align_get_bits(gb);
    length = svq3_get_ue_golomb(gb);
    if (!length) {
#ifndef CLEAR_ONCE
        IDWTELEM *buf = b->ibuf;
        for (y = 0; y < b->height; y++) {
            memset(buf, 0, b->width*sizeof(IDWTELEM));
            buf += b->stride;
        }
#endif
        return;
    }

    quant = svq3_get_ue_golomb(gb);

    if (is_arith)
        ff_dirac_init_arith_decoder(&s->arith, gb, length);
    else
        align_get_bits(gb);

    top = 0;
    for (cb_y = 0; cb_y < cb_height; cb_y++) {
        bottom = (b->height * (cb_y+1)) / cb_height;
        left = 0;
        for (cb_x = 0; cb_x < cb_width; cb_x++) {
            right = (b->width * (cb_x+1)) / cb_width;
            codeblock(s, b, left, right, top, bottom, &quant, blockcnt_one, is_arith);
            left = right;
        }
        top = bottom;
    }

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
    enum dirac_subband orientation;
    int level;

    /* Unpack all subbands at all levels. */
    for (level = 0; level < s->wavelet_depth; level++) {
        for (orientation = !!level; orientation < 4; orientation++) {
            SubBand *b = &s->plane[comp].band[level][orientation];
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

        p->width  = s->source.width  >> (i ? s->chroma_x_shift : 0);
        p->height = s->source.height >> (i ? s->chroma_y_shift : 0);
        p->padded_width  = w = CALC_PADDING(p->width , s->wavelet_depth);
        p->padded_height = h = CALC_PADDING(p->height, s->wavelet_depth);
        p->idwt_stride = FFALIGN(w, 16);

        for (level = s->wavelet_depth-1; level >= 0; level--) {
            for (orientation = level ? 1 : 0; orientation < 4; orientation++) {
                SubBand *b = &p->band[level][orientation];

                b->ibuf   = p->idwt_buf;
                b->level  = level;
                b->stride = p->idwt_stride << (s->wavelet_depth - level);
                b->width  = w>>1;
                b->height = h>>1;
                b->orientation = orientation;

                if (orientation & 1)
                    b->ibuf += w>>1;
                if (orientation > 1)
                    b->ibuf += b->stride>>1;

                if (level)
                    b->parent = &p->band[level-1][orientation];
            }
            w = w>>1;
            h = h>>1;
        }

        if (i > 0) {
            p->xblen = s->plane[0].xblen >> s->chroma_x_shift;
            p->yblen = s->plane[0].yblen >> s->chroma_y_shift;
            p->xbsep = s->plane[0].xbsep >> s->chroma_x_shift;
            p->ybsep = s->plane[0].ybsep >> s->chroma_y_shift;
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

    if (s->plane[0].xbsep < s->plane[0].xblen/2 || s->plane[0].ybsep < s->plane[0].yblen/2) {
        av_log(s->avctx, AV_LOG_ERROR, "Block separation too small\n");
        return -1;
    }
    if (s->plane[0].xbsep > s->plane[0].xblen || s->plane[0].ybsep > s->plane[0].yblen) {
        av_log(s->avctx, AV_LOG_ERROR, "Block seperation greater than size\n");
        return -1;
    }
    if (s->plane[0].xblen > MAX_BLOCKSIZE || s->plane[0].yblen > MAX_BLOCKSIZE) {
        av_log(s->avctx, AV_LOG_ERROR, "Unsupported large block size\n");
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
    int res = dirac_get_arith_bit(&s->arith, CTX_PMODE_REF1);

    res ^= mode_prediction(s, x, y, DIRAC_REF_MASK_REF1, 0);
    s->blmotion[y * s->blwidth + x].use_ref |= res;
    if (s->num_refs == 2) {
        res = dirac_get_arith_bit(&s->arith, CTX_PMODE_REF2);
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
        int res = dirac_get_arith_bit(&s->arith, CTX_GLOBAL_BLOCK);
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

    res = dirac_get_arith_int(&s->arith, CTX_DC_F1, CTX_DC_DATA);
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

    res = dirac_get_arith_int(&s->arith, CTX_MV_F1, CTX_MV_DATA);
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
    int x, y, q, p;

    length = svq3_get_ue_golomb(gb);
    ff_dirac_init_arith_decoder(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
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
    int x, y, q, p;

    align_get_bits(gb);

    s->sbwidth  = DIVRNDUP(s->source.width,  s->plane[0].xbsep << 2);
    s->sbheight = DIVRNDUP(s->source.height, s->plane[0].ybsep << 2);
    s->blwidth  = s->sbwidth  << 2;
    s->blheight = s->sbheight << 2;

    memset(s->blmotion, 0, s->blwidth * s->blheight * sizeof(*s->blmotion));

    /* Superblock splitmodes. */
    length = svq3_get_ue_golomb(gb);
    ff_dirac_init_arith_decoder(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
            int res = dirac_get_arith_uint(&s->arith, CTX_SB_F1,
                                           CTX_SB_DATA);
            s->sbsplit[y * s->sbwidth + x] = res + split_prediction(s, x, y);
            s->sbsplit[y * s->sbwidth + x] %= 3;
        }

    /* Prediction modes. */
    length = svq3_get_ue_golomb(gb);
    ff_dirac_init_arith_decoder(&s->arith, gb, length);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
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

    /* Unpack the motion vectors. */
    for (i = 0; i < s->num_refs; i++) {
        dirac_unpack_motion_vectors(s, i, 0);
        dirac_unpack_motion_vectors(s, i, 1);
    }

    /* Unpack the DC values for all the three components (YUV). */
    for (comp = 0; comp < 3; comp++) {
        /* Unpack the DC values. */
        length = svq3_get_ue_golomb(gb);
        ff_dirac_init_arith_decoder(&s->arith, gb, length);
        for (y = 0; y < s->sbheight; y++)
            for (x = 0; x < s->sbwidth; x++) {
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
    }

    return 0;
}

static int dirac_unpack_idwt_params(DiracContext *s)
{
    GetBitContext *gb = &s->gb;
    int i, level;

    align_get_bits(gb);

    s->zero_res = s->num_refs ? get_bits1(gb) : 0;
    if (s->zero_res)
        return 0;

    s->wavelet_idx = svq3_get_ue_golomb(gb);

    if (s->wavelet_idx > 6)
        return -1;

    s->wavelet_depth = svq3_get_ue_golomb(gb);
    if (s->wavelet_depth > MAX_DECOMPOSITIONS) {
        av_log(s->avctx, AV_LOG_ERROR, "too many dwt decompositions\n");
        return -1;
    }

    if (!s->low_delay) {
        /* Codeblock paramaters (core syntax only) */
        if (get_bits1(gb)) {
            for (i = 0; i <= s->wavelet_depth; i++) {
                s->codeblocksh[i] = svq3_get_ue_golomb(gb);
                s->codeblocksv[i] = svq3_get_ue_golomb(gb);
            }

            s->codeblock_mode = svq3_get_ue_golomb(gb);
            if (s->codeblock_mode > 1) {
                av_log(s->avctx, AV_LOG_ERROR, "unknown codeblock mode\n");
                return -1;
            }
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
                    s->quant_matrix[level][i] = ff_dirac_default_qmat[s->wavelet_idx][level][i];

                    // haar with no shift differs for different depths
                    if (s->wavelet_idx == 3)
                        s->quant_matrix[level][i] += 4*(s->wavelet_depth-1 - level);
                }
        }
    }
    return 0;
}

static int obmc_weight(int i, int blen, int offset)
{
#define ROLLOFF(i) offset == 1 ? ((i) ? 5 : 3) : \
    (1 + (6*(i) + offset - 1) / (2*offset - 1))

    if (i < 2*offset)
        return ROLLOFF(i);
    else if (i > blen-1 - 2*offset)
        return ROLLOFF(blen-1 - i);
    else
        return 8;
}

static void init_obmc_weights(DiracContext *s)
{
    int x, y, p;
    for (p = 0; p < 2; p++)
        for (y = 0; y < s->plane[p].yblen; y++) {
            int wy = obmc_weight(y, s->plane[p].yblen, s->plane[p].yoffset);
            for (x = 0; x < s->plane[p].xblen; x++) {
                int wx = obmc_weight(x, s->plane[p].xblen, s->plane[p].xoffset);
                s->obmc_weight[p][y*MAX_BLOCKSIZE + x] = wx*wy;
            }
        }
}

// fixme: chroma only guarantees 2 byte alignment, and -> dsp
static void dirac_put_intra_dc(uint8_t *dst, int stride, int dc, int width, int height)
{
    int x, y;
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x+=4)
            *(uint32_t*)(dst+x) = dc*0x01010101;
        dst += stride;
    }
}

static void dirac_add_yblock(uint8_t *dst, int dst_stride, uint8_t *obmc_curr,
                    uint8_t *obmc_last, int obmc_stride, uint8_t *obmc_weight,
                    int xblen, int yblen, int xbsep, int ybsep)
{
}

static int dirac_decode_frame_internal(DiracContext *s)
{
    IDWTELEM *line;
    int16_t *mcline;
    int comp;
    int x, y;

    init_obmc_weights(s);

    for (comp = 0; comp < 3; comp++) {
        Plane *p = &s->plane[comp];
        uint8_t *frame = s->current_picture->data[comp];
        // fixme: do subpel interpolation and don't force a NULL dereference for invalid streams
        uint8_t *src[2][4] = {
            { s->num_refs     ? s->ref_pics[0]->data[comp] : NULL },
            { s->num_refs > 1 ? s->ref_pics[1]->data[comp] : NULL }
        };
        int width, height, stride = s->current_picture->linesize[comp];

        width  = s->source.width  >> (comp ? s->chroma_x_shift : 0);
        height = s->source.height >> (comp ? s->chroma_y_shift : 0);

        // TODO: check to see which is faster for inter
#ifdef CLEAR_ONCE
        memset(p->idwt_buf, 0, p->idwt_stride * p->padded_height * sizeof(IDWTELEM));
#endif
        if (!s->zero_res)
            decode_component(s, comp);

        ff_spatial_idwt2(p->idwt_buf, p->padded_width, p->padded_height,
                         p->idwt_stride, s->wavelet_idx+2, s->wavelet_depth);

        if (!s->num_refs) {
            line = p->idwt_buf;

            for (y = 0; y < height; y++) {
                for (x = 0; x < width; x++)
                    frame[x] = av_clip_uint8(line[x] + 128);

                line  += p->idwt_stride;
                frame += s->current_picture->linesize[comp];
            }
        } else {
#if 0
            int obmc_stride = FFALIGN(p->xblen * p->current_blwidth, 16);
            for (y = 0; y < p->current_blheight; y++) {
                uint8_t *obmc_last = s->obmc_buffer + (y&1 ? obmc_stride*p->yblen : 0) + p->xoffset;
                uint8_t *obmc_curr = s->obmc_buffer + (y&1 ? 0 : obmc_stride*p->yblen) + p->xoffset;
                uint8_t *dst = obmc_curr;
                int offset = y*p->ybsep*stride - 2*p->xoffset - 2*p->yoffset*stride;

                for (x = 0; x < p->current_blwidth; x++) {
                    struct dirac_blockmotion *block = &s->blmotion[y*s->blwidth + x];
                    int ref = block->use_ref;

                    switch (ref) {
                    case 0: // DC
                        dirac_put_intra_dc(dst, obmc_stride, block->dc[comp],
                                           p->xblen, p->yblen);
                        break;
                    case 1:
                    case 2:
                        s->dsp.put_dirac_tab[s->mv_precision](dst, obmc_stride,
                            src[ref-1], stride, offset, block->vect[ref-1][0],
                            block->vect[ref-1][1], p->xblen, p->yblen);
                        break;
                    case 3:
                        s->dsp.put_dirac_tab[s->mv_precision](dst, obmc_stride,
                            src[0], stride, offset, block->vect[0][0], block->vect[0][1],
                            p->xblen, p->yblen);
                        s->dsp.avg_dirac_tab[s->mv_precision](dst, obmc_stride,
                            src[1], stride, offset, block->vect[1][0], block->vect[1][1],
                            p->xblen, p->yblen);
                        break;
                    }
                    dst += p->xblen;
                    offset += p->xbsep;
                }

            }
#endif
        }

#if 0
        if (s->num_refs) {
            if (dirac_motion_compensation(s, comp)) {
                return -1;
            }
        }

        /* Copy the decoded coefficients into the frame and also add
           the data calculated by MC. */
        line = p->idwt_buf;
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
#endif
#if 0
        if (s->current_picture->reference) {
            int edge_width = FFMIN(FFMAX(s->plane[comp].xblen, s->plane[comp].yblen), EDGE_WIDTH);
            s->dsp.draw_edges(s->current_picture->data[comp], s->current_picture->linesize[comp],
                              s->plane[comp].width, s->plane[comp].height, edge_width);
        }
#endif
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
    int i;
    GetBitContext *gb = &s->gb;

    picnum= s->current_picture->display_picture_number = get_bits_long(gb, 32);

    s->ref_pics[0] = s->ref_pics[1] = NULL;
    for (i = 0; i < s->num_refs; i++)
        s->ref_pics[i] = get_frame(s->ref_frames, picnum + dirac_get_se_golomb(gb));

    /* Retire the reference frames that are not used anymore. */
    if (s->current_picture->reference) {
        retire = picnum + dirac_get_se_golomb(gb);
        if (retire != picnum) {
            AVFrame *retire_pic = remove_frame(s->ref_frames, retire);

            if (retire_pic) {
                if (retire_pic->reference & DELAYED_PIC_REF)
                    retire_pic->reference = DELAYED_PIC_REF;
                else
                    retire_pic->reference = 0;
            } else
                av_log(s->avctx, AV_LOG_DEBUG, "Frame to retire not found\n");
        }

        // if reference array is full, remove the oldest as per the spec
        while (add_frame(s->ref_frames, MAX_REFERENCE_FRAMES, s->current_picture)) {
            av_log(s->avctx, AV_LOG_ERROR, "Reference frame overflow\n");
            remove_frame(s->ref_frames, s->ref_frames[0]->display_picture_number);
        }
    }

    if (s->num_refs) {
        if (dirac_unpack_prediction_parameters(s))
            return -1;
        if (dirac_unpack_block_motion_data(s))
            return -1;
    }
    if (dirac_unpack_idwt_params(s))
        return -1;

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

        if (ff_dirac_parse_sequence_header(avctx, &s->gb, &s->source))
            return -1;

        avcodec_get_chroma_sub_sample(avctx->pix_fmt, &s->chroma_x_shift, &s->chroma_y_shift);

        if (alloc_sequence_buffers(s))
            return -1;
        s->seen_sequence_header = 1;
    } else if (parse_code == pc_eos) {
        free_sequence_buffers(s);
        s->seen_sequence_header = 0;
    } else if (parse_code == pc_aux_data) {
        if (buf[13] == 1) {     // encoder implementation/version
            int ver[3];
            // versions newer than 1.0.7 store quant delta for all codeblocks
            if (sscanf(buf+14, "Schroedinger %d.%d.%d", ver, ver+1, ver+2) == 3)
                if (ver[0] > 1 || ver[1] > 0 || ver[2] > 7)
                    s->new_delta_quant = 1;
        }
    } else if (parse_code & 0x8) {
        // picture data unit
        if (!s->seen_sequence_header) {
            av_log(avctx, AV_LOG_DEBUG, "Dropping frame without sequence header\n");
            return -1;
        }

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
        AVFrame *delayed_frame = remove_frame(s->delay_frames, avctx->frame_number);

        s->current_picture->reference |= DELAYED_PIC_REF;
        if (add_frame(s->delay_frames, MAX_DELAY, s->current_picture))
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
    .long_name = NULL_IF_CONFIG_SMALL("BBC Dirac VC-2"),
};
