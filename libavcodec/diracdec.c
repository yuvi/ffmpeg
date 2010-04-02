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

#include "avcodec.h"
#include "dsputil.h"
#include "get_bits.h"
#include "bytestream.h"
#include "golomb.h"
#include "dirac_arith.h"
#include "mpeg12data.h"
#include "dwt.h"
#include "dirac.h"

#undef printf

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
    int i, w, h, top_padding;

    for (i = 0; i < 3; i++) {
        w = s->source.width  >> (i ? s->chroma_x_shift : 0);
        h = s->source.height >> (i ? s->chroma_y_shift : 0);

        // we allocate the max we support here since num decompositions can
        // change from frame to frame. Stride is aligned to 16 for SIMD, and
        // 1<<MAX_DWT_LEVELS top padding to avoid if(y>0) in arith decoding
        // MAX_BLOCKSIZE padding for MC: blocks can spill up to half of that
        // on each side
        top_padding = FFMAX(1<<MAX_DWT_LEVELS, MAX_BLOCKSIZE/2);
        w = FFALIGN(CALC_PADDING(w, MAX_DWT_LEVELS), 8);
        h = top_padding + CALC_PADDING(h, MAX_DWT_LEVELS) + MAX_BLOCKSIZE/2;

        s->plane[i].idwt_buf_base = av_mallocz((w+MAX_BLOCKSIZE)*h * sizeof(IDWTELEM));
        s->plane[i].idwt_tmp      = av_malloc((w+16) * sizeof(IDWTELEM));
        s->plane[i].idwt_buf      = s->plane[i].idwt_buf_base + top_padding*w;
        if (!s->plane[i].idwt_buf_base || !s->plane[i].idwt_tmp)
            return AVERROR(ENOMEM);
    }

    // fixme: allocate using real stride here
    s->sbsplit  = av_malloc(sbwidth * sbheight);
    s->blmotion = av_malloc(sbwidth * sbheight * 4 * sizeof(*s->blmotion));

    if (!s->sbsplit || !s->blmotion)
        return AVERROR(ENOMEM);
    return 0;
}

static void free_sequence_buffers(DiracContext *s)
{
    int i;
    for (i = 0; i < 3; i++) {
        av_freep(&s->plane[i].idwt_buf_base);
        av_freep(&s->plane[i].idwt_tmp);
    }
    av_freep(&s->sbsplit);
    av_freep(&s->blmotion);
}

static av_cold int decode_init(AVCodecContext *avctx)
{
    DiracContext *s = avctx->priv_data;
    s->avctx = avctx;

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

static inline void coeff_unpack_arith(DiracArith *c, int qfactor, int qoffset,
                                      SubBand *b, IDWTELEM *buf, int x, int y)
{
    int coeff, sign;
    int sign_pred = 0;
    int pred_ctx = CTX_ZPZN_F1;

    // Check if the parent subband has a 0 in the corresponding position
    if (b->parent)
        pred_ctx += !!b->parent->ibuf[b->parent->stride * (y>>1) + (x>>1)] << 1;

    if (b->orientation == subband_hl)
        sign_pred = buf[-b->stride];

    // Determine if the pixel has only zeros in its neighbourhood
    if (x) {
        pred_ctx += !(buf[-1] | buf[-b->stride] | buf[-1-b->stride]);
        if (b->orientation == subband_lh)
            sign_pred = buf[-1];
    } else {
        pred_ctx += !buf[-b->stride];
    }

    coeff = dirac_get_arith_uint(c, pred_ctx, CTX_COEFF_DATA);
    if (coeff) {
        coeff = (coeff*qfactor + qoffset + 2)>>2;
        sign = dirac_get_arith_bit(c, SIGN_CTX(sign_pred));
        coeff = (coeff ^ -sign) + sign;
    }
    *buf = coeff;
}

static inline int coeff_unpack_golomb(GetBitContext *gb, int qfactor, int qoffset)
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
                             GetBitContext *gb, DiracArith *c,
                             int left, int right, int top, int bottom,
                             int blockcnt_one, int is_arith)
{
    int x, y, zero_block;
    int qoffset, qfactor;
    IDWTELEM *buf;

    // check for any coded coefficients in this codeblock
    if (!blockcnt_one) {
        if (is_arith)
            zero_block = dirac_get_arith_bit(c, CTX_ZERO_BLOCK);
        else
            zero_block = get_bits1(gb);

        if (zero_block)
            return;
    }

    if (s->codeblock_mode && (s->new_delta_quant || !blockcnt_one)) {
        if (is_arith)
            b->quant += dirac_get_arith_int(c, CTX_DELTA_Q_F, CTX_DELTA_Q_DATA);
        else
            b->quant += dirac_get_se_golomb(gb);
    }

    b->quant = FFMIN(b->quant, MAX_QUANT);

    qfactor = ff_dirac_qscale_tab[b->quant];
    // TODO: context pointer?
    if (!s->num_refs)
        qoffset = ff_dirac_qoffset_intra_tab[b->quant];
    else
        qoffset = ff_dirac_qoffset_inter_tab[b->quant];

    buf = b->ibuf + top*b->stride;
    for (y = top; y < bottom; y++) {
        for (x = left; x < right; x++) {
            if (is_arith)
                coeff_unpack_arith(c, qfactor, qoffset, b, buf+x, x, y);
            else
                buf[x] = coeff_unpack_golomb(gb, qfactor, qoffset);
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
            // magic number division by 3
            buf[x] += ((pred+1)*21845 + 10922) >> 16;
        }
        buf += b->stride;
    }
}

static av_always_inline
void decode_subband_internal(DiracContext *s, SubBand *b, int is_arith)
{
    int cb_x, cb_y, left, right, top, bottom;
    DiracArith c;
    GetBitContext gb;
    int cb_width  = s->codeblock[b->level + (b->orientation != subband_ll)].width;
    int cb_height = s->codeblock[b->level + (b->orientation != subband_ll)].height;
    int blockcnt_one = (cb_width + cb_height) == 2;

    if (!b->length)
        return;

    init_get_bits(&gb, b->coeff_data, b->length*8);

    if (is_arith)
        ff_dirac_init_arith_decoder(&c, &gb, b->length);

    top = 0;
    for (cb_y = 0; cb_y < cb_height; cb_y++) {
        bottom = (b->height * (cb_y+1)) / cb_height;
        left = 0;
        for (cb_x = 0; cb_x < cb_width; cb_x++) {
            right = (b->width * (cb_x+1)) / cb_width;
            codeblock(s, b, &gb, &c, left, right, top, bottom, blockcnt_one, is_arith);
            left = right;
        }
        top = bottom;
    }

    if (b->orientation == subband_ll && s->num_refs == 0)
        intra_dc_prediction(b);
}

static int decode_subband_arith(AVCodecContext *avctx, void *b)
{
    DiracContext *s = avctx->priv_data;
    decode_subband_internal(s, b, 1);
    return 0;
}

static int decode_subband_golomb(AVCodecContext *avctx, void *arg)
{
    DiracContext *s = avctx->priv_data;
    SubBand **b = arg;
    decode_subband_internal(s, *b, 0);
    return 0;
}

/**
 * Decode a single component
 *
 * @param coeffs coefficients for this component
 */
static void decode_component(DiracContext *s, int comp)
{
    AVCodecContext *avctx = s->avctx;
    SubBand *bands[3*MAX_DWT_LEVELS+1];
    enum dirac_subband orientation;
    int level, num_bands = 0;

    /* Unpack all subbands at all levels. */
    for (level = 0; level < s->wavelet_depth; level++) {
        for (orientation = !!level; orientation < 4; orientation++) {
            SubBand *b = &s->plane[comp].band[level][orientation];
            bands[num_bands++] = b;

            align_get_bits(&s->gb);
            b->length = svq3_get_ue_golomb(&s->gb);
            if (b->length) {
                b->quant = svq3_get_ue_golomb(&s->gb);
                align_get_bits(&s->gb);
                b->coeff_data = s->gb.buffer + get_bits_count(&s->gb)/8;
                b->length = FFMIN(b->length, get_bits_left(&s->gb)/8);
                skip_bits_long(&s->gb, b->length*8);
            }
        }
        // arithmetic coding has inter-level dependencies, so we can only execute one level at a time
        if (s->is_arith)
            avctx->execute(avctx, decode_subband_arith, &s->plane[comp].band[level][!!level],
                           NULL, 4-!!level, sizeof(SubBand));
    }
    // golomb coding has no inter-level dependencies, so we can execute all subbands in parallel
    if (!s->is_arith)
        avctx->execute(avctx, decode_subband_golomb, bands, NULL, num_bands, sizeof(SubBand*));
}

static av_always_inline
void lowdelay_subband(DiracContext *s, GetBitContext *gb,
                      int quant, int slice_x, int slice_y, int bits_end,
                      SubBand *b1, SubBand *b2)
{
    int left   = b1->width * slice_x    / s->lowdelay.num_x;
    int right  = b1->width *(slice_x+1) / s->lowdelay.num_x;
    int top    = b1->height* slice_y    / s->lowdelay.num_y;
    int bottom = b1->height*(slice_y+1) / s->lowdelay.num_y;

    int qfactor = ff_dirac_qscale_tab[FFMIN(quant, MAX_QUANT)];
    int qoffset = ff_dirac_qoffset_intra_tab[FFMIN(quant, MAX_QUANT)];

    IDWTELEM *buf1 =      b1->ibuf + top*b1->stride;
    IDWTELEM *buf2 = b2 ? b2->ibuf + top*b2->stride : NULL;
    int x, y;

    // we have to constantly check for overread since the spec explictly
    // requires this, with the meaning that all remaining coeffs are set to 0
    if (get_bits_count(gb) >= bits_end)
        return;

    for (y = top; y < bottom; y++) {
        for (x = left; x < right; x++) {
            buf1[x] = coeff_unpack_golomb(gb, qfactor, qoffset);
            if (get_bits_count(gb) >= bits_end)
                return;
            if (buf2) {
                buf2[x] = coeff_unpack_golomb(gb, qfactor, qoffset);
                if (get_bits_count(gb) >= bits_end)
                    return;
            }
        }
        buf1 += b1->stride;
        if (buf2)
            buf2 += b2->stride;
    }
}

struct lowdelay_slice {
    GetBitContext gb;
    int slice_x;
    int slice_y;
    int bytes;
};

static int decode_lowdelay_slice(AVCodecContext *avctx, void *arg)
{
    DiracContext *s = avctx->priv_data;
    struct lowdelay_slice *slice = arg;
    GetBitContext *gb = &slice->gb;
    enum dirac_subband orientation;
    int level, quant, chroma_bits, chroma_end;

    int quant_base  = get_bits(gb, 7);
    int length_bits = av_log2(8*slice->bytes)+1;
    int luma_bits   = get_bits_long(gb, length_bits);
    int luma_end    = get_bits_count(gb) + FFMIN(luma_bits, get_bits_left(gb));

    for (level = 0; level < s->wavelet_depth; level++)
        for (orientation = !!level; orientation < 4; orientation++) {
            quant = FFMAX(quant_base - s->lowdelay.quant[level][orientation], 0);
            lowdelay_subband(s, gb, quant, slice->slice_x, slice->slice_y, luma_end,
                             &s->plane[0].band[level][orientation], NULL);
        }

    // consume any unused bits from luma
    skip_bits_long(gb, get_bits_count(gb) - luma_end);

    chroma_bits = 8*slice->bytes - 7 - length_bits - luma_bits;
    chroma_end = get_bits_count(gb) + FFMIN(chroma_bits, get_bits_left(gb));

    for (level = 0; level < s->wavelet_depth; level++)
        for (orientation = !!level; orientation < 4; orientation++) {
            quant = FFMAX(quant_base - s->lowdelay.quant[level][orientation], 0);
            lowdelay_subband(s, gb, quant, slice->slice_x, slice->slice_y, chroma_end,
                             &s->plane[1].band[level][orientation],
                             &s->plane[2].band[level][orientation]);
        }

    return 0;
}

static void decode_lowdelay(DiracContext *s)
{
    AVCodecContext *avctx = s->avctx;
    int slice_x, slice_y, bytes, bufsize;
    const uint8_t *buf;
    struct lowdelay_slice *slices;
    int slice_num = 0;

    slices = av_mallocz(s->lowdelay.num_x * s->lowdelay.num_y * sizeof(struct lowdelay_slice));

    align_get_bits(&s->gb);
    buf = s->gb.buffer + get_bits_count(&s->gb)/8;
    bufsize = get_bits_left(&s->gb);

    for (slice_y = 0; slice_y < s->lowdelay.num_y; slice_y++)
        for (slice_x = 0; slice_x < s->lowdelay.num_x; slice_x++) {
            bytes = (slice_num+1) * s->lowdelay.bytes.num / s->lowdelay.bytes.den
                   - slice_num    * s->lowdelay.bytes.num / s->lowdelay.bytes.den;

            slices[slice_num].bytes   = bytes;
            slices[slice_num].slice_x = slice_x;
            slices[slice_num].slice_y = slice_y;
            init_get_bits(&slices[slice_num].gb, buf, bufsize);
            slice_num++;

            buf     += bytes;
            bufsize -= bytes*8;
            if (bufsize <= 0)
                goto end;
        }
end:
    avctx->execute(avctx, decode_lowdelay_slice, slices, NULL, slice_num,
                   sizeof(struct lowdelay_slice));

    intra_dc_prediction(&s->plane[0].band[0][0]);
    intra_dc_prediction(&s->plane[1].band[0][0]);
    intra_dc_prediction(&s->plane[2].band[0][0]);
    av_free(slices);
}

static void init_planes(DiracContext *s)
{
    int i, w, h, level, orientation;

    for (i = 0; i < 3; i++) {
        Plane *p = &s->plane[i];

        p->width  = s->source.width  >> (i ? s->chroma_x_shift : 0);
        p->height = s->source.height >> (i ? s->chroma_y_shift : 0);
        p->width  = w = CALC_PADDING(p->width , s->wavelet_depth);
        p->height = h = CALC_PADDING(p->height, s->wavelet_depth);
        p->idwt_stride = FFALIGN(w, 16);

        for (level = s->wavelet_depth-1; level >= 0; level--) {
            w = w>>1;
            h = h>>1;
            for (orientation = !!level; orientation < 4; orientation++) {
                SubBand *b = &p->band[level][orientation];

                b->ibuf   = p->idwt_buf;
                b->level  = level;
                b->stride = p->idwt_stride << (s->wavelet_depth - level);
                b->width  = w;
                b->height = h;
                b->orientation = orientation;

                if (orientation & 1)
                    b->ibuf += w;
                if (orientation > 1)
                    b->ibuf += b->stride>>1;

                if (level)
                    b->parent = &p->band[level-1][orientation];
            }
        }

        if (i > 0) {
            p->xblen = s->plane[0].xblen >> s->chroma_x_shift;
            p->yblen = s->plane[0].yblen >> s->chroma_y_shift;
            p->xbsep = s->plane[0].xbsep >> s->chroma_x_shift;
            p->ybsep = s->plane[0].ybsep >> s->chroma_y_shift;
        }

        p->xedge = p->xblen - p->xbsep;
        p->yedge = p->yblen - p->ybsep;
    }
}

/**
 * Unpack the motion compensation parameters
 */
static int dirac_unpack_prediction_parameters(DiracContext *s)
{
    GetBitContext *gb = &s->gb;
    unsigned idx, ref;

    static const uint8_t default_blen[] = {
        4, 12, 16, 24
    };
    static const uint8_t default_bsep[] = {
        4, 8, 12, 16
    };

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
        s->plane[0].xblen = default_blen[idx-1];
        s->plane[0].yblen = default_blen[idx-1];
        s->plane[0].xbsep = default_bsep[idx-1];
        s->plane[0].ybsep = default_bsep[idx-1];
    }

    if (s->plane[0].xbsep < s->plane[0].xblen/2 || s->plane[0].ybsep < s->plane[0].yblen/2) {
        av_log(s->avctx, AV_LOG_ERROR, "Block separation too small\n");
        return -1;
    }
    if (s->plane[0].xbsep > s->plane[0].xblen || s->plane[0].ybsep > s->plane[0].yblen) {
        av_log(s->avctx, AV_LOG_ERROR, "Block seperation greater than size\n");
        return -1;
    }
    if (FFMAX(s->plane[0].xblen, s->plane[0].yblen) > MAX_BLOCKSIZE) {
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
        memset(s->globalmc, 0, sizeof(s->globalmc));
        av_log(s->avctx, AV_LOG_WARNING, "GMC not fully supported\n");
        for (ref = 0; ref < s->num_refs; ref++) {
            /* Pan/tilt parameters. */
            if (get_bits1(gb)) {
                s->globalmc[ref].pan_tilt[0] = dirac_get_se_golomb(gb);
                s->globalmc[ref].pan_tilt[1] = dirac_get_se_golomb(gb);
            }

            /* Rotation/shear parameters. */
            if (get_bits1(gb)) {
                s->globalmc[ref].zrs_exp = svq3_get_ue_golomb(gb);
                s->globalmc[ref].zrs[0][0] = dirac_get_se_golomb(gb);
                s->globalmc[ref].zrs[0][1] = dirac_get_se_golomb(gb);
                s->globalmc[ref].zrs[1][0] = dirac_get_se_golomb(gb);
                s->globalmc[ref].zrs[1][1] = dirac_get_se_golomb(gb);
            } else {
                s->globalmc[ref].zrs[0][0] = 1;
                s->globalmc[ref].zrs[1][1] = 1;
            }

            /* Perspective parameters. */
            if (get_bits1(gb)) {
                s->globalmc[ref].perspective_exp = svq3_get_ue_golomb(gb);
                s->globalmc[ref].perspective[0] = dirac_get_se_golomb(gb);
                s->globalmc[ref].perspective[1] = dirac_get_se_golomb(gb);
            }
        }
    }

    /* Picture prediction mode. May be used in the future. */
    if (svq3_get_ue_golomb(gb)) {
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
    s->blmotion[y * s->blwidth + x].ref |= res;
    if (s->num_refs == 2) {
        res = dirac_get_arith_bit(&s->arith, CTX_PMODE_REF2);
        res ^= mode_prediction(s, x, y, DIRAC_REF_MASK_REF2, 1);
        s->blmotion[y * s->blwidth + x].ref |= res << 1;
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
    if (s->blmotion[y * s->blwidth + x].ref & 3) {
        int res = dirac_get_arith_bit(&s->arith, CTX_GLOBAL_BLOCK);
        res ^= mode_prediction(s, x, y, DIRAC_REF_MASK_GLOBAL, 2);
        s->blmotion[y * s->blwidth + x].ref |= res << 2;
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

    if (s->blmotion[y * s->blwidth + x].ref & 3) {
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
    if ((s->blmotion[y * s->blwidth + x].ref & refmask) != ref + 1)
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
            int res = dirac_get_arith_uint(&s->arith, CTX_SB_F1, CTX_SB_DATA);
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
    if (s->wavelet_depth > MAX_DWT_LEVELS) {
        av_log(s->avctx, AV_LOG_ERROR, "too many dwt decompositions\n");
        return -1;
    }

    if (!s->low_delay) {
        /* Codeblock paramaters (core syntax only) */
        if (get_bits1(gb)) {
            for (i = 0; i <= s->wavelet_depth; i++) {
                s->codeblock[i].width = svq3_get_ue_golomb(gb);
                s->codeblock[i].height = svq3_get_ue_golomb(gb);
            }

            s->codeblock_mode = svq3_get_ue_golomb(gb);
            if (s->codeblock_mode > 1) {
                av_log(s->avctx, AV_LOG_ERROR, "unknown codeblock mode\n");
                return -1;
            }
        } else
            for (i = 0; i <= s->wavelet_depth; i++)
                s->codeblock[i].width = s->codeblock[i].height = 1;
    } else {
        s->lowdelay.num_x     = svq3_get_ue_golomb(gb);
        s->lowdelay.num_y     = svq3_get_ue_golomb(gb);
        s->lowdelay.bytes.num = svq3_get_ue_golomb(gb);
        s->lowdelay.bytes.den = svq3_get_ue_golomb(gb);

        if (get_bits1(gb)) {
            // custom quantization matrix
            s->lowdelay.quant[0][0] = svq3_get_ue_golomb(gb);
            for (level = 0; level < s->wavelet_depth; level++) {
                s->lowdelay.quant[level][1] = svq3_get_ue_golomb(gb);
                s->lowdelay.quant[level][2] = svq3_get_ue_golomb(gb);
                s->lowdelay.quant[level][3] = svq3_get_ue_golomb(gb);
            }
        } else {
            // default quantization matrix
            for (level = 0; level < s->wavelet_depth; level++)
                for (i = 0; i < 4; i++) {
                    s->lowdelay.quant[level][i] = ff_dirac_default_qmat[s->wavelet_idx][level][i];

                    // haar with no shift differs for different depths
                    if (s->wavelet_idx == 3)
                        s->lowdelay.quant[level][i] += 4*(s->wavelet_depth-1 - level);
                }
        }
    }
    return 0;
}

static int obmc_weight(int i, int blen, int edge)
{
#define ROLLOFF(i) edge == 2 ? ((i) ? 5 : 3) : \
    (1 + (6*(i) + edge/2 - 1) / (edge - 1))

    if (i < edge)
        return ROLLOFF(i);
    else if (i > blen-1 - edge)
        return ROLLOFF(blen-1 - i);
    else
        return 8;
}

static void init_obmc_weights(DiracContext *s)
{
    int x, y, i;
    for (i = 0; i < 2; i++) {
        Plane *p = &s->plane[i];
        for (y = 0; y < p->yblen; y++) {
            int wy = obmc_weight(y, p->yblen, p->yedge);
            for (x = 0; x < p->xblen; x++) {
                int wx = obmc_weight(x, p->xblen, p->xedge);
                s->obmc_weight[i][y*MAX_BLOCKSIZE + x] = wx*wy;
            }
        }
    }
}

static void put_block(int16_t *dst, int dst_stride,
                      uint8_t *src, int src_stride,
                      uint8_t *obmc_weight, int width, int height)
{
    int x, y;
    for (y = 0; y < height; y++) {
        for (x = 0; x < height; x++)
            dst[x] += src[x] * obmc_weight[x];
        dst += dst_stride;
        src += src_stride;
        obmc_weight += MAX_BLOCKSIZE;
    }
}

static void avg_block(int16_t *dst, int dst_stride,
                      uint8_t *src0, uint8_t *src1, int src_stride,
                      uint8_t *obmc_weight, int width, int height)
{
    int x, y;
    for (y = 0; y < height; y++) {
        for (x = 0; x < height; x++)
            dst[x] += ((src0[x] + src1[x] + 1)>>1) * obmc_weight[x];
        dst  += dst_stride;
        src0 += src_stride;
        src1 += src_stride;
        obmc_weight += MAX_BLOCKSIZE;
    }
}

static void mc_block(DiracContext *s, int plane, int x, int y)
{
    Plane *p = &s->plane[plane];
    int dst_stride = s->ref_pics[0]->linesize[plane];

    int16_t *dst = p->idwt_buf + x*p->xbsep - p->xedge +
                 p->idwt_stride*(y*p->ybsep - p->yedge);
    uint8_t *src = s->ref_pics[0]->dest[plane] + x*p->xbsep - p->xedge +
                                     dst_stride*(y*p->ybsep - p->yedge);
}

static int dirac_decode_frame_internal(DiracContext *s)
{
    DWTContext d;
    int comp;
    int x, y;

    init_obmc_weights(s);

    for (comp = 0; comp < 3; comp++) {
        Plane *p = &s->plane[comp];
        memset(p->idwt_buf, 0, p->idwt_stride * p->height * sizeof(IDWTELEM));
    }

    if (!s->zero_res && s->low_delay)
        decode_lowdelay(s);

    for (comp = 0; comp < 3; comp++) {
        Plane *p = &s->plane[comp];
        uint8_t *frame = s->current_picture->data[comp];
        int width, height, stride = s->current_picture->linesize[comp];

        width  = s->source.width  >> (comp ? s->chroma_x_shift : 0);
        height = s->source.height >> (comp ? s->chroma_y_shift : 0);

        if (!s->zero_res && !s->low_delay)
            decode_component(s, comp);

        if (ff_spatial_idwt_init2(&d, p->idwt_buf, p->width, p->height, p->idwt_stride,
                                  s->wavelet_idx+2, s->wavelet_depth, p->idwt_tmp))
            return -1;

        if (!s->num_refs) {
            for (y = 0; y < height; y += 16) {
                ff_spatial_idwt_slice2(&d, y+16);
                s->dsp.put_signed_rect_clamped(frame + y*stride, stride,
                        p->idwt_buf + y*p->idwt_stride, p->idwt_stride, width, 16);
            }
        } else {
            int dst_x = -p->xedge;
            for (y = 0; y < height; y += p->ybsep) {
                int dst_y = -p->yedge;

                ff_spatial_idwt_slice2(&d, y+p->yblen);
                
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
