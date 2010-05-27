/**
 * VP8 compatible video decoder
 *
 * Copyright (C) 2010 David Conrad
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

#include "avcodec.h"
#include "vp56.h"
#include "vp8data.h"
#include "h264pred.h"
#include "rectangle.h"

typedef struct {
    uint8_t segment;
    uint8_t skip;
    uint8_t mode;
    uint8_t uvmode;     ///< could be packed in with mode
} VP8Macroblock;

typedef struct {
    AVCodecContext *avctx;
    DSPContext dsp;
    H264PredContext hpc;
    AVFrame frames[4];
    AVFrame *framep[6];
    uint8_t *edge_emu_buffer_alloc;
    uint8_t *edge_emu_buffer;
    VP56RangeCoder c;
    int sub_version;

    int mb_width;   /* number of horizontal MB */
    int mb_height;  /* number of vertical MB */

    int keyframe;
    int referenced; ///< update last frame with the current one
    int clamp;

    int num_partitions;
    struct {
        VP56RangeCoder c;
    } partition[8];

    VP8Macroblock *macroblocks;
    uint8_t *intra4x4_pred_mode;
    uint8_t *intra4x4_pred_mode_base;
    int intra4x4_stride;

    /**
     * For coeff decode, we need to know whether the above block had non-zero
     * coefficients. This means for each macroblock, we need data for 4 luma
     * blocks, 2 u blocks, 2 v blocks, and the luma dc block, for a total of 9
     * per macroblock. We keep the last row in top_nnz.
     */
    uint8_t (*top_nnz)[9];

#define MAX_NUM_SEGMENTS 4
    struct {
        int enabled;
        int absolute_vals;
        int update_map;
        int8_t quant[MAX_NUM_SEGMENTS];
        int8_t lf_level[MAX_NUM_SEGMENTS];
    } segments;

    struct {
        int type;
        int level;
        int sharpness;
    } filter;

    struct {
        int enabled;
        int8_t mode[4];
        int8_t ref[4];
    } lf_delta;

    struct {
        // [0] - DC qmul  [1] - AC qmul
        int16_t luma_qmul[2];
        int16_t luma_dc_qmul[2];    ///< luma dc-only block quant
        int16_t chroma_qmul[2];
    } qmat[4];  ///< [segment] (fixme: rename this segment, dunno what to call current .segments)

    int golden_sign_bias;
    int altref_sign_bias;

    int mbskip_enabled;

    struct {
        uint8_t segmentid[3];
        uint8_t mbskip;
        uint8_t intra;
        uint8_t last;
        uint8_t golden;
        uint8_t pred16x16[4];
        uint8_t pred8x8c[3];
        uint8_t token[4][8][3][NUM_DCT_TOKENS-1];
        uint8_t mvc[2][19];
    } prob;
} VP8Context;

#define RL24(p) (AV_RL16(p) + ((p)[2] << 16))

// XXX: vp56_size_changed
static int update_dimensions(VP8Context *s, int width, int height)
{
    int i;
    if (avcodec_check_dimensions(s->avctx, width, height))
        return -1;

    avcodec_set_dimensions(s->avctx, width, height);

    s->mb_width  = (s->avctx->coded_width +15) / 16;
    s->mb_height = (s->avctx->coded_height+15) / 16;

    // we allocate a border around the top/left of intra4x4 modes
    // this is 4 blocks on the left to keep alignment for fill_rectangle
    s->intra4x4_stride = 4*(s->mb_width+1);

    s->macroblocks = av_realloc(s->macroblocks,
                                s->mb_width*s->mb_height*sizeof(*s->macroblocks));
    s->intra4x4_pred_mode_base = av_realloc(s->intra4x4_pred_mode_base,
                                            s->intra4x4_stride*(4*s->mb_height+1));
    s->intra4x4_pred_mode = s->intra4x4_pred_mode_base + 4 + s->intra4x4_stride;

    // zero the edges used for context prediction
    memset(s->intra4x4_pred_mode_base, 0, s->intra4x4_stride);
    for (i = 0; i < 4*s->mb_height; i++)
        s->intra4x4_pred_mode[i*s->intra4x4_stride-1] = 0;

    s->top_nnz = av_realloc(s->top_nnz, s->mb_width*sizeof(*s->top_nnz));

    return 0;
}

static void parse_segment_info(VP8Context *s)
{
    VP56RangeCoder *c = &s->c;
    int i;

    av_log(s->avctx, AV_LOG_INFO, "segmented\n");

    s->segments.update_map = vp8_rac_get(c);

    if (vp8_rac_get(c)) { // update segment feature data
        s->segments.absolute_vals = vp8_rac_get(c);

        for (i = 0; i < MAX_NUM_SEGMENTS; i++)
            s->segments.quant[i] = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 7) : 0;

        for (i = 0; i < MAX_NUM_SEGMENTS; i++)
            s->segments.lf_level[i] = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 6) : 0;
    }
    if (s->segments.update_map)
        for (i = 0; i < 3; i++)
            s->prob.segmentid[i] = vp8_rac_get(c) ? vp8_rac_get_uint(c, 8) : 255;
}

static void update_lf_deltas(VP8Context *s)
{
    VP56RangeCoder *c = &s->c;
    int i;

    for (i = 0; i < 4; i++)
        s->lf_delta.ref[i]  = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 6) : 0;

    for (i = 0; i < 4; i++)
        s->lf_delta.mode[i] = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 6) : 0;
}

static int setup_partitions(VP8Context *s, const uint8_t *buf, int buf_size)
{
    const uint8_t *sizes = buf;
    int i;

    s->num_partitions = 1 << vp8_rac_get_uint(&s->c, 2);
    av_log(s->avctx, AV_LOG_INFO, "%d partitions\n", s->num_partitions);

    buf      += 3*(s->num_partitions-1);
    buf_size -= 3*(s->num_partitions-1);
    if (buf_size < 0)
        return -1;

    for (i = 0; i < s->num_partitions-1; i++) {
        int size = RL24(sizes + 3*i);
        if (buf_size - size < 0)
            return -1;

        vp56_init_range_decoder(&s->partition[i].c, buf, size);
        buf      += size;
        buf_size -= size;
    }
    vp56_init_range_decoder(&s->partition[i].c, buf, buf_size);

    return 0;
}

static void get_quants(VP8Context *s)
{
    VP56RangeCoder *c = &s->c;

    int yac_qi     = vp8_rac_get_uint(c, 7);
    int ydc_delta  = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 4) : 0;
    int y2dc_delta = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 4) : 0;
    int y2ac_delta = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 4) : 0;
    int uvdc_delta = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 4) : 0;
    int uvac_delta = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 4) : 0;

    // fixme: segments
    s->qmat[0].luma_qmul[0]    =         vp8_dc_qlookup[av_clip(yac_qi + ydc_delta , 0, 127)];
    s->qmat[0].luma_qmul[1]    =         vp8_ac_qlookup[av_clip(yac_qi             , 0, 127)];
    s->qmat[0].luma_dc_qmul[0] =     2 * vp8_dc_qlookup[av_clip(yac_qi + y2dc_delta, 0, 127)];
    s->qmat[0].luma_dc_qmul[1] =   155 * vp8_ac_qlookup[av_clip(yac_qi + y2ac_delta, 0, 127)] / 100;
    s->qmat[0].chroma_qmul[0]  = av_clip(vp8_dc_qlookup[av_clip(yac_qi + uvdc_delta, 0, 127)], 0, 132);
    s->qmat[0].chroma_qmul[1]  =         vp8_ac_qlookup[av_clip(yac_qi + uvac_delta, 0, 127)];
}

static void update_refs(VP8Context *s)
{
    VP56RangeCoder *c = &s->c;
    
    int update_golden = vp8_rac_get(c);
    int update_altref = vp8_rac_get(c);

    if (!update_golden) {
        vp8_rac_get_uint(c, 2); // 0: none  1: last frame  2: alt ref frame
    }

    if (!update_altref) {
        vp8_rac_get_uint(c, 2); // 0: none  1: last frame  2: golden frame
    }
}

static int decode_frame_header(VP8Context *s, const uint8_t *buf, int buf_size)
{
    VP56RangeCoder *c = &s->c;

    int invisible, header_size;
    int width, height, hscale, vscale;
    int i, j, k, l;

    s->keyframe = s->framep[VP56_FRAME_CURRENT]->key_frame = !(buf[0] & 1);

    s->sub_version = (buf[0]>>1) & 7;
    invisible      = !(buf[0] & 0x10);
    header_size    = RL24(buf) >> 5;
    buf      += 3;
    buf_size -= 3;

    av_log(s->avctx, AV_LOG_INFO, "sub version %d, invisible %d suze %d\n",
           s->sub_version, invisible, header_size);

    if (s->keyframe) {
        if (RL24(buf) != 0x2a019d) {
            av_log(s->avctx, AV_LOG_ERROR, "Invalid start code 0x%x\n", RL24(buf));
            return AVERROR_INVALIDDATA;
        }
        width  = AV_RL16(buf+3) & 0x3fff;
        height = AV_RL16(buf+5) & 0x3fff;
        hscale = buf[4] >> 6;
        vscale = buf[6] >> 6;
        buf      += 7;
        buf_size -= 7;

        av_log(s->avctx, AV_LOG_INFO, "dim %dx%d scale %dx%d\n", width, height, hscale, vscale);

        if (!s->macroblocks || /* first frame */
            width != s->avctx->width || height != s->avctx->height)
            update_dimensions(s, width, height);

        memcpy(s->prob.token    , vp8_token_default_probs , sizeof(s->prob.token));
        memcpy(s->prob.pred16x16, vp8_pred16x16_prob_intra, sizeof(s->prob.pred16x16));
        memcpy(s->prob.pred8x8c , vp8_pred8x8c_prob_intra , sizeof(s->prob.pred8x8c));
        memset(&s->segments, 0, sizeof(s->segments));
    }

    if (header_size > buf_size) {
        av_log(s->avctx, AV_LOG_ERROR, "Header size larger than data\n");
        return AVERROR_INVALIDDATA;
    }
    vp56_init_range_decoder(c, buf, header_size);
    buf      += header_size;
    buf_size -= header_size;

    if (s->keyframe) {
        if (vp8_rac_get(c))
            av_log(s->avctx, AV_LOG_WARNING, "Unspecified colorspace\n");
        s->clamp = vp8_rac_get(c);
    }

    if ((s->segments.enabled = vp8_rac_get(c)))
        parse_segment_info(s);
    else
        s->segments.update_map = 0; // FIXME: move this to some init function?

    s->filter.type      = vp8_rac_get(c);
    s->filter.level     = vp8_rac_get_uint(c, 6);
    s->filter.sharpness = vp8_rac_get_uint(c, 3);

    if ((s->lf_delta.enabled = vp8_rac_get(c)))
        if (vp8_rac_get(c))
            update_lf_deltas(s);

    if (setup_partitions(s, buf, buf_size)) {
        av_log(s->avctx, AV_LOG_ERROR, "Invalid partitions\n");
        return AVERROR_INVALIDDATA;
    }

    get_quants(s);

    if (!s->keyframe) {
        update_refs(s);
        s->golden_sign_bias = vp8_rac_get(c);
        s->altref_sign_bias = vp8_rac_get(c);
    }

    if (vp8_rac_get(c)) {
        // reset probabilities (yay for being omitted from the spec)
    }

    s->referenced = s->keyframe || vp8_rac_get(c);

    for (i = 0; i < 4; i++)
        for (j = 0; j < 8; j++)
            for (k = 0; k < 3; k++)
                for (l = 0; l < NUM_DCT_TOKENS-1; l++)
                    if (vp56_rac_get_prob(c, vp8_token_update_probs[i][j][k][l]))
                        s->prob.token[i][j][k][l] = vp8_rac_get_uint(c, 8);

    if ((s->mbskip_enabled = vp8_rac_get(c)))
        s->prob.mbskip = vp8_rac_get_uint(c, 8);

    if (!s->keyframe) {
        s->prob.intra  = vp8_rac_get_uint(c, 8);
        s->prob.last   = vp8_rac_get_uint(c, 8);
        s->prob.golden = vp8_rac_get_uint(c, 8);

        if (vp8_rac_get(c))
            for (i = 0; i < 4; i++)
                s->prob.pred16x16[i] = vp8_rac_get_uint(c, 8);
        if (vp8_rac_get(c))
            for (i = 0; i < 3; i++)
                s->prob.pred8x8c[i]  = vp8_rac_get_uint(c, 8);

        // 17.2 MV probability update
        for (i = 0; i < 2; i++)
            for (j = 0; j < 19; j++)
                if (vp56_rac_get_prob(c, vp8_mv_update_prob[i][j]))
                    s->prob.mvc[i][j] = vp8_rac_get_nn(c);
    } else {
        // reset s->prob.mvc
        memcpy(s->prob.mvc, vp8_mv_default_prob, 19*2);
    }

    return 0;
}

static inline void decode_intra4x4_modes(VP56RangeCoder *c, uint8_t *intra4x4,
                                         int stride, int keyframe)
{
    int x, y, t, l;
    const uint8_t *ctx = vp8_pred4x4_prob_inter;

    for (y = 0; y < 4; y++) {
        for (x = 0; x < 4; x++) {
            if (keyframe) {
                t = intra4x4[x - stride];
                l = intra4x4[x - 1];
                ctx = vp8_pred4x4_prob_intra[t][l];
            }
            intra4x4[x] = vp8_rac_get_tree(c, vp8_pred4x4_tree, ctx);
        }
        intra4x4 += stride;
    }
}

static void decode_mb_mode(VP8Context *s, VP8Macroblock *mb, uint8_t *intra4x4)
{
    VP56RangeCoder *c = &s->c;

    if (s->segments.update_map)
        mb->segment = vp8_rac_get_tree(c, vp8_segmentid_tree, s->prob.segmentid);

    mb->skip = s->mbskip_enabled ? vp56_rac_get_prob(c, s->prob.mbskip) : 0;

    if (s->keyframe) {
        mb->mode = vp8_rac_get_tree(c, vp8_pred16x16_tree_intra, s->prob.pred16x16);

        if (mb->mode == MODE_I4x4)
            decode_intra4x4_modes(c, intra4x4, s->intra4x4_stride, 1);
        else
            fill_rectangle(intra4x4, 4, 4, s->intra4x4_stride, vp8_pred4x4_mode[mb->mode], 1);

        mb->uvmode = vp8_rac_get_tree(c, vp8_pred8x8c_tree, vp8_pred8x8c_prob_intra);
    } else if (vp56_rac_get_prob(c, s->prob.intra)) {
        // inter MB, 16.2
    } else {
        // intra MB, 16.1
        mb->mode = vp8_rac_get_tree(c, vp8_pred16x16_tree_inter, s->prob.pred16x16);

        if (mb->mode == MODE_I4x4)
            decode_intra4x4_modes(c, intra4x4, s->intra4x4_stride, 0);

        mb->uvmode = vp8_rac_get_tree(c, vp8_pred8x8c_tree, vp8_pred8x8c_prob_inter);
    }
}

/**
 * @param i initial coeff index, 0 unless a separate DC block is coded
 * @param zero_nhood the initial prediction context for number of surrounding
 *                   all-zero blocks (only left/top, so 0-2)
 * @param qmul[0] dc dequant factor
 * @param qmul[1] ac dequant factor
 * @return 1 if any non-zero coeffs were decoded, 0 otherwise
 */
static int decode_block_coeffs(VP56RangeCoder *c, DCTELEM block[16],
                               uint8_t probs[8][3][NUM_DCT_TOKENS-1],
                               int i, int zero_nhood, int16_t qmul[2])
{
    int token, nonzero = 0;

    for (; i < 16; i++) {
        token = vp8_rac_get_tree(c, vp8_coeff_tree, probs[vp8_coeff_band[i]][zero_nhood]);

        if (token == DCT_EOB)
            break;
        else if (token >= DCT_CAT1)
            token = vp8_rac_get_coeff(c, vp8_dct_cat_prob[token-DCT_CAT1]);

        // after the first token, the non-zero prediction context becomes
        // based on the last decoded coeff
        if (!token) {
            zero_nhood = 0;
            continue;
        } else if (token == 1)
            zero_nhood = 1;
        else
            zero_nhood = 2;

        // todo: full [16] qmat? load into register?
        block[zigzag_scan[i]] = (vp8_rac_get(c) ? -token : token) * qmul[!!i];
        nonzero = 1;
    }
    return nonzero;
}

// todo: save nnz in a usable form for dc-only idct
static void decode_mb_coeffs(VP8Context *s, VP56RangeCoder *c, VP8Macroblock *mb,
                             DCTELEM block[6][4][16],
                             uint8_t t_nnz[9], uint8_t l_nnz[9])
{
    LOCAL_ALIGNED_16(DCTELEM, dc,[16]);
    int i, x, y, luma_start = 0, luma_ctx = 3;
    int l_nnz_pred, nnz_pred, nnz = 0;
    int segment = s->segments.enabled ? mb->segment : 0;

    s->dsp.clear_blocks((DCTELEM *)block);

    // also SPLIT_MV (4MV?)
    if (mb->mode != MODE_I4x4) {
        AV_ZERO128(dc);
        AV_ZERO128(dc+8);
        nnz_pred = t_nnz[8] + l_nnz[8];

        // decode DC values and do hadamard
        nnz = decode_block_coeffs(c, dc, s->prob.token[1], 0, nnz_pred,
                                  s->qmat[segment].luma_dc_qmul);
        s->dsp.vp8_luma_dc_wht(block, dc);
        luma_start = 1;
        luma_ctx = 0;
    }
    l_nnz[8] = t_nnz[8] = nnz;

    // luma blocks
    for (y = 0; y < 4; y++) {
        l_nnz_pred = l_nnz[y];
        for (x = 0; x < 4; x++) {
            nnz_pred = l_nnz_pred + t_nnz[x];
            nnz = decode_block_coeffs(c, block[y][x], s->prob.token[luma_ctx], luma_start,
                                      nnz_pred, s->qmat[segment].luma_qmul);
            t_nnz[x] = l_nnz_pred = nnz;
        }
        l_nnz[y] = l_nnz_pred;
    }

    // chroma blocks
    // TODO: what to do about dimensions? 2nd dim for luma is x,
    // but for chroma it's (y<<1)|x
    for (i = 4; i < 6; i++)
        for (y = 0; y < 2; y++) {
            l_nnz_pred = l_nnz[i+2*y];
            for (x = 0; x < 2; x++) {
                nnz_pred = l_nnz_pred + t_nnz[i+2*x];
                nnz = decode_block_coeffs(c, block[i][(y<<1)+x], s->prob.token[2], 0,
                                          nnz_pred, s->qmat[segment].chroma_qmul);
                t_nnz[i+2*x] = l_nnz_pred = nnz;
            }
            l_nnz[i+2*y] = l_nnz_pred;
        }
}

static int vp8_decode_frame(AVCodecContext *avctx, void *data, int *data_size,
                            AVPacket *avpkt)
{
    VP8Context *s = avctx->priv_data;
    LOCAL_ALIGNED_16(DCTELEM, block,[6],[4][16]);
    int ret, mb_x, mb_y, i, y;
    VP8Macroblock *mb;
    AVFrame *frame;

    if ((ret = decode_frame_header(s, avpkt->data, avpkt->size)) < 0)
        return ret;

    if (s->keyframe) {
        if (s->framep[VP56_FRAME_GOLDEN]->data[0])
            avctx->release_buffer(avctx, s->framep[VP56_FRAME_GOLDEN]);
        avctx->get_buffer(avctx, s->framep[VP56_FRAME_GOLDEN]);
        frame = s->framep[VP56_FRAME_GOLDEN];
    } else {
        // inter buffer stuff
        return 0;
    }

    mb = s->macroblocks;
    memset(s->top_nnz, 0, s->mb_width*sizeof(*s->top_nnz));

    // top edge of 127 for intra prediction
    for (i = 0; i < 3; i++)
        memset(frame->data[i] - frame->linesize[i], 127, frame->linesize[i]);

    for (mb_y = 0; mb_y < s->mb_height; mb_y++) {
        VP56RangeCoder *c = &s->partition[mb_y%s->num_partitions].c;
        uint8_t *intra4x4 = s->intra4x4_pred_mode + 4*mb_y*s->intra4x4_stride;
        uint8_t (*t_nnz)[9] = s->top_nnz;
        uint8_t l_nnz[9] = { 0 };   // AV_ZERO64
        uint8_t *dst[3];

        // left edge of 129 for intra prediction
        for (i = 0; i < 3; i++) {
            dst[i] = frame->data[i] + (16>>!!i)*mb_y*frame->linesize[i];
            for (y = 0; y < 16>>!!i; y++)
                dst[i][y*frame->linesize[i]-1] = 129;
        }

        for (mb_x = 0; mb_x < s->mb_width; mb_x++) {
            decode_mb_mode(s, mb, intra4x4 + 4*mb_x);

            if (!mb->skip) {
                decode_mb_coeffs(s, c, mb, block, *t_nnz, l_nnz);
            } else {
                memset(l_nnz, 0, 9);
                memset(t_nnz, 0, 9);
            }

            t_nnz++;
            mb++;
        }
    }

    // init the intra pred probabilities for inter frames
    // this seems like it'll be a bit tricky for frame-base multithreading
    if (s->keyframe) {
        memcpy(s->prob.pred16x16, vp8_pred16x16_prob_inter, sizeof(s->prob.pred16x16));
        memcpy(s->prob.pred8x8c , vp8_pred8x8c_prob_inter , sizeof(s->prob.pred8x8c));
    }

    return 0;
}

static av_cold int vp8_decode_init(AVCodecContext *avctx)
{
    VP8Context *s = avctx->priv_data;
    int i;

    s->avctx = avctx;
    avctx->pix_fmt = PIX_FMT_YUV420P;

    dsputil_init(&s->dsp, avctx);
    ff_h264_pred_init(&s->hpc, CODEC_ID_VP8);

    // intra pred needs edge emulation among other things
    if (avctx->flags&CODEC_FLAG_EMU_EDGE) {
        av_log(avctx, AV_LOG_ERROR, "Edge emulation not supproted\n");
        return AVERROR_PATCHWELCOME;
    }

    for (i=0; i<4; i++)
        s->framep[i] = &s->frames[i];
    s->framep[VP56_FRAME_UNUSED] = s->framep[VP56_FRAME_GOLDEN];
    s->framep[VP56_FRAME_UNUSED2] = s->framep[VP56_FRAME_GOLDEN2];
    s->edge_emu_buffer_alloc = NULL;

    return 0;
}

static av_cold int vp8_decode_free(AVCodecContext *avctx)
{
    VP8Context *s = avctx->priv_data;

    if (s->framep[VP56_FRAME_GOLDEN]->data[0])
        avctx->release_buffer(avctx, s->framep[VP56_FRAME_GOLDEN]);
    if (s->framep[VP56_FRAME_GOLDEN2]->data[0])
        avctx->release_buffer(avctx, s->framep[VP56_FRAME_GOLDEN2]);
    if (s->framep[VP56_FRAME_PREVIOUS]->data[0])
        avctx->release_buffer(avctx, s->framep[VP56_FRAME_PREVIOUS]);

    av_freep(&s->macroblocks);
    av_freep(&s->intra4x4_pred_mode_base);
    av_freep(&s->top_nnz);

    return 0;
}

AVCodec vp8_decoder = {
    "vp8",
    AVMEDIA_TYPE_VIDEO,
    CODEC_ID_VP8,
    sizeof(VP8Context),
    vp8_decode_init,
    NULL,
    vp8_decode_free,
    vp8_decode_frame,
    CODEC_CAP_DR1,
    .long_name = NULL_IF_CONFIG_SMALL("On2 VP8"),
};
