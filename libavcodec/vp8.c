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

#define LEFT_TOP_MARGIN     (16 << 3)
#define RIGHT_BOTTOM_MARGIN (16 << 3)

typedef struct {
    uint8_t segment;
    uint8_t skip;
    uint8_t mode;
    uint8_t uvmode;     ///< could be packed in with mode
    uint8_t ref_frame;
    uint8_t partitioning;
    VP56mv mv;
    VP56mv bmv[16];
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
    int linesize[3];

    int keyframe;
    int referenced; ///< update last frame with the current one
    int clamp;

    int num_partitions;
    struct {
        VP56RangeCoder c;
    } partition[8];

    VP8Macroblock *macroblocks;
    VP8Macroblock *macroblocks_base;
    int mb_stride;
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
    DECLARE_ALIGNED(8, uint8_t, left_nnz)[9];

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

    int sign_bias[4]; ///< one state [0, 1] per ref frame type

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

    s->mb_stride = s->mb_width + 1;
    s->macroblocks_base = av_realloc(s->macroblocks_base,
                                s->mb_stride*(s->mb_height+1)*sizeof(*s->macroblocks));
    s->macroblocks = s->macroblocks_base + 1 + s->mb_stride;
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

        if (!s->macroblocks_base || /* first frame */
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
        s->sign_bias[VP56_FRAME_GOLDEN]               = vp8_rac_get(c);
        s->sign_bias[VP56_FRAME_GOLDEN2 /* altref */] = vp8_rac_get(c);
    } else {
        s->sign_bias[VP56_FRAME_GOLDEN]               = 0;
        s->sign_bias[VP56_FRAME_GOLDEN2]              = 0;
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

static inline void clamp_mv(VP8Context *s, VP56mv *dst, const VP56mv *src,
                            int mb_x, int mb_y)
{
    dst->x = av_clip(src->x, -((mb_x << 7) + LEFT_TOP_MARGIN),
                     ((s->mb_width - 1 - mb_x) << 7) + RIGHT_BOTTOM_MARGIN);
    dst->y = av_clip(src->y, -((mb_y << 7) + LEFT_TOP_MARGIN),
                     ((s->mb_height - 1 - mb_y) << 7) + RIGHT_BOTTOM_MARGIN);
}

static void find_near_mvs(VP8Context *s, VP8Macroblock *mb,
                          VP56mv near[2], VP56mv *best, int cnt[4])
{
    VP8Macroblock *mb_edge[3] = { mb - 1 /* left */,
                                  mb - s->mb_stride /* top */,
                                  mb - s->mb_stride - 1 /* top-left */ };
    enum { EDGE_LEFT, EDGE_TOP, EDGE_TOPLEFT };
    VP56mv near_mv[4]  = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
    enum { CNT_INTRA, CNT_NEAREST, CNT_NEAR, CNT_SPLITMV };
    int idx = CNT_INTRA, n;

    /* Process MB on top, left and top-left */
    for (n = 0; n < 3; n++) {
        VP8Macroblock *edge = mb_edge[n];
        if (edge->ref_frame != VP56_FRAME_CURRENT) {
            if (edge->mv.x && edge->mv.y) {
                VP56mv tmp = edge->mv;
                if (s->sign_bias[mb->ref_frame] != s->sign_bias[edge->ref_frame]) {
                    tmp.x *= -1;
                    tmp.y *= -1;
                }
                if (tmp.x != near_mv[idx].x && tmp.y != near_mv[idx].y)
                    near_mv[++idx] = tmp;
                cnt[idx]       += 1 + (n != 2);
            } else
                cnt[CNT_INTRA] += 1 + (n != 2);
        }
    }

    /* If we have three distinct MV's, attempt merge of top-left with left */
    if (cnt[CNT_SPLITMV] &&
        near_mv[1+EDGE_LEFT].x == near_mv[1+EDGE_TOPLEFT].x &&
        near_mv[1+EDGE_LEFT].y == near_mv[1+EDGE_TOPLEFT].y)
        cnt[CNT_NEAREST] += 1;

    cnt[CNT_SPLITMV] = ((mb_edge[EDGE_LEFT]->mode   == VP8_MVMODE_SPLIT) +
                        (mb_edge[EDGE_TOP]->mode    == VP8_MVMODE_SPLIT)) * 2 +
                       (mb_edge[EDGE_TOPLEFT]->mode == VP8_MVMODE_SPLIT);

    /* Swap near and nearest if necessary */
    if (cnt[CNT_NEAR] > cnt[CNT_NEAREST]) {
        FFSWAP(int,    cnt[CNT_NEAREST],     cnt[CNT_NEAR]);
        FFSWAP(VP56mv, near_mv[CNT_NEAREST], near_mv[CNT_NEAR]);
    }

    /* Use near_mv[0] to store the "best" MV */
    if (cnt[CNT_NEAREST] >= cnt[CNT_INTRA])
        near_mv[CNT_INTRA] = near_mv[CNT_NEAREST];

    *best   = near_mv[CNT_INTRA];
    near[0] = near_mv[CNT_NEAREST];
    near[1] = near_mv[CNT_NEAR];
}

/**
 * Motion vector coding, 17.1.
 */
static int read_mv_component(VP56RangeCoder *c, const uint8_t *p)
{
    int x = 0;

    if (vp56_rac_get_prob(c, p[0])) {
        int i;

        for (i = 0; i < 3; i++)
            x += vp56_rac_get_prob(c, p[9 + i]) << i;
        for (i = 9; i > 3; i--)
            x += vp56_rac_get_prob(c, p[9 + i]) << i;
        if (!(x & 0xFFF0) || vp56_rac_get_prob(c, p[12]))
            x += 8;
    } else
        x = vp8_rac_get_tree(c, vp8_small_mvtree, &p[2]);

    return (x && vp56_rac_get_prob(c, p[1])) ? -x : x;
}

static const uint8_t *get_submv_prob(const VP56mv *left, const VP56mv *above)
{
    int lez = (left->x  == 0 && left->y  == 0);

    if (left->x  == above->x && left->y == above->y)
        return lez ? vp8_submv_prob[4] : vp8_submv_prob[3];
    if (above->x == 0 && above->y == 0)
        return vp8_submv_prob[2];
    return lez ? vp8_submv_prob[1] : vp8_submv_prob[0];
}

/**
 * Split motion vector prediction, 16.4.
 */
static void decode_splitmvs(VP8Context    *s,  VP56RangeCoder *c,
                            VP8Macroblock *mb, VP56mv         *base_mv,
                            uint8_t       *intra4x4mode)
{
    int part_idx = mb->partitioning =
        vp8_rac_get_tree(c, vp8_mbsplit_tree, vp8_mbsplit_prob);
    int n, num = vp8_mbsplit_count[part_idx];
    VP56mv part_mv[16];
    int part_mode[16];

    for (n = 0; n < num; n++) {
        int k = part_idx == 2 ? ((n & 2) << 2) | ((n & 1) << 1) :
               (part_idx == 3 ? n : n << (3 - 2 * part_idx));
        const VP56mv *left  = (k & 3)  ? &mb->bmv[k - 1] : &mb[-1].bmv[k + 3],
                     *above = (k >> 2) ? &mb->bmv[k - 4] : &mb[-1].bmv[k + 12];

        part_mode[n] = vp8_rac_get_tree(c, vp8_submv_ref_tree,
                                        get_submv_prob(left, above));
        switch (part_mode[n]) {
        case VP8_SUBMVMODE_NEW4X4:
            part_mv[n].x = base_mv->x + read_mv_component(c, s->prob.mvc[0]);
            part_mv[n].y = base_mv->y + read_mv_component(c, s->prob.mvc[1]);
            break;
        case VP8_SUBMVMODE_ZERO4X4:
            part_mv[n].x = 0;
            part_mv[n].y = 0;
            break;
        case VP8_SUBMVMODE_LEFT4X4:
            part_mv[n] = *left;
            break;
        case VP8_SUBMVMODE_TOP4X4:
            part_mv[n] = *above;
            break;
        }

        /* fill out over the 4x4 blocks in MB */
        for (k = 0; k < 16; k++)
            if (vp8_mbsplits[part_idx][k] == n) {
                mb->bmv[k]      = part_mv[n];
                intra4x4mode[k] = part_mode[n];
            }
    }
}

static void decode_mb_mode(VP8Context *s, VP8Macroblock *mb, int mb_x, int mb_y,
                           uint8_t *intra4x4)
{
    VP56RangeCoder *c = &s->c;
    int n;

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
        mb->ref_frame = VP56_FRAME_CURRENT;
    } else if (vp56_rac_get_prob(c, s->prob.intra)) {
        VP56mv near[2], best;
        int cnt[4] = { 0, 0, 0, 0 };
        uint8_t p[4];

        // inter MB, 16.2
        mb->ref_frame = vp56_rac_get_prob(c, s->prob.last) ?
             (vp56_rac_get_prob(c, s->prob.golden) ?
              VP56_FRAME_GOLDEN2 /* altref */ : VP56_FRAME_GOLDEN) : VP56_FRAME_PREVIOUS;

        // motion vectors, 16.3
        find_near_mvs(s, mb, near, &best, cnt);
        for (n = 0; n < 4; n++)
            p[n] = vp8_mode_contexts[cnt[n]][n];
        mb->mode = vp8_rac_get_tree(c, vp8_pred16x16_tree_mvinter, p);
        switch (mb->mode) {
        case VP8_MVMODE_SPLIT:
            decode_splitmvs(s, c, mb, &best, intra4x4);
            mb->mv = mb->bmv[15];
            break;
        case VP8_MVMODE_ZERO:
            mb->mv.x = 0;
            mb->mv.y = 0;
            break;
        case VP8_MVMODE_NEAREST:
            clamp_mv(s, &mb->mv, &near[0], mb_x, mb_y);
            break;
        case VP8_MVMODE_NEAR:
            clamp_mv(s, &mb->mv, &near[1], mb_x, mb_y);
            break;
        case VP8_MVMODE_NEW:
            mb->mv.x = read_mv_component(c, s->prob.mvc[0]);
            mb->mv.y = read_mv_component(c, s->prob.mvc[1]);
            clamp_mv(s, &mb->mv, &mb->mv, mb_x, mb_y);
            break;
        }
        if (mb->mode != VP8_MVMODE_SPLIT) {
            for (n = 0; n < 16; n++)
                mb->bmv[n] = mb->mv;
            fill_rectangle(intra4x4, 4, 4, s->intra4x4_stride, mb->mode, 1);
        }
    } else {
        // intra MB, 16.1
        mb->mode = vp8_rac_get_tree(c, vp8_pred16x16_tree_inter, s->prob.pred16x16);

        if (mb->mode == MODE_I4x4)
            decode_intra4x4_modes(c, intra4x4, s->intra4x4_stride, 0);
        else
            fill_rectangle(intra4x4, 4, 4, s->intra4x4_stride, vp8_pred4x4_mode[mb->mode], 1);

        mb->uvmode = vp8_rac_get_tree(c, vp8_pred8x8c_tree, vp8_pred8x8c_prob_inter);
        mb->ref_frame = VP56_FRAME_CURRENT;
    }
}

// todo: optimize (see ff_h264_check_intra_pred_mode)
// also, what happens on inter frames?
static int check_intra_pred_mode(int mode, int mb_x, int mb_y)
{
    if (mode == DC_PRED8x8) {
        if (!mb_x && !mb_y)
            mode = DC_128_PRED8x8;
        else if (!mb_y)
            mode = LEFT_DC_PRED8x8;
        else if (!mb_x)
            mode = TOP_DC_PRED8x8;
    }
    return mode;
}

static int check_intra4x4_pred_mode(int mode, int x, int y)
{
    if (mode == DC_PRED) {
        if (!x && !y)
            mode = DC_128_PRED;
        else if (!y)
            mode = LEFT_DC_PRED;
        else if (!x)
            mode = TOP_DC_PRED;
    }
    return mode;
}

static void intra_predict(VP8Context *s, uint8_t *dst[3], VP8Macroblock *mb,
                          uint8_t *bmode, DCTELEM block[6][4][16], int mb_x, int mb_y)
{
    DECLARE_ALIGNED(4, static const uint8_t, tr_rightedge)[4] = { 127, 127, 127, 127 };
    int x, y, mode;

    // fixme: special DC modes
    if (mb->mode < MODE_I4x4) {
        mode = check_intra_pred_mode(mb->mode, mb_x, mb_y);
        s->hpc.pred16x16[mode](dst[0], s->linesize[0]);
    } else {
        // all blocks on the right edge use the top right edge of
        // the top macroblock (since the right mb isn't decoded yet)
        const uint8_t *tr_right = dst[0] - s->linesize[0] + 16;
        uint8_t *i4x4dst = dst[0];

        // use 127 for top right blocks that don't exist
        if (mb_x == s->mb_width-1)
            tr_right = tr_rightedge;

        for (y = 0; y < 4; y++) {
            for (x = 0; x < 3; x++) {
                uint8_t *tr = i4x4dst+4*x - s->linesize[0]+4;
                mode = check_intra4x4_pred_mode(vp8_pred4x4_func[bmode[x]], mb_x+x, mb_y+y);
                s->hpc.pred4x4[mode](i4x4dst+4*x, tr, s->linesize[0]);
                if (!mb->skip)
                    s->dsp.vp8_idct_add(i4x4dst+4*x, block[y][x], s->linesize[0]);
            }
            mode = check_intra4x4_pred_mode(vp8_pred4x4_func[bmode[x]], mb_x+x, mb_y+y);
            s->hpc.pred4x4[mode](i4x4dst+4*x, tr_right, s->linesize[0]);
            if (!mb->skip)
                s->dsp.vp8_idct_add(i4x4dst+4*x, block[y][x], s->linesize[0]);

            i4x4dst += 4*s->linesize[0];
            bmode += s->intra4x4_stride;
        }
    }

    mode = check_intra_pred_mode(mb->uvmode, mb_x, mb_y);
    s->hpc.pred8x8[mode](dst[1], s->linesize[1]);
    s->hpc.pred8x8[mode](dst[2], s->linesize[2]);
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
    int offset = 0;

    for (; i < 16; i++) {
        token = vp8_rac_get_tree2(c, vp8_coeff_tree, probs[vp8_coeff_band[i]][zero_nhood], offset);

        if (token == DCT_EOB)
            break;
        else if (token >= DCT_CAT1) {
            int cat = token-DCT_CAT1;
            token = vp8_rac_get_coeff(c, vp8_dct_cat_prob[cat]);
            token += vp8_dct_cat_offset[cat];
        }

        // after the first token, the non-zero prediction context becomes
        // based on the last decoded coeff
        if (!token) {
            zero_nhood = 0;
            offset = 1;
            continue;
        } else if (token == 1)
            zero_nhood = 1;
        else
            zero_nhood = 2;

        // todo: full [16] qmat? load into register?
        block[zigzag_scan[i]] = (vp8_rac_get(c) ? -token : token) * qmul[!!i];
        nonzero = 1;
        offset = 0;
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
    int nnz_pred, nnz;
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
        l_nnz[8] = t_nnz[8] = nnz;
        s->dsp.vp8_luma_dc_wht(block, dc);
        luma_start = 1;
        luma_ctx = 0;
    }

    // luma blocks
    for (y = 0; y < 4; y++)
        for (x = 0; x < 4; x++) {
            nnz_pred = l_nnz[y] + t_nnz[x];
            nnz = decode_block_coeffs(c, block[y][x], s->prob.token[luma_ctx], luma_start,
                                      nnz_pred, s->qmat[segment].luma_qmul);
            t_nnz[x] = l_nnz[y] = nnz;
        }

    // chroma blocks
    // TODO: what to do about dimensions? 2nd dim for luma is x,
    // but for chroma it's (y<<1)|x
    for (i = 4; i < 6; i++)
        for (y = 0; y < 2; y++)
            for (x = 0; x < 2; x++) {
                nnz_pred = l_nnz[i+2*y] + t_nnz[i+2*x];
                nnz = decode_block_coeffs(c, block[i][(y<<1)+x], s->prob.token[2], 0,
                                          nnz_pred, s->qmat[segment].chroma_qmul);
                t_nnz[i+2*x] = l_nnz[i+2*y] = nnz;
            }
}

static void idct_mb(VP8Context *s, uint8_t *y_dst, uint8_t *u_dst, uint8_t *v_dst,
                    VP8Macroblock *mb, DCTELEM block[6][4][16])
{
    int x, y;

    if (mb->mode != MODE_I4x4)
        for (y = 0; y < 4; y++) {
            for (x = 0; x < 4; x++)
                s->dsp.vp8_idct_add(y_dst+4*x, block[y][x], s->linesize[0]);
            y_dst += 4*s->linesize[0];
        }

    for (y = 0; y < 2; y++) {
        for (x = 0; x < 2; x++) {
            s->dsp.vp8_idct_add(u_dst+4*x, block[4][(y<<1)+x], s->linesize[1]);
            s->dsp.vp8_idct_add(v_dst+4*x, block[5][(y<<1)+x], s->linesize[2]);
        }
        u_dst += 4*s->linesize[1];
        v_dst += 4*s->linesize[2];
    }
}

static int vp8_decode_frame(AVCodecContext *avctx, void *data, int *data_size,
                            AVPacket *avpkt)
{
    VP8Context *s = avctx->priv_data;
    LOCAL_ALIGNED_16(DCTELEM, block,[6],[4][16]);
    int ret, mb_x, mb_y, i, y;
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

    for (i = 0; i < 3; i++)
        s->linesize[i] = frame->linesize[i];

    memset(s->top_nnz, 0, s->mb_width*sizeof(*s->top_nnz));

    // top edge of 127 for intra prediction
    for (i = 0; i < 3; i++)
        memset(frame->data[i] - frame->linesize[i], 127, frame->linesize[i]);

    for (mb_y = 0; mb_y < s->mb_height; mb_y++) {
        VP56RangeCoder *c = &s->partition[mb_y%s->num_partitions].c;
        VP8Macroblock *mb = s->macroblocks + mb_y*s->mb_stride;
        uint8_t *intra4x4 = s->intra4x4_pred_mode + 4*mb_y*s->intra4x4_stride;
        uint8_t *dst[3];

        memset(s->left_nnz, 0, sizeof(s->left_nnz));

        // left edge of 129 for intra prediction
        for (i = 0; i < 3; i++) {
            dst[i] = frame->data[i] + (16>>!!i)*mb_y*frame->linesize[i];
            for (y = 0; y < 16>>!!i; y++)
                dst[i][y*frame->linesize[i]-1] = 129;
        }

        for (mb_x = 0; mb_x < s->mb_width; mb_x++) {
            decode_mb_mode(s, mb, mb_x, mb_y, intra4x4 + 4*mb_x);

            if (!mb->skip)
                decode_mb_coeffs(s, c, mb, block, s->top_nnz[mb_x], s->left_nnz);

            if (mb->mode <= MODE_I4x4) {
                intra_predict(s, dst, mb, intra4x4 + 4*mb_x, block, mb_x, mb_y);
            } else {
                // inter prediction
            }

            if (!mb->skip) {
                idct_mb(s, dst[0], dst[1], dst[2], mb, block);
            } else {
                AV_ZERO64(s->left_nnz);
                AV_WN64(s->top_nnz[mb_x], 0);   // array of 9, so unaligned

                // Reset DC block if it wouldn't exist if the mb wasn't skipped
                // SPLIT_MV too...
                if (mb->mode != MODE_I4x4) {
                    s->left_nnz[8] = 0;
                    s->top_nnz[mb_x][8] = 0;
                }
            }

            for (i = 0; i < 3; i++)
                dst[i] += 16 >> !!i;
            mb++;
        }
    }

    // init the intra pred probabilities for inter frames
    // this seems like it'll be a bit tricky for frame-base multithreading
    if (s->keyframe) {
        memcpy(s->prob.pred16x16, vp8_pred16x16_prob_inter, sizeof(s->prob.pred16x16));
        memcpy(s->prob.pred8x8c , vp8_pred8x8c_prob_inter , sizeof(s->prob.pred8x8c));
    }

    *(AVFrame*)data = *frame;
    *data_size = sizeof(AVFrame);

    return avpkt->size;
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

    av_freep(&s->macroblocks_base);
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
