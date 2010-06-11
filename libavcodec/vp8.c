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
#include "vp8dsp.h"
#include "h264pred.h"
#include "rectangle.h"

typedef struct {
    uint8_t segment;
    uint8_t skip;
    // todo: make it possible to check for at least (i4x4 or split_mv)
    // in one op. are others needed?
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
    VP8DSPContext vp8dsp;
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
    int update_golden; ///< copy altref (-2), last (-1) or cur (1) to golden
    int update_altref; ///< copy golden (-2), last (-1) or cur (1) to altref

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

    /**
     * This is the index plus one of the last non-zero coeff
     * for each of the blocks in the current macroblock.
     * So, 0 -> no coeffs
     *     1 -> dc-only (special transform)
     *     2+-> full transform
     */
    DECLARE_ALIGNED(16, uint8_t, non_zero_count_cache)[6][4];
    DECLARE_ALIGNED(16, DCTELEM, block)[6][4][16];

    /**
     * Base parameters for segmentation, i.e. per-macroblock parameters.
     * These must be kept unchanged even if segmentation is not used for
     * a frame, since the values persist between interframes.
     */
    struct {
        int enabled;
        int absolute_vals;
        int update_map;
        int8_t base_quant[4];
        int8_t filter_level[4];     ///< base loop filter level
    } segmentation;

    /**
     * Macroblocks can have one of 4 different quants in a frame when
     * segmentation is enabled.
     * If segmentation is disabled, only the first segment's values are used.
     */
    struct {
        // [0] - DC qmul  [1] - AC qmul
        int16_t luma_qmul[2];
        int16_t luma_dc_qmul[2];    ///< luma dc-only block quant
        int16_t chroma_qmul[2];
    } qmat[4];

    struct {
        int simple;
        int level;
        int sharpness;
    } filter;

    struct {
        int enabled;    ///< whether each mb can have a different strength based on mode/ref

        /**
         * filter strength adjustment for the following macroblock modes:
         * [0] - i4x4
         * [1] - zero mv
         * [2] - inter modes except for zero or split mv
         * [3] - split mv
         *  i16x16 modes never have any adjustment
         */
        int8_t mode[4];

        /**
         * filter strength adjustment for macroblocks that reference:
         * (TODO: make sure this is right)
         * [0] - intra / VP56_FRAME_CURRENT
         * [1] - VP56_FRAME_PREVIOUS
         * [2] - VP56_FRAME_GOLDEN
         * [3] - altref / VP56_FRAME_GOLDEN2
         */
        int8_t ref[4];
    } lf_delta;

    int sign_bias[4]; ///< one state [0, 1] per ref frame type

    int mbskip_enabled;

    /**
     * These are all of the updatable probabilities for binary decisions.
     * They are only implictly reset on keyframes, making it quite likely
     * for an interframe to desync if a prior frame's header was corrupt
     * or missing outright!
     */
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

    s->segmentation.update_map = vp8_rac_get(c);

    if (vp8_rac_get(c)) { // update segment feature data
        s->segmentation.absolute_vals = vp8_rac_get(c);

        for (i = 0; i < 4; i++)
            s->segmentation.base_quant[i] = vp8_rac_get(c) ? vp8_rac_get_sint(c, 7) : 0;

        for (i = 0; i < 4; i++)
            s->segmentation.filter_level[i] = vp8_rac_get(c) ? vp8_rac_get_sint(c, 6) : 0;
    }
    if (s->segmentation.update_map)
        for (i = 0; i < 3; i++)
            s->prob.segmentid[i] = vp8_rac_get(c) ? vp8_rac_get_uint(c, 8) : 255;
}

static void update_lf_deltas(VP8Context *s)
{
    VP56RangeCoder *c = &s->c;
    int i;

    for (i = 0; i < 4; i++)
        s->lf_delta.ref[i]  = vp8_rac_get(c) ? vp8_rac_get_sint(c, 6) : 0;

    for (i = 0; i < 4; i++)
        s->lf_delta.mode[i] = vp8_rac_get(c) ? vp8_rac_get_sint(c, 6) : 0;
}

static int setup_partitions(VP8Context *s, const uint8_t *buf, int buf_size)
{
    const uint8_t *sizes = buf;
    int i;

    s->num_partitions = 1 << vp8_rac_get_uint(&s->c, 2);

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
    int i, base_qi;

    int yac_qi     = vp8_rac_get_uint(c, 7);
    int ydc_delta  = vp8_rac_get(c) ? vp8_rac_get_sint(c, 4) : 0;
    int y2dc_delta = vp8_rac_get(c) ? vp8_rac_get_sint(c, 4) : 0;
    int y2ac_delta = vp8_rac_get(c) ? vp8_rac_get_sint(c, 4) : 0;
    int uvdc_delta = vp8_rac_get(c) ? vp8_rac_get_sint(c, 4) : 0;
    int uvac_delta = vp8_rac_get(c) ? vp8_rac_get_sint(c, 4) : 0;

    for (i = 0; i < 4; i++) {
        if (s->segmentation.enabled) {
            base_qi = s->segmentation.base_quant[i];
            if (!s->segmentation.absolute_vals)
                base_qi += yac_qi;
        } else
            base_qi = yac_qi;

        s->qmat[i].luma_qmul[0]    =       vp8_dc_qlookup[av_clip(base_qi + ydc_delta , 0, 127)];
        s->qmat[i].luma_qmul[1]    =       vp8_ac_qlookup[av_clip(base_qi             , 0, 127)];
        s->qmat[i].luma_dc_qmul[0] =   2 * vp8_dc_qlookup[av_clip(base_qi + y2dc_delta, 0, 127)];
        s->qmat[i].luma_dc_qmul[1] = 155 * vp8_ac_qlookup[av_clip(base_qi + y2ac_delta, 0, 127)] / 100;
        s->qmat[i].chroma_qmul[0]  =       vp8_dc_qlookup[av_clip(base_qi + uvdc_delta, 0, 127)];
        s->qmat[i].chroma_qmul[1]  =       vp8_ac_qlookup[av_clip(base_qi + uvac_delta, 0, 127)];

        s->qmat[i].luma_dc_qmul[1] = FFMAX(s->qmat[i].luma_dc_qmul[1], 8);
        s->qmat[i].chroma_qmul[0]  = FFMIN(s->qmat[i].chroma_qmul[0], 132);
    }
}

static void update_refs(VP8Context *s)
{
    VP56RangeCoder *c = &s->c;
    
    s->update_golden = vp8_rac_get(c);
    s->update_altref = vp8_rac_get(c);

    if (!s->update_golden) {
        s->update_golden = -vp8_rac_get_uint(c, 2); // 0: none  1: last frame  2: alt ref frame
    }

    if (!s->update_altref) {
        s->update_altref = -vp8_rac_get_uint(c, 2); // 0: none  1: last frame  2: golden frame
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

    if (invisible)
        av_log(s->avctx, AV_LOG_WARNING, "Invisible frame!\n");

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

        if (hscale || vscale)
            av_log(s->avctx, AV_LOG_WARNING, "Spatial scale by %dx%d!\n", hscale, vscale);

        if (!s->macroblocks_base || /* first frame */
            width != s->avctx->width || height != s->avctx->height)
            update_dimensions(s, width, height);

        memcpy(s->prob.token    , vp8_token_default_probs , sizeof(s->prob.token));
        memcpy(s->prob.pred16x16, vp8_pred16x16_prob_intra, sizeof(s->prob.pred16x16));
        memcpy(s->prob.pred8x8c , vp8_pred8x8c_prob_intra , sizeof(s->prob.pred8x8c));
        memset(&s->segmentation, 0, sizeof(s->segmentation));
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
        vp8_rac_get(c); // whether we can skip clamping in dsp functions
    }

    if ((s->segmentation.enabled = vp8_rac_get(c)))
        parse_segment_info(s);
    else
        s->segmentation.update_map = 0; // FIXME: move this to some init function?

    s->filter.simple    = vp8_rac_get(c);
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
        s->update_golden = s->update_altref = 1;
        s->sign_bias[VP56_FRAME_GOLDEN]               = 0;
        s->sign_bias[VP56_FRAME_GOLDEN2]              = 0;
    }

    if (!vp8_rac_get(c)) {
        av_log(s->avctx, AV_LOG_WARNING, "Reset probabilities, not yet implemented\n");
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
#define MARGIN (16 << 2)
    dst->x = av_clip(src->x, -((mb_x << 6) + MARGIN),
                     ((s->mb_width  - 1 - mb_x) << 6) + MARGIN);
    dst->y = av_clip(src->y, -((mb_y << 6) + MARGIN),
                     ((s->mb_height - 1 - mb_y) << 6) + MARGIN);
}

static void find_near_mvs(VP8Context *s, VP8Macroblock *mb, int mb_x, int mb_y,
                          VP56mv near[2], VP56mv *best, int cnt[4])
{
    VP8Macroblock *mb_edge[3] = { mb - s->mb_stride     /* top */,
                                  mb - 1                /* left */,
                                  mb - s->mb_stride - 1 /* top-left */ };
    enum { EDGE_TOP, EDGE_LEFT, EDGE_TOPLEFT };
    VP56mv near_mv[4]  = { { 0, 0 }, { 0, 0 }, { 0, 0 }, { 0, 0 } };
    enum { CNT_INTRA, CNT_NEAREST, CNT_NEAR, CNT_SPLITMV };
    int idx = CNT_INTRA, n;

    /* Process MB on top, left and top-left */
    for (n = 0; n < 3; n++) {
        VP8Macroblock *edge = mb_edge[n];
        if (edge->ref_frame != VP56_FRAME_CURRENT) {
            if (edge->mv.x || edge->mv.y) {
                VP56mv tmp = edge->mv;
                if (s->sign_bias[mb->ref_frame] != s->sign_bias[edge->ref_frame]) {
                    tmp.x *= -1;
                    tmp.y *= -1;
                }
                if (tmp.x != near_mv[idx].x || tmp.y != near_mv[idx].y)
                    near_mv[++idx] = tmp;
                cnt[idx]       += 1 + (n != 2);
            } else
                cnt[CNT_INTRA] += 1 + (n != 2);
        }
    }

    /* If we have three distinct MV's, attempt merge of first and last */
    if (cnt[CNT_SPLITMV] &&
        near_mv[1+EDGE_TOP].x == near_mv[1+EDGE_TOPLEFT].x &&
        near_mv[1+EDGE_TOP].y == near_mv[1+EDGE_TOPLEFT].y)
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

    clamp_mv(s,  best,    &near_mv[CNT_INTRA],   mb_x, mb_y);
    clamp_mv(s, &near[0], &near_mv[CNT_NEAREST], mb_x, mb_y);
    clamp_mv(s, &near[1], &near_mv[CNT_NEAR],    mb_x, mb_y);
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
    int lez = (left->x == 0 && left->y == 0);

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
        int k = vp8_mbfirstidx[part_idx][n];
        const VP56mv *left  = (k & 3) ? &mb->bmv[k - 1] : &mb[-1].bmv[k + 3],
                     *above = (k > 3) ? &mb->bmv[k - 4] : &mb[-s->mb_stride].bmv[k + 12];

        part_mode[n] = vp8_rac_get_tree(c, vp8_submv_ref_tree,
                                        get_submv_prob(left, above));
        switch (part_mode[n]) {
        case VP8_SUBMVMODE_NEW4X4:
            part_mv[n].y = base_mv->y + read_mv_component(c, s->prob.mvc[0]);
            part_mv[n].x = base_mv->x + read_mv_component(c, s->prob.mvc[1]);
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
                //intra4x4mode[k] = part_mode[n];
            }
    }
}

static void decode_mb_mode(VP8Context *s, VP8Macroblock *mb, int mb_x, int mb_y,
                           uint8_t *intra4x4)
{
    VP56RangeCoder *c = &s->c;
    int n;

    if (s->segmentation.update_map)
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
        if (vp56_rac_get_prob(c, s->prob.last))
            mb->ref_frame = vp56_rac_get_prob(c, s->prob.golden) ?
                VP56_FRAME_GOLDEN2 /* altref */ : VP56_FRAME_GOLDEN;
        else
            mb->ref_frame = VP56_FRAME_PREVIOUS;

        // motion vectors, 16.3
        find_near_mvs(s, mb, mb_x, mb_y, near, &best, cnt);
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
            mb->mv.y = best.y + read_mv_component(c, s->prob.mvc[0]);
            mb->mv.x = best.x + read_mv_component(c, s->prob.mvc[1]);
            clamp_mv(s, &mb->mv, &mb->mv, mb_x, mb_y);
            break;
        }
        if (mb->mode != VP8_MVMODE_SPLIT) {
            for (n = 0; n < 16; n++)
                mb->bmv[n] = mb->mv;
            //fill_rectangle(intra4x4, 4, 4, s->intra4x4_stride, mb->mode, 1);
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

static void intra_predict(VP8Context *s, uint8_t *dst[3], VP8Macroblock *mb,
                          uint8_t *bmode, int mb_x, int mb_y)
{
    DECLARE_ALIGNED(4, uint8_t, tr_extend)[4];
    int x, y, mode, nnz;

    if (mb->mode < MODE_I4x4) {
        mode = check_intra_pred_mode(mb->mode, mb_x, mb_y);
        s->hpc.pred16x16[mode](dst[0], s->linesize[0]);
    } else {
        // all blocks on the right edge use the top right edge of
        // the top macroblock (since the right mb isn't decoded yet)
        const uint8_t *tr_right = dst[0] - s->linesize[0] + 16;
        uint8_t *i4x4dst = dst[0];

        // extend the right edge of the top macroblock for prediction
        // could do sliced draw_edge for the same effect
        if (mb_x == s->mb_width-1) {
            AV_WN32A(tr_extend, tr_right[-1]*0x01010101);
            tr_right = tr_extend;
        }

        for (y = 0; y < 4; y++) {
            for (x = 0; x < 3; x++) {
                uint8_t *tr = i4x4dst+4*x - s->linesize[0]+4;

                s->hpc.pred4x4[vp8_pred4x4_func[bmode[x]]](i4x4dst+4*x, tr, s->linesize[0]);

                nnz = s->non_zero_count_cache[y][x];
                if (nnz) {
                    if (nnz == 1)
                        s->vp8dsp.vp8_idct_dc_add(i4x4dst+4*x, s->block[y][x], s->linesize[0]);
                    else
                        s->vp8dsp.vp8_idct_add(i4x4dst+4*x, s->block[y][x], s->linesize[0]);
                }
            }

            s->hpc.pred4x4[vp8_pred4x4_func[bmode[x]]](i4x4dst+4*x, tr_right, s->linesize[0]);

            nnz = s->non_zero_count_cache[y][x];
            if (nnz) {
                if (nnz == 1)
                    s->vp8dsp.vp8_idct_dc_add(i4x4dst+4*x, s->block[y][x], s->linesize[0]);
                else
                    s->vp8dsp.vp8_idct_add(i4x4dst+4*x, s->block[y][x], s->linesize[0]);
            }

            i4x4dst += 4*s->linesize[0];
            bmode += s->intra4x4_stride;
        }
    }

    mode = check_intra_pred_mode(mb->uvmode, mb_x, mb_y);
    s->hpc.pred8x8[mode](dst[1], s->linesize[1]);
    s->hpc.pred8x8[mode](dst[2], s->linesize[2]);
}

/**
 * Generic MC function.
 *
 * @param s VP8 decoding context
 * @param luma 1 for luma (Y) planes, 0 for chroma (Cb/Cr) planes
 * @param submv 1 for 4x4 block (sub-MV), 0 for full MB (16x16 and 8x8)
 * @param dst target buffer for block data at block position
 * @param src reference picture buffer at origin (0, 0)
 * @param mv motion vector (relative to block position) to get pixel data from
 * @param x_off horizontal position of block from origin (0, 0)
 * @param y_off vertical position of block from origin (0, 0)
 * @param block_w width of block (16, 8 or 4)
 * @param block_h height of block (always same as block_w)
 * @param width width of src/dst plane data
 * @param height height of src/dst plane data
 * @param linesize size of a single line of plane data, including padding
 */
static void vp8_mc(VP8Context *s, int luma, int submv,
                   uint8_t *dst, uint8_t *src, const VP56mv *mv,
                   int x_off, int y_off, int block_w, int block_h,
                   int width, int height, int linesize)
{
    uint8_t edge_emu_buf[21 * linesize];
    int mx = (mv->x << luma)&7;
    int my = (mv->y << luma)&7;

    x_off += mv->x >> (3 - luma);
    y_off += mv->y >> (3 - luma);

    // edge emulation
    src += y_off * linesize + x_off;
    if (x_off < 2 || x_off >= width  - block_w - 3 ||
        y_off < 2 || y_off >= height - block_h - 3) {
        ff_emulated_edge_mc(edge_emu_buf, src - 2 * linesize - 2, linesize,
                            block_w + 5, block_h + 5,
                            x_off - 2, y_off - 2, width, height);
        src = edge_emu_buf + 2 + linesize * 2;
    }

    s->vp8dsp.put_vp8_epel_pixels_tab[block_w>>3][!!my][!!mx](dst, src, linesize, mx, my);
}

/**
 * Apply motion vectors to prediction buffer, chapter 18.
 */
static void inter_predict(VP8Context *s, uint8_t *dst[3], VP8Macroblock *mb,
                          uint8_t *bmode, int mb_x, int mb_y)
{
    int x_off = mb_x << 4, y_off = mb_y << 4;
    int width = s->avctx->width, height = s->avctx->height;
    VP56mv uvmv;

    if (mb->mode < VP8_MVMODE_SPLIT) {
        /* Y */
        vp8_mc(s, 1, 0, dst[0], s->framep[mb->ref_frame]->data[0], &mb->mv,
               x_off, y_off, 16, 16, width, height, s->linesize[0]);

        /* U/V */
        uvmv = mb->mv;
        if (s->sub_version == 3) {
            uvmv.x &= ~7;
            uvmv.y &= ~7;
        }
        x_off >>= 1; y_off >>= 1; width >>= 1; height >>= 1;
        vp8_mc(s, 0, 0, dst[1], s->framep[mb->ref_frame]->data[1], &uvmv,
               x_off, y_off, 8, 8, width, height, s->linesize[1]);
        vp8_mc(s, 0, 0, dst[2], s->framep[mb->ref_frame]->data[2], &uvmv,
               x_off, y_off, 8, 8, width, height, s->linesize[2]);
    } else {
        int x, y;

        /* Y */
        for (x = 0; x < 4; x++) {
            for (y = 0; y < 4; y++) {
                vp8_mc(s, 1, 1, dst[0] + s->linesize[0] * 4 * y + x * 4,
                       s->framep[mb->ref_frame]->data[0], &mb->bmv[y * 4 + x],
                       x * 4 + x_off, y * 4 + y_off, 4, 4,
                       width, height, s->linesize[0]);
            }
        }

        /* U/V */
        x_off >>= 1; y_off >>= 1; width >>= 1; height >>= 1;
        for (x = 0; x < 2; x++) {
            for (y = 0; y < 2; y++) {
                uvmv.x = mb->bmv[ y * 2      * 4 + x * 2    ].x +
                         mb->bmv[ y * 2      * 4 + x * 2 + 1].x +
                         mb->bmv[(y * 2 + 1) * 4 + x * 2    ].x +
                         mb->bmv[(y * 2 + 1) * 4 + x * 2 + 1].x;
                uvmv.y = mb->bmv[ y * 2      * 4 + x * 2    ].y +
                         mb->bmv[ y * 2      * 4 + x * 2 + 1].y +
                         mb->bmv[(y * 2 + 1) * 4 + x * 2    ].y +
                         mb->bmv[(y * 2 + 1) * 4 + x * 2 + 1].y;
                uvmv.x = (uvmv.x + (uvmv.x < 0 ? -2 : 2)) / 4;
                uvmv.y = (uvmv.y + (uvmv.y < 0 ? -2 : 2)) / 4;
                if (s->sub_version == 3) {
                    uvmv.x &= ~7;
                    uvmv.y &= ~7;
                }
                vp8_mc(s, 0, 1, dst[1] + s->linesize[1] * 4 * y + x * 4,
                       s->framep[mb->ref_frame]->data[1], &uvmv,
                       x * 4 + x_off, y * 4 + y_off, 4, 4, 
                       width, height, s->linesize[1]);
                vp8_mc(s, 0, 1, dst[2] + s->linesize[2] * 4 * y + x * 4,
                       s->framep[mb->ref_frame]->data[2], &uvmv,
                       x * 4 + x_off, y * 4 + y_off, 4, 4, 
                       width, height, s->linesize[2]);
            }
        }
    }
}

/**
 * @param i initial coeff index, 0 unless a separate DC block is coded
 * @param zero_nhood the initial prediction context for number of surrounding
 *                   all-zero blocks (only left/top, so 0-2)
 * @param qmul[0] dc dequant factor
 * @param qmul[1] ac dequant factor
 * @return 0 if no coeffs were decoded
 *         otherwise, the index of the last coeff decoded plus one
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
        nonzero = i+1;
        offset = 0;
    }
    return nonzero;
}

static void decode_mb_coeffs(VP8Context *s, VP56RangeCoder *c, VP8Macroblock *mb,
                             uint8_t t_nnz[9], uint8_t l_nnz[9])
{
    LOCAL_ALIGNED_16(DCTELEM, dc,[16]);
    int i, x, y, luma_start = 0, luma_ctx = 3;
    int nnz_pred, nnz;
    int segment = s->segmentation.enabled ? mb->segment : 0;

    s->dsp.clear_blocks((DCTELEM *)s->block);

    if (mb->mode != MODE_I4x4 && mb->mode != VP8_MVMODE_SPLIT) {
        AV_ZERO128(dc);
        AV_ZERO128(dc+8);
        nnz_pred = t_nnz[8] + l_nnz[8];

        // decode DC values and do hadamard
        nnz = decode_block_coeffs(c, dc, s->prob.token[1], 0, nnz_pred,
                                  s->qmat[segment].luma_dc_qmul);
        l_nnz[8] = t_nnz[8] = !!nnz;
        s->vp8dsp.vp8_luma_dc_wht(s->block, dc);
        luma_start = 1;
        luma_ctx = 0;
    }

    // luma blocks
    for (y = 0; y < 4; y++)
        for (x = 0; x < 4; x++) {
            nnz_pred = l_nnz[y] + t_nnz[x];
            nnz = decode_block_coeffs(c, s->block[y][x], s->prob.token[luma_ctx], luma_start,
                                      nnz_pred, s->qmat[segment].luma_qmul);
            // nnz+luma_start may be one more than the actual last index, but we don't care
            s->non_zero_count_cache[y][x] = nnz + luma_start;
            t_nnz[x] = l_nnz[y] = !!nnz;
        }

    // chroma blocks
    // TODO: what to do about dimensions? 2nd dim for luma is x,
    // but for chroma it's (y<<1)|x
    for (i = 4; i < 6; i++)
        for (y = 0; y < 2; y++)
            for (x = 0; x < 2; x++) {
                nnz_pred = l_nnz[i+2*y] + t_nnz[i+2*x];
                nnz = decode_block_coeffs(c, s->block[i][(y<<1)+x], s->prob.token[2], 0,
                                          nnz_pred, s->qmat[segment].chroma_qmul);
                s->non_zero_count_cache[i][(y<<1)+x] = nnz;
                t_nnz[i+2*x] = l_nnz[i+2*y] = !!nnz;
            }
}

static void idct_mb(VP8Context *s, uint8_t *y_dst, uint8_t *u_dst, uint8_t *v_dst,
                    VP8Macroblock *mb)
{
    int x, y, nnz;

    if (mb->mode != MODE_I4x4)
        for (y = 0; y < 4; y++) {
            for (x = 0; x < 4; x++) {
                nnz = s->non_zero_count_cache[y][x];
                if (nnz) {
                    if (nnz == 1)
                        s->vp8dsp.vp8_idct_dc_add(y_dst+4*x, s->block[y][x], s->linesize[0]);
                    else
                        s->vp8dsp.vp8_idct_add(y_dst+4*x, s->block[y][x], s->linesize[0]);
                }
            }
            y_dst += 4*s->linesize[0];
        }

    for (y = 0; y < 2; y++) {
        for (x = 0; x < 2; x++) {
            nnz = s->non_zero_count_cache[4][(y<<1)+x];
            if (nnz) {
                if (nnz == 1)
                    s->vp8dsp.vp8_idct_dc_add(u_dst+4*x, s->block[4][(y<<1)+x], s->linesize[1]);
                else
                    s->vp8dsp.vp8_idct_add(u_dst+4*x, s->block[4][(y<<1)+x], s->linesize[1]);
            }

            nnz = s->non_zero_count_cache[5][(y<<1)+x];
            if (nnz) {
                if (nnz == 1)
                    s->vp8dsp.vp8_idct_dc_add(v_dst+4*x, s->block[5][(y<<1)+x], s->linesize[2]);
                else
                    s->vp8dsp.vp8_idct_add(v_dst+4*x, s->block[5][(y<<1)+x], s->linesize[2]);
            }
        }
        u_dst += 4*s->linesize[1];
        v_dst += 4*s->linesize[2];
    }
}

// TODO: can we calculate this less often?
static void filter_level_for_mb(VP8Context *s, VP8Macroblock *mb, int *level, int *inner, int *hev_thresh)
{
    int interior_limit, filter_level;

    if (s->segmentation.enabled) {
        filter_level = s->segmentation.filter_level[mb->segment];
        if (!s->segmentation.absolute_vals)
            filter_level += s->filter.level;
    } else
        filter_level = s->filter.level;

    if (s->lf_delta.enabled) {
        filter_level += s->lf_delta.ref[mb->ref_frame];

        if (mb->ref_frame == VP56_FRAME_CURRENT) {
            if (mb->mode == MODE_I4x4)
                filter_level += s->lf_delta.mode[0];
        } else {
            if (mb->mode == VP8_MVMODE_ZERO)
                filter_level += s->lf_delta.mode[1];
            else if (mb->mode == VP8_MVMODE_SPLIT)
                filter_level += s->lf_delta.mode[3];
            else
                filter_level += s->lf_delta.mode[2];
        }
    }
    filter_level = av_clip(filter_level, 0, 63);

    interior_limit = filter_level;
    if (s->filter.sharpness) {
        interior_limit >>= s->filter.sharpness > 4 ? 2 : 1;
        interior_limit = FFMIN(interior_limit, 9 - s->filter.sharpness);
    }
    interior_limit = FFMAX(interior_limit, 1);

    *level = filter_level;
    *inner = interior_limit;

    if (hev_thresh) {
        *hev_thresh = 0;

        if (s->keyframe) {
            if (filter_level >= 40)
                *hev_thresh = 2;
            else if (filter_level >= 15)
                *hev_thresh = 1;
        } else {
            if (filter_level >= 40)
                *hev_thresh = 3;
            else if (filter_level >= 20)
                *hev_thresh = 2;
            else if (filter_level >= 15)
                *hev_thresh = 1;
        }
    }
}

// TODO: look at backup_mb_border / xchg_mb_border in h264.c
static void filter_mb(VP8Context *s, uint8_t *dst[3], VP8Macroblock *mb, int mb_x, int mb_y)
{
    int filter_level, inner_limit, hev_thresh;

    filter_level_for_mb(s, mb, &filter_level, &inner_limit, &hev_thresh);
    if (!filter_level)
        return;

    if (mb_x) {
        s->vp8dsp.vp8_h_loop_filter16(dst[0], s->linesize[0], filter_level+2, inner_limit, hev_thresh);
        s->vp8dsp.vp8_h_loop_filter8 (dst[1], s->linesize[1], filter_level+2, inner_limit, hev_thresh);
        s->vp8dsp.vp8_h_loop_filter8 (dst[2], s->linesize[2], filter_level+2, inner_limit, hev_thresh);
    }

    if (!mb->skip || mb->mode == MODE_I4x4) {
        s->vp8dsp.vp8_h_loop_filter16_inner(dst[0]+ 4, s->linesize[0], filter_level, inner_limit, hev_thresh);
        s->vp8dsp.vp8_h_loop_filter16_inner(dst[0]+ 8, s->linesize[0], filter_level, inner_limit, hev_thresh);
        s->vp8dsp.vp8_h_loop_filter16_inner(dst[0]+12, s->linesize[0], filter_level, inner_limit, hev_thresh);
        s->vp8dsp.vp8_h_loop_filter8_inner (dst[1]+ 4, s->linesize[1], filter_level, inner_limit, hev_thresh);
        s->vp8dsp.vp8_h_loop_filter8_inner (dst[2]+ 4, s->linesize[2], filter_level, inner_limit, hev_thresh);
    }

    if (mb_y) {
        s->vp8dsp.vp8_v_loop_filter16(dst[0], s->linesize[0], filter_level+2, inner_limit, hev_thresh);
        s->vp8dsp.vp8_v_loop_filter8 (dst[1], s->linesize[1], filter_level+2, inner_limit, hev_thresh);
        s->vp8dsp.vp8_v_loop_filter8 (dst[2], s->linesize[2], filter_level+2, inner_limit, hev_thresh);
    }

    if (!mb->skip || mb->mode == MODE_I4x4) {
        s->vp8dsp.vp8_v_loop_filter16_inner(dst[0]+ 4*s->linesize[0], s->linesize[0], filter_level, inner_limit, hev_thresh);
        s->vp8dsp.vp8_v_loop_filter16_inner(dst[0]+ 8*s->linesize[0], s->linesize[0], filter_level, inner_limit, hev_thresh);
        s->vp8dsp.vp8_v_loop_filter16_inner(dst[0]+12*s->linesize[0], s->linesize[0], filter_level, inner_limit, hev_thresh);
        s->vp8dsp.vp8_v_loop_filter8_inner (dst[1]+ 4*s->linesize[1], s->linesize[1], filter_level, inner_limit, hev_thresh);
        s->vp8dsp.vp8_v_loop_filter8_inner (dst[2]+ 4*s->linesize[2], s->linesize[2], filter_level, inner_limit, hev_thresh);
    }
}

static void filter_mb_simple(VP8Context *s, uint8_t *dst, VP8Macroblock *mb, int mb_x, int mb_y)
{
    int filter_level, inner_limit, mbedge_lim, bedge_lim;

    filter_level_for_mb(s, mb, &filter_level, &inner_limit, NULL);
    if (!filter_level)
        return;

    mbedge_lim = 2*(filter_level+2) + inner_limit;
     bedge_lim = 2* filter_level    + inner_limit;

    if (mb_x)
        s->vp8dsp.vp8_h_loop_filter_simple(dst, s->linesize[0], mbedge_lim);
    if (!mb->skip || mb->mode == MODE_I4x4) {
        s->vp8dsp.vp8_h_loop_filter_simple(dst+ 4, s->linesize[0], bedge_lim);
        s->vp8dsp.vp8_h_loop_filter_simple(dst+ 8, s->linesize[0], bedge_lim);
        s->vp8dsp.vp8_h_loop_filter_simple(dst+12, s->linesize[0], bedge_lim);
    }

    if (mb_y)
        s->vp8dsp.vp8_v_loop_filter_simple(dst, s->linesize[0], mbedge_lim);
    if (!mb->skip || mb->mode == MODE_I4x4) {
        s->vp8dsp.vp8_v_loop_filter_simple(dst+ 4*s->linesize[0], s->linesize[0], bedge_lim);
        s->vp8dsp.vp8_v_loop_filter_simple(dst+ 8*s->linesize[0], s->linesize[0], bedge_lim);
        s->vp8dsp.vp8_v_loop_filter_simple(dst+12*s->linesize[0], s->linesize[0], bedge_lim);
    }
}

static void filter_mb_row(VP8Context *s, int mb_y)
{
    VP8Macroblock *mb = s->macroblocks + mb_y*s->mb_stride;
    uint8_t *dst[3] = {
        s->framep[VP56_FRAME_CURRENT]->data[0] + 16*mb_y*s->linesize[0],
        s->framep[VP56_FRAME_CURRENT]->data[1] +  8*mb_y*s->linesize[1],
        s->framep[VP56_FRAME_CURRENT]->data[2] +  8*mb_y*s->linesize[2],
    };
    int mb_x;

    for (mb_x = 0; mb_x < s->mb_width; mb_x++) {
        filter_mb(s, dst, mb++, mb_x, mb_y);
        dst[0] += 16;
        dst[1] += 8;
        dst[2] += 8;
    }
}

static void filter_mb_row_simple(VP8Context *s, int mb_y)
{
    uint8_t *dst = s->framep[VP56_FRAME_CURRENT]->data[0] + 16*mb_y*s->linesize[0];
    VP8Macroblock *mb = s->macroblocks + mb_y*s->mb_stride;
    int mb_x;

    for (mb_x = 0; mb_x < s->mb_width; mb_x++) {
        filter_mb_simple(s, dst, mb++, mb_x, mb_y);
        dst += 16;
    }
}

static int vp8_decode_frame(AVCodecContext *avctx, void *data, int *data_size,
                            AVPacket *avpkt)
{
    VP8Context *s = avctx->priv_data;
    int ret, mb_x, mb_y, i, y;

    if ((ret = decode_frame_header(s, avpkt->data, avpkt->size)) < 0)
        return ret;

    for (i = 0; i < 4; i++)
        if (!s->frames[i].data[0] ||
            (&s->frames[i] != s->framep[VP56_FRAME_PREVIOUS] &&
             &s->frames[i] != s->framep[VP56_FRAME_GOLDEN] &&
             &s->frames[i] != s->framep[VP56_FRAME_GOLDEN2])) {
            s->framep[VP56_FRAME_CURRENT] = &s->frames[i];
            break;
        }
    if (s->framep[VP56_FRAME_CURRENT]->data[0])
        avctx->release_buffer(avctx, s->framep[VP56_FRAME_CURRENT]);
    avctx->get_buffer(avctx, s->framep[VP56_FRAME_CURRENT]);

    memset(s->top_nnz, 0, s->mb_width*sizeof(*s->top_nnz));

    for (i = 0; i < 3; i++) {
        s->linesize[i] = s->framep[VP56_FRAME_CURRENT]->linesize[i];

        // top edge of 127 for intra prediction
        memset(s->framep[VP56_FRAME_CURRENT]->data[i] - s->linesize[i]-1, 127, s->linesize[i]+1);
    }

    for (mb_y = 0; mb_y < s->mb_height; mb_y++) {
        VP56RangeCoder *c = &s->partition[mb_y%s->num_partitions].c;
        VP8Macroblock *mb = s->macroblocks + mb_y*s->mb_stride;
        uint8_t *intra4x4 = s->intra4x4_pred_mode + 4*mb_y*s->intra4x4_stride;
        uint8_t *dst[3];

        memset(s->left_nnz, 0, sizeof(s->left_nnz));

        dst[0] = s->framep[VP56_FRAME_CURRENT]->data[0] + 16*mb_y*s->linesize[0];
        dst[1] = s->framep[VP56_FRAME_CURRENT]->data[1] +  8*mb_y*s->linesize[1];
        dst[2] = s->framep[VP56_FRAME_CURRENT]->data[2] +  8*mb_y*s->linesize[2];

        // left edge of 129 for intra prediction
        for (i = 0; i < 3; i++)
            for (y = 0; y < 16>>!!i; y++)
                dst[i][y*s->linesize[i]-1] = 129;

        for (mb_x = 0; mb_x < s->mb_width; mb_x++) {
            decode_mb_mode(s, mb, mb_x, mb_y, intra4x4 + 4*mb_x);

            if (!mb->skip)
                decode_mb_coeffs(s, c, mb, s->top_nnz[mb_x], s->left_nnz);
            else {
                AV_ZERO128(s->non_zero_count_cache);    // luma
                AV_ZERO64(s->non_zero_count_cache[4]);  // chroma
            }

            if (mb->mode <= MODE_I4x4) {
                intra_predict(s, dst, mb, intra4x4 + 4*mb_x, mb_x, mb_y);
            } else {
                inter_predict(s, dst, mb, intra4x4 + 4*mb_x, mb_x, mb_y);
            }

            if (!mb->skip) {
                idct_mb(s, dst[0], dst[1], dst[2], mb);
            } else {
                AV_ZERO64(s->left_nnz);
                AV_WN64(s->top_nnz[mb_x], 0);   // array of 9, so unaligned

                // Reset DC block if it would exist if the mb wasn't skipped
                if (mb->mode != MODE_I4x4 && mb->mode != VP8_MVMODE_SPLIT) {
                    s->left_nnz[8]      = 0;
                    s->top_nnz[mb_x][8] = 0;
                }
            }

            dst[0] += 16;
            dst[1] += 8;
            dst[2] += 8;
            mb++;
        }
        if (mb_y && s->filter.level) {
            if (s->filter.simple)
                filter_mb_row_simple(s, mb_y-1);
            else
                filter_mb_row(s, mb_y-1);
        }
    }
    if (s->filter.level) {
        if (s->filter.simple)
            filter_mb_row_simple(s, mb_y-1);
        else
            filter_mb_row(s, mb_y-1);
    }

    // init the intra pred probabilities for inter frames
    // this seems like it'll be a bit tricky for frame-base multithreading
    if (s->keyframe) {
        memcpy(s->prob.pred16x16, vp8_pred16x16_prob_inter, sizeof(s->prob.pred16x16));
        memcpy(s->prob.pred8x8c , vp8_pred8x8c_prob_inter , sizeof(s->prob.pred8x8c));
    }

    if (s->update_golden == -2 && s->update_altref == -2) {
        FFSWAP(AVFrame *, s->framep[VP56_FRAME_GOLDEN],
                          s->framep[VP56_FRAME_GOLDEN2]);
    } else {
        if (s->update_golden) {
            if (s->framep[VP56_FRAME_GOLDEN]->data[0] &&
                s->framep[VP56_FRAME_GOLDEN] != s->framep[VP56_FRAME_GOLDEN2] &&
                s->framep[VP56_FRAME_GOLDEN] != s->framep[VP56_FRAME_PREVIOUS])
                avctx->release_buffer(avctx, s->framep[VP56_FRAME_GOLDEN]);
            s->framep[VP56_FRAME_GOLDEN] =
                s->framep[s->update_golden ==  1 ? VP56_FRAME_CURRENT :
                          s->update_golden == -1 ? VP56_FRAME_PREVIOUS :
                                                   VP56_FRAME_GOLDEN2];
        }
        if (s->update_altref) {
            if (s->framep[VP56_FRAME_GOLDEN2]->data[0] &&
                s->framep[VP56_FRAME_GOLDEN2] != s->framep[VP56_FRAME_GOLDEN] &&
                s->framep[VP56_FRAME_GOLDEN2] != s->framep[VP56_FRAME_PREVIOUS])
                avctx->release_buffer(avctx, s->framep[VP56_FRAME_GOLDEN2]);
            s->framep[VP56_FRAME_GOLDEN2] = 
                s->framep[s->update_altref ==  1 ? VP56_FRAME_CURRENT :
                          s->update_altref == -1 ? VP56_FRAME_PREVIOUS :
                                                   VP56_FRAME_GOLDEN];
        }
    }
    if (s->referenced) { // move cur->prev
        if (s->framep[VP56_FRAME_PREVIOUS]->data[0] &&
            s->framep[VP56_FRAME_GOLDEN] != s->framep[VP56_FRAME_PREVIOUS] &&
            s->framep[VP56_FRAME_GOLDEN2] != s->framep[VP56_FRAME_PREVIOUS])
            avctx->release_buffer(avctx, s->framep[VP56_FRAME_PREVIOUS]);
        s->framep[VP56_FRAME_PREVIOUS] = s->framep[VP56_FRAME_CURRENT];
    }

    *(AVFrame*)data = *s->framep[VP56_FRAME_CURRENT];
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
    ff_vp8dsp_init(&s->vp8dsp);

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

    if (s->frames[VP56_FRAME_GOLDEN].data[0])
        avctx->release_buffer(avctx, &s->frames[VP56_FRAME_GOLDEN]);
    if (s->frames[VP56_FRAME_GOLDEN2].data[0])
        avctx->release_buffer(avctx, &s->frames[VP56_FRAME_GOLDEN2]);
    if (s->frames[VP56_FRAME_PREVIOUS].data[0])
        avctx->release_buffer(avctx, &s->frames[VP56_FRAME_PREVIOUS]);
    if (s->frames[VP56_FRAME_CURRENT].data[0])
        avctx->release_buffer(avctx, &s->frames[VP56_FRAME_CURRENT]);

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
