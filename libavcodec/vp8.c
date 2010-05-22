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

typedef struct {
    AVCodecContext *avctx;
    DSPContext dsp;
    AVFrame frames[4];
    AVFrame *framep[6];
    uint8_t *edge_emu_buffer_alloc;
    uint8_t *edge_emu_buffer;
    VP56RangeCoder c;
    int sub_version;

    int keyframe;
    int referenced; ///< update last frame with the current one

    int num_partitions;
    struct {
        VP56RangeCoder c;
    } partition[8];

#define MAX_NUM_SEGMENTS 4
    struct {
        int enabled;
        int mode;
        int8_t data[2][MAX_NUM_SEGMENTS];
        uint8_t prob[3];
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
        int yac_qi;
        int ydc_delta;
        int y2dc_delta;
        int y2ac_delta;
        int uvdc_delta;
        int uvac_delta;
    } dequant;

    int golden_sign_bias;
    int altref_sign_bias;

    int mbskip_enabled;

    struct {
        uint8_t mbskip;
        uint8_t intra;
        uint8_t last;
        uint8_t golden;
        uint8_t intra_pred[2][4]; // [plane][mode_tree]
        uint8_t coeff[4][8][3][NUM_DCT_TOKENS-1];
    } prob;
} VP8Context;

#define RL24(p) (AV_RL16(p) + ((p)[2] << 16))

static void parse_segment_info(VP8Context *s)
{
    VP56RangeCoder *c = &s->c;
    int i, opt, segment;
    int update_map = vp8_rac_get(c);

    av_log(s->avctx, AV_LOG_INFO, "segmented\n");

    if (vp8_rac_get(c)) { // update segment feature data
        s->segments.mode = vp8_rac_get(c);
        memset(s->segments.data, 0, sizeof(s->segments.data));

        for (opt = 0; opt < 2; opt++)
            for (segment = 0; segment < MAX_NUM_SEGMENTS; segment++)
                if (vp8_rac_get(c))
                    s->segments.data[opt][segment] = vp8_rac_get_sint2(c, 6+opt);
    }
    if (update_map)
        for (i = 0; i < 3; i++)
            s->segments.prob[i] = vp8_rac_get(c) ? vp8_rac_get_uint(c, 8) : 255;
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

    s->dequant.yac_qi     = vp8_rac_get_uint(c, 7);
    s->dequant.ydc_delta  = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 4) : 0;
    s->dequant.y2dc_delta = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 4) : 0;
    s->dequant.y2ac_delta = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 4) : 0;
    s->dequant.uvdc_delta = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 4) : 0;
    s->dequant.uvac_delta = vp8_rac_get(c) ? vp8_rac_get_sint2(c, 4) : 0;
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

        if (/*!s->macroblocks ||*/ /* first frame */
            width != s->avctx->width || height != s->avctx->height)
            avcodec_set_dimensions(s->avctx, width, height);

        memcpy(s->prob.coeff, vp8_default_coeff_probs, sizeof(s->prob.coeff));
    }

    if (header_size > buf_size) {
        av_log(s->avctx, AV_LOG_ERROR, "Header size larger than data\n");
        return AVERROR_INVALIDDATA;
    }
    vp56_init_range_decoder(c, buf, header_size);
    buf      += header_size;
    buf_size -= header_size;

    if (s->keyframe) {
        int colorspec = vp8_rac_get(c);
        int clamp     = vp8_rac_get(c);
    }

    if ((s->segments.enabled = vp8_rac_get(c)))
        parse_segment_info(s);

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
                    if (vp56_rac_get_prob(c, vp8_coeff_update_probs[i][j][k][l]))
                        s->prob.coeff[i][j][k][l] = vp8_rac_get_uint(c, 8);

    if ((s->mbskip_enabled = vp8_rac_get(c)))
        s->prob.mbskip = vp8_rac_get_uint(c, 8);

    if (!s->keyframe) {
        s->prob.intra  = vp8_rac_get_uint(c, 8);
        s->prob.last   = vp8_rac_get_uint(c, 8);
        s->prob.golden = vp8_rac_get_uint(c, 8);

        if (vp8_rac_get(c))
            for (i = 0; i < 4; i++)
                s->prob.intra_pred[0][i] = vp8_rac_get_uint(c, 8);
        if (vp8_rac_get(c))
            for (i = 0; i < 3; i++)
                s->prob.intra_pred[1][i] = vp8_rac_get_uint(c, 8);

        // 17.2 MV probability update
    }

    return 0;
}

static int vp8_decode_frame(AVCodecContext *avctx, void *data, int *data_size,
                            AVPacket *avpkt)
{
    VP8Context *s = avctx->priv_data;
    int ret;

    if ((ret = decode_frame_header(s, avpkt->data, avpkt->size)) < 0)
        return ret;

    return 0;
}

static av_cold int vp8_decode_init(AVCodecContext *avctx)
{
    VP8Context *s = avctx->priv_data;
    int i;

    s->avctx = avctx;
    avctx->pix_fmt = PIX_FMT_YUV420P;

    dsputil_init(&s->dsp, avctx);

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
