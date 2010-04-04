/*
 * Copyright (c) 2010 David Conrad
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

static int ass_get_duration(const uint8_t *p)
{
    int sh, sm, ss, sc, eh, em, es, ec;
    uint64_t start, end;

    if (sscanf(p, "%*[^,],%d:%d:%d%*c%d,%d:%d:%d%*c%d",
               &sh, &sm, &ss, &sc, &eh, &em, &es, &ec) != 8)
        return 0;
    start = 3600000*sh + 60000*sm + 1000*ss + 10*sc;
    end   = 3600000*eh + 60000*em + 1000*es + 10*ec;
    return end - start;
}

static int decode(AVCodecContext *avctx, void *data, int *data_size,
                  AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size = avpkt->size;
    int i, level, field;
    uint8_t *dst;

    AVSubtitle *sub = data;
    memset(sub, 0, sizeof(*sub));

    sub->end_display_time = ass_get_duration(avpkt->data);
    sub->num_rects = 1;
    sub->rects = av_mallocz(sizeof(*sub->rects));
    sub->rects[0] = av_mallocz(sizeof(AVSubtitleRect));
    sub->rects[0]->text = av_mallocz(buf_size);
    sub->rects[0]->ass = av_mallocz(buf_size);

    memcpy(sub->rects[0]->ass, buf, buf_size);
    sub->rects[0]->type = SUBTITLE_ASS;

    // strip out all formatting for the text approximation
    level = field = 0;
    dst = sub->rects[0]->text;
    for (i = 0; i < buf_size; i++) {
        if (field >= 9) {
            level += (buf[i] == '{');
            level -= (buf[i] == '}');
            level = FFMAX(level, 0);
            if (!level)
                *dst++ = buf[i];
        }
        field += (buf[i] == ',');
    }
    *dst = 0;

    *data_size = 1;
    return buf_size;
}

AVCodec ssa_decoder = {
    "ssa",
    AVMEDIA_TYPE_SUBTITLE,
    CODEC_ID_SSA,
    .decode = decode,
    .long_name = NULL_IF_CONFIG_SMALL("SSA/ASS Subtitles"),
};
