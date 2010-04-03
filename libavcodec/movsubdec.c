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
#include "bytestream.h"

static int decode(AVCodecContext *avctx, void *data, int *data_size,
                  AVPacket *avpkt)
{
    const uint8_t *buf = avpkt->data;
    int buf_size;

    AVSubtitle *sub = data;
    memset(sub, 0, sizeof(*sub));

    if (avpkt->size < 2) {
        *data_size = 0;
        return avpkt->size;
    }
    buf_size = FFMIN(avpkt->size-2, AV_RB16(buf));

    // we don't parse any styles/other atoms
    sub->num_rects = 1;
    sub->rects = av_mallocz(sizeof(*sub->rects));
    sub->rects[0] = av_mallocz(sizeof(AVSubtitleRect));
    sub->rects[0]->text = av_mallocz(buf_size+1);

    memcpy(sub->rects[0]->text, buf+2, buf_size);
    sub->rects[0]->text[buf_size] = 0;
    sub->rects[0]->type = SUBTITLE_TEXT;

    *data_size = 1;
    return avpkt->size;
}

AVCodec mov_text_decoder = {
    "ttxt",
    AVMEDIA_TYPE_SUBTITLE,
    CODEC_ID_MOV_TEXT,
    .decode = decode,
    .long_name = NULL_IF_CONFIG_SMALL("MOV/MP4 Timed Text Subtitles"),
};
