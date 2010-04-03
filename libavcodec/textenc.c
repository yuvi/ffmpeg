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

static int encode(AVCodecContext *avctx, uint8_t *buf, int buf_size,
                  void *data)
{
    AVSubtitle *sub = data;
    int i, len, bufpos = 0;;

    if (sub->num_rects == 0 || sub->rects == NULL)
        return -1;

    for (i = 0; i < sub->num_rects; i++) {
        if (!sub->rects[i]->text)
            continue;

        len = FFMIN(strlen(sub->rects[i]->text), buf_size-bufpos-1);
        memcpy(buf+bufpos, sub->rects[i]->text, len);

        // separate different rects by a newline
        buf[bufpos+len] = '\n';
        bufpos += len+1;

        if (bufpos >= buf_size)
            break;
    }

    return bufpos-1;
}

AVCodec text_encoder = {
    "text",
    AVMEDIA_TYPE_SUBTITLE,
    CODEC_ID_TEXT,
    .encode = encode,
    .long_name = NULL_IF_CONFIG_SMALL("Text Subtitles"),
};
