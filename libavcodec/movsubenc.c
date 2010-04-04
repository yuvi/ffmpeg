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

static int init(AVCodecContext *avctx)
{
    uint8_t *buf;
    avctx->extradata_size = 48;
    avctx->extradata = buf = av_mallocz(avctx->extradata_size +
                                    FF_INPUT_BUFFER_PADDING_SIZE);

    bytestream_put_be32(&buf, 0);   // displayFlags
    *buf++ = 0;                     // horizontal-justification
    *buf++ = 0;                     // vertical-justification
    bytestream_put_be32(&buf, 0);   // background-color-rgba

    // default BoxRecord
    bytestream_put_be16(&buf, 0);   // top
    bytestream_put_be16(&buf, 0);   // left
    bytestream_put_be16(&buf, 0);   // bottom
    bytestream_put_be16(&buf, 0);   // right

    // default StyleRecord
    bytestream_put_be16(&buf, 0);   // startChar
    bytestream_put_be16(&buf, 0);   // endChar
    bytestream_put_be16(&buf, 1);   // font ID
    *buf++ = 0;                     // facy style flags
    *buf++ = 12;                    // font size
    *buf++ = 0xff;                  // text color red
    *buf++ = 0xff;                  // text color green
    *buf++ = 0xff;                  // text color blue
    *buf++ = 0xff;                  // text color alpha

    // font table
    bytestream_put_be32(&buf, 18);  // size
    bytestream_put_buffer(&buf, "ftab", 4);
    bytestream_put_be16(&buf, 1);   // entry count
    // FontRecord
    bytestream_put_be16(&buf, 1);   // font ID
    *buf++ = 5;
    bytestream_put_buffer(&buf, "Arial", 5);

    avctx->time_base = (AVRational){1,1000};

    // quicktime scales the text such that there's two lines in the visible
    // height. One line taking up 1/12 of the height seems a reasonable default
    avctx->height /= 6;
    return 0;
}

static int encode(AVCodecContext *avctx, uint8_t *buf, int buf_size,
                  void *data)
{
    AVSubtitle *sub = data;
    int i, len, bufpos = 2;

    if (sub->num_rects == 0 || sub->rects == NULL)
        return -1;

    // TODO: we can do something useful with multiple rects
    for (i = 0; i < sub->num_rects; i++) {
        uint8_t *text = sub->rects[i]->text;
        if (!text)
            continue;

        len = FFMIN(strlen(text), buf_size-bufpos-1);

        // strip leading whitespace (so we can write nothing if there's nothing else)
        // TODO: crate these null packets in movenc.c and discard these at textdec.c
        while (len && (*text == '\n' || *text == ' ')) {
            buf++;
            len--;
        }

        memcpy(buf+bufpos, text, len);

        // separate different rects by a newline
        buf[bufpos+len] = '\n';
        bufpos += len+1;

        if (bufpos >= buf_size)
            break;
    }

    bufpos = FFMIN(bufpos-1, 0xffff);
    AV_WB16(buf, bufpos-2);

    return bufpos;
}

AVCodec mov_text_encoder = {
    "ttxt",
    AVMEDIA_TYPE_SUBTITLE,
    CODEC_ID_MOV_TEXT,
    .init = init,
    .encode = encode,
    .long_name = NULL_IF_CONFIG_SMALL("MOV/MP4 Timed Text Subtitles"),
};
