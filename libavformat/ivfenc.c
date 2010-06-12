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

#include "avformat.h"
#include "riff.h"
#include "libavutil/intreadwrite.h"

struct ivf_ctx {
    int64_t nframes;
    int64_t nframes_offset;
};

static int write_header(AVFormatContext *s)
{
    struct ivf_ctx *ivf = s->priv_data;
    AVStream *st = s->streams[0];

    if (s->nb_streams > 1 || st->codec->codec_type != AVMEDIA_TYPE_VIDEO) {
        av_log(s, AV_LOG_ERROR, "Only one video stream supported!\n");
        return AVERROR_INVALIDDATA;
    }

    put_tag(s->pb, "DKIF");
    put_le16(s->pb, 0); // version
    put_le16(s->pb, 32); // header size

    put_le32(s->pb, st->codec->codec_tag);
    put_le16(s->pb, st->codec->width);
    put_le16(s->pb, st->codec->height);
    put_le32(s->pb, st->time_base.den);
    put_le32(s->pb, st->time_base.num);
    ivf->nframes_offset = url_ftell(s->pb);
    put_le64(s->pb, 0);

    return 0;
}

static int write_packet(AVFormatContext *s, AVPacket *pkt)
{
    struct ivf_ctx *ivf = s->priv_data;
    ivf->nframes++;

    put_le32(s->pb, pkt->size);
    put_le64(s->pb, pkt->dts);
    put_buffer(s->pb, pkt->data, pkt->size);

    return 0;
}

static int write_trailer(AVFormatContext *s)
{
    struct ivf_ctx *ivf = s->priv_data;

    if (!url_is_streamed(s->pb)) {
        url_fseek(s->pb, ivf->nframes_offset, SEEK_SET);
        put_le64(s->pb, ivf->nframes);
    }

    return 0;
}

AVOutputFormat ivf_muxer = {
    "ivf",
    NULL_IF_CONFIG_SMALL("On2 IVF"),
    NULL,
    NULL,
    sizeof(int64_t),
    CODEC_ID_VP8,
    CODEC_ID_NONE,
    write_header,
    write_packet,
    write_trailer,
    .flags= AVFMT_VARIABLE_FPS,
    .codec_tag = (const AVCodecTag*[]){ff_codec_bmp_tags, 0},
};
