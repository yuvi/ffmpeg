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

typedef struct {
    unsigned int subnum;
} SubRipContext;

static int write_header(AVFormatContext *s)
{
    av_set_pts_info(s->streams[0], 64, 1, 1000);
    return 0;
}

static int write_packet(AVFormatContext *s, AVPacket *pkt)
{
    SubRipContext *srt = s->priv_data;
    char buf[256];
    int len, i;

    int h,m,sec,ms, eh,em,esec,ems;
    int64_t pts = pkt->pts;
    int64_t end = pkt->pts + pkt->duration;

    if (end >= 100*60*60*1000) {
        av_log(s, AV_LOG_ERROR, "SRT can't handle 100 or more hours\n");
        return -1;
    }

    h  = pts / (60*60*1000);
    eh = end / (60*60*1000);
    pts -= h * (60*60*1000);
    end -= eh* (60*60*1000);

    m  = pts / (60*1000);
    em = end / (60*1000);
    pts -= m * (60*1000);
    end -= em* (60*1000);

    sec  = pts / 1000;
    esec = end / 1000;
    pts -= sec * 1000;
    end -= esec* 1000;

    ms  = pts;
    ems = end;

    len = snprintf(buf, sizeof(buf), "%d\n%02d:%02d:%02d,%03d --> %02d:%02d:%02d,%03d\n",
        ++srt->subnum, h,m,sec,ms, eh,em,esec,ems);

    put_buffer(s->pb, buf, len);
    put_buffer(s->pb, pkt->data, pkt->size);
    put_buffer(s->pb, "\n\n", 2);
    put_flush_packet(s->pb);

    return 0;
}

AVOutputFormat srt_muxer = {
    "subrip",
    NULL_IF_CONFIG_SMALL("SubRip format"),
    .extensions = "srt",
    .priv_data_size = sizeof(SubRipContext),
    .write_packet = write_packet,
    .subtitle_codec = CODEC_ID_TEXT,
};
