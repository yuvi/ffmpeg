/*
 * frame MD5 encoder (for codec/format testing)
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

#include "libavutil/md5.h"
#include "avformat.h"

static int framemd5_write_packet(struct AVFormatContext *s, AVPacket *pkt)
{
    char buf[256], md5string[32+2];
    uint8_t md5sum[16];
    int i;

    av_md5_sum(md5sum, pkt->data, pkt->size);

    snprintf(buf, sizeof(buf), "%d, %"PRId64", %d, ", pkt->stream_index, pkt->dts, pkt->size);
    for (i = 0; i < 16; i++)
        snprintf(md5string+2*i, 4, "%02x\n", md5sum[i]);

    put_buffer(s->pb, buf, strlen(buf));
    put_buffer(s->pb, md5string, 33);
    put_flush_packet(s->pb);
    return 0;
}

AVOutputFormat framemd5_muxer = {
    .name = "framemd5",
    .video_codec = CODEC_ID_RAWVIDEO,
    .audio_codec = CODEC_ID_PCM_S16LE,
    .write_packet = framemd5_write_packet,
    .long_name = NULL_IF_CONFIG_SMALL("framemd5 testing format"),
};
