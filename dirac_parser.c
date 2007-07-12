/* -*-  indent-tabs-mode:nil; c-basic-offset:4;  -*- */
/*
 * Dirac parser
 *
 * Copyright (c) 2007 Marco Gerards <marco@gnu.org>
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

/**
 * @file dirac_parser.c
 * Dirac Parser
 * @author Marco Gerards <marco@gnu.org>
 */

#include "parser.h"

#define DEBUG 1

#define DIRAC_PARSE_INFO_PREFIX 0x42424344

/**
 * finds the end of the current frame in the bitstream.
 * @return the position of the first byte of the next frame, or -1
 */
static int find_frame_end(ParseContext *pc, const uint8_t *buf, int buf_size)
{
    uint32_t state;
    int parse_info_found;
    int i = 0;

    parse_info_found = pc->frame_start_found;
    state = pc->state;

    if (! parse_info_found) {
        for(i = 0; i < buf_size; i++) {
            state = (state << 8) | buf[i];
            if (state == DIRAC_PARSE_INFO_PREFIX) {
                parse_info_found = 1;
                break;
            }
        }
    }

    if (parse_info_found) {
        for(; i < buf_size; i++) {
            state = (state << 8) | buf[i];
            if (state == DIRAC_PARSE_INFO_PREFIX) {
                pc->frame_start_found = 0;
                pc->state = -1;
                return i - 3;
            }
        }
    }

    pc->frame_start_found = parse_info_found;
    pc->state = state;

    return END_NOT_FOUND;
}

static int dirac_parse(AVCodecParserContext *s, AVCodecContext *avctx,
                       const uint8_t **poutbuf, int *poutbuf_size,
                       const uint8_t *buf, int buf_size)
{
    ParseContext *pc = s->priv_data;
    int next;

    if(s->flags & PARSER_FLAG_COMPLETE_FRAMES){
        next = buf_size;
    }else{
        next = find_frame_end(pc, buf, buf_size);

        if (ff_combine_frame(pc, next, &buf, &buf_size) < 0) {
            *poutbuf = NULL;
            *poutbuf_size = 0;
            return buf_size;
        }
    }

    *poutbuf = buf;
    *poutbuf_size = buf_size;
    return next;
}

AVCodecParser dirac_parser = {
    { CODEC_ID_DIRAC },
    sizeof(ParseContext),
    NULL,
    dirac_parse,
    ff_parse_close,
};
