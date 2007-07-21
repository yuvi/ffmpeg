/* -*-  indent-tabs-mode:nil; c-basic-offset:4;  -*- */
/*
 * Copyright (C) 2007 Marco Gerards <marco@gnu.org>
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

#include <stdio.h>

static int byte = 0;
static int cnt = 0;

static int getbit(void)
{
    int ret = (byte & 128)? 1:0;
    byte <<= 1;
    cnt--;
    if (cnt < 0)
        ret = 1;

    return ret;
}

static int dirac_golomb_val(i) {
    unsigned int val = 0;

    byte = i;
    cnt = 8;

    while (! getbit()) {
        if (getbit())
            val++;
        val <<= 1;
    }
    val >>= 1;

    printf("%d, ", val);
    return val;
}

static int dirac_golomb_bits(i) {
    unsigned int val = 0;
    int pos = 0;

    byte = i;
    cnt = 8;

    while (! getbit()) {
        if (getbit())
            val++;
        val <<= 1;
        pos += 2;
    }
    val >>= 1;
    pos++;

    printf("%d,", pos);
    return val;
}


int main (void) {
    unsigned int i;

    for (i = 0; i <= 255; i++) {
        dirac_golomb_val(i);
        if (i % 16 == 15)
            printf("\n");
    }

    printf ("\n\n\n");

    for (i = 0; i <= 255; i++) {
        dirac_golomb_bits(i);
        if (i % 16 == 15)
            printf("\n");
    }

    return 0;
}
