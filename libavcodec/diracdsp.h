/*
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

#ifndef AVCODEC_DIRACDSP_H
#define AVCODEC_DIRACDSP_H

void ff_diracdsp_init(DSPContext *c, AVCodecContext *avctx);

#define DECL_DIRAC_PIXOP(PFX, EXT) \
    void ff_ ## PFX ## _dirac_pixels8_ ## EXT(uint8_t *dst, uint8_t *src[5], int stride, int h); \
    void ff_ ## PFX ## _dirac_pixels16_ ## EXT(uint8_t *dst, uint8_t *src[5], int stride, int h); \
    void ff_ ## PFX ## _dirac_pixels32_ ## EXT(uint8_t *dst, uint8_t *src[5], int stride, int h)

DECL_DIRAC_PIXOP(put, c);
DECL_DIRAC_PIXOP(avg, c);
DECL_DIRAC_PIXOP(put, l2_c);
DECL_DIRAC_PIXOP(avg, l2_c);
DECL_DIRAC_PIXOP(put, l4_c);
DECL_DIRAC_PIXOP(avg, l4_c);

#endif
