/*
 * MMX optimized discrete wavelet transform
 * Copyright (c) 2002-2004 Michael Niedermayer <michaelni@gmx.at>
 * Copyright (c) 2009-2010 David Conrad
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

#include "dsputil_mmx.h"
#include "dwt.h"

#define COMPOSE_VERTICAL(ext, align) \
void ff_vertical_compose53iL0##ext(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2, int width); \
void ff_vertical_compose_dirac53iH0##ext(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2, int width); \
void ff_vertical_compose_dd137iL0##ext(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2, IDWTELEM *b3, IDWTELEM *b4, int width); \
void ff_vertical_compose_dd97iH0##ext(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2, IDWTELEM *b3, IDWTELEM *b4, int width); \
void ff_vertical_compose_haariL0##ext(IDWTELEM *b0, IDWTELEM *b1, int width); \
void ff_vertical_compose_haariH0##ext(IDWTELEM *b0, IDWTELEM *b1, int width); \
\
static void vertical_compose53iL0##ext(IDWTELEM *b0, IDWTELEM *b1, \
                                             IDWTELEM *b2, int width) \
{ \
    int i, width_align = width&~(align-1); \
\
    for(i=width_align; i<width; i++) \
        b1[i] = COMPOSE_53iL0(b0[i], b1[i], b2[i]); \
\
    ff_vertical_compose53iL0##ext(b0, b1, b2, width_align); \
} \
\
static void vertical_compose_dirac53iH0##ext(IDWTELEM *b0, IDWTELEM *b1, \
                                             IDWTELEM *b2, int width) \
{ \
    int i, width_align = width&~(align-1); \
\
    for(i=width_align; i<width; i++) \
        b1[i] = COMPOSE_DIRAC53iH0(b0[i], b1[i], b2[i]); \
\
    ff_vertical_compose_dirac53iH0##ext(b0, b1, b2, width_align); \
} \
\
static void vertical_compose_dd137iL0##ext(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2, \
                                           IDWTELEM *b3, IDWTELEM *b4, int width) \
{ \
    int i, width_align = width&~(align-1); \
\
    for(i=width_align; i<width; i++) \
        b2[i] = COMPOSE_DD137iL0(b0[i], b1[i], b2[i], b3[i], b4[i]); \
\
    ff_vertical_compose_dd137iL0##ext(b0, b1, b2, b3, b4, width_align); \
} \
\
static void vertical_compose_dd97iH0##ext(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2, \
                                          IDWTELEM *b3, IDWTELEM *b4, int width) \
{ \
    int i, width_align = width&~(align-1); \
\
    for(i=width_align; i<width; i++) \
        b2[i] = COMPOSE_DD97iH0(b0[i], b1[i], b2[i], b3[i], b4[i]); \
\
    ff_vertical_compose_dd97iH0##ext(b0, b1, b2, b3, b4, width_align); \
} \
static void vertical_compose_haariL0##ext(IDWTELEM *b0, IDWTELEM *b1, int width) \
{ \
    int i, width_align = width&~(align-1); \
\
    for(i=width_align; i<width; i++) \
        b0[i] = COMPOSE_HAARiL0(b0[i], b1[i]); \
\
    ff_vertical_compose_haariL0##ext(b0, b1, width_align); \
} \
\
static void vertical_compose_haariH0##ext(IDWTELEM *b0, IDWTELEM *b1, int width) \
{ \
    int i, width_align = width&~(align-1); \
\
    for(i=width_align; i<width; i++) \
        b0[i] = COMPOSE_HAARiH0(b0[i], b1[i]); \
\
    ff_vertical_compose_haariH0##ext(b0, b1, width_align); \
} \

#if HAVE_YASM
COMPOSE_VERTICAL(_mmx, 4)
COMPOSE_VERTICAL(_sse2, 8)
#endif

void ff_spatial_idwt_init_mmx(DWTContext *d, enum dwt_type type)
{
#if HAVE_YASM
    // fixme: call dsputil_init
    mm_flags = mm_support();

    if (!(mm_flags & FF_MM_MMX))
        return;

    switch (type) {
    case DWT_DIRAC_DD9_7:
        d->vertical_compose_l0 = vertical_compose53iL0_mmx;
        d->vertical_compose_h0 = vertical_compose_dd97iH0_mmx;
        break;
    case DWT_DIRAC_LEGALL5_3:
        d->vertical_compose_l0 = vertical_compose53iL0_mmx;
        d->vertical_compose_h0 = vertical_compose_dirac53iH0_mmx;
        break;
    case DWT_DIRAC_DD13_7:
        d->vertical_compose_l0 = vertical_compose_dd137iL0_mmx;
        d->vertical_compose_h0 = vertical_compose_dd97iH0_mmx;
        break;
    case DWT_DIRAC_HAAR0:
    case DWT_DIRAC_HAAR1:
        d->vertical_compose_l0 = vertical_compose_haariL0_mmx;
        d->vertical_compose_h0 = vertical_compose_haariH0_mmx;
        break;
    }

    if (!(mm_flags & FF_MM_SSE2))
        return;

    switch (type) {
    case DWT_DIRAC_DD9_7:
        d->vertical_compose_l0 = vertical_compose53iL0_sse2;
        d->vertical_compose_h0 = vertical_compose_dd97iH0_sse2;
        break;
    case DWT_DIRAC_LEGALL5_3:
        d->vertical_compose_l0 = vertical_compose53iL0_sse2;
        d->vertical_compose_h0 = vertical_compose_dirac53iH0_sse2;
        break;
    case DWT_DIRAC_DD13_7:
        d->vertical_compose_l0 = vertical_compose_dd137iL0_sse2;
        d->vertical_compose_h0 = vertical_compose_dd97iH0_sse2;
        break;
    case DWT_DIRAC_HAAR0:
    case DWT_DIRAC_HAAR1:
        d->vertical_compose_l0 = vertical_compose_haariL0_sse2;
        d->vertical_compose_h0 = vertical_compose_haariH0_sse2;
        break;
    }
#endif // HAVE_YASM
}
