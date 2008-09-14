/*
 * Copyright (C) 2004 the ffmpeg project
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
 * @file libavcodec/x86/vp3dsp_sse2.c
 * SSE2-optimized functions cribbed from the original VP3 source code.
 */

#include "libavcodec/dsputil.h"
#include "dsputil_mmx.h"
#include "vp3dsp_sse2.h"

DECLARE_ALIGNED(16, const uint16_t, ff_vp3_idct_data)[7 * 8] =
{
    64277,64277,64277,64277,64277,64277,64277,64277,
    60547,60547,60547,60547,60547,60547,60547,60547,
    54491,54491,54491,54491,54491,54491,54491,54491,
    46341,46341,46341,46341,46341,46341,46341,46341,
    36410,36410,36410,36410,36410,36410,36410,36410,
    25080,25080,25080,25080,25080,25080,25080,25080,
    12785,12785,12785,12785,12785,12785,12785,12785
};


// This macro expects the following registers to contain:
// r1 = I(7)
// r2 = I(3)
// r3 = I(1)
// r7 = I(5)
// and calculates each row op0-7 in its respective register
#define VP3_1D_IDCT_SSE2(ADD, SHIFT, r0, r1, r2, r3, r4, r5, r6, r7) \
    "movdqa "C(3)", " #r6" \n\t"     /* xmm6 = c3 */ \
    "movdqa " #r2", " #r4" \n\t"     /* xmm4 = i3 */ \
    "pmulhw " #r6", " #r4" \n\t"     /* xmm4 = c3 * i3 - i3 */ \
    "movdqa "C(5)", " #r0" \n\t"     /* xmm0 = c5 */ \
    "pmulhw " #r7", " #r6" \n\t"     /* xmm6 = c3 * i5 - i5 */ \
    "movdqa " #r0", " #r5" \n\t"     /* xmm5 = c5 */ \
    "pmulhw " #r2", " #r0" \n\t"     /* xmm0 = c5 * i3 - i3 */ \
    "pmulhw " #r7", " #r5" \n\t"     /* xmm5 = c5 * i5 - i5 */ \
    "paddw  " #r2", " #r4" \n\t"     /* xmm4 = c3 * i3 */ \
    "paddw  " #r7", " #r6" \n\t"     /* xmm6 = c3 * i5 */ \
    "paddw  " #r0", " #r2" \n\t"     /* xmm2 = c5 * i3 */ \
    "movdqa "C(1)", " #r0" \n\t"     /* xmm0 = c1 */ \
    "paddw  " #r5", " #r7" \n\t"     /* xmm7 = c5 * i5 */ \
    "movdqa " #r0", " #r5" \n\t"     /* xmm5 = c1 */ \
    "pmulhw " #r3", " #r0" \n\t"     /* xmm0 = c1 * i1 - i1 */ \
    "paddsw " #r7", " #r4" \n\t"     /* xmm4 = c3 * i3 + c5 * i5 = C */ \
    "pmulhw " #r1", " #r5" \n\t"     /* xmm5 = c1 * i7 - i7 */ \
    "movdqa "C(7)", " #r7" \n\t"     /* xmm7 = c7 */ \
    "psubsw " #r2", " #r6" \n\t"     /* xmm6 = c3 * i5 - c5 * i3 = D */ \
    "paddw  " #r3", " #r0" \n\t"     /* xmm0 = c1 * i1 */ \
    "pmulhw " #r7", " #r3" \n\t"     /* xmm3 = c7 * i1 */ \
    "movdqa "I(2)", " #r2" \n\t"     /* xmm2 = i2 */ \
    "pmulhw " #r1", " #r7" \n\t"     /* xmm7 = c7 * i7 */ \
    "paddw  " #r1", " #r5" \n\t"     /* xmm5 = c1 * i7 */ \
    "movdqa " #r2", " #r1" \n\t"     /* xmm1 = i2 */ \
    "pmulhw "C(2)", " #r2" \n\t"     /* xmm2 = i2 * c2 -i2 */ \
    "psubsw " #r5", " #r3" \n\t"     /* xmm3 = c7 * i1 - c1 * i7 = B */ \
    "movdqa "I(6)", " #r5" \n\t"     /* xmm5 = i6 */ \
    "paddsw " #r7", " #r0" \n\t"     /* xmm0 = c1 * i1 + c7 * i7 = A */ \
    "movdqa " #r5", " #r7" \n\t"     /* xmm7 = i6 */ \
    "psubsw " #r4", " #r0" \n\t"     /* xmm0 = A - C */ \
    "pmulhw "C(2)", " #r5" \n\t"     /* xmm5 = c2 * i6 - i6 */ \
    "paddw  " #r1", " #r2" \n\t"     /* xmm2 = i2 * c2 */ \
    "pmulhw "C(6)", " #r1" \n\t"     /* xmm1 = c6 * i2 */ \
    "paddsw " #r4", " #r4" \n\t"     /* xmm4 = C + C */ \
    "paddsw " #r0", " #r4" \n\t"     /* xmm4 = A + C = C. */ \
    "psubsw " #r6", " #r3" \n\t"     /* xmm3 = B - D */ \
    "paddw  " #r7", " #r5" \n\t"     /* xmm5 = c2 * i6 */ \
    "paddsw " #r6", " #r6" \n\t"     /* xmm6 = D + D */ \
    "pmulhw "C(6)", " #r7" \n\t"     /* xmm7 = c6 * i6 */ \
    "paddsw " #r3", " #r6" \n\t"     /* xmm6 = B + D = D. */ \
    "movdqa " #r4", "I(1)" \n\t"     /* Save C. at I(1) */ \
    "psubsw " #r5", " #r1" \n\t"     /* xmm1 = c6 * i2 - c2 * i6 = H */ \
    "movdqa "C(4)", " #r4" \n\t"     /* xmm4 = c4 */ \
    "movdqa " #r3", " #r5" \n\t"     /* xmm5 = B - D */ \
    "pmulhw " #r4", " #r3" \n\t"     /* xmm3 = ( c4 -1 ) * ( B - D ) */ \
    "paddsw " #r2", " #r7" \n\t"     /* xmm7 = c2 * i2 + c6 * i6 = G */ \
    "movdqa " #r6", "I(2)" \n\t"     /* Save D. at I(2) */ \
    "movdqa " #r0", " #r2" \n\t"     /* xmm2 = A - C */ \
    "movdqa "I(0)", " #r6" \n\t"     /* xmm6 = i0 */ \
    "pmulhw " #r4", " #r0" \n\t"     /* xmm0 = ( c4 - 1 ) * ( A - C ) = A. */ \
    "paddw  " #r3", " #r5" \n\t"     /* xmm5 = c4 * ( B - D ) = B. */ \
    "movdqa "I(4)", " #r3" \n\t"     /* xmm3 = i4 */ \
    "psubsw " #r1", " #r5" \n\t"     /* xmm5 = B. - H = B.. */ \
    "paddw  " #r0", " #r2" \n\t"     /* xmm2 = c4 * ( A - C) = A. */ \
    "psubsw " #r3", " #r6" \n\t"     /* xmm6 = i0 - i4 */ \
    "movdqa " #r6", " #r0" \n\t"     /* xmm0 = i0 - i4 */ \
    "pmulhw " #r4", " #r6" \n\t"     /* xmm6 = (c4 - 1) * (i0 - i4) = F */ \
    "paddsw " #r3", " #r3" \n\t"     /* xmm3 = i4 + i4 */ \
    "paddsw " #r1", " #r1" \n\t"     /* xmm1 = H + H */ \
    "paddsw " #r0", " #r3" \n\t"     /* xmm3 = i0 + i4 */ \
    "paddsw " #r5", " #r1" \n\t"     /* xmm1 = B. + H = H. */ \
    "pmulhw " #r3", " #r4" \n\t"     /* xmm4 = ( c4 - 1 ) * ( i0 + i4 )  */ \
    "paddw  " #r0", " #r6" \n\t"     /* xmm6 = c4 * ( i0 - i4 ) */ \
    "psubsw " #r2", " #r6" \n\t"     /* xmm6 = F - A. = F. */ \
    "paddsw " #r2", " #r2" \n\t"     /* xmm2 = A. + A. */ \
    "movdqa "I(1)", " #r0" \n\t"     /* Load        C. from I(1) */ \
    "paddsw " #r6", " #r2" \n\t"     /* xmm2 = F + A. = A.. */ \
    "paddw  " #r3", " #r4" \n\t"     /* xmm4 = c4 * ( i0 + i4 ) = 3 */ \
    "psubsw " #r1", " #r2" \n\t"     /* xmm2 = A.. - H. = R2 */ \
    ADD(r2)                          /* Adjust R2 and R1 before shifting */ \
    "paddsw " #r1", " #r1" \n\t"     /* xmm1 = H. + H. */ \
    "paddsw " #r2", " #r1" \n\t"     /* xmm1 = A.. + H. = R1 */ \
    SHIFT(r2)                        /* xmm2 = op2 */ \
    "psubsw " #r7", " #r4" \n\t"     /* xmm4 = E - G = E. */ \
    SHIFT(r1)                        /* xmm1 = op1 */ \
    "movdqa "I(2)", " #r3" \n\t"     /* Load D. from I(2) */ \
    "paddsw " #r7", " #r7" \n\t"     /* xmm7 = G + G */ \
    "paddsw " #r4", " #r7" \n\t"     /* xmm7 = E + G = G. */ \
    "psubsw " #r3", " #r4" \n\t"     /* xmm4 = E. - D. = R4 */ \
    ADD(r4)                          /* Adjust R4 and R3 before shifting */ \
    "paddsw " #r3", " #r3" \n\t"     /* xmm3 = D. + D. */ \
    "paddsw " #r4", " #r3" \n\t"     /* xmm3 = E. + D. = R3 */ \
    SHIFT(r4)                        /* xmm4 = op4 */ \
    "psubsw " #r5", " #r6" \n\t"     /* xmm6 = F. - B..= R6 */ \
    SHIFT(r3)                        /* xmm3 = op3 */ \
    ADD(r6)                          /* Adjust R6 and R5 before shifting */ \
    "paddsw " #r5", " #r5" \n\t"     /* xmm5 = B.. + B.. */ \
    "paddsw " #r6", " #r5" \n\t"     /* xmm5 = F. + B.. = R5 */ \
    SHIFT(r6)                        /* xmm6 = op6 */ \
    SHIFT(r5)                        /* xmm5 = op5 */ \
    "psubsw " #r0", " #r7" \n\t"     /* xmm7 = G. - C. = R7 */ \
    ADD(r7)                          /* Adjust R7 and R0 before shifting */ \
    "paddsw " #r0", " #r0" \n\t"     /* xmm0 = C. + C. */ \
    "paddsw " #r7", " #r0" \n\t"     /* xmm0 = G. + C. */ \
    SHIFT(r7)                        /* xmm7 = op7 */ \
    SHIFT(r0)                        /* xmm0 = op0 */

#define PUT_BLOCK(r0, r1, r2, r3, r4, r5, r6, r7) \
    "movdqa " #r0 ", " I(0) "\n\t" \
    "movdqa " #r1 ", " I(1) "\n\t" \
    "movdqa " #r2 ", " I(2) "\n\t" \
    "movdqa " #r3 ", " I(3) "\n\t" \
    "movdqa " #r4 ", " I(4) "\n\t" \
    "movdqa " #r5 ", " I(5) "\n\t" \
    "movdqa " #r6 ", " I(6) "\n\t" \
    "movdqa " #r7 ", " I(7) "\n\t"

#define LOAD_ODD_ROWS(r1, r3, r5, r7) \
    "movdqa " I(1) ", " #r1 " \n\t" \
    "movdqa " I(3) ", " #r3 " \n\t" \
    "movdqa " I(5) ", " #r5 " \n\t" \
    "movdqa " I(7) ", " #r7 " \n\t"

#define STORE_EVEN_ROWS(r0, r2, r4, r6) \
    "movdqa " #r0 ", " I(0) "\n\t" \
    "movdqa " #r2 ", " I(2) "\n\t" \
    "movdqa " #r4 ", " I(4) "\n\t" \
    "movdqa " #r6 ", " I(6) "\n\t"

#define NOP(xmm)
#define SHIFT4(xmm) "psraw  $4, "#xmm"\n\t"
#define ADD8(xmm)   "paddsw %2, "#xmm"\n\t"

void ff_vp3_idct_sse2(int16_t *input_data)
{
#define I(x) AV_STRINGIFY(16*x)"(%0)"
#define C(x) AV_STRINGIFY(16*(x-1))"(%1)"

    __asm__ volatile (
        LOAD_ODD_ROWS(%%xmm3, %%xmm2, %%xmm7, %%xmm1)
        VP3_1D_IDCT_SSE2(NOP, NOP, %%xmm0, %%xmm1, %%xmm2, %%xmm3, %%xmm4, %%xmm5, %%xmm6, %%xmm7)

        TRANSPOSE8(%%xmm0, %%xmm1, %%xmm2, %%xmm3, %%xmm4, %%xmm5, %%xmm6, %%xmm7, (%0))
        STORE_EVEN_ROWS(%%xmm0, %%xmm7, %%xmm6, %%xmm2)

        VP3_1D_IDCT_SSE2(ADD8, SHIFT4, %%xmm0, %%xmm1, %%xmm3, %%xmm5, %%xmm2, %%xmm7, %%xmm6, %%xmm4)
        PUT_BLOCK(%%xmm0, %%xmm1, %%xmm3, %%xmm5, %%xmm2, %%xmm7, %%xmm6, %%xmm4)
        :: "r"(input_data), "r"(ff_vp3_idct_data), "m"(ff_pw_8)
    );
}

void ff_vp3_idct_put_sse2(uint8_t *dest, int line_size, DCTELEM *block)
{
    ff_vp3_idct_sse2(block);
    put_signed_pixels_clamped_mmx(block, dest, line_size);
}

void ff_vp3_idct_add_sse2(uint8_t *dest, int line_size, DCTELEM *block)
{
    ff_vp3_idct_sse2(block);
    add_pixels_clamped_mmx(block, dest, line_size);
}
