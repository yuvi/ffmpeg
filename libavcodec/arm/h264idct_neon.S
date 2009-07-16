/*
 * Copyright (c) 2008 Mans Rullgard <mans@mansr.com>
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

#include "asm.S"

        preserve8
        .fpu neon

        .text

function ff_h264_idct_add_neon, export=1
        mov             r3,  #(1<<5)
        vmov.i16        d16, #0
        vmov.16         d16[0],   r3
        vld1.64         {d0-d3},  [r1,:128]
        vadd.i16        d0,  d0,  d16

        vswp            d1,  d2
        vadd.i16        d4,  d0,  d1
        vshr.s16        q8,  q1,  #1
        vsub.i16        d5,  d0,  d1
        vadd.i16        d6,  d2,  d17
        vsub.i16        d7,  d16, d3
        vadd.i16        q0,  q2,  q3
        vsub.i16        q1,  q2,  q3

        vtrn.16         d0,  d1
        vtrn.16         d3,  d2
        vtrn.32         d0,  d3
        vtrn.32         d1,  d2

        vadd.i16        d4,  d0,  d3
        vld1.32         {d18[0]}, [r0,:32], r2
        vswp            d1,  d3
        vshr.s16        q8,  q1,  #1
        vld1.32         {d19[1]}, [r0,:32], r2
        vsub.i16        d5,  d0,  d1
        vld1.32         {d18[1]}, [r0,:32], r2
        vadd.i16        d6,  d16, d3
        vld1.32         {d19[0]}, [r0,:32], r2
        vsub.i16        d7,  d2,  d17
        sub             r0,  r0,  r2, lsl #2
        vadd.i16        q0,  q2,  q3
        vsub.i16        q1,  q2,  q3

        vshr.s16        q0,  q0,  #6
        vshr.s16        q1,  q1,  #6

        vaddw.u8        q0,  q0,  d18
        vaddw.u8        q1,  q1,  d19

        vqmovun.s16     d0,  q0
        vqmovun.s16     d1,  q1

        vst1.32         {d0[0]},  [r0,:32], r2
        vst1.32         {d1[1]},  [r0,:32], r2
        vst1.32         {d0[1]},  [r0,:32], r2
        vst1.32         {d1[0]},  [r0,:32], r2

        bx              lr
        .endfunc

function ff_h264_idct_dc_add_neon, export=1
        vld1.16         {d2[],d3[]}, [r1,:16]
        vrshr.s16       q1,  q1,  #6
        vld1.32         {d0[0]},  [r0,:32], r2
        vld1.32         {d0[1]},  [r0,:32], r2
        vaddw.u8        q2,  q1,  d0
        vld1.32         {d1[0]},  [r0,:32], r2
        vld1.32         {d1[1]},  [r0,:32], r2
        vaddw.u8        q1,  q1,  d1
        vqmovun.s16     d0,  q2
        vqmovun.s16     d1,  q1
        sub             r0,  r0,  r2, lsl #2
        vst1.32         {d0[0]},  [r0,:32], r2
        vst1.32         {d0[1]},  [r0,:32], r2
        vst1.32         {d1[0]},  [r0,:32], r2
        vst1.32         {d1[1]},  [r0,:32], r2
        bx              lr
        .endfunc

function ff_h264_idct_add16_neon, export=1
        push            {r4-r8,lr}
        mov             r4,  r0
        mov             r5,  r1
        mov             r1,  r2
        mov             r2,  r3
        ldr             r6,  [sp, #24]
        movrel          r7,  scan8
        mov             ip,  #16
1:      ldrb            r8,  [r7], #1
        ldr             r0,  [r5], #4
        ldrb            r8,  [r6, r8]
        subs            r8,  r8,  #1
        blt             2f
        ldrsh           lr,  [r1]
        add             r0,  r0,  r4
        movne           lr,  #0
        cmp             lr,  #0
        adrne           lr,  ff_h264_idct_dc_add_neon
        adreq           lr,  ff_h264_idct_add_neon
        blx             lr
2:      subs            ip,  ip,  #1
        add             r1,  r1,  #32
        bne             1b
        pop             {r4-r8,pc}
        .endfunc

function ff_h264_idct_add16intra_neon, export=1
        push            {r4-r8,lr}
        mov             r4,  r0
        mov             r5,  r1
        mov             r1,  r2
        mov             r2,  r3
        ldr             r6,  [sp, #24]
        movrel          r7,  scan8
        mov             ip,  #16
1:      ldrb            r8,  [r7], #1
        ldr             r0,  [r5], #4
        ldrb            r8,  [r6, r8]
        add             r0,  r0,  r4
        cmp             r8,  #0
        ldrsh           r8,  [r1]
        adrne           lr,  ff_h264_idct_add_neon
        adreq           lr,  ff_h264_idct_dc_add_neon
        cmpeq           r8,  #0
        blxne           lr
        subs            ip,  ip,  #1
        add             r1,  r1,  #32
        bne             1b
        pop             {r4-r8,pc}
        .endfunc

function ff_h264_idct_add8_neon, export=1
        push            {r4-r10,lr}
        ldm             r0,  {r4,r9}
        add             r5,  r1,  #16*4
        add             r1,  r2,  #16*32
        mov             r2,  r3
        ldr             r6,  [sp, #32]
        movrel          r7,  scan8+16
        mov             ip,  #8
1:      ldrb            r8,  [r7], #1
        ldr             r0,  [r5], #4
        ldrb            r8,  [r6, r8]
        tst             ip,  #4
        addeq           r0,  r0,  r4
        addne           r0,  r0,  r9
        cmp             r8,  #0
        ldrsh           r8,  [r1]
        adrne           lr,  ff_h264_idct_add_neon
        adreq           lr,  ff_h264_idct_dc_add_neon
        cmpeq           r8,  #0
        blxne           lr
        subs            ip,  ip,  #1
        add             r1,  r1,  #32
        bne             1b
        pop             {r4-r10,pc}
        .endfunc


        .macro SUMSUB_AB sum, diff, a, b
        vadd.s16    \sum,  \a, \b
        vsub.s16    \diff, \a, \b
        .endm

        // sum = a + b>>shift   sub = a>>shift - b
        .macro SUMSUB_SHR shift sum sub a b t0 t1
        vshr.s16    \t0,  \b, #\shift
        vshr.s16    \t1,  \a, #\shift
        vadd.s16    \sum, \a, \t0
        vsub.s16    \sub, \t1, \b
        .endm

        // sum = a>>shift + b   sub = a - b>>shift
        .macro SUMSUB_SHR2 shift sum sub a b t0 t1
        vshr.s16    \t0,  \a, #\shift
        vshr.s16    \t1,  \b, #\shift
        vadd.s16    \sum, \t0, \b
        vsub.s16    \sub, \a, \t1
        .endm

        // a += 1.5*ma   b -= 1.5*mb
        .macro SUMSUB_MULL a b ma mb t0 t1
        vshr.s16    \t0, \ma, #1
        vshr.s16    \t1, \mb, #1
        vadd.s16    \t0, \t0, \ma
        vadd.s16    \t1, \t1, \mb
        vadd.s16    \a,  \a,  \t0
        vsub.s16    \b,  \b,  \t1
        .endm

        .macro IDCT8_1D type
.ifc \type, col
        vswp            d21, d28
.endif
        SUMSUB_AB       q0,  q1,  q8,  q12              // a0/a2
.ifc \type, row
        vld1.64         {d28-d31}, [r1,:64]!
.else
        vswp            d19, d26
.endif
        SUMSUB_SHR   1, q2,  q3,  q10, q14,  q8, q12    // a6/a4
.ifc \type, col
        vswp            d23, d30
.endif
        SUMSUB_AB       q8,  q10, q13, q11
        SUMSUB_MULL     q8,  q10, q9,  q15,  q12, q14   // a7/a1
        SUMSUB_AB       q14, q15, q15, q9
        SUMSUB_MULL     q15, q14, q13, q11,  q12, q9    // a5/a3

        SUMSUB_SHR   2, q13, q14, q14, q15,  q11, q9    // b3/b5
        SUMSUB_SHR2  2, q12, q15, q8,  q10,  q11, q9    // b1/b7

        SUMSUB_AB       q10, q2,  q0,  q2               // b0/b6
        SUMSUB_AB       q11, q3,  q1,  q3               // b2/b4

        SUMSUB_AB       q8,  q15, q10, q15
        SUMSUB_AB       q9,  q14, q11, q14
        SUMSUB_AB       q10, q13, q3,  q13
.ifc \type, row
        vtrn.16         q8,  q9
.endif
        SUMSUB_AB       q11, q12, q2,  q12
        .endm

function ff_h264_idct8_add_neon, export=1
        vld1.64         {d16-d19}, [r1,:64]!
        vld1.64         {d20-d23}, [r1,:64]!
        vld1.64         {d24-d27}, [r1,:64]!

        IDCT8_1D row
        vtrn.16         q10, q11
        vtrn.16         q12, q13
        vtrn.16         q14, q15
        vtrn.32         q8,  q10
        vtrn.32         q9,  q11
        vtrn.32         q12, q14
        vtrn.32         q13, q15
        vswp            d17, d24
        IDCT8_1D col

        vld1.64         {d0}, [r0,:64], r2
        vrshr.s16       q8,  q8,  #6
        vld1.64         {d1}, [r0,:64], r2
        vrshr.s16       q9,  q9,  #6
        vld1.64         {d2}, [r0,:64], r2
        vrshr.s16       q10, q10, #6
        vld1.64         {d3}, [r0,:64], r2
        vrshr.s16       q11, q11, #6
        vld1.64         {d4}, [r0,:64], r2
        vrshr.s16       q12, q12, #6
        vld1.64         {d5}, [r0,:64], r2
        vrshr.s16       q13, q13, #6
        vld1.64         {d6}, [r0,:64], r2
        vrshr.s16       q14, q14, #6
        vld1.64         {d7}, [r0,:64], r2
        vrshr.s16       q15, q15, #6

        sub             r1,  r1,  #128
        sub             r0,  r0,  r2,  lsl #3
        vaddw.u8        q8,  q8,  d0
        vaddw.u8        q9,  q9,  d1
        vaddw.u8        q10, q10, d2
        vqmovun.s16     d0,  q8
        vqmovun.s16     d1,  q9
        vqmovun.s16     d2,  q10
        vaddw.u8        q11, q11, d3
        vst1.64         {d0}, [r0,:64], r2
        vaddw.u8        q12, q12, d4
        vst1.64         {d1}, [r0,:64], r2
        vaddw.u8        q13, q13, d5
        vst1.64         {d2}, [r0,:64], r2
        vqmovun.s16     d3,  q11
        vqmovun.s16     d4,  q12
        vaddw.u8        q14, q14, d6
        vaddw.u8        q15, q15, d7
        vst1.64         {d3}, [r0,:64], r2
        vqmovun.s16     d5,  q13
        vst1.64         {d4}, [r0,:64], r2
        vqmovun.s16     d6,  q14
        vqmovun.s16     d7,  q15
        vst1.64         {d5}, [r0,:64], r2
        vst1.64         {d6}, [r0,:64], r2
        vst1.64         {d7}, [r0,:64], r2
        bx              lr
        .endfunc

function ff_h264_idct8_dc_add_neon, export=1
        vld1.16         {d30[]}, [r1,:16]
        vmov            d31, d30
        vld1.32         {d0},  [r0,:64], r2
        vrshr.s16       q15, q15, #6
        vld1.32         {d1},  [r0,:64], r2
        vld1.32         {d2},  [r0,:64], r2
        vaddw.u8        q8,  q15, d0
        vld1.32         {d3},  [r0,:64], r2
        vaddw.u8        q9,  q15, d1
        vld1.32         {d4},  [r0,:64], r2
        vaddw.u8        q10, q15, d2
        vld1.32         {d5},  [r0,:64], r2
        vaddw.u8        q11, q15, d3
        vld1.32         {d6},  [r0,:64], r2
        vaddw.u8        q12, q15, d4
        vld1.32         {d7},  [r0,:64], r2
        vaddw.u8        q13, q15, d5
        vaddw.u8        q14, q15, d6
        vaddw.u8        q15, q15, d7
        vqmovun.s16     d0,  q8
        vqmovun.s16     d1,  q9
        vqmovun.s16     d2,  q10
        vqmovun.s16     d3,  q11
        sub             r0,  r0,  r2, lsl #3
        vst1.32         {d0},  [r0,:64], r2
        vqmovun.s16     d4,  q12
        vst1.32         {d1},  [r0,:64], r2
        vqmovun.s16     d5,  q13
        vst1.32         {d2},  [r0,:64], r2
        vqmovun.s16     d6,  q14
        vst1.32         {d3},  [r0,:64], r2
        vqmovun.s16     d7,  q15
        vst1.32         {d4},  [r0,:64], r2
        vst1.32         {d5},  [r0,:64], r2
        vst1.32         {d6},  [r0,:64], r2
        vst1.32         {d7},  [r0,:64], r2
        bx              lr
        .endfunc

function ff_h264_idct8_add4_neon, export=1
        push            {r4-r8,lr}
        mov             r4,  r0
        mov             r5,  r1
        mov             r1,  r2
        mov             r2,  r3
        ldr             r6,  [sp, #24]
        movrel          r7,  scan8
        mov             ip,  #16
1:      ldrb            r8,  [r7], #4
        ldr             r0,  [r5], #16
        ldrb            r8,  [r6, r8]
        subs            r8,  r8,  #1
        blt             2f
        ldrsh           lr,  [r1]
        add             r0,  r0,  r4
        movne           lr,  #0
        cmp             lr,  #0
        adrne           lr,  ff_h264_idct8_dc_add_neon
        adreq           lr,  ff_h264_idct8_add_neon
        blx             lr
2:      subs            ip,  ip,  #4
        add             r1,  r1,  #128
        bne             1b
        pop             {r4-r8,pc}
        .endfunc

        .section .rodata
scan8:  .byte           4+1*8, 5+1*8, 4+2*8, 5+2*8
        .byte           6+1*8, 7+1*8, 6+2*8, 7+2*8
        .byte           4+3*8, 5+3*8, 4+4*8, 5+4*8
        .byte           6+3*8, 7+3*8, 6+4*8, 7+4*8
        .byte           1+1*8, 2+1*8
        .byte           1+2*8, 2+2*8
        .byte           1+4*8, 2+4*8
        .byte           1+5*8, 2+5*8
