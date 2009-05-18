/*
 * Copyright (c) 2009 David Conrad
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

.fpu neon

.section .rodata

.macro vc1_mspel_filter n t0 t1 t2 t3
.align 4
vc1_mspel_filter_\n\():
.rept 8
    .byte \t0
.endr
.rept 8
    .byte \t1
.endr
.rept 8
    .byte \t2
.endr
.rept 8
    .byte \t3
.endr
.endm

vc1_mspel_filter 1,  4, 53, 18, 3
vc1_mspel_filter 2,  1,  9,  9, 1
vc1_mspel_filter 3,  3, 18, 53, 4

vc1_mspel_filter16_1:
.short  4, 53, 18, 3

vc1_mspel_filter16_2:
.short  1,  9,  9, 1

vc1_mspel_filter16_3:
.short  3, 18, 53, 4


.text
.align


// d24-d27: filter  q14: rounder  q15: shift
function put_mspel_v_lowpass_neon
    mov         r3,  #2
    vld1.64     {d16}, [r1], r2
    vld1.64     {d17}, [r1], r2
    vld1.64     {d18}, [r1], r2

1:  subs        r3,  r3,  #1
    vld1.64     {d19}, [r1], r2
    vmull.u8    q0,  d17, d25
    vmlal.u8    q0,  d18, d26
    vmlsl.u8    q0,  d16, d24
    vmlsl.u8    q0,  d19, d27
    vadd.s16    q0,  q0,  q14
    vshl.s16    q0,  q0,  q15
    vqmovun.s16 d0,  q0
    vst1.64     {d0}, [r0,:64], r2

    vld1.64     {d16}, [r1], r2
    vmull.u8    q1,  d18, d25
    vmlal.u8    q1,  d19, d26
    vmlsl.u8    q1,  d17, d24
    vmlsl.u8    q1,  d16, d27
    vadd.s16    q1,  q1,  q14
    vshl.s16    q1,  q1,  q15
    vqmovun.s16 d2,  q1
    vst1.64     {d2}, [r0,:64], r2

    vld1.64     {d17}, [r1], r2
    vmull.u8    q2,  d19, d25
    vmlal.u8    q2,  d16, d26
    vmlsl.u8    q2,  d18, d24
    vmlsl.u8    q2,  d17, d27
    vadd.s16    q2,  q2,  q14
    vshl.s16    q2,  q2,  q15
    vqmovun.s16 d4,  q2
    vst1.64     {d4}, [r0,:64], r2

    vld1.64     {d18}, [r1], r2
    vmull.u8    q3,  d16, d25
    vmlal.u8    q3,  d17, d26
    vmlsl.u8    q3,  d19, d24
    vmlsl.u8    q3,  d18, d27
    vadd.s16    q3,  q3,  q14
    vshl.s16    q3,  q3,  q15
    vqmovun.s16 d6,  q3
    vst1.64     {d6}, [r0,:64], r2

    bgt         1b
    bx          lr
.endfunc

function put_mspel_h_lowpass_neon
    mov         r3,  #4

1:  vld1.64     {d16-d17}, [r1], r2
    pld         [r1]
    vext.8      d18, d16, d17, #2
    vext.8      d19, d16, d17, #3
    vmull.u8    q0,  d18, d26
    vext.8      d17, d16, d17, #1
    vmlsl.u8    q0,  d19, d27
    vmlal.u8    q0,  d17, d25
    vmlsl.u8    q0,  d16, d24

    vld1.64     {d20-d21}, [r1], r2
    pld         [r1]

    vext.8      d22, d20, d21, #2
    vext.8      d23, d20, d21, #3
    vmull.u8    q1,  d22, d26
    vext.8      d21, d20, d21, #1
    vmlal.u8    q1,  d21, d25
    vmlsl.u8    q1,  d20, d24
    vmlsl.u8    q1,  d23, d27
    vadd.s16    q0,  q0,  q14


    vshl.s16    q0,  q0,  q15

    vqmovun.s16 d0,  q0

    vst1.64     {d0}, [r0,:64], r2
    subs        r3,  r3,  #1
    vadd.s16    q1,  q1,  q14
    vshl.s16    q1,  q1,  q15
    vqmovun.s16 d2,  q1
    vst1.64     {d2}, [r0,:64], r2

    bgt         1b
    bx          lr
.endfunc

// filter for 2d bicubic interpolation
// need to filter 11 values vertically
function put_mspel_2d_neon
    mov         r3,  #2
    vld1.64     {d16-d17}, [r1], r2
    vld1.64     {d18-d19}, [r1], r2
    vld1.64     {d20-d21}, [r1], r2
    mov         ip,  lr

1:  subs        r3,  r3,  #1
    vld1.64     {d22-d23}, [r1], r2
    vmull.u8    q4,  d18, d25
    vmlal.u8    q4,  d20, d26
    vmlsl.u8    q4,  d16, d24
    vmlsl.u8    q4,  d22, d27
    vmull.u8    q5,  d19, d25
    vmlal.u8    q5,  d21, d26
    vmlsl.u8    q5,  d17, d24
    vmlsl.u8    q5,  d23, d27
    vadd.s16    q4,  q4,  q14
    vld1.64     {d16-d17}, [r1], r2

    vshl.s16    q4,  q4,  q15
    vadd.s16    q5,  q5,  q14
    vshl.s16    q5,  q5,  q15

//    bl          put_mspel_h_lowpass16_neon

    vmull.u8    q2,  d20, d25
    vmlal.u8    q2,  d22, d26
    vmlsl.u8    q2,  d18, d24
    vmlsl.u8    q2,  d16, d27
    vadd.s16    q2,  q4,  q14
    vshl.s16    q2,  q4,  q15

    vmull.u8    q3,  d21, d25
    vmlal.u8    q3,  d23, d26
    vmlsl.u8    q3,  d19, d24
    vmlsl.u8    q3,  d17, d27
    vadd.s16    q3,  q3,  q14
    vshl.s16    q3,  q3,  q15

    bl          put_mspel_h_lowpass16_neon

    vld1.64     {d18-d19}, [r1], r2
    vmull.u8    q4,  d22, d25
    vmlal.u8    q4,  d16, d26
    vmlsl.u8    q4,  d20, d24
    vmlsl.u8    q4,  d18, d27
    vadd.s16    q4,  q4,  q14
    vshl.s16    q4,  q4,  q15

    vmull.u8    q5,  d23, d25
    vmlal.u8    q5,  d17, d26
    vmlsl.u8    q5,  d21, d24
    vmlsl.u8    q5,  d19, d27
    vadd.s16    q5,  q5,  q14
    vshl.s16    q5,  q5,  q15

//    bl          put_mspel_h_lowpass16_neon

    vld1.64     {d20-d21}, [r1], r2
    vmull.u8    q2,  d16, d25
    vmlal.u8    q2,  d18, d26
    vmlsl.u8    q2,  d22, d24
    vmlsl.u8    q2,  d20, d27
    vadd.s16    q2,  q4,  q14
    vshl.s16    q2,  q4,  q15

    vmull.u8    q3,  d17, d25
    vmlal.u8    q3,  d19, d26
    vmlsl.u8    q3,  d23, d24
    vmlsl.u8    q3,  d21, d27
    vadd.s16    q3,  q3,  q14
    vshl.s16    q3,  q3,  q15

    bl          put_mspel_h_lowpass16_neon

    bgt         1b
    vpop        {d8-d15}
    bx          ip
.endfunc


// horizontal filter for 2d bicubic interpolation
// q4-q5 have 1 row of vertically filtered pixels
function put_mspel_h_lowpass16_neon
    vext.16     q6,  q4,  q5,  #2
    vext.16     q7,  q4,  q5,  #3
    vext.16     q5,  q4,  q5,  #1

    vmul.s16    d12, d12, d2[2]
    vmls.s16    d12, d14, d2[3]
    vmla.s16    d12, d10, d2[1]
    vmls.s16    d12, d8,  d2[0]

    vmul.s16    d13, d13, d2[2]
    vmls.s16    d13, d15, d2[3]
    vmla.s16    d13, d11, d2[1]
    vmls.s16    d13, d9,  d2[0]

    vext.16     q4,  q2,  q3,  #2
    vext.16     q5,  q2,  q3,  #3
    vext.16     q3,  q2,  q3,  #1

    vmul.s16    d14, d8,  d2[2]
    vmls.s16    d14, d10, d2[3]
    vmla.s16    d14, d6,  d2[1]
    vmls.s16    d14, d4,  d2[0]

    vadd.s16    q6,  q6,  q0

    vmul.s16    d15, d9,  d2[2]
    vmls.s16    d15, d11, d2[3]
    vmla.s16    d15, d7,  d2[1]
    vmls.s16    d15, d5,  d2[0]

    vshr.s16    q6,  q6,  #7


    vqmovun.s16 d12, q6
    vst1.64     {d12}, [r0,:64], r2

    vadd.s16    q7,  q7,  q0
    vqmovun.s16 d14, q7
    vst1.64     {d14}, [r0,:64], r2
    bx          lr
.endfunc


.macro MSPEL_HV a b shift
.if \b == 0
function ff_put_vc1_mspel_mc\a\()0_neon, export=1
    movrel      ip,  vc1_mspel_filter_\a
    rsb         r3,  r3,  #(1<<(\shift -1))
    vld1.64     {d24-d27}, [ip,:128]
    vdup.16     q14, r3
    vmvn.i16    q15, #\shift -1
    sub         r1,  r1,  #1
    b           put_mspel_h_lowpass_neon
.endfunc

.elseif \a == 0
function ff_put_vc1_mspel_mc0\b\()_neon, export=1
    movrel      ip,  vc1_mspel_filter_\b
    add         r3,  r3,  #(1<<(\shift -1))-1
    vld1.64     {d24-d27}, [ip,:128]
    vdup.16     q14, r3
    vmvn.i16    q15, #\shift -1
    sub         r1,  r1,  r2
    b           put_mspel_v_lowpass_neon
.endfunc

.else
function ff_put_vc1_mspel_mc\a\b\()_neon, export=1
    movrel      ip,  vc1_mspel_filter_\b
    vpush       {d8-d15}
    vld1.64     {d24-d27}, [ip,:128]

    movrel      ip,  vc1_mspel_filter16_\a
    vld1.64     {d2}, [ip,:64]

    rsb         ip,  r3,  #64
.if (1<<(\shift -1))-1
    add         r3,  r3,  #(1<<(\shift -1))-1
.endif
    vdup.16     q14, r3
    vdup.16     q0,  ip

    sub         r1,  r1,  r2
    sub         r1,  r1,  #1

    vmvn.i16    q15, #\shift -1

    b           put_mspel_2d_neon
.endfunc
.endif
.endm

MSPEL_HV  0, 1,  6
MSPEL_HV  0, 2,  4
MSPEL_HV  0, 3,  6

MSPEL_HV  1, 0,  6
MSPEL_HV  1, 1,  5
MSPEL_HV  1, 2,  3
MSPEL_HV  1, 3,  5

MSPEL_HV  2, 0,  4
MSPEL_HV  2, 1,  3
MSPEL_HV  2, 2,  1
MSPEL_HV  2, 3,  3

MSPEL_HV  3, 0,  6
MSPEL_HV  3, 1,  5
MSPEL_HV  3, 2,  3
MSPEL_HV  3, 3,  5


// dest = (2*(p1 - q1) - 5*(p0 - q0) + 4) >> 3
// clobbers q12, q13
.macro VC1_LOOP_FILTER_A0 dest, p1, p0, q0, q1
    vsubl.u8    q12, \p0, \q0
    vsubl.u8    q13, \p1, \q1
    vmul.s16    q12, q12, q15
    vadd.s16    q13, q13, q13
    vsub.s16    q12, q13, q12
    vadd.s16    q12, q12, q14
    vshr.s16    \dest, q12, #3
.endm

// q14: if()  q15: pw5 
.macro VC1_FILTER a0, a1, a2
    vabs.s16    \a1, \a1
    vabs.s16    \a2, \a2
    vabs.s16    q8,  \a0

    vdup.16     q14, r2
    vcgt.u16    q14, q14, q8    // if (a0 < pq)

    vmin.u16    \a1, \a1, \a2   // a3
    vcgt.u16    q9,  q8,  \a1   // if (a2 < a0 || a1 < a0)
    vand        q14, q14, q9    // if (a0 < pq) && (a2 < a0 || a1 < a0)

    vsubl.u8    q13, d0,  d1    // q13 - clip (signed)
    vabs.s16    q9,  q13
    vshr.u16    q9,  q9,  #1    // clip = abs((s[-1]-s[0]) / 2)

    vcgt.s16    q10, q9,  #0    // if (clip)
    vand        q14, q14, q10   // vc1_filter_line() return value
    vdup.16     d28, d28[2]
    vdup.16     d29, d29[2]

    vsub.s16    q8,  \a1, q8    // a3-a0
    vmul.s16    q12, q8,  q15   // q12 - d (signed)
    vabs.s16    q8,  q12        // abs(5*(a3-a0))
    vshr.u16    q8,  q8,  #3    // d = abs(5*(a3-a0)/8)

    vshr.s16    \a0, \a0, #15   // a0_sign
    vshr.s16    q13, q13, #15   // clip_sign
    vshr.s16    q12, q12, #15   // d_sign

    veor        q12, q12, \a0   // d_sign ^= a0_sign
    veor        q13, q13, q12   // d_sign ^ clip_sign
    vbic        q14, q14, q13   // if !(d_sign ^ clip_sign)

    vmin.u16    q8,  q8,  q9    // d = FFMIN(d, clip)
    veor        q8,  q8, q13
    vsub.s16    q8,  q8, q13    // restore sign
    vand        q8,  q8, q14

// better way to do 8 bit unsigned +/- 8 bit signed -> 8 bit unsigned?
    vmovl.u8    q1,  d0
    vmovl.u8    q2,  d1
    vsub.s16    q1,  q1,  q8
    vsub.s16    q2,  q2,  q8
    vqmovun.s16 d0,  q1
    vqmovun.s16 d1,  q2
.endm

// void vc1_v_loop_filter8_c(uint8_t *src, int stride, int pq)
function ff_vc1_v_loop_filter8_neon, export=1
    sub         r3, r0, r1, lsl #2      // r3 = src-4*stride
    sub         r0, r0, r1
    vmov.u16    q14, #4
    vmov.u16    q15, #5

    vld1.64     {d2}, [r3,:64], r1
    vld1.64     {d3}, [r3,:64], r1
    vld1.64     {d4}, [r3,:64], r1
    vld1.64     {d0}, [r3,:64], r1
    vld1.64     {d1}, [r3,:64], r1
    vld1.64     {d5}, [r3,:64], r1
    vld1.64     {d6}, [r3,:64], r1
    vld1.64     {d7}, [r3,:64]

    VC1_LOOP_FILTER_A0  q1, d2, d3, d4, d0
    VC1_LOOP_FILTER_A0  q2, d4, d0, d1, d5
    VC1_LOOP_FILTER_A0  q3, d1, d5, d6, d7
    VC1_FILTER q2, q1, q3

    vst1.64     {d0}, [r0,:64], r1
    vst1.64     {d1}, [r0,:64]
    bx          lr
.endfunc
