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

#include "asm.S"

.text

// ((p0 - p3) + 3*(p2 - p1) + 4) >> 3 has range -127, 128
// so we instead calculate the negative, so it fits in signed 8 bits

function ff_vp3_v_loop_filter_armv6, export=1
    push    {r4-r10,lr}
    sub     r0,  r0,  r1
    ldr     r2,  [r2, #129*4]
    ldr     r3,  =0x030003
    ldr     ip,  =0xff00ff
    mov     lr,  #8
1:
    ldr     r4,  [r0,-r1]
    ldr     r5,  [r0]
    ldr     r6,  [r0, r1]
    ldr     r7,  [r0, r1, lsl #1]
    subs    lr,  lr,  #4

    uxtb16  r10, r4
    uxtb16  r8,  r5
    uxtb16  r9,  r6
    ssub16  r8,  r8,  r9
    sadd16  r9,  r8,  r8
    sadd16  r8,  r8,  r9            // 3*(p1-p2)
    uxtb16  r9,  r7
    ssub16  r9,  r9,  r10
    sadd16  r8,  r8,  r9
    sadd16  r8,  r8,  r3            // (p3 - p0) + 3*(p1 - p2) + 3
    uxtb16  r9,  r5,  ror #8
    and     r8,  ip,  r8, lsr #3

    uxtb16  r10, r6,  ror #8
    ssub16  r9,  r9,  r10
    sadd16  r10, r9,  r9
    sadd16  r9,  r9,  r10
    uxtb16  r4,  r4,  ror #8
    uxtb16  r7,  r7,  ror #8
    ssub16  r4,  r7,  r4
    sadd16  r4,  r4,  r9
    sadd16  r4,  r4,  r3
    mov     r10, #0
    and     r4,  ip,  r4, lsr #3
    orr     r8,  r8,  r4, lsl #8    // 4 filter values

    ssub8   r9,  r10, r8
    sel     r9,  r9,  r8            // abs(filter_value)
    uqsub8  r7,  r2,  r9
    uqsub8  r9,  r7,  r9
    usub8   r9,  r7,  r9
// add and sub, then select the right one based on the original sign
    uqsub8  r7,  r6,  r9
    uqadd8  r6,  r6,  r9
    uqadd8  r4,  r5,  r9
    uqsub8  r5,  r5,  r9
    ssub8   r10, r8,  r10
    sel     r6,  r6,  r7
    sel     r5,  r5,  r4
    str     r6,  [r0, r1]
    str     r5,  [r0], #4
    bgt     1b
    pop     {r4-r10,pc}
endfunc

function ff_vp3_h_loop_filter_armv6, export=1
    push    {r4-r10,lr}
    sub     ip,  r0,  #1
    sub     r0,  r0,  #2
    ldr     r3,  =0x010003
    mov     r10, #4
    mov     lr,  #8
1:
    ldr     r4,  [r0], r1
    ldr     r7,  [r0], r1
    subs    lr,  lr,  #2

    uxtb16  r5,  r4,  ror #8
    uxtb16  r6,  r4,  ror #16
    ssub16  r6,  r6,  r5        // p0-p3  p2-p1
    smlad   r6,  r6,  r3,  r10  // (p0-p3) + 3*(p2-p1) + 4

    uxtb16  r8,  r7,  ror #8
    uxtb16  r9,  r7,  ror #16
    ssub16  r9,  r9,  r8
    smlad   r9,  r9,  r3,  r10

.macro filter p1 p2 r
    asr     \r,  \r,  #3
    ldr     \r,  [r2, \r, lsl #2] // filter value
    uxtb    \p2, \p1, ror #16     // p2
    uxtb    \p1, \p1, ror #8      // p1
    sub     \p2, \p2, \r
    add     \p1, \p1, \r
    usat    \p2, #8,  \p2
    usat    \p1, #8,  \p1
    strb    \p2, [ip, #1]
    strb    \p1, [ip], r1
.endm

    filter  r4,  r5,  r6
    filter  r7,  r8,  r9
    bgt     1b
    pop     {r4-r10,pc}
endfunc
