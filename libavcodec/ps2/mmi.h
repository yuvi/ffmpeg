/*
 * copyright (c) 2002 Leon van Stuivenberg
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

#ifndef FFMPEG_MMI_H
#define FFMPEG_MMI_H

#define align16 __attribute__ ((aligned (16)))

/*
#define r0 $zero
#define r1 $at          //assembler!
#define r2 $v0          //return
#define r3 $v1          //return
#define r4 $a0          //arg
#define r5 $a1          //arg
#define r6 $a2          //arg
#define r7 $a3          //arg
#define r8 $t0          //temp
#define r9 $t1          //temp
#define r10 $t2         //temp
#define r11 $t3         //temp
#define r12 $t4         //temp
#define r13 $t5         //temp
#define r14 $t6         //temp
#define r15 $t7         //temp
#define r16 $s0         //saved temp
#define r17 $s1         //saved temp
#define r18 $s2         //saved temp
#define r19 $s3         //saved temp
#define r20 $s4         //saved temp
#define r21 $s5         //saved temp
#define r22 $s6         //saved temp
#define r23 $s7         //saved temp
#define r24 $t8         //temp
#define r25 $t9         //temp
#define r26 $k0         //kernel
#define r27 $k1         //kernel
#define r28 $gp         //global ptr
#define r29 $sp         //stack ptr
#define r30 $fp         //frame ptr
#define r31 $ra         //return addr
*/


#define         lq(base, off, reg)        \
        asm volatile ("lq " #reg ", %0("#base ")" : : "i" (off) )

#define         lq2(mem, reg)        \
        asm volatile ("lq " #reg ", %0" : : "r" (mem))

#define         sq(reg, off, base)        \
        asm volatile ("sq " #reg ", %0("#base ")" : : "i" (off) )

/*
#define         ld(base, off, reg)        \
        asm volatile ("ld " #reg ", " #off "("#base ")")
*/

#define         ld3(base, off, reg)        \
        asm volatile (".word %0" : : "i" ( 0xdc000000 | (base<<21) | (reg<<16) | (off)))

#define         ldr3(base, off, reg)        \
        asm volatile (".word %0" : : "i" ( 0x6c000000 | (base<<21) | (reg<<16) | (off)))

#define         ldl3(base, off, reg)        \
        asm volatile (".word %0" : : "i" ( 0x68000000 | (base<<21) | (reg<<16) | (off)))

/*
#define         sd(reg, off, base)        \
        asm volatile ("sd " #reg ", " #off "("#base ")")
*/
//seems assembler has bug encoding mnemonic 'sd', so DIY
#define         sd3(reg, off, base)        \
        asm volatile (".word %0" : : "i" ( 0xfc000000 | (base<<21) | (reg<<16) | (off)))

#define         sw(reg, off, base)        \
        asm volatile ("sw " #reg ", " #off "("#base ")")

#define         sq2(reg, mem)        \
        asm volatile ("sq " #reg ", %0" : : "m" (*(mem)))

#define         pinth(rs, rt, rd) \
        asm volatile ("pinth  " #rd ", " #rs ", " #rt )

#define         phmadh(rs, rt, rd) \
        asm volatile ("phmadh " #rd ", " #rs ", " #rt )

#define         pcpyud(rs, rt, rd) \
        asm volatile ("pcpyud " #rd ", " #rs ", " #rt )

#define         pcpyld(rs, rt, rd) \
        asm volatile ("pcpyld " #rd ", " #rs ", " #rt )

#define         pcpyh(rt, rd) \
        asm volatile ("pcpyh  " #rd ", " #rt )

#define         paddw(rs, rt, rd) \
        asm volatile ("paddw  " #rd ", " #rs ", " #rt )

#define         pextlw(rs, rt, rd) \
        asm volatile ("pextlw " #rd ", " #rs ", " #rt )

#define         pextuw(rs, rt, rd) \
        asm volatile ("pextuw " #rd ", " #rs ", " #rt )

#define         pextlh(rs, rt, rd) \
        asm volatile ("pextlh " #rd ", " #rs ", " #rt )

#define         pextuh(rs, rt, rd) \
        asm volatile ("pextuh " #rd ", " #rs ", " #rt )

#define         psubw(rs, rt, rd) \
        asm volatile ("psubw  " #rd ", " #rs ", " #rt )

#define         psraw(rt, sa, rd) \
        asm volatile ("psraw  " #rd ", " #rt ", %0" : : "i"(sa) )

#define         ppach(rs, rt, rd) \
        asm volatile ("ppach  " #rd ", " #rs ", " #rt )

#define         ppacb(rs, rt, rd) \
        asm volatile ("ppacb  " #rd ", " #rs ", " #rt )

#define         prevh(rt, rd) \
        asm volatile ("prevh  " #rd ", " #rt )

#define         pmulth(rs, rt, rd) \
        asm volatile ("pmulth " #rd ", " #rs ", " #rt )

#define         pmaxh(rs, rt, rd) \
        asm volatile ("pmaxh " #rd ", " #rs ", " #rt )

#define         pminh(rs, rt, rd) \
        asm volatile ("pminh " #rd ", " #rs ", " #rt )

#define         pinteh(rs, rt, rd) \
        asm volatile ("pinteh  " #rd ", " #rs ", " #rt )

#define         paddh(rs, rt, rd) \
        asm volatile ("paddh  " #rd ", " #rs ", " #rt )

#define         psubh(rs, rt, rd) \
        asm volatile ("psubh  " #rd ", " #rs ", " #rt )

#define         psrah(rt, sa, rd) \
        asm volatile ("psrah  " #rd ", " #rt ", %0" : : "i"(sa) )

#define         pmfhl_uw(rd) \
        asm volatile ("pmfhl.uw  " #rd)

#define         pextlb(rs, rt, rd) \
        asm volatile ("pextlb  " #rd ", " #rs ", " #rt )

#endif /* FFMPEG_MMI_H */
