;******************************************************************************
;* MMX optimized DSP utils
;* Copyright (c) 2000, 2001 Fabrice Bellard
;* Copyright (c) 2002-2004 Michael Niedermayer <michaelni@gmx.at>
;*
;* This file is part of FFmpeg.
;*
;* FFmpeg is free software; you can redistribute it and/or
;* modify it under the terms of the GNU Lesser General Public
;* License as published by the Free Software Foundation; either
;* version 2.1 of the License, or (at your option) any later version.
;*
;* FFmpeg is distributed in the hope that it will be useful,
;* but WITHOUT ANY WARRANTY; without even the implied warranty of
;* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
;* Lesser General Public License for more details.
;*
;* You should have received a copy of the GNU Lesser General Public
;* License along with FFmpeg; if not, write to the Free Software
;* 51, Inc., Foundation Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA
;******************************************************************************

%include "x86inc.asm"

SECTION_RODATA

pb_128: times 8 db 0x80

SECTION .text
INIT_MMX

%macro PUT_PIXELS_CLAMPED_MMX 1
    movq        m0, [r0+%1]
    movq        m1, [r0+%1+16]
    movq        m2, [r0+%1+32]
    movq        m3, [r0+%1+48]
    packuswb    m0, [r0+%1+8]
    packuswb    m1, [r0+%1+24]
    packuswb    m2, [r0+%1+40]
    packuswb    m3, [r0+%1+56]
    movq      [r1], m0
    movq   [r1+r2], m1
    movq [r1+2*r2], m2
    movq   [r1+r4], m3
%endm

;void put_pixels_clamped_mmx(const DCTELEM *block, uint8_t *pixels, int line_size)
cglobal put_pixels_clamped_mmx, 3,4,4
    lea r4, [r2+2*r2]
    PUT_PIXELS_CLAMPED_MMX 0
    lea r1, [r1+4*r2]
    PUT_PIXELS_CLAMPED_MMX 64
    RET

%macro PUT_SIGNED_PIXELS_CLAMPED_MMX 1
    movq        m0, [r0+%1]
    movq        m1, [r0+%1+16]
    movq        m2, [r0+%1+32]
    movq        m3, [r0+%1+48]
    packuswb    m0, [r0+%1+8]
    packuswb    m1, [r0+%1+24]
    packuswb    m2, [r0+%1+40]
    packuswb    m3, [r0+%1+56]
    paddb       m0, m4
    paddb       m1, m4
    paddb       m2, m4
    paddb       m3, m4
    movq      [r1], m0
    movq   [r1+r2], m1
    movq [r1+2*r2], m2
    movq   [r1+r4], m3
%endm

;void put_signed_pixels_clamped_mmx(const DCTELEM *block, uint8_t *pixels, int line_size)
cglobal put_signed_pixels_clamped_mmx, 3,4,5
    movq        m4, [pb_128 GLOBAL]
    lea         r3, [r2+2*r2]
    PUT_SIGNED_PIXELS_CLAMPED_MMX 0
    lea         r1, [r1+4*r2]
    PUT_SIGNED_PIXELS_CLAMPED_MMX 64
    RET

;void add_pixels_clamped_mmx(const DCTELEM *block, uint8_t *pixels, int line_size)
cglobal add_pixels_clamped_mmx, 3,4,8
    pxor        m7, m7
    mov         r3, 4
.loop:
    movq        m0, [r0]
    movq        m1, [r0+8]
    movq        m2, [r0+16]
    movq        m3, [r0+24]
    movq        m4, [r1]
    movq        m6, [r1+r2]
    movq        m5, m4
    punpcklbw   m4, m7
    punpckhbw   m5, m7
    paddsw      m0, m4
    paddsw      m1, m5
    movq        m5, m6
    punpcklbw   m6, m7
    punpckhbw   m5, m7
    paddsw      m2, m6
    paddsw      m3, m5
    packuswb    m0, m1
    packuswb    m2, m3
    movq      [r1], m0
    movq   [r1+r2], m2
    add         r0, 32
    lea         r1, [r1+2*r2]
    dec         r3d
    jg .loop
    RET

; %1 = prefix
; %2 = width
; %3 = extension
%macro PUT_PIX 2
%if %2 == mmsize/2
    %define load movh
    %define store movh
%else
    %define load movu
    %define store mova
%endif

cglobal %1_pixels%2_%3, 4,5,4
    lea         r4, [r2+2*r2]
.loop:
    load        m0, [r1]
    load        m1, [r1+r2]
    load        m2, [r1+2*r2]
    load        m3, [r1+r4]
    PAVGB       m0, [r0]
    PAVGB       m1, [r0+r2]
    PAVGB       m2, [r0+2*r2]
    PAVGB       m3, [r0+r4]
    store      [r0], m0
    store   [r0+r2], m1
    store [r0+2*r2], m2
    store   [r0+r4], m3
%if %2 == 2*mmsize
    load        m0, [r1+8]
    load        m1, [r1+r2+8]
    load        m2, [r1+2*r2+8]
    load        m3, [r1+r4+8]
    PAVGB       m0, [r0+8]
    PAVGB       m1, [r0+r2+8]
    PAVGB       m2, [r0+2*r2+8]
    PAVGB       m3, [r0+r4+8]
    store      [r0+8], m4
    store   [r0+r2+8], m5
    store [r0+2*r2+8], m6
    store   [r0+r4+8], m7
%endif
    lea         r1, [r1+4*r2]
    lea         r0, [r0+4*r2]
    sub         r3d, 4
    jg .loop
    RET
%endm

%define PAVGB NOP
PUT_PIX put,  4, mmx
PUT_PIX put,  8, mmx
PUT_PIX put, 16, mmx

INIT_XMM
PUT_PIX put, 16, sse2

%define PAVGB pavgb
INIT_MMX
PUT_PIX avg,  4, mmx2
PUT_PIX avg,  8, mmx2
PUT_PIX avg, 16, mmx2

INIT_XMM
PUT_PIX avg, 16, sse2
