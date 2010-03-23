;******************************************************************************
;* MMX optimized discrete wavelet trasnform
;* Copyright (c) 2010 David Conrad
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
pw_1: times 8 dw 1
pw_2: times 8 dw 2
pw_8: times 8 dw 8
pw_16: times 8 dw 16
pw_1991: times 4 dw 9,-1

section .text

; %1 -= (%2 + %3 + 2)>>2     %4 is pw_2
%macro COMPOSE_53iL0 4
    paddw   %2, %3
    paddw   %2, %4
    psraw   %2, 2
    psubw   %1, %2
%endm

; m1 = %1 + (-m0 + 9*m1 + 9*%2 -%3 + 8)>>4
; if %4 is supplied, %1 is loaded from there
; m2: clobbered  m3: pw_8  m4: pw_1991
%macro COMPOSE_DD97iH0 3-4
    paddw   m0, %3
    paddw   m1, %2
    psubw   m0, m3
    mova    m2, m1
    punpcklwd m1, m0
    punpckhwd m2, m0
    pmaddwd m1, m4
    pmaddwd m2, m4
%if %0 > 3
    mova    %1, %4
%endif
    psrad   m1, 4
    psrad   m2, 4
    packssdw m1, m2
    paddw   m1, %1
%endm

%macro COMPOSE_VERTICAL 1
; void vertical_compose53iL0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
;                                  int width)
cglobal vertical_compose53iL0_%1, 4,4,1, b0, b1, b2, width
    mova    m2, [pw_2 GLOBAL]
.loop:
    sub     widthd, mmsize/2
    mova    m1, [b0q+2*widthq]
    mova    m0, [b1q+2*widthq]
    COMPOSE_53iL0 m0, m1, [b2q+2*widthq], m2
    mova    [b1q+2*widthq], m0
    jg      .loop
    REP_RET

; void vertical_compose_dirac53iH0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
;                                  int width)
cglobal vertical_compose_dirac53iH0_%1, 4,4,1, b0, b1, b2, width
    mova    m1, [pw_1 GLOBAL]
.loop:
    sub     widthd, mmsize/2
    mova    m0, [b0q+2*widthq]
    paddw   m0, [b2q+2*widthq]
    paddw   m0, m1
    psraw   m0, 1
    paddw   m0, [b1q+2*widthq]
    mova    [b1q+2*widthq], m0
    jg      .loop
    REP_RET

; void vertical_compose_dd97iH0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
;                               IDWTELEM *b3, IDWTELEM *b4, int width)
cglobal vertical_compose_dd97iH0_%1, 6,6,5, b0, b1, b2, b3, b4, width
    mova    m3, [pw_8 GLOBAL]
    mova    m4, [pw_1991 GLOBAL]
.loop:
    sub     widthd, mmsize/2
    mova    m0, [b0q+2*widthq]
    mova    m1, [b1q+2*widthq]
    COMPOSE_DD97iH0 [b2q+2*widthq], [b3q+2*widthq], [b4q+2*widthq]
    mova    [b2q+2*widthq], m1
    jg      .loop
    REP_RET

; void vertical_compose_dd137iL0(IDWTELEM *b0, IDWTELEM *b1, IDWTELEM *b2,
;                                IDWTELEM *b3, IDWTELEM *b4, int width)
cglobal vertical_compose_dd137iL0_%1, 6,6,6, b0, b1, b2, b3, b4, width
    mova    m3, [pw_16 GLOBAL]
    mova    m4, [pw_1991 GLOBAL]
.loop:
    sub     widthd, mmsize/2
    mova    m0, [b0q+2*widthq]
    mova    m1, [b1q+2*widthq]
    mova    m5, [b2q+2*widthq]
    paddw   m0, [b4q+2*widthq]
    paddw   m1, [b3q+2*widthq]
    psubw   m0, m3
    mova    m2, m1
    punpcklwd m1, m0
    punpckhwd m2, m0
    pmaddwd m1, m4
    pmaddwd m2, m4
    psrad   m1, 5
    psrad   m2, 5
    packssdw m1, m2
    psubw   m5, m1
    mova    [b2q+2*widthq], m5
    jg      .loop
    REP_RET

; void vertical_compose_haariL0(IDWTELEM *b0, IDWTELEM *b1, int width)
cglobal vertical_compose_haariL0_%1, 3,3,3, b0, b1, width
    mova    m2, [pw_1 GLOBAL]
.loop:
    sub     widthd, mmsize/2
    mova    m1, [b1q+2*widthq]
    mova    m0, [b0q+2*widthq]
    paddw   m1, m2
    psraw   m1, 1
    psubw   m0, m1
    mova    [b0q+2*widthq], m0
    jg      .loop
    REP_RET

; void vertical_compose_haariH0(IDWTELEM *b0, IDWTELEM *b1, int width)
cglobal vertical_compose_haariH0_%1, 3,3,1, b0, b1, width
.loop:
    sub     widthd, mmsize/2
    mova    m0, [b0q+2*widthq]
    paddw   m0, [b1q+2*widthq]
    mova    [b0q+2*widthq], m0
    jg      .loop
    REP_RET
%endmacro ; COMPOSE_VERTICAL

INIT_MMX
COMPOSE_VERTICAL mmx

INIT_XMM
COMPOSE_VERTICAL sse2


; void horizontal_compose_dd97i(IDWTELEM *b, int width, IDWTELEM *tmp)
cglobal horizontal_compose_dd97i_ssse3, 3,6,8, b, width, tmp, b_w2, w2, x
    mov    w2d, widthd
    xor     xd, xd
    sar    w2d, 1
    lea  b_w2q, [bq+widthq]
    movu    m4, [bq+widthq]
    mova    m7, [pw_2 GLOBAL]
    pslldq  m4, 14
.lowpass_loop:
    movu    m1, [b_w2q + 2*xq]
    mova    m0, [bq    + 2*xq]
    mova    m2, m1
    palignr m1, m4, 14
    mova    m4, m2
    COMPOSE_53iL0 m0, m1, m2, m7
    mova    [tmpq + 2*xq], m0
    add     xd, mmsize/2
    cmp     xd, w2d
    jl      .lowpass_loop

    ; extend the edges
    mov     xw, [tmpq]
    mov     [tmpq-2], xw
    mov     xw, [tmpq+2*w2q-2]
    mov     [tmpq+2*w2q  ], xw
    mov     [tmpq+2*w2q+2], xw

    xor     xd, xd
    and    w2d, ~(mmsize/2 - 1)
    cmp    w2d, mmsize/2
    jl      .end

    mova    m7, [tmpq-16]
    mova    m0, [tmpq]
    mova    m5, [pw_1 GLOBAL]
    mova    m3, [pw_8 GLOBAL]
    mova    m4, [pw_1991 GLOBAL]
.highpass_loop:
    mova    m6, m0
    palignr m0, m7, 14
    mova    m7, [tmpq + 2*xq + 16]
    mova    m1, m7
    mova    m2, m7
    palignr m1, m6, 2
    palignr m2, m6, 4
    COMPOSE_DD97iH0 m0, m6, m2, [b_w2q + 2*xq]
    mova    m0, m7
    mova    m7, m6

    ; shift and interleave
    paddw   m6, m5
    paddw   m1, m5
    psraw   m6, 1
    psraw   m1, 1
    mova    m2, m6
    punpcklwd m6, m1
    punpckhwd m2, m1
    mova    [bq+4*xq   ], m6
    mova    [bq+4*xq+16], m2

    add     xd, mmsize/2
    cmp     xd, w2d
    jl      .highpass_loop
.end:
    mov     eax, xd
    RET
