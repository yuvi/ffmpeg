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
pw_3: times 8 dw 3
pw_7: times 8 dw 7
pw_16: times 8 dw 16

section .text

%macro UNPACK_ADD 6
    mov%5   %1, %3
    mov%6   m5, %4
    mova    m4, %1
    mova    %2, m5
    punpcklbw %1, m7
    punpcklbw m5, m7
    punpckhbw m4, m7
    punpckhbw %2, m7
    paddw   %1, m5
    paddw   %2, m4
%endmacro

%macro HPEL_FILTER 1
; dirac_hpel_filter_v_sse2(uint8_t *dst, uint8_t *src, int stride, int width);
cglobal dirac_hpel_filter_v_%1, 4,6,8, dst, src, stride, width, src0, stridex3
    mov     src0q, srcq
    lea     stridex3q, [3*strideq]
    sub     src0q, stridex3q
    pxor    m7, m7
.loop:
    ; 7*(src[0] + src[1])
    UNPACK_ADD m0, m1, [srcq], [srcq + strideq], a,a
    pmullw  m0, [pw_7 GLOBAL]
    pmullw  m1, [pw_7 GLOBAL]

    ; 3*( ... + src[-2] + src[3])
    UNPACK_ADD m2, m3, [src0q + strideq], [srcq + stridex3q], a,a
    paddw   m0, m2
    paddw   m1, m3
    pmullw  m0, [pw_3 GLOBAL]
    pmullw  m1, [pw_3 GLOBAL]

    ; ... - 7*(src[-1] + src[2])
    UNPACK_ADD m2, m3, [src0q + strideq*2], [srcq + strideq*2], a,a
    pmullw  m2, [pw_7 GLOBAL]
    pmullw  m3, [pw_7 GLOBAL]
    psubw   m0, m2
    psubw   m1, m3

    ; ... - (src[-3] + src[4])
    UNPACK_ADD m2, m3, [src0q], [srcq + strideq*4], a,a
    psubw   m0, m2
    psubw   m1, m3

    paddw   m0, [pw_16 GLOBAL]
    paddw   m1, [pw_16 GLOBAL]
    psraw   m0, 5
    psraw   m1, 5
    packuswb m0, m1
    mova    [dstq], m0
    add     dstq, mmsize
    add     srcq, mmsize
    add     src0q, mmsize
    sub     widthd, mmsize
    jg      .loop
    RET

; dirac_hpel_filter_h_sse2(uint8_t *dst, uint8_t *src, int width);
cglobal dirac_hpel_filter_h_%1, 3,3,8, dst, src, width
    dec     widthd
    pxor    m7, m7
    and     widthd, ~(mmsize-1)
.loop:
    ; 7*(src[0] + src[1])
    UNPACK_ADD m0, m1, [srcq + widthq], [srcq + widthq + 1], a,u
    pmullw  m0, [pw_7 GLOBAL]
    pmullw  m1, [pw_7 GLOBAL]

    ; 3*( ... + src[-2] + src[3])
    UNPACK_ADD m2, m3, [srcq + widthq - 2], [srcq + widthq + 3], u,u
    paddw   m0, m2
    paddw   m1, m3
    pmullw  m0, [pw_3 GLOBAL]
    pmullw  m1, [pw_3 GLOBAL]

    ; ... - 7*(src[-1] + src[2])
    UNPACK_ADD m2, m3, [srcq + widthq - 1], [srcq + widthq + 2], u,u
    pmullw  m2, [pw_7 GLOBAL]
    pmullw  m3, [pw_7 GLOBAL]
    psubw   m0, m2
    psubw   m1, m3

    ; ... - (src[-3] + src[4])
    UNPACK_ADD m2, m3, [srcq + widthq - 3], [srcq + widthq + 4], u,u
    psubw   m0, m2
    psubw   m1, m3

    paddw   m0, [pw_16 GLOBAL]
    paddw   m1, [pw_16 GLOBAL]
    psraw   m0, 5
    psraw   m1, 5
    packuswb m0, m1
    mova    [dstq + widthq], m0
    sub     widthd, mmsize
    jge     .loop
    RET
%endmacro


%ifndef ARCH_X86_64
INIT_MMX
HPEL_FILTER mmx
%endif

INIT_XMM
HPEL_FILTER sse2