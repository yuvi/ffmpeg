;******************************************************************************
;* MMX optimized DSP utils
;* Copyright (c) 2008 Loren Merritt
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
%include "x86util.asm"

SECTION_RODATA

pw_8: times 8 dw 8
C1: times 8 dw 64277
C2: times 8 dw 60547
C3: times 8 dw 54491
C4: times 8 dw 46341
C5: times 8 dw 36410
C6: times 8 dw 25080
C7: times 8 dw 12785

SECTION .text

%macro M16 3
    mova    %3, %1
    pmulhw  %1, %2
    paddw   %1, %3
%endmacro

%macro VP3_IDCT10 1
%ifdef ARCH_X86_64
    %define xC4 m9
%else
    %define xC4 [C4 GLOBAL]
%endif
    mova        m7, m1
    M16         m1, [C1 GLOBAL], m4 ; A
    pmulhw      m7, [C7 GLOBAL]     ; B

    mova        m5, m3
    M16         m3, [C3 GLOBAL], m4 ; C
    M16         m5, [C5 GLOBAL], m4 ;-D

    M16         m0, xC4, m4         ; E
%if %1
    paddw       m0, [pw_8 GLOBAL]
%endif

    mova        m6, m2
    M16         m2, [C2 GLOBAL], m4 ; G
    pmulhw      m6, [C6 GLOBAL]     ; H

    SUMSUB_BADC m3, m1, m7, m5, m4  ; m3 = Cd, m5 = Dd
    M16         m1, xC4, m4         ; Ad
    M16         m7, xC4, m4         ; Bd

    SUMSUB_BA   m6, m7, m4          ; Hd, Bdd
    mova        m4, m0              ; F

%ifdef ARCH_X86_64
    SUMSUB_BADC m2, m0, m1, m4, m8  ; Gd, Ed, Add, Fd
    SUMSUB_BADC m3, m2, m6, m1, m8  ; 0, 7, 1, 2
    SUMSUB_BADC m5, m0, m7, m4, m8  ; 3, 4, 5, 6
%else
    SUMSUB_BADC m2, m0, m1, m4      ; Gd, Ed, Add, Fd
    SUMSUB_BADC m3, m2, m6, m1      ; 0, 7, 1, 2
    SUMSUB_BADC m5, m0, m7, m4      ; 3, 4, 5, 6
%endif
    SWAP 0,3
    SWAP 1,6
    SWAP 2,6
    SWAP 4,5
    SWAP 5,7
    SWAP 6,7
%endmacro

%macro VP3_STORE_DIFF 4
    movh       %2, %4
    punpcklbw  %2, %3
    psraw      %1, 4
    paddsw     %1, %2
    packuswb   %1, %1
    movh       %4, %1
%endmacro

INIT_XMM
cglobal vp3_idct10_add_sse2, 3,4,10, dst, stride, block, stridex3
    mova  m0, [r2+ 0]
    mova  m1, [r2+16]
    mova  m2, [r2+32]
    mova  m3, [r2+48]
%ifdef ARCH_X86_64
    mova  m9, [C4 GLOBAL]
%endif

    VP3_IDCT10 0
    TRANSPOSE8x4W  0,1,2,3,4,5,6,7
    VP3_IDCT10 1

    pxor  m8, m8
    lea   r3, [r1+2*r1]

    VP3_STORE_DIFF  m0, m9, m8, [r0]
    VP3_STORE_DIFF  m1, m0, m8, [r0+r1]
    VP3_STORE_DIFF  m2, m0, m8, [r0+2*r1]
    VP3_STORE_DIFF  m3, m0, m8, [r0+r3]
    lea   r0, [r0+4*r1]
    VP3_STORE_DIFF  m4, m0, m8, [r0]
    VP3_STORE_DIFF  m5, m0, m8, [r0+r1]
    VP3_STORE_DIFF  m6, m0, m8, [r0+2*r1]
    VP3_STORE_DIFF  m7, m0, m8, [r0+r3]
    RET
