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
pb_f: times 16 db 15
pb_zzzzzzzz77777777: times 8 db -1
pb_7: times 8 db 7
pb_zzzz3333zzzzbbbb: db -1,-1,-1,-1,3,3,3,3,-1,-1,-1,-1,11,11,11,11
pb_zz11zz55zz99zzdd: db -1,-1,1,1,-1,-1,5,5,-1,-1,9,9,-1,-1,13,13

pb_1: times 8 db 1
pb_128: times 8 db 0x80
pb_fe: times 8 db 0xfe

section .text align=16

%macro PSWAPD_SSE 2
    pshufw %1, %2, 0x4e
%endmacro
%macro PSWAPD_3DN1 2
    movq  %1, %2
    psrlq %1, 32
    punpckldq %1, %2
%endmacro

%macro FLOAT_TO_INT16_INTERLEAVE6 1
; void ff_float_to_int16_interleave6_sse(int16_t *dst, const float **src, int len)
cglobal float_to_int16_interleave6_%1, 2,7,0, dst, src, src1, src2, src3, src4, src5
%ifdef ARCH_X86_64
    %define lend r10d
    mov     lend, r2d
%else
    %define lend dword r2m
%endif
    mov src1q, [srcq+1*gprsize]
    mov src2q, [srcq+2*gprsize]
    mov src3q, [srcq+3*gprsize]
    mov src4q, [srcq+4*gprsize]
    mov src5q, [srcq+5*gprsize]
    mov srcq,  [srcq]
    sub src1q, srcq
    sub src2q, srcq
    sub src3q, srcq
    sub src4q, srcq
    sub src5q, srcq
.loop:
    cvtps2pi   mm0, [srcq]
    cvtps2pi   mm1, [srcq+src1q]
    cvtps2pi   mm2, [srcq+src2q]
    cvtps2pi   mm3, [srcq+src3q]
    cvtps2pi   mm4, [srcq+src4q]
    cvtps2pi   mm5, [srcq+src5q]
    packssdw   mm0, mm3
    packssdw   mm1, mm4
    packssdw   mm2, mm5
    pswapd     mm3, mm0
    punpcklwd  mm0, mm1
    punpckhwd  mm1, mm2
    punpcklwd  mm2, mm3
    pswapd     mm3, mm0
    punpckldq  mm0, mm2
    punpckhdq  mm2, mm1
    punpckldq  mm1, mm3
    movq [dstq   ], mm0
    movq [dstq+16], mm2
    movq [dstq+ 8], mm1
    add srcq, 8
    add dstq, 24
    sub lend, 2
    jg .loop
    emms
    RET
%endmacro ; FLOAT_TO_INT16_INTERLEAVE6

%define pswapd PSWAPD_SSE
FLOAT_TO_INT16_INTERLEAVE6 sse
%define cvtps2pi pf2id
%define pswapd PSWAPD_3DN1
FLOAT_TO_INT16_INTERLEAVE6 3dnow
%undef pswapd
FLOAT_TO_INT16_INTERLEAVE6 3dn2
%undef cvtps2pi



; void ff_add_hfyu_median_prediction_mmx2(uint8_t *dst, const uint8_t *top, const uint8_t *diff, int w, int *left, int *left_top)
cglobal add_hfyu_median_prediction_mmx2, 6,6,0, dst, top, diff, w, left, left_top
    movq    mm0, [topq]
    movq    mm2, mm0
    movd    mm4, [left_topq]
    psllq   mm2, 8
    movq    mm1, mm0
    por     mm4, mm2
    movd    mm3, [leftq]
    psubb   mm0, mm4 ; t-tl
    add    dstq, wq
    add    topq, wq
    add   diffq, wq
    neg      wq
    jmp .skip
.loop:
    movq    mm4, [topq+wq]
    movq    mm0, mm4
    psllq   mm4, 8
    por     mm4, mm1
    movq    mm1, mm0 ; t
    psubb   mm0, mm4 ; t-tl
.skip:
    movq    mm2, [diffq+wq]
%assign i 0
%rep 8
    movq    mm4, mm0
    paddb   mm4, mm3 ; t-tl+l
    movq    mm5, mm3
    pmaxub  mm3, mm1
    pminub  mm5, mm1
    pminub  mm3, mm4
    pmaxub  mm3, mm5 ; median
    paddb   mm3, mm2 ; +residual
%if i==0
    movq    mm7, mm3
    psllq   mm7, 56
%else
    movq    mm6, mm3
    psrlq   mm7, 8
    psllq   mm6, 56
    por     mm7, mm6
%endif
%if i<7
    psrlq   mm0, 8
    psrlq   mm1, 8
    psrlq   mm2, 8
%endif
%assign i i+1
%endrep
    movq [dstq+wq], mm7
    add      wq, 8
    jl .loop
    movzx   r2d, byte [dstq-1]
    mov [leftq], r2d
    movzx   r2d, byte [topq-1]
    mov [left_topq], r2d
    RET


%macro ADD_HFYU_LEFT_LOOP 1 ; %1 = is_aligned
    add     srcq, wq
    add     dstq, wq
    neg     wq
%%.loop:
    mova    m1, [srcq+wq]
    mova    m2, m1
    psllw   m1, 8
    paddb   m1, m2
    mova    m2, m1
    pshufb  m1, m3
    paddb   m1, m2
    pshufb  m0, m5
    mova    m2, m1
    pshufb  m1, m4
    paddb   m1, m2
%if mmsize == 16
    mova    m2, m1
    pshufb  m1, m6
    paddb   m1, m2
%endif
    paddb   m0, m1
%if %1
    mova    [dstq+wq], m0
%else
    movq    [dstq+wq], m0
    movhps  [dstq+wq+8], m0
%endif
    add     wq, mmsize
    jl %%.loop
    mov     eax, mmsize-1
    sub     eax, wd
    movd    m1, eax
    pshufb  m0, m1
    movd    eax, m0
    RET
%endmacro

; int ff_add_hfyu_left_prediction(uint8_t *dst, const uint8_t *src, int w, int left)
INIT_MMX
cglobal add_hfyu_left_prediction_ssse3, 3,3,7, dst, src, w, left
.skip_prologue:
    mova    m5, [pb_7 GLOBAL]
    mova    m4, [pb_zzzz3333zzzzbbbb GLOBAL]
    mova    m3, [pb_zz11zz55zz99zzdd GLOBAL]
    movd    m0, leftm
    psllq   m0, 56
    ADD_HFYU_LEFT_LOOP 1

INIT_XMM
cglobal add_hfyu_left_prediction_sse4, 3,3,7, dst, src, w, left
    mova    m5, [pb_f GLOBAL]
    mova    m6, [pb_zzzzzzzz77777777 GLOBAL]
    mova    m4, [pb_zzzz3333zzzzbbbb GLOBAL]
    mova    m3, [pb_zz11zz55zz99zzdd GLOBAL]
    movd    m0, leftm
    pslldq  m0, 15
    test    srcq, 15
    jnz add_hfyu_left_prediction_ssse3.skip_prologue
    test    dstq, 15
    jnz .unaligned
    ADD_HFYU_LEFT_LOOP 1
.unaligned:
    ADD_HFYU_LEFT_LOOP 0


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

INIT_MMX
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
%macro PIXOP_FPEL 3
%if %2 < mmsize
    %define load movh
    %define store movh
%else
    %define load movu
    %define store mova
%endif

;void put_pixels(uint8_t *block, const uint8_t *pixels, int line_size, int h)
cglobal %1_pixels%2_%3, 4,5,5
    lea         r4, [r2+2*r2]
.loop:
%assign off 0
%rep (%2+4) / mmsize    ; 2x for width 16 on mmx, 1x otherwise
    load        m0, [r1      +off]
    load        m1, [r1+  r2 +off]
    load        m2, [r1+2*r2 +off]
    load        m3, [r1+  r4 +off]
    OP          m0, [r0      +off], m4
    OP          m1, [r0+  r2 +off], m4
    OP          m2, [r0+2*r2 +off], m4
    OP          m3, [r0+  r4 +off], m4
    store      [r0+off], m0
    store   [r0+r2+off], m1
    store [r0+2*r2+off], m2
    store   [r0+r4+off], m3
%assign off off+mmsize
%endrep
    lea         r1, [r1+4*r2]
    lea         r0, [r0+4*r2]
    sub         r3d, 4
    jg .loop
    RET
%endm

%macro PIXOP_HPEL 3
%if %2 < mmsize
    %define load movh
    %define store movh
%else
    %define load movu
    %define store mova
%endif

cglobal %1_pixels%2_x2_%3, 4,5,5
    lea         r4, [r2+2*r2]
.loop:
%assign off 0
%rep (%2+4) / mmsize    ; 2x for width 16 on mmx, 1x otherwise
    load        m0, [r1      +off]
    load        m1, [r1+  r2 +off]
    load        m2, [r1+2*r2 +off]
    load        m3, [r1+  r4 +off]
    PAVGB       m0, [r1      +off+1], m4
    PAVGB       m1, [r1+  r2 +off+1], m4
    PAVGB       m2, [r1+2*r2 +off+1], m4
    PAVGB       m3, [r1+  r4 +off+1], m4
    OP          m0, [r0      +off], m4
    OP          m1, [r0+  r2 +off], m4
    OP          m2, [r0+2*r2 +off], m4
    OP          m3, [r0+  r4 +off], m4
    store      [r0+off], m0
    store   [r0+r2+off], m1
    store [r0+2*r2+off], m2
    store   [r0+r4+off], m3
%assign off off+mmsize
%endrep
    lea         r1, [r1+4*r2]
    lea         r0, [r0+4*r2]
    sub         r3d, 4
    jg .loop
    RET

cglobal %1_pixels%2_y2_%3, 4,5,6
    lea         r4, [r2+2*r2]
    load        m0, [r1]
%if %2 > mmsize
    load        m3, [r1+8]
%endif
.loop:
    load        m1, [r1+r2]
    load        m2, [r1+2*r2]
    PAVGB       m0, m1, m4
    PAVGB       m1, m2, m4
    OP          m0, [r0], m4
    OP          m1, [r0+r2], m4
    store     [r0], m0
    store  [r0+r2], m1
    load        m1, [r1+r4]
    load        m0, [r1+4*r2]
    PAVGB       m2, m1, m4
    PAVGB       m1, m0, m4
    OP          m2, [r0+2*r2], m4
    OP          m1, [r0+r4], m4
    store [r0+2*r2], m2
    store   [r0+r4], m1
%if %2 > mmsize
    load        m4, [r1+r2+8]
    load        m5, [r1+2*r2+8]
    PAVGB       m3, m4, m2
    PAVGB       m4, m5, m2
    OP          m3, [r0+8], m2
    OP          m4, [r0+r2+8], m2
    store     [r0+8], m3
    store  [r0+r2+8], m4
    load        m4, [r1+r4+8]
    load        m3, [r1+4*r2+8]
    PAVGB       m5, m4, m2
    PAVGB       m4, m3, m2
    OP          m5, [r0+2*r2+8], m2
    OP          m4, [r0+r4+8], m2
    store [r0+2*r2+8], m5
    store   [r0+r4+8], m4
%endif
    lea         r1, [r1+4*r2]
    lea         r0, [r0+4*r2]
    sub         r3d, 4
    jg .loop
    RET
%endm

; PIXOP3 - only interpolated hpel positions
; %1 = prefix
; %2 = extension
%macro PIXOP3 2
    PIXOP_HPEL %1, 16, %2
    PIXOP_HPEL %1, 8, %2
    PIXOP_HPEL %1, 4, %2
%endm

; PIXOP4 - all pixel functions
%macro PIXOP4 2
    PIXOP_FPEL %1, 16, %2
    PIXOP_FPEL %1, 8, %2
    PIXOP_FPEL %1, 4, %2
    PIXOP3     %1, %2
%endm

%macro NOP 1+
%endmacro

; TODO: make use of the 4th reg
%macro PAVGB_MMX_NO_RND 3-4
    mova    %3, %1
    pand    %1, %2
    pxor    %3, %2
%if %0 == 4
    pand    %3, %4
%else
    pand    %3, [pb_fe GLOBAL]
%endif
    psrlq   %3, 1
    paddb   %1, %3
%endmacro

%macro PAVGB_MMX 3-4
    mova    %3, %1
    por     %1, %2
    pxor    %3, %2
%if %0 == 4
    pand    %3, %4
%else
    pand    %3, [pb_fe GLOBAL]
%endif
    psrlq   %3, 1
    psubb   %1, %3
%endmacro

; this does incorrect rounding if %1 is 0
%macro PAVGB_MMX2_NO_RND 2-4
%if %0 == 4
    psubb   %1, %4
%else
    psubb   %1, [pb_1 GLOBAL]
%endif
    pavgb   %1, %2
%endmacro

%macro PAVGB_MMX2 2-4
    pavgb   %1, %2
%endmacro

%macro PAVGB_3DNOW 2-4
    pavgusb %1, %2
%endmacro


%define PAVGB PAVGB_MMX
%define OP NOP
PIXOP4 put, mmx
%define OP PAVGB_MMX
PIXOP4 avg, mmx

%define PAVGB PAVGB_MMX_NO_RND
%define OP NOP
PIXOP3 put_no_rnd, mmx
%define OP PAVGB_MMX
PIXOP3 avg_no_rnd, mmx

%define PAVGB PAVGB_3DNOW
%define OP NOP
PIXOP3 put, 3dnow
%define OP PAVGB_3DNOW
PIXOP4 avg, 3dnow

%define PAVGB PAVGB_MMX2
%define OP NOP
PIXOP3 put, mmx2
%define OP PAVGB_MMX2
PIXOP4 avg, mmx2

%define PAVGB PAVGB_MMX2_NO_RND
%define OP NOP
PIXOP3 put_no_rnd, mmx2
%define OP PAVGB_MMX2
PIXOP3 avg_no_rnd, mmx2

INIT_XMM
%define PAVGB PAVGB_MMX2
%define OP NOP
PIXOP_FPEL put, 16, sse2
PIXOP_HPEL put, 16, sse2
%define OP PAVGB_MMX2
PIXOP_FPEL avg, 16, sse2
PIXOP_HPEL avg, 16, sse2

%define PAVGB PAVGB_MMX2_NO_RND
%define OP NOP
PIXOP_HPEL put_no_rnd, 16, sse2
%define OP PAVGB_MMX2
PIXOP_HPEL avg_no_rnd, 16, sse2
