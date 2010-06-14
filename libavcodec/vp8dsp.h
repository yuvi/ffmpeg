/**
 * VP8 compatible video decoder
 *
 * Copyright (C) 2010 David Conrad
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


#ifndef AVCODEC_VP8DSP_H
#define AVCODEC_VP8DSP_H

typedef struct VP8DSPContext {
    void (*vp8_luma_dc_wht)(DCTELEM block[4][4][16], DCTELEM dc[16]);
    void (*vp8_idct_add)(uint8_t *dst, DCTELEM block[16], int stride);
    void (*vp8_idct_dc_add)(uint8_t *dst, DCTELEM block[16], int stride);

    // loop filter applied to edges between macroblocks
    void (*vp8_v_loop_filter16)(uint8_t *dst, int stride, int flim_E, int flim_I, int hev_thresh);
    void (*vp8_h_loop_filter16)(uint8_t *dst, int stride, int flim_E, int flim_I, int hev_thresh);
    void (*vp8_v_loop_filter8)(uint8_t *dst, int stride, int flim_E, int flim_I, int hev_thresh);
    void (*vp8_h_loop_filter8)(uint8_t *dst, int stride, int flim_E, int flim_I, int hev_thresh);

    // loop filter applied to inner macroblock edges
    void (*vp8_v_loop_filter16_inner)(uint8_t *dst, int stride, int flim_E, int flim_I, int hev_thresh);
    void (*vp8_h_loop_filter16_inner)(uint8_t *dst, int stride, int flim_E, int flim_I, int hev_thresh);
    void (*vp8_v_loop_filter8_inner)(uint8_t *dst, int stride, int flim_E, int flim_I, int hev_thresh);
    void (*vp8_h_loop_filter8_inner)(uint8_t *dst, int stride, int flim_E, int flim_I, int hev_thresh);

    void (*vp8_v_loop_filter_simple)(uint8_t *dst, int stride, int flim);
    void (*vp8_h_loop_filter_simple)(uint8_t *dst, int stride, int flim);

    /**
     * first dimension: width>>3, height is assumed equal to width
     * second dimension: whether vertical interpolation is needed
     * third dimension: whether horizontal interposation is needed
     * so something like put_vp8_epel_pixels_tab[width>>3][!!my][!!mx](..., mx, my)
     */
    epel_mc_func put_vp8_epel_pixels_tab[3][2][2];    
} VP8DSPContext;

void ff_put_vp8_pixels16_c(uint8_t *dst, const uint8_t *src, int stride, int mx, int my);
void ff_put_vp8_pixels8_c(uint8_t *dst, const uint8_t *src, int stride, int mx, int my);
void ff_put_vp8_pixels4_c(uint8_t *dst, const uint8_t *src, int stride, int mx, int my);

void ff_vp8dsp_init(VP8DSPContext *c);

#endif /* AVCODEC_VP8DSP_H */