/* -*-  indent-tabs-mode:nil; c-basic-offset:4;  -*- */
/*
 * Copyright (C) 2007 Marco Gerards <marco@gnu.org>
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

/**
 * @file libavcodec/dirac_wavelet.c
 * Dirac wavelet functions
 * @author Marco Gerards <marco@gnu.org>
 */

#define DEBUG 1

#include "avcodec.h"
#include "dirac_wavelet.h"

/**
 * Reorder coefficients so the IDWT synthesis can run in place
 *
 * @param data   coefficients
 * @param synth  output buffer
 * @param level  subband level
 */
static void dirac_subband_idwt_interleave(int16_t *data, int width,
                                          int height, int padded_width,
                                          int16_t *synth, int level)
{
    int x, y;
    int synth_width     = width << 1;
    int16_t *synth_line = synth;
    int16_t *line_ll    = data;
    int16_t *line_lh    = data + height * padded_width;
    int16_t *line_hl    = data                         + width;
    int16_t *line_hh    = data + height * padded_width + width;

    /* Interleave the coefficients. */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            synth_line[(x << 1)                  ] = line_ll[x];
            synth_line[(x << 1)               + 1] = line_hl[x];
            synth_line[(x << 1) + synth_width    ] = line_lh[x];
            synth_line[(x << 1) + synth_width + 1] = line_hh[x];
        }

        synth_line += synth_width << 1;
        line_ll    += padded_width;
        line_lh    += padded_width;
        line_hl    += padded_width;
        line_hh    += padded_width;
    }
}

static void dirac_subband_dwt_deinterleave(int16_t *data, int width,
                                           int height, int padded_width,
                                           int16_t *synth, int level)
{
    int x, y;
    int synth_width     = width << 1;
    int16_t *synth_line = synth;
    int16_t *line_ll    = data;
    int16_t *line_lh    = data + height * padded_width;
    int16_t *line_hl    = data                         + width;
    int16_t *line_hh    = data + height * padded_width + width;

    /* Deinterleave the coefficients. */
    for (y = 0; y < height; y++) {
        for (x = 0; x < width; x++) {
            line_ll[x] = synth_line[(x << 1)                  ];
            line_hl[x] = synth_line[(x << 1)               + 1];
            line_lh[x] = synth_line[(x << 1) + synth_width    ];
            line_hh[x] = synth_line[(x << 1) + synth_width + 1];
        }

        synth_line += synth_width << 1;
        line_ll    += padded_width;
        line_lh    += padded_width;
        line_hl    += padded_width;
        line_hh    += padded_width;
    }
}

/**
 * IDWT transform (5,3) for a specific subband
 *
 * @param data coefficients to transform
 * @param level level of the current transform
 * @return 0 when successful, otherwise -1 is returned
 */
int dirac_subband_idwt_53(AVCodecContext *avctx, int width, int height,
                          int padded_width, int16_t *data, int16_t *synth,
                          int level)
{
    int16_t *synthline;
    int x, y;
    int synth_width = width  << 1;
    int synth_height = height << 1;

    dirac_subband_idwt_interleave(data, width, height,
                                  padded_width, synth, level);

    /* Vertical synthesis: Lifting stage 1. */
    synthline = synth;
    for (x = 0; x < synth_width; x++) {
        synthline[x] -= (synthline[synth_width + x]
                       + synthline[synth_width + x]
                       + 2) >> 2;
    }
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x] -= (synthline[x - synth_width]
                           + synthline[x + synth_width]
                           + 2) >> 2;
        }
        synthline += (synth_width << 1);
    }
    synthline = synth + (synth_height - 2) * synth_width;
    for (x = 0; x < synth_width; x++) {
        synthline[x] -= (synthline[x - synth_width]
                       + synthline[x + synth_width]
                       + 2) >> 2;
    }

    /* Vertical synthesis: Lifting stage 2. */
    synthline = synth + synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] += (synthline[x - synth_width]
                       + synthline[x + synth_width]
                       + 1) >> 1;
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x + synth_width] += (synthline[x]
                                         + synthline[x + synth_width * 2]
                                         + 1) >> 1;
        }
        synthline += (synth_width << 1);
    }
    synthline = synth + (synth_height - 1) * synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] += (synthline[x - synth_width]
                       + synthline[x - synth_width]
                       + 1) >> 1;

    /* Horizontal synthesis. */
    synthline = synth;
    for (y = 0; y < synth_height; y++) {

        /* Lifting stage 1. */
        synthline[0] -= (synthline[1]
                       + synthline[1]
                       + 2) >> 2;
        data[0] = (synthline[0] + 1) >> 1;

        for (x = 1; x < width - 2; x += 2) {
        // even
            synthline[2*x] -= (synthline[2*x - 1]
                             + synthline[2*x + 1]
                             + 2) >> 2;
            data[2*x] = (synthline[2*x] + 1) >> 1;
            synthline[2*x + 2] -= (synthline[2*x + 1]
                                 + synthline[2*x + 3]
                                 + 2) >> 2;
            data[2*x + 2] = (synthline[2*x + 2] + 1) >> 1;
        // odd
            synthline[2*x - 1] += (synthline[2*x - 2]
                                 + synthline[2*x]
                                 + 1) >> 1;
            data[2*x - 1] = (synthline[2*x - 1] + 1) >> 1;
            synthline[2*x + 1] += (synthline[2*x]
                                 + synthline[2*x + 2]
                                 + 1) >> 1;
            data[2*x + 1] = (synthline[2*x + 1] + 1) >> 1;
        }

        synthline[synth_width - 2] -= (synthline[synth_width - 3]
                                     + synthline[synth_width - 1]
                                     + 2) >> 2;
        data[synth_width - 2] = (synthline[synth_width - 2] + 1) >> 1;

        synthline[synth_width - 3] += (synthline[synth_width - 4]
                                       + synthline[synth_width - 2]
                                       + 1) >> 1;
        data[synth_width - 3] = (synthline[synth_width - 3] + 1) >> 1;

        synthline[synth_width - 1] += (synthline[synth_width - 2]
                                     + synthline[synth_width - 2]
                                     + 1) >> 1;
        data[synth_width - 1] = (synthline[synth_width - 1] + 1) >> 1;

        synthline += synth_width;
        data      += padded_width;
    }

    return 0;
}

/**
 * DWT transform (5,3) for a specific subband
 *
 * @param data coefficients to transform
 * @param level level of the current transform
 * @return 0 when successful, otherwise -1 is returned
 */
int dirac_subband_dwt_53(AVCodecContext *avctx, int width, int height,
                         int padded_width, int16_t *data, int level)
{
    int16_t *synth, *synthline, *dataline;
    int x, y;
    int synth_width = width  << 1;
    int synth_height = height << 1;

    if (avcodec_check_dimensions(avctx, synth_width, synth_height)) {
        av_log(avctx, AV_LOG_ERROR, "avcodec_check_dimensions() failed\n");
        return -1;
    }

    synth = av_malloc(synth_width * synth_height * sizeof(int16_t));
    if (!synth) {
        av_log(avctx, AV_LOG_ERROR, "av_malloc() failed\n");
        return -1;
    }

    /* Shift in one bit that is used for additional precision and copy
       the data to the buffer. */
    synthline = synth;
    dataline  = data;
    for (y = 0; y < synth_height; y++) {
        for (x = 0; x < synth_width; x++)
            synthline[x] = dataline[x] << 1;
        synthline += synth_width;
        dataline  += padded_width;
    }

    /* Horizontal synthesis. */
    synthline = synth;
    dataline  = data;
    for (y = 0; y < synth_height; y++) {
        /* Lifting stage 2. */
        for (x = 0; x < width - 1; x++) {
            synthline[2 * x + 1] -= (synthline[2 * x]
                                   + synthline[2 * x + 2]
                                   + 1) >> 1;
        }
        synthline[synth_width - 1] -= (synthline[synth_width - 2]
                                     + synthline[synth_width - 2]
                                     + 1) >> 1;
        /* Lifting stage 1. */
        synthline[0] += (synthline[1]
                       + synthline[1]
                       + 2) >> 2;
        for (x = 1; x < width - 1; x++) {
            synthline[2 * x] += (synthline[2 * x - 1]
                               + synthline[2 * x + 1]
                               + 2) >> 2;
        }
        synthline[synth_width - 2] += (synthline[synth_width - 3]
                                     + synthline[synth_width - 1]
                                     + 2) >> 2;

        synthline += synth_width;
        dataline  += padded_width;
    }

    /* Vertical synthesis: Lifting stage 2. */
    synthline = synth + synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] -= (synthline[x - synth_width]
                       + synthline[x + synth_width]
                       + 1) >> 1;
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x + synth_width] -= (synthline[x]
                                         + synthline[x + synth_width * 2]
                                         + 1) >> 1;
        }
        synthline += (synth_width << 1);
    }
    synthline = synth + (synth_height - 1) * synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] -= (synthline[x - synth_width]
                       + synthline[x - synth_width]
                       + 1) >> 1;

    /* Vertical synthesis: Lifting stage 1. */
    synthline = synth;
    for (x = 0; x < synth_width; x++) {
        synthline[x] += (synthline[synth_width + x]
                       + synthline[synth_width + x]
                       + 2) >> 2;
    }
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x] += (synthline[x - synth_width]
                           + synthline[x + synth_width]
                           + 2) >> 2;
        }
        synthline += (synth_width << 1);
    }
    synthline = synth + (synth_height - 2) * synth_width;
    for (x = 0; x < synth_width; x++) {
        synthline[x] += (synthline[x - synth_width]
                       + synthline[x + synth_width]
                       + 2) >> 2;
    }


    dirac_subband_dwt_deinterleave(data, width, height,
                                   padded_width, synth, level);

    av_free(synth);

    return 0;
}


/**
 * IDWT transform (9,7) for a specific subband
 *
 * @param data coefficients to transform
 * @param level level of the current transform
 * @return 0 when successful, otherwise -1 is returned
 */
int dirac_subband_idwt_97(AVCodecContext *avctx, int width, int height,
                          int padded_width, int16_t *data, int16_t *synth,
                          int level)
{
    int16_t *synthline;
    int x, y;
    int synth_width = width  << 1;
    int synth_height = height << 1;

    dirac_subband_idwt_interleave(data, width, height, padded_width, synth, level);

    /* Vertical synthesis: Lifting stage 1. */
    synthline = synth;
    for (x = 0; x < synth_width; x++)
        synthline[x] -= (synthline[x + synth_width]
                       + synthline[x + synth_width]
                       + 2) >> 2;
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x] -= (synthline[x - synth_width]
                           + synthline[x + synth_width]
                           + 2) >> 2;
        }
        synthline += synth_width << 1;
    }
    synthline = synth + (synth_height - 2) * synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] -= (synthline[x - synth_width]
                       + synthline[x + synth_width]
                       + 2) >> 2;

    /* Vertical synthesis: Lifting stage 2. */
    synthline = synth + synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] += (-     synthline[x -     synth_width]
                         + 9 * synthline[x -     synth_width]
                         + 9 * synthline[x +     synth_width]
                         -     synthline[x + 3 * synth_width]
                                   + 8) >> 4;
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 2; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x + synth_width] += (-     synthline[x - 2 * synth_width]
                                           + 9 * synthline[x                  ]
                                           + 9 * synthline[x + 2 * synth_width]
                                           -     synthline[x + 4 * synth_width]
                                           + 8) >> 4;
        }
        synthline += synth_width << 1;
    }
    synthline = synth + (synth_height - 1) * synth_width;
    for (x = 0; x < synth_width; x++) {
        synthline[x] += (-     synthline[x - 3 * synth_width]
                         + 9 * synthline[x -     synth_width]
                         + 9 * synthline[x -     synth_width]
                         -     synthline[x -     synth_width]
                                   + 8) >> 4;
        synthline[x - synth_width * 2] += (-     synthline[x - 5* synth_width]
                                           + 9 * synthline[x - 3* synth_width]
                                           + 9 * synthline[x -    synth_width]
                                           -     synthline[x -    synth_width]
                                           + 8) >> 4;
    }

    /* Horizontal synthesis. */
    synthline = synth;
    for (y = 0; y < synth_height; y++) {
        /* Lifting stage 1. */
        synthline[0] -= (synthline[1]
                       + synthline[1]
                       + 2) >> 2;
        data[0] = (synthline[0] + 1) >> 1;

        for (x = 1; x < width - 1; x++) {
            synthline[2 * x] -= (synthline[2 * x - 1]
                               + synthline[2 * x + 1]
                               + 2) >> 2;
            data[2 * x] = (synthline[2 * x] + 1) >> 1;
        }
        synthline[synth_width - 2] -= ( synthline[synth_width - 3]
                                      + synthline[synth_width - 1]
                                      + 2) >> 2;
        data[synth_width - 2] = (synthline[synth_width - 2] + 1) >> 1;

        /* Lifting stage 2. */
        synthline[1] += (-     synthline[0]
                         + 9 * synthline[0]
                         + 9 * synthline[2]
                         -     synthline[4]
                         + 8) >> 4;
        data[1] = (synthline[1] + 1) >> 1;
        for (x = 1; x < width - 2; x++) {
            synthline[2*x + 1] += (-     synthline[2 * x - 2]
                                   + 9 * synthline[2 * x    ]
                                   + 9 * synthline[2 * x + 2]
                                   -     synthline[2 * x + 4]
                                   + 8) >> 4;
            data[2 * x + 1] = (synthline[2 * x + 1] + 1) >> 1;
        }
        synthline[synth_width - 1] += (-     synthline[synth_width - 4]
                                       + 9 * synthline[synth_width - 2]
                                       + 9 * synthline[synth_width - 2]
                                       -     synthline[synth_width - 2]
                                       + 8) >> 4;
        data[synth_width - 1] = (synthline[synth_width - 1] + 1) >> 1;
        synthline[synth_width - 3] += (-     synthline[synth_width - 6]
                                       + 9 * synthline[synth_width - 4]
                                       + 9 * synthline[synth_width - 2]
                                       -     synthline[synth_width - 2]
                                       + 8) >> 4;
        data[synth_width - 3] = (synthline[synth_width - 3] + 1) >> 1;
        synthline += synth_width;
        data      += padded_width;
    }

    return 0;
}

/**
 * DWT transform (9,7) for a specific subband
 *
 * @param data coefficients to transform
 * @param level level of the current transform
 * @return 0 when successful, otherwise -1 is returned
 */
int dirac_subband_dwt_97(AVCodecContext *avctx, int width, int height,
                                int padded_width,
                                int16_t *data, int level)
{
    int16_t *synth, *synthline, *dataline;
    int x, y;
    int synth_width = width  << 1;
    int synth_height = height << 1;

    if (avcodec_check_dimensions(avctx, synth_width, synth_height)) {
        av_log(avctx, AV_LOG_ERROR, "avcodec_check_dimensions() failed\n");
        return -1;
    }

    synth = av_malloc(synth_width * synth_height * sizeof(int16_t));
    if (!synth) {
        av_log(avctx, AV_LOG_ERROR, "av_malloc() failed\n");
        return -1;
    }

    /* Shift in one bit that is used for additional precision and copy
       the data to the buffer. */
    synthline = synth;
    dataline  = data;
    for (y = 0; y < synth_height; y++) {
        for (x = 0; x < synth_width; x++)
            synthline[x] = dataline[x] << 1;
        synthline += synth_width;
        dataline  += padded_width;
    }

    /* Horizontal synthesis. */
    synthline = synth;
    for (y = 0; y < synth_height; y++) {
        /* Lifting stage 2. */
        synthline[1] -= (-     synthline[0]
                         + 9 * synthline[0]
                         + 9 * synthline[2]
                         -     synthline[4]
                         + 8) >> 4;
        for (x = 1; x < width - 2; x++) {
            synthline[2*x + 1] -= (-     synthline[2 * x - 2]
                                   + 9 * synthline[2 * x    ]
                                   + 9 * synthline[2 * x + 2]
                                   -     synthline[2 * x + 4]
                                   + 8) >> 4;
        }
        synthline[synth_width - 1] -= (-     synthline[synth_width - 4]
                                       + 9 * synthline[synth_width - 2]
                                       + 9 * synthline[synth_width - 2]
                                       -     synthline[synth_width - 2]
                                       + 8) >> 4;
        synthline[synth_width - 3] -= (-     synthline[synth_width - 6]
                                       + 9 * synthline[synth_width - 4]
                                       + 9 * synthline[synth_width - 2]
                                       -     synthline[synth_width - 2]
                                       + 8) >> 4;
        /* Lifting stage 1. */
        synthline[0] += (synthline[1]
                       + synthline[1]
                       + 2) >> 2;
        for (x = 1; x < width - 1; x++) {
            synthline[2 * x] += (synthline[2 * x - 1]
                               + synthline[2 * x + 1]
                               + 2) >> 2;
        }
        synthline[synth_width - 2] += ( synthline[synth_width - 3]
                                      + synthline[synth_width - 1]
                                      + 2) >> 2;

        synthline += synth_width;
    }

    /* Vertical synthesis: Lifting stage 2. */
    synthline = synth + synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] -= (-     synthline[x -     synth_width]
                         + 9 * synthline[x -     synth_width]
                         + 9 * synthline[x +     synth_width]
                         -     synthline[x + 3 * synth_width]
                                   + 8) >> 4;
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 2; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x + synth_width] -= (-     synthline[x - 2 * synth_width]
                                           + 9 * synthline[x                  ]
                                           + 9 * synthline[x + 2 * synth_width]
                                           -     synthline[x + 4 * synth_width]
                                           + 8) >> 4;
        }
        synthline += synth_width << 1;
    }
    synthline = synth + (synth_height - 1) * synth_width;
    for (x = 0; x < synth_width; x++) {
        synthline[x] -= (-     synthline[x - 3 * synth_width]
                         + 9 * synthline[x -     synth_width]
                         + 9 * synthline[x -     synth_width]
                         -     synthline[x -     synth_width]
                                   + 8) >> 4;
        synthline[x - synth_width * 2] -= (-     synthline[x - 5* synth_width]
                                           + 9 * synthline[x - 3* synth_width]
                                           + 9 * synthline[x -    synth_width]
                                           -     synthline[x -    synth_width]
                                           + 8) >> 4;
    }

    /* Vertical synthesis: Lifting stage 1. */
    synthline = synth;
    for (x = 0; x < synth_width; x++)
        synthline[x] += (synthline[x + synth_width]
                       + synthline[x + synth_width]
                       + 2) >> 2;
    synthline = synth + (synth_width << 1);
    for (y = 1; y < height - 1; y++) {
        for (x = 0; x < synth_width; x++) {
            synthline[x] += (synthline[x - synth_width]
                           + synthline[x + synth_width]
                           + 2) >> 2;
        }
        synthline += synth_width << 1;
    }
    synthline = synth + (synth_height - 2) * synth_width;
    for (x = 0; x < synth_width; x++)
        synthline[x] += (synthline[x - synth_width]
                       + synthline[x + synth_width]
                       + 2) >> 2;

    dirac_subband_dwt_deinterleave(data, width, height,
                                   padded_width, synth, level);

    av_free(synth);

    return 0;
}
