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
 * @file diracenc.c
 * Dirac Encoder
 * @author Marco Gerards <marco@gnu.org>
 */

#define DEBUG 1

#include "dirac.h"
#include "avcodec.h"
#include "dsputil.h"
#include "bitstream.h"
#include "bytestream.h"
#include "golomb.h"
#include "dirac_arith.h"
#include "dirac_wavelet.h"
#include "mpeg12data.h"

static int encode_init(AVCodecContext *avctx){
    DiracContext *s = avctx->priv_data;
    av_log_set_level(AV_LOG_DEBUG);

    /* XXX: Choose a better size somehow.  */
    s->encodebuf = av_malloc(1 << 20);

    if (!s->encodebuf) {
        av_log(s->avctx, AV_LOG_ERROR, "av_malloc() failed\n");
        return -1;
    }

    return 0;
}

static int encode_end(AVCodecContext *avctx)
{
    DiracContext *s = avctx->priv_data;

    av_free(s->encodebuf);

    return 0;
}

/**
 * DWT
 *
 * @param coeffs coefficients to transform
 * @return returns 0 on succes, otherwise -1
 */
int dirac_dwt(DiracContext *s, int16_t *coeffs) {
    int level;
    int width, height;

    /* XXX: make depth configurable.  */
    for (level = s->frame_decoding.wavelet_depth; level >= 1; level--) {
        width  = subband_width(s, level);
        height = subband_height(s, level);

        if (s->refs)
        dirac_subband_dwt_53(s->avctx, width, height, s->padded_width, coeffs, level);
        else
            dirac_subband_dwt_95(s->avctx, width, height, s->padded_width, coeffs, level);
    }

    return 0;
}

static void dirac_encode_parse_info(DiracContext *s, int parsecode) {
    put_bits(&s->pb, 32, DIRAC_PARSE_INFO_PREFIX);
    put_bits(&s->pb, 8,  parsecode);
    /* XXX: These will be filled in after encoding.  */
    put_bits(&s->pb, 32, 0);
    put_bits(&s->pb, 32, 0);
}

static void dirac_encode_sequence_parameters(DiracContext *s) {
    AVCodecContext *avctx = s->avctx;
    struct sequence_parameters *seq = &s->sequence;
    const struct sequence_parameters *seqdef;
    int video_format = 0;

    seqdef = &dirac_sequence_parameters_defaults[video_format];

    /* Fill in defaults for the sequence parameters.  */
    s->sequence = *seqdef;

    /* Fill in the sequence parameters using the information set by
       the user. XXX: Only support YUV420P for now.  */
    seq->luma_width    = avctx->width;
    seq->luma_height   = avctx->height;
    seq->chroma_width  = avctx->width  / 2;
    seq->chroma_height = avctx->height / 2;
    seq->video_depth   = 8;
    seq->chroma_format = 2;

    /* Set video format to 0.  In the future a best match is perhaps
       better.  */
    dirac_set_ue_golomb(&s->pb, video_format);


    /* Override image dimensions.  */
    if (seq->luma_width != seqdef->luma_width
        || seq->luma_height != seqdef->luma_height) {
        put_bits(&s->pb, 1, 1);

        dirac_set_ue_golomb(&s->pb, seq->luma_width);
        dirac_set_ue_golomb(&s->pb, seq->luma_height);
    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override chroma format.  */
    if (seq->chroma_format != seqdef->chroma_format) {
        put_bits(&s->pb, 1, 1);

        /* XXX: Hardcoded to 4:2:0.  */
        dirac_set_ue_golomb(&s->pb, 2);
    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override video depth.  */
    if (seq->video_depth != seqdef->video_depth) {
        put_bits(&s->pb, 1, 1);

        dirac_set_ue_golomb(&s->pb, seq->video_depth);
    } else {
        put_bits(&s->pb, 1, 0);
    }
}

static void dirac_encode_source_parameters(DiracContext *s) {
    AVCodecContext *avctx = s->avctx;
    struct source_parameters *source = &s->source;
    const struct source_parameters *sourcedef;
    int video_format = 0;

    sourcedef = &dirac_source_parameters_defaults[video_format];

    /* Fill in defaults for the source parameters.  */
    s->source = *sourcedef;

    /* Fill in the source parameters using the information set by the
       user. XXX: No support for interlacing.  */
    source->interlaced         = 0;
    source->frame_rate.num     = avctx->time_base.den;
    source->frame_rate.den     = avctx->time_base.num;

    if (avctx->sample_aspect_ratio.num != 0)
        source->aspect_ratio = avctx->sample_aspect_ratio;

    /* Override interlacing options.  */
    if (source->interlaced != sourcedef->interlaced) {
        put_bits(&s->pb, 1, 1);

        put_bits(&s->pb, 1, source->interlaced);

        /* Override top field first flag.  */
        if (source->top_field_first != sourcedef->top_field_first) {
            put_bits(&s->pb, 1, 1);

            put_bits(&s->pb, 1, source->top_field_first);

        } else {
            put_bits(&s->pb, 1, 0);
        }

        /* Override sequential fields flag.  */
        if (source->sequential_fields != sourcedef->sequential_fields) {
            put_bits(&s->pb, 1, 1);

            put_bits(&s->pb, 1, source->sequential_fields);

        } else {
            put_bits(&s->pb, 1, 0);
        }

    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override frame rate.  */
    if (av_cmp_q(source->frame_rate, sourcedef->frame_rate) != 0) {
        put_bits(&s->pb, 1, 1);

        /* XXX: Some default frame rates can be used.  For now just
           set the index to 0 and write the frame rate.  */
        dirac_set_ue_golomb(&s->pb, 0);

        dirac_set_ue_golomb(&s->pb, source->frame_rate.num);
        dirac_set_ue_golomb(&s->pb, source->frame_rate.den);
    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override aspect ratio.  */
    if (av_cmp_q(source->aspect_ratio, sourcedef->aspect_ratio) != 0) {
        put_bits(&s->pb, 1, 1);

        /* XXX: Some default aspect ratios can be used.  For now just
           set the index to 0 and write the aspect ratio.  */
        dirac_set_ue_golomb(&s->pb, 0);

        dirac_set_ue_golomb(&s->pb, source->aspect_ratio.num);
        dirac_set_ue_golomb(&s->pb, source->aspect_ratio.den);
    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override clean area.  */
    if (source->clean_width != sourcedef->clean_width
        || source->clean_height != sourcedef->clean_height
        || source->clean_left_offset != sourcedef->clean_left_offset
        || source->clean_right_offset != sourcedef->clean_right_offset) {
        put_bits(&s->pb, 1, 1);

        dirac_set_ue_golomb(&s->pb, source->clean_width);
        dirac_set_ue_golomb(&s->pb, source->clean_height);
        dirac_set_ue_golomb(&s->pb, source->clean_left_offset);
        dirac_set_ue_golomb(&s->pb, source->clean_right_offset);
    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override signal range.  */
    if (source->luma_offset != sourcedef->luma_offset
        || source->luma_excursion != sourcedef->luma_excursion
        || source->chroma_offset != sourcedef->chroma_offset
        || source->chroma_excursion != sourcedef->chroma_excursion) {
        put_bits(&s->pb, 1, 1);

        /* XXX: Some default signal ranges can be used.  For now just
           set the index to 0 and write the signal range.  */
        dirac_set_ue_golomb(&s->pb, 0);

        dirac_set_ue_golomb(&s->pb, source->luma_offset);
        dirac_set_ue_golomb(&s->pb, source->luma_excursion);
        dirac_set_ue_golomb(&s->pb, source->chroma_offset);
        dirac_set_ue_golomb(&s->pb, source->chroma_excursion);
    } else {
        put_bits(&s->pb, 1, 0);
    }

    /* Override color spec.  */
    /* XXX: For now this won't be overridden at all.  Just set this to
       defaults.  */
    put_bits(&s->pb, 1, 0);
}

static void dirac_encode_access_unit_header(DiracContext *s) {
    /* First write the Access Unit Parse Parameters.  */

    dirac_set_ue_golomb(&s->pb, 0); /* version major */
    dirac_set_ue_golomb(&s->pb, 1); /* version minor */
    dirac_set_ue_golomb(&s->pb, 0); /* profile */
    dirac_set_ue_golomb(&s->pb, 0); /* level */

    dirac_encode_sequence_parameters(s);
    dirac_encode_source_parameters(s);
    /* Fill in defaults for the decoding parameters.  */
    s->decoding = dirac_decoding_parameters_defaults[0];
}



static void encode_coeff(DiracContext *s, int16_t *coeffs, int level,
                         int orientation, int x, int y) {
    int parent = 0;
    int nhood;
    int idx;
    int coeff;
    int xpos, ypos;
    struct dirac_arith_context_set *context;
    int16_t *coeffp;

    xpos   = coeff_posx(s, level, orientation, x);
    ypos   = coeff_posy(s, level, orientation, y);

    coeffp = &coeffs[xpos + ypos * s->padded_width];
    coeff  = *coeffp;

    /* The value of the pixel belonging to the lower level.  */
    if (level >= 2) {
        int px = coeff_posx(s, level - 1, orientation, x >> 1);
        int py = coeff_posy(s, level - 1, orientation, y >> 1);
        parent = coeffs[s->padded_width * py + px] != 0;
    }

    /* Determine if the pixel has only zeros in its neighbourhood.  */
    nhood = zero_neighbourhood(s, coeffp, y, x);

    /* Calculate an index into context_sets_waveletcoeff.  */
    idx = parent * 6 + (!nhood) * 3;
    idx += sign_predict(s, coeffp, orientation, y, x);

    context = &dirac_context_sets_waveletcoeff[idx];

    /* XXX: Quantization.  */

    /* Write out the coefficient.  */
    dirac_arith_write_int(&s->arith, context, coeff);
}

static void encode_codeblock(DiracContext *s, int16_t *coeffs, int level,
                             int orientation, int xpos, int ypos) {
    int blockcnt_one = (s->codeblocksh[level] + s->codeblocksv[level]) == 2;
    int left, right, top, bottom;
    int x, y;

    left   = (subband_width(s, level)  *  xpos     ) / s->codeblocksh[level];
    right  = (subband_width(s, level)  * (xpos + 1)) / s->codeblocksh[level];
    top    = (subband_height(s, level) *  ypos     ) / s->codeblocksv[level];
    bottom = (subband_height(s, level) * (ypos + 1)) / s->codeblocksv[level];

    if (!blockcnt_one) {
        int zero = 0;
        for (y = top; y < bottom; y++) {
            for (x = left; x < right; x++) {
                int xpos, ypos;
                xpos   = coeff_posx(s, level, orientation, x);
                ypos   = coeff_posy(s, level, orientation, y);

                if (coeffs[xpos + ypos * s->padded_width] != 0) {
                    zero = 0;
                    break;
                }
            }
        }

        dirac_arith_put_bit(&s->arith, ARITH_CONTEXT_ZERO_BLOCK, zero);

        if (zero)
            return;
    }

    for (y = top; y < bottom; y++)
        for (x = left; x < right; x++)
            encode_coeff(s, coeffs, level, orientation, x, y);
}

static void intra_dc_coding(DiracContext *s, int16_t *coeffs) {
    int x, y;
    int16_t *line = coeffs + (subband_height(s, 0) - 1) * s->padded_width;

    /* Just do the inverse of intra_dc_prediction.  Start at the right
       bottom corner and remove the predicted value from the
       coefficient, the decoder can easily reconstruct this.  */

    for (y = subband_height(s, 0) - 1; y >= 0; y--) {
        for (x = subband_width(s, 0) - 1; x >= 0; x--) {
            line[x] -= intra_dc_coeff_prediction(s, &line[x], x, y);
        }
        line -= s->padded_width;
    }
}

static inline void dirac_arithblk_writelen(DiracContext *s,
                                           PutBitContext *pb) {
    int length ;
    dirac_arith_coder_flush(&s->arith);
    flush_put_bits(pb);
    length = put_bits_count(pb) / 8;
    dirac_set_ue_golomb(&s->pb, length);
}

static inline void dirac_arithblk_writedata(DiracContext *s,
                                            PutBitContext *pb) {
    int length;
    char *buf;

    length = put_bits_count(pb) / 8;

    align_put_bits(&s->pb);
    /* XXX: Use memmove.  */
    flush_put_bits(&s->pb);
    buf = pbBufPtr(&s->pb);
    memcpy(buf, s->encodebuf, length);
    skip_put_bytes(&s->pb, length);
}

static int encode_subband(DiracContext *s, int level,
                          int orientation, int16_t *coeffs) {
    int xpos, ypos;
    PutBitContext pb;

    /* Encode the data.  */

    init_put_bits(&pb, s->encodebuf, (1 << 20) * 8);
    dirac_arith_coder_init(&s->arith, &pb);

    if (level == 0 && s->refs == 0)
        intra_dc_coding(s, coeffs);

    for (ypos = 0; ypos < s->codeblocksv[level]; ypos++)
        for (xpos = 0; xpos < s->codeblocksh[level]; xpos++)
            encode_codeblock(s, coeffs, level, orientation, xpos, ypos);

    dirac_arithblk_writelen(s, &pb);

    /* Write quantizer index.  XXX: No quantization?  */
    dirac_set_ue_golomb(&s->pb, 0);

    dirac_arithblk_writedata(s, &pb);

    return 0;
}

static int dirac_encode_component(DiracContext *s, int comp) {
    int level;
    subband_t subband;
    int16_t *coeffs;
    int x, y;

    align_put_bits(&s->pb);

    if (comp == 0) {
        s->width         = s->sequence.luma_width;
        s->height        = s->sequence.luma_height;
        s->padded_width  = s->padded_luma_width;
        s->padded_height = s->padded_luma_height;
    } else {
        s->width         = s->sequence.chroma_width;
        s->height        = s->sequence.chroma_height;
        s->padded_width  = s->padded_chroma_width;
        s->padded_height = s->padded_chroma_height;
    }

    coeffs = av_mallocz(s->padded_width * s->padded_height * sizeof(int16_t));
    if (! coeffs) {
        av_log(s->avctx, AV_LOG_ERROR, "av_malloc() failed\n");
        return -1;
    }

    for (y = 0; y < s->height; y++) {
        for (x = 0; x < s->width; x++) {
            coeffs[y * s->padded_width + x] =
                s->picture.data[comp][y * s->picture.linesize[comp] + x];
        }
    }

    /* Subtract motion compensated data to calculate the residue.  */
    if (s->refs) {
        int x, y;
        int16_t *coeffline;
        int16_t *mcline;

        /* Calculate the data that should be subtracted from the
           pixels to produce the residue.  */
        if (dirac_motion_compensation(s, coeffs, comp)) {
            av_freep(&s->sbsplit);
            av_freep(&s->blmotion);
            av_freep(&s->mcpic);

            return -1;
        }

        coeffline = coeffs;
        mcline    = s->mcpic;
        for (y = 0; y < s->height; y++) {
            for (x = 0; x < s->width; x++) {
                int16_t coeff = mcline[x] + (1 << (s->total_wt_bits - 1));
                coeffline[x] -= coeff >> s->total_wt_bits;
            }
            coeffline += s->padded_width;
            mcline    += s->width;
        }

        av_freep(&s->mcpic);
    }


    for (y = 0; y < s->height; y++) {
        for (x = s->width; x < s->padded_width; x++)
            coeffs[y * s->padded_width + x] = coeffs[y * s->padded_width + x + s->padded_width - 1];
    }

    for (y = s->height; y < s->padded_height; y++) {
        for (x = 0; x < s->padded_width; x++)
            coeffs[y * s->padded_width + x] = coeffs[(s->height - 1) * s->padded_width + x];
    }

    dirac_dwt(s, coeffs);

    encode_subband(s, 0, subband_ll, coeffs);
    for (level = 1; level <= 4; level++) {
        for (subband = 1; subband <= subband_hh; subband++) {
            encode_subband(s, level, subband, coeffs);
        }
    }

    av_free(coeffs);

    return 0;
}


static void blockmode_encode(DiracContext *s, int x, int y) {
    int res = s->blmotion[y * s->blwidth + x].use_ref & DIRAC_REF_MASK_REF1;
    res ^= mode_prediction(s, x, y, DIRAC_REF_MASK_REF1, 0);
    dirac_arith_put_bit(&s->arith, ARITH_CONTEXT_PMODE_REF2, res);

    if (s->refs == 2) {
        res = (s->blmotion[y * s->blwidth + x].use_ref
               & DIRAC_REF_MASK_REF2) >> 1;
        res ^= mode_prediction(s, x, y, DIRAC_REF_MASK_REF2, 1);
        dirac_arith_put_bit(&s->arith, ARITH_CONTEXT_PMODE_REF2, res);
    }
}

static void blockglob_encode(DiracContext *s, int x, int y) {
    /* Global motion compensation is not used at all.  */
    if (!s->globalmc_flag)
        return;

    /* Global motion compensation is not used for this block.  */
    if (s->blmotion[y * s->blwidth + x].use_ref & 3) {
        int res = (s->blmotion[y * s->blwidth + x].use_ref
                   & DIRAC_REF_MASK_GLOBAL) >> 2;
        res ^= mode_prediction(s, x, y, DIRAC_REF_MASK_GLOBAL, 2);
        dirac_arith_put_bit(&s->arith, ARITH_CONTEXT_GLOBAL_BLOCK, res);
    }
}

static void dirac_pack_motion_vector(DiracContext *s,
                                     int ref, int dir,
                                     int x, int y) {
    int res;
    const int refmask = (ref + 1) | DIRAC_REF_MASK_GLOBAL;

    /* First determine if for this block in the specific reference
       frame a motion vector is required.  */
    if ((s->blmotion[y * s->blwidth + x].use_ref & refmask) != ref + 1)
        return;

    res = s->blmotion[y * s->blwidth + x].vect[ref][dir];
    res -= motion_vector_prediction(s, x, y, ref, dir);
    dirac_arith_write_int(&s->arith, &dirac_context_set_mv, res);
}

static void dirac_pack_motion_vectors(DiracContext *s,
                                      int ref, int dir) {
    PutBitContext pb;
    int x, y;

    init_put_bits(&pb, s->encodebuf, (1 << 20) * 8);
    dirac_arith_coder_init(&s->arith, &pb);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
                        int q, p;
            int blkcnt = 1 << s->sbsplit[y * s->sbwidth + x];
            int step = 4 >> s->sbsplit[y * s->sbwidth + x];

            for (q = 0; q < blkcnt; q++)
                for (p = 0; p < blkcnt; p++) {
                    dirac_pack_motion_vector(s, ref, dir,
                                             4 * x + p * step,
                                             4 * y + q * step);
                }
        }

    dirac_arithblk_writelen(s, &pb);
    dirac_arithblk_writedata(s, &pb);
}

static void pack_block_dc(DiracContext *s, int x, int y, int comp) {
    int res;

    if (s->blmotion[y * s->blwidth + x].use_ref & 3)
        return;

    res = s->blmotion[y * s->blwidth + x].dc[comp];
    res -= block_dc_prediction(s, x, y, comp);
    dirac_arith_write_int(&s->arith, &dirac_context_set_dc, res);
}

static int dirac_encode_blockdata(DiracContext *s) {
    int i;
    int comp;
    int x, y;
    PutBitContext pb;

#define DIVRNDUP(a, b) ((a + b - 1) / b)

    s->sbwidth  = DIVRNDUP(s->sequence.luma_width,
                           (s->frame_decoding.luma_xbsep << 2));
    s->sbheight = DIVRNDUP(s->sequence.luma_height,
                           (s->frame_decoding.luma_ybsep << 2));
    s->blwidth  = s->sbwidth  << 2;
    s->blheight = s->sbheight << 2;

    /* XXX: This should be done before calling this function, but for
       now it is more convenient to do this here.  */
    s->sbsplit  = av_mallocz(s->sbwidth * s->sbheight * sizeof(int));
    if (!s->sbsplit) {
        av_log(s->avctx, AV_LOG_ERROR, "avcodec_check_dimensions() failed\n");
        return -1;
    }

    s->blmotion = av_mallocz(s->blwidth * s->blheight * sizeof(*s->blmotion));
    if (!s->blmotion) {
        av_freep(&s->sbsplit);
        av_log(s->avctx, AV_LOG_ERROR, "avcodec_check_dimensions() failed\n");
        return -1;
    }

    /* XXX: Fill the Motion Vectors with semi-random data for
       testing.  */
    for (x = 0; x < s->blwidth; x++) {
        for (y = 0; y < s->blheight; y++) {
            struct dirac_blockmotion *bl = &s->blmotion[y * s->blwidth + x];

#if 0
            bl->use_ref = (x + y) % 2;
#else
            bl->use_ref = 1;
#endif
            bl->vect[0][0] = (y % 18) - 9;
            bl->vect[0][1] = (x % 18) - 9;
            bl->vect[1][0] = (y % 7)  - 5;
            bl->vect[1][1] = (x % 7)  - 5;
            if (!bl->use_ref) {
                bl->dc[0] = x - y;
                bl->dc[1] = x + y;
                bl->dc[2] = x * y;
            }
        }
    }

    /* Superblock splitmodes.  XXX: Just (for now) encode "2", so that
       blocks are not split at all.  */
    init_put_bits(&pb, s->encodebuf, (1 << 20) * 8);
    dirac_arith_coder_init(&s->arith, &pb);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
            int res;
            s->sbsplit[y * s->sbwidth + x] = 2;
            res = s->sbsplit[y * s->sbwidth + x] - split_prediction(s, x, y);

            /* This should be unsigned, because this is under modulo
               3, it is ok to add 3.  */
            if (res < 0)
                res += 3;

            dirac_arith_write_uint(&s->arith, &dirac_context_set_split, res);
        }
    dirac_arithblk_writelen(s, &pb);
    dirac_arithblk_writedata(s, &pb);

    /* Prediction modes.  */
    init_put_bits(&pb, s->encodebuf, (1 << 20) * 8);
    dirac_arith_coder_init(&s->arith, &pb);
    for (y = 0; y < s->sbheight; y++)
        for (x = 0; x < s->sbwidth; x++) {
            int q, p;
            int blkcnt = 1 << s->sbsplit[y * s->sbwidth + x];
            int step   = 4 >> s->sbsplit[y * s->sbwidth + x];

            for (q = 0; q < blkcnt; q++)
                for (p = 0; p < blkcnt; p++) {
                    blockmode_encode(s,
                                     4 * x + p * step,
                                     4 * y + q * step);
                    blockglob_encode(s,
                                     4 * x + p * step,
                                     4 * y + q * step);
                }
        }
    dirac_arithblk_writelen(s, &pb);
    dirac_arithblk_writedata(s, &pb);

    /* Pack the motion vectors.  */
    for (i = 0; i < s->refs; i++) {
        dirac_pack_motion_vectors(s, i, 0);
        dirac_pack_motion_vectors(s, i, 1);
    }

    /* Unpack the DC values for all the three components (YUV).  */
    for (comp = 0; comp < 3; comp++) {
        /* Unpack the DC values.  */
        init_put_bits(&pb, s->encodebuf, (1 << 20) * 8);
        dirac_arith_coder_init(&s->arith, &pb);
        for (y = 0; y < s->sbheight; y++)
            for (x = 0; x < s->sbwidth; x++) {
                int q, p;
                int blkcnt = 1 << s->sbsplit[y * s->sbwidth + x];
                int step   = 4 >> s->sbsplit[y * s->sbwidth + x];

                for (q = 0; q < blkcnt; q++)
                    for (p = 0; p < blkcnt; p++) {
                        pack_block_dc(s,
                                      4 * x + p * step,
                                      4 * y + q * step,
                                      comp);
                    }
            }
        dirac_arithblk_writelen(s, &pb);
        dirac_arithblk_writedata(s, &pb);
    }

    return 0;
}

static int dirac_pack_prediction_parameters(DiracContext *s) {
    PutBitContext *pb = &s->pb;

    /* Use default block parameters.  */
    put_bits(pb, 1, 0);

    /* Motion vector precision: Use qpel interpolation (2 bits
       precision) for now.  */
    put_bits(pb, 1, 1);
    dirac_set_ue_golomb(pb, 2);

    /* Do not use Global Motion Estimation.  */
    put_bits(pb, 1, 0);

    /* Do not encode the picture prediction mode, this is not yet
       used.  */
    put_bits(pb, 1, 0);

    /* Use default weights for the reference frames.  */
    put_bits(pb, 1, 0);

    s->chroma_hshift = s->sequence.chroma_format > 0;
    s->chroma_vshift = s->sequence.chroma_format > 1;
    s->sequence.chroma_width  = s->sequence.luma_width  >> s->chroma_hshift;
    s->sequence.chroma_height = s->sequence.luma_height >> s->chroma_vshift;

    s->frame_decoding.chroma_xblen = (s->frame_decoding.luma_xblen
                                      >> s->chroma_hshift);
    s->frame_decoding.chroma_yblen = (s->frame_decoding.luma_yblen
                                      >> s->chroma_vshift);
    s->frame_decoding.chroma_xbsep = (s->frame_decoding.luma_xbsep
                                      >> s->chroma_hshift);
    s->frame_decoding.chroma_ybsep = (s->frame_decoding.luma_ybsep
                                      >> s->chroma_vshift);

    return 0;
}


static int dirac_encode_frame(DiracContext *s) {
    PutBitContext *pb = &s->pb;
    int comp;
    int i;

    s->frame_decoding = s->decoding;

    /* Round up to a multiple of 2^depth.  */
    s->padded_luma_width    = CALC_PADDING(s->sequence.luma_width,
                                           s->frame_decoding.wavelet_depth);
    s->padded_luma_height   = CALC_PADDING(s->sequence.luma_height,
                                           s->frame_decoding.wavelet_depth);
    s->padded_chroma_width  = CALC_PADDING(s->sequence.chroma_width,
                                           s->frame_decoding.wavelet_depth);
    s->padded_chroma_height = CALC_PADDING(s->sequence.chroma_height,
                                           s->frame_decoding.wavelet_depth);

    /* Set defaults for the codeblocks.  */
    for (i = 0; i <= s->frame_decoding.wavelet_depth; i++) {
        if (s->refs == 0) {
            s->codeblocksh[i] = i <= 2 ? 1 : 4;
            s->codeblocksv[i] = i <= 2 ? 1 : 3;
        } else {
            if (i <= 1) {
                s->codeblocksh[i] = 1;
                s->codeblocksv[i] = 1;
            } else if (i == 2) {
                s->codeblocksh[i] = 8;
                s->codeblocksv[i] = 6;
            } else {
                s->codeblocksh[i] = 12;
                s->codeblocksv[i] = 8;
            }
        }
    }

    /* Write picture header.  */
    s->picnum = s->avctx->frame_number - 1;
    put_bits(pb, 32, s->picnum);

    for (i = 0; i < s->refs; i++)
        dirac_set_se_golomb(pb, s->ref[i] - s->picnum);

    /* Write retire pictures list.  */
    if (s->refcnt == 0) {
        dirac_set_ue_golomb(pb, 0);
    } else if (s->refs == 0) {
        /* This is a new intra frame, remove all old reference
           frames.  */
        dirac_set_ue_golomb(pb, 1);
        dirac_set_se_golomb(pb, s->refframes[0].frame.display_picture_number
                            - s->picnum);
    } else {
        dirac_set_ue_golomb(pb, 0);
    }

    /* Pack the ME data.  */
    if (s->refs) {
        align_put_bits(pb);
        if (dirac_pack_prediction_parameters(s))
            return -1;
        align_put_bits(pb);
        if (dirac_encode_blockdata(s))
            return -1;
    }

    align_put_bits(pb);

    /* Wavelet transform parameters.  */
    if (s->refs == 0) {
        s->zero_res = 0;
    } else {
        /* XXX: Actually, calculate if the residue is zero.  */
        s->zero_res = 0;
        put_bits(pb, 1, 0);
    }

    /* Do not override default filter.  */
    put_bits(pb, 1, 1);

    /* Set the default filter to LeGall for inter frames and
       Deslauriers-Debuc for intra frames.  */
    if (s->refs)
    dirac_set_ue_golomb(pb, 1);
    else
        dirac_set_ue_golomb(pb, 0);

    /* Do not override the default depth.  */
    put_bits(pb, 1, 0);

    /* Use spatial partitioning.  */
    put_bits(pb, 1, 1);

    /* Do not override spatial partitioning.  */
    put_bits(pb, 1, 0);

    /* Codeblock mode.  */
    dirac_set_ue_golomb(pb, 0);


    /* Write the transform data.  */
    for (comp = 0; comp < 3; comp++) {
        if (dirac_encode_component(s, comp))
            return -1;
    }

    return 0;
}

static int encode_frame(AVCodecContext *avctx, unsigned char *buf,
                        int buf_size, void *data) {
    DiracContext *s = avctx->priv_data;
    AVFrame *picture = data;
    unsigned char *dst = &buf[5];
    int reference;
    int size;
    static int intercnt = 0;

    reference = (s->next_parse_code & 0x04) == 0x04;
    s->refs   = s->next_parse_code & 0x03;

    dprintf(avctx, "Encoding frame %p size=%d of type=%02X isref=%d refs=%d\n",
            buf, buf_size, s->next_parse_code, reference, s->refs);

    init_put_bits(&s->pb, buf, buf_size);
    s->avctx = avctx;
    s->picture = *picture;

    if (s->next_parse_code == 0) {
        dirac_encode_parse_info(s, pc_access_unit_header);
        dirac_encode_access_unit_header(s);
        s->next_parse_code = 0x0C;
    } else if (s->next_parse_code == 0x0C) {
        dirac_encode_parse_info(s, 0x0C);
        dirac_encode_frame(s);
        s->next_parse_code = 0x09;
    } else if (s->next_parse_code == 0x09) {
        s->ref[0] = s->refframes[0].frame.display_picture_number;
        dirac_encode_parse_info(s, 0x09);
        dirac_encode_frame(s);
        if (++intercnt == 5) {
            s->next_parse_code = 0x0C;
            intercnt = 0;
        }
    }

    flush_put_bits(&s->pb);
    size = put_bits_count(&s->pb) / 8;

    bytestream_put_be32(&dst, size);
    bytestream_put_be32(&dst, s->prev_size);
    s->prev_size = size;

    if (reference) {
        AVFrame f;
        int data_size;
        int decsize;

        avcodec_get_frame_defaults(&f);
        avcodec_get_frame_defaults(&s->picture);

        /* Use the decoder to create the reference frame.  */
        decsize = dirac_decode_frame(avctx, &f, &data_size, buf, size);
        if (decsize == -1)
            return -1;

        assert(f.reference);
    }

    return size;
}

#ifdef CONFIG_ENCODERS
AVCodec dirac_encoder = {
    "dirac",
    CODEC_TYPE_VIDEO,
    CODEC_ID_DIRAC,
    sizeof(DiracContext),
    encode_init,
    encode_frame,
    encode_end,
    .pix_fmts = (enum PixelFormat[]) {PIX_FMT_YUV420P, -1}
};
#endif
