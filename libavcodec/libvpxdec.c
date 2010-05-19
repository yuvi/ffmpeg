/*
 *  Copyright (c) 2010, Google, Inc.
 * 
 *  All rights reserved.
 * 
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 * 
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 
 *  - Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials provided
 *    with the distribution.
 * 
 *  - Neither the name of Google nor the names of its contributors may
 *    be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 *  Subject to the terms and conditions of the above License, Google
 *  hereby grants to You a perpetual, worldwide, non-exclusive,
 *  no-charge, royalty-free, irrevocable (except as stated in this
 *  section) patent license to make, have made, use, offer to sell, sell,
 *  import, and otherwise transfer this implementation of VP8, where such
 *  license applies only to those patent claims, both currently owned by
 *  Google and acquired in the future, licensable by Google that are
 *  necessarily infringed by this implementation of VP8. If You or your
 *  agent or exclusive licensee institute or order or agree to the
 *  institution of patent litigation against any entity (including a
 *  cross-claim or counterclaim in a lawsuit) alleging that this
 *  implementation of VP8 or any code incorporated within this
 *  implementation of VP8 constitutes direct or contributory patent
 *  infringement, or inducement of patent infringement, then any rights
 *  granted to You under this License for this implementation of VP8
 *  shall terminate as of the date such litigation is filed.
 */

/*!\file
   \brief VP8 decoder support via libvpx
*/

#include "avcodec.h"

#ifndef HAVE_STDINT_H
# define HAVE_STDINT_H 1
#endif
#define VPX_CODEC_DISABLE_COMPAT 1
#include <vpx_decoder.h>
#include <vp8dx.h>

#include <assert.h>

typedef struct VP8DecoderContext {
  vpx_codec_ctx_t decoder;
} vp8dctx_t;

static av_cold int vp8_init(AVCodecContext* avctx)
{
  vp8dctx_t* const ctx = avctx->priv_data;
  vpx_codec_iface_t* const iface = &vpx_codec_vp8_dx_algo;
  vpx_codec_dec_cfg_t deccfg = { /*token partitions+1 would be a decent choice*/
                                 .threads= FFMIN(avctx->thread_count,16) };
  vp8_postproc_cfg_t ppcfg;
#if (defined(CONFIG_LIBPOSTPROC) && CONFIG_LIBPOSTPROC) ||\
    (defined(CONFIG_POSTPROC) && CONFIG_POSTPROC)
  const vpx_codec_flags_t flags = 0;
#else
  const vpx_codec_flags_t flags = VPX_CODEC_USE_POSTPROC;
#endif

  av_log(avctx,AV_LOG_INFO,"%s\n",vpx_codec_version_str());
  av_log(avctx,AV_LOG_VERBOSE,"%s\n",vpx_codec_build_config());

  if(vpx_codec_dec_init(&ctx->decoder,iface,&deccfg,flags)!=VPX_CODEC_OK) {
    const char* error = vpx_codec_error(&ctx->decoder);
    av_log(avctx,AV_LOG_ERROR,"Failed to initialize decoder: %s\n",error);
    return -1;
  }

  /*FIXME set based on user parameters. for now we'll disable based on
    libpostproc presence in mplayer/ffmpeg based builds*/
  if(flags&VPX_CODEC_USE_POSTPROC) {
    ppcfg.post_proc_flag   = VP8_DEMACROBLOCK|VP8_DEBLOCK|VP8_ADDNOISE;
    ppcfg.deblocking_level = 5;
    ppcfg.noise_level      = 1;
    vpx_codec_control(&ctx->decoder,VP8_SET_POSTPROC,&ppcfg);
  }

  avctx->pix_fmt = PIX_FMT_YUV420P;
  return 0;
}

static int vp8_decode(AVCodecContext* avctx,
                      void* data, int* data_size,
                      AVPacket *avpkt)
{
  const uint8_t* const buf = avpkt->data;
  const int buf_size = avpkt->size;
  vp8dctx_t* const ctx = avctx->priv_data;
  AVFrame* const picture = data;
  vpx_codec_iter_t iter = NULL;
  vpx_image_t* img;

  if(vpx_codec_decode(&ctx->decoder,buf,buf_size,NULL,0)!=VPX_CODEC_OK) {
    const char* error = vpx_codec_error(&ctx->decoder);
    const char* detail = vpx_codec_error_detail(&ctx->decoder);

    av_log(avctx,AV_LOG_ERROR,"Failed to decode frame: %s\n",error);
    if(detail) av_log(avctx,AV_LOG_ERROR,"  Additional information: %s\n",detail);
    return -1;
  }

  if( (img= vpx_codec_get_frame(&ctx->decoder,&iter)) ) {
    assert(img->fmt==IMG_FMT_I420);

    if((int)img->d_w!=avctx->width || (int)img->d_h!=avctx->height) {
      av_log(avctx,AV_LOG_INFO,"dimension change! %dx%d -> %dx%d\n",
        avctx->width,avctx->height,img->d_w,img->d_h);
      if(avcodec_check_dimensions(avctx,img->d_w,img->d_h))
        return -1;
      avcodec_set_dimensions(avctx,img->d_w,img->d_h);
    }
    picture->data[0] = img->planes[0];
    picture->data[1] = img->planes[1];
    picture->data[2] = img->planes[2];
    picture->data[3] = img->planes[3];
    picture->linesize[0] = img->stride[0];
    picture->linesize[1] = img->stride[1];
    picture->linesize[2] = img->stride[2];
    picture->linesize[3] = img->stride[3];
    *data_size = sizeof(AVPicture);
  }
  return buf_size;
}

static av_cold int vp8_free(AVCodecContext *avctx)
{
  vp8dctx_t* const ctx = avctx->priv_data;
  vpx_codec_destroy(&ctx->decoder);
  return 0;
}

AVCodec libvpx_vp8_decoder = {
  "libvpx_vp8",
  AVMEDIA_TYPE_VIDEO,
  CODEC_ID_VP8,
  sizeof(vp8dctx_t),
  vp8_init,
  NULL, /*encode*/
  vp8_free,
  vp8_decode,
  0, /*capabilities*/
  .long_name = NULL_IF_CONFIG_SMALL("libvpx VP8"),
};
