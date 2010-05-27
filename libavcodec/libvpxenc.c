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
   \brief VP8 encoding support via libvpx

   \par VPx SDK param./#AVCodecContext param./Assoc. cli opt {{{
   <table>
   <tr><td>g_threads</td><td>AVCodecContext::thread_count</td><td>-threads</td></tr>
   <tr><td>g_lag_in_frames</td><td>AVCodecContext::lag</td><td>-lag</td></tr>
   <tr><td>rc_target_bitrate</td><td>AVCodecContext::bit_rate/1000</td><td>-vb</td></tr>
   <tr><td>kf_max_dist</td><td>AVCodecContext::gop_size</td><td>-g</td></tr>
   <tr><td>kf_min_dist</td><td>AVCodecContext::keyint_min</td><td>-keyint_min</td></tr>
   <tr><td><code>VPX_KF_FIXED</code></td><td>AVCodecContext::keyint_min == AVCodecContext::gop_size</td><td>-keyint_min -g</td></tr>
   <tr><td>rc_min_quantizer</td><td>((AVCodecContext::qmin*5+1)&gt;&gt;2) - 1</td><td>-qmin</td></tr>
   <tr><td>rc_max_quantizer</td><td>((AVCodecContext::qmax*5+1)&gt;&gt;2) - 1</td><td>-qmax</td></tr>
   <tr><td>rc_buf_sz</td><td>AVCodecContext::rc_buffer_size*1000 / AVCodecContext::bit_rate</td><td>-bufsize -vb</td></tr>
   <tr><td>rc_buf_initial_sz</td><td>AVCodecContext::rc_initial_buffer_occupancy*1000 / AVCodecContext::bit_rate</td><td>-rc_init_occupancy -vb</td></tr>
   <tr><td>rc_buf_optimal_sz</td><td>AVCodecContext::rc_optimal_buffer_occupancy*1000 / AVCodecContext::bit_rate</td><td>-rc_opt_occupancy -vb</td></tr>
   <tr><td>rc_dropframe_thresh</td><td>AVCodecContext::frame_skip_threshold</td><td>-skip_threshold</td></tr>
   <tr><td>rc_undershoot_pct</td><td>AVCodecContext::rc_buffer_aggressivity*100</td><td>-rc_buf_aggressivity</td></tr>
   <tr><td>rc_overshoot_pct</td><td>AVCodecContext::bit_rate_tolerance*100 / AVCodecContext::bit_rate</td><td>-bt -vb</td></tr>
   <tr><td>rc_resize_allowed</td><td>AVCodecContext::spatial_rsmpl</td><td>-spatial_rsmpl</td></tr>
   <tr><td>rc_resize_up_thresh</td><td>AVCodecContext::spatial_rsmpl_up</td><td>-spatial_rsmpl_up</td></tr>
   <tr><td>rc_resize_down_thresh</td><td>AVCodecContext::spatial_rsmpl_down</td><td>-spatial_rsmpl_down</td></tr>
   <tr><td>rc_2pass_vbr_bias_pct</td><td>AVCodecContext::vbr_bias</td><td>-vbr_bias</td></tr>
   <tr><td>rc_2pass_vbr_maxsection_pct</td><td>AVCodecContext::rc_max_rate*100 / AVCodecContext::bit_rate</td><td>-maxrate -vb</td></tr>
   <tr><td>rc_2pass_vbr_minsection_pct</td><td>AVCodecContext::rc_min_rate*100 / AVCodecContext::bit_rate</td><td>-minrate -vb</td></tr>
   <tr><td>VPX_CBR</td><td>AVCodecContext::rc_min_rate == AVCodecContext::rc_max_rate &amp;&amp; AVCodecContext::rc_min_rate == AVCodecContext::bit_rate</td><td>-minrate -maxrate -vb</td></tr>
   <tr><td>g_profile</td><td>AVCodecContext::profile</td><td>-profile</td></tr>
   <tr><td>g_error_resilient</td><td>AVCodecContext::level&lt;0</td><td>-level</td></tr>
   <tr><td><code>VPX_DL_BEST_QUALITY(1)/VPX_DL_GOOD_QUALITY(2)/VPX_DL_REALTIME(3)</code></td><td>#FFABS(AVCodecContext::level)/100</td><td>-level</td></tr>
   <tr><td><code>VP8E_SET_CPUUSED</code></td><td>#FFABS(AVCodecContext::level)%%100-16</td><td>-level</td></tr>
   <tr><td><code>VP8E_SET_NOISE_SENSITIVITY</code></td><td>AVCodecContext::noise_reduction</td><td>-nr</td></tr>
   <tr><td><code>VP8E_SET_SHARPNESS</code></td><td>AVCodecContext::sharpness</td><td>-sharpness</td></tr>
   <tr><td><code>VP8E_SET_ENABLEAUTOALTREF</code></td><td>AVCodecContext::altref</td><td>-altref</td></tr>
   <tr><td><code>VP8E_SET_ARNR_MAXFRAMES</code></td><td>AVCodecContext::ar_max_frames</td><td>-ar_max_frames</td></tr>
   <tr><td><code>VP8E_SET_ARNR_TYPE</code></td><td>AVCodecContext::ar_type</td><td>-ar_type</td></tr>
   <tr><td><code>VP8E_SET_ARNR_STRENGTH</code></td><td>AVCodecContext::ar_strength</td><td>-ar_strength</td></tr>
   <tr><td><code>VP8E_SET_STATIC_THRESHOLD</code></td><td>AVCodecContext::mb_static_threshold</td><td>-mb_static_threshold</td></tr>
   <tr><td><code>VP8E_SET_TOKEN_PARTITIONS</code></td><td>AVCodecContext::token_partitions</td><td>-token_partitions</td></tr>
   </table> }}}
*/

#include "avcodec.h"
#include "libavutil/base64.h"

#ifndef HAVE_STDINT_H
# define HAVE_STDINT_H 1
#endif
#define VPX_CODEC_DISABLE_COMPAT 1
#include <vpx/vpx_encoder.h>
#include <vpx/vp8cx.h>

/*!Portion of #vpx_codec_cx_pkt_t from vpx_encoder.h.
   One encoded frame returned from the library.*/
typedef struct FrameListData {
  void                    *buf;      /**< compressed data buffer */
  size_t                   sz;       /**< length of compressed data */
  vpx_codec_pts_t          pts;      /**< time stamp to show frame
                                          (in timebase units) */
  unsigned long            duration; /**< duration to show frame
                                          (in timebase units) */
  vpx_codec_frame_flags_t  flags;    /**< flags for this frame */
  struct FrameListData    *next;
} coded_frame_t;

typedef struct VP8EncoderContext {
  vpx_codec_ctx_t encoder;
  vpx_image_t rawimg;
  vpx_fixed_buf_t twopass_stats;
  unsigned long deadline; //i.e., RT/GOOD/BEST
  coded_frame_t* coded_frame_list;
} vp8ctx_t;

static int vp8_free(AVCodecContext *avctx);
static void free_frame_list(coded_frame_t* list);
static void free_coded_frame(coded_frame_t* cx_frame);
static void coded_frame_add(void* list, coded_frame_t* cx_frame);
static void log_encoder_error(AVCodecContext *avctx, const char* desc);
static void dump_enc_cfg(AVCodecContext *avctx, const vpx_codec_enc_cfg_t* cfg);

static av_cold int vp8_init(AVCodecContext *avctx)
{
  vp8ctx_t* const ctx = avctx->priv_data;
  vpx_codec_iface_t* const iface = &vpx_codec_vp8_cx_algo;
  const int log_field_width = -30;
  const unsigned int usage = 0; //should be 0 for current VP8
  const vpx_codec_flags_t flags = 0;
  int cpuused = 3;
  vpx_codec_enc_cfg_t enccfg;
  vpx_codec_err_t res;

  av_log(avctx,AV_LOG_INFO,"%s\n",vpx_codec_version_str());
  av_log(avctx,AV_LOG_VERBOSE,"%s\n",vpx_codec_build_config());

  if((res= vpx_codec_enc_config_default(iface,&enccfg,usage))!=VPX_CODEC_OK) {
    av_log(avctx,AV_LOG_ERROR,"Failed to get config: %s\n",vpx_codec_err_to_string(res));
    return -1;
  }
  dump_enc_cfg(avctx,&enccfg);

  enccfg.g_w = avctx->width;
  enccfg.g_h = avctx->height;

  /*With altref set an additional frame at the same pts may be produced.
    Increasing the time_base gives the library a window to place these frames
    ensuring strictly increasing timestamps.*/
  if(avctx->altref) {
    avctx->ticks_per_frame = 2;
    avctx->time_base = av_mul_q(avctx->time_base,(AVRational){1,avctx->ticks_per_frame});
  }
  enccfg.g_timebase.num = avctx->time_base.num;
  enccfg.g_timebase.den = avctx->time_base.den;

  enccfg.g_threads = FFMIN(avctx->thread_count,64);
  if(avctx->flags&CODEC_FLAG_PASS1) enccfg.g_pass= VPX_RC_FIRST_PASS;
  else if(avctx->flags&CODEC_FLAG_PASS2) enccfg.g_pass= VPX_RC_LAST_PASS;
  else enccfg.g_pass= VPX_RC_ONE_PASS;

  enccfg.rc_resize_allowed     = avctx->spatial_rsmpl;
  enccfg.rc_resize_up_thresh   = avctx->spatial_rsmpl_up;
  enccfg.rc_resize_down_thresh = avctx->spatial_rsmpl_down;
  enccfg.rc_dropframe_thresh   = avctx->frame_skip_threshold;

  if(avctx->rc_min_rate==avctx->rc_max_rate && avctx->rc_min_rate==avctx->bit_rate)
    enccfg.rc_end_usage = VPX_CBR;
  enccfg.rc_target_bitrate = (unsigned int)av_rescale_rnd(avctx->bit_rate,1,1000,AV_ROUND_NEAR_INF);
  enccfg.rc_overshoot_pct = avctx->bit_rate_tolerance*100/avctx->bit_rate;

  //convert [1,51] -> [0,63]
  enccfg.rc_min_quantizer = ((avctx->qmin*5+1)>>2) - 1;
  enccfg.rc_max_quantizer = ((avctx->qmax*5+1)>>2) - 1;
  if(avctx->rc_buffer_size)
    enccfg.rc_buf_sz = avctx->rc_buffer_size*1000/avctx->bit_rate;
  if(avctx->rc_initial_buffer_occupancy)
    enccfg.rc_buf_initial_sz = avctx->rc_initial_buffer_occupancy*1000/avctx->bit_rate;
  if(avctx->rc_optimal_buffer_occupancy)
    enccfg.rc_buf_optimal_sz = avctx->rc_optimal_buffer_occupancy*1000/avctx->bit_rate;
  if(avctx->rc_buffer_aggressivity)
    enccfg.rc_undershoot_pct = (unsigned int)round(avctx->rc_buffer_aggressivity*100);

  enccfg.rc_2pass_vbr_bias_pct= avctx->vbr_bias;
  if(avctx->rc_min_rate)
    enccfg.rc_2pass_vbr_minsection_pct = avctx->rc_min_rate*100/avctx->bit_rate;
  if(avctx->rc_max_rate)
    enccfg.rc_2pass_vbr_maxsection_pct = avctx->rc_max_rate*100/avctx->bit_rate;

  if(avctx->keyint_min==avctx->gop_size) enccfg.kf_mode= VPX_KF_FIXED;
  //_enc_init() will balk if kf_min_dist is set in this case
  if(enccfg.kf_mode!=VPX_KF_AUTO) enccfg.kf_min_dist= avctx->keyint_min;
  enccfg.kf_max_dist = avctx->gop_size;
  enccfg.g_lag_in_frames= avctx->lag;

  if(enccfg.g_pass==VPX_RC_FIRST_PASS) enccfg.g_lag_in_frames= 0;
  if(enccfg.g_pass==VPX_RC_LAST_PASS) {
    int decode_size;

    if(!avctx->stats_in) {
      av_log(avctx,AV_LOG_ERROR,"No stats file for second pass\n");
      return -1;
    }

    ctx->twopass_stats.sz  = strlen(avctx->stats_in) * 3/4;
    ctx->twopass_stats.buf = av_malloc(ctx->twopass_stats.sz);
    if(!ctx->twopass_stats.buf) {
      av_log(avctx,AV_LOG_ERROR,"Stat buffer alloc (%zu bytes) failed\n",ctx->twopass_stats.sz);
      return AVERROR(ENOMEM);
    }
    decode_size =
      av_base64_decode(ctx->twopass_stats.buf, avctx->stats_in, ctx->twopass_stats.sz);
    if(decode_size<0) {
      av_log(avctx,AV_LOG_ERROR,"Stat buffer decode failed\n");
      return -1;
    }

    ctx->twopass_stats.sz      = decode_size;
    enccfg.rc_twopass_stats_in = ctx->twopass_stats;
  }

  if(avctx->profile!=FF_PROFILE_UNKNOWN) enccfg.g_profile= avctx->profile;
  switch(FFABS(avctx->level)/100) {
  case 1:          ctx->deadline = VPX_DL_BEST_QUALITY; break;
  case 2: default: ctx->deadline = VPX_DL_GOOD_QUALITY; break;
  case 3:          ctx->deadline = VPX_DL_REALTIME; break;
  }
  av_log(avctx,AV_LOG_DEBUG,"Using deadline: %lu\n",ctx->deadline);

  if(avctx->level!=FF_LEVEL_UNKNOWN) {
    enccfg.g_error_resilient = avctx->level<0;
    cpuused = FFABS(avctx->level)%100-16;
  }

  dump_enc_cfg(avctx,&enccfg);
  /* Construct Encoder Context */
  res = vpx_codec_enc_init(&ctx->encoder,iface,&enccfg,flags);
  if(res!=VPX_CODEC_OK) { log_encoder_error(avctx,"Failed to initialize encoder"); return -1; }

  //codec control failures are currently treated only as warnings
  av_log(avctx,AV_LOG_DEBUG,"vpx_codec_control\n");
  #define codecctl(id,val)\
    do if(av_log(avctx,AV_LOG_DEBUG,"%*s%d\n",log_field_width," "#id":",val),\
          (res= vpx_codec_control(&ctx->encoder,id,val))!=VPX_CODEC_OK) {\
      log_encoder_error(avctx,"Failed to set "#id" codec control");\
    } while(0);
  codecctl(VP8E_SET_CPUUSED,cpuused);
  codecctl(VP8E_SET_NOISE_SENSITIVITY,avctx->noise_reduction);
  codecctl(VP8E_SET_SHARPNESS,avctx->sharpness);
  codecctl(VP8E_SET_ENABLEAUTOALTREF,avctx->altref);
  codecctl(VP8E_SET_ARNR_MAXFRAMES,avctx->ar_max_frames);
  codecctl(VP8E_SET_ARNR_TYPE,avctx->ar_type);
  codecctl(VP8E_SET_ARNR_STRENGTH,avctx->ar_strength);
  codecctl(VP8E_SET_STATIC_THRESHOLD,avctx->mb_static_threshold);
  codecctl(VP8E_SET_TOKEN_PARTITIONS,av_log2(avctx->token_partitions));
  #undef codecctl

  //provide dummy value to initialize wrapper, values will be updated each _encode()
  vpx_img_wrap(&ctx->rawimg,VPX_IMG_FMT_I420,avctx->width,avctx->height,1,(unsigned char*)1);

  avctx->coded_frame = avcodec_alloc_frame();
  if(!avctx->coded_frame) {
    av_log(avctx,AV_LOG_ERROR,"Error allocating coded frame\n");
    vp8_free(avctx);
    return AVERROR(ENOMEM);
  }
  return 0;
}

static inline void cx_pktcpy(coded_frame_t* dst, const vpx_codec_cx_pkt_t* src)
{
  dst->pts      = src->data.frame.pts;
  dst->duration = src->data.frame.duration;
  dst->flags    = src->data.frame.flags;
  dst->sz       = src->data.frame.sz;
  dst->buf      = src->data.frame.buf;
}

/*!Store coded frame information in format suitable for return from encode().
   Write buffer information from \a cx_frame to \a buf & \a buf_size.
   Timing/frame details to \a coded_frame.
   \return Frame size written to \a buf on success
   \return -1 on error*/
static int storeframe(AVCodecContext *avctx, coded_frame_t* cx_frame,
                      uint8_t* buf, int buf_size, AVFrame* coded_frame)
{
  if((int)cx_frame->sz<=buf_size) {
    buf_size = cx_frame->sz;
    memcpy(buf,cx_frame->buf,buf_size);
    coded_frame->pts = cx_frame->pts;
    coded_frame->key_frame = !!(cx_frame->flags&VPX_FRAME_IS_KEY);

    if(coded_frame->key_frame)
      coded_frame->pict_type = FF_I_TYPE;
    else
      coded_frame->pict_type = FF_P_TYPE;
  } else {
    av_log(avctx,AV_LOG_ERROR,
      "Compressed frame larger than storage provided! (%zu/%d)\n",
      cx_frame->sz,buf_size);
    return -1;
  }
  return buf_size;
}

/*!Queue multiple output frames from the encoder, returning the front-most.
   In cases where vpx_codec_get_cx_data() returns more than 1 frame append
   the frame queue. Return the head frame if available.
   \return Stored frame size
   \return -1 on error*/
static int queue_frames(AVCodecContext *avctx, uint8_t* buf, int buf_size, AVFrame* coded_frame)
{
  vp8ctx_t* const ctx = avctx->priv_data;
  const vpx_codec_cx_pkt_t* pkt;
  vpx_codec_iter_t iter = NULL;
  int size = 0;

  if(ctx->coded_frame_list) {
    coded_frame_t* const cx_frame = ctx->coded_frame_list;
    /*return the leading frame if we've already begun queueing*/
    size = storeframe(avctx,cx_frame,buf,buf_size,coded_frame);
    if(size<0)
      return -1;
    ctx->coded_frame_list = cx_frame->next;
    free_coded_frame(cx_frame);
  }

  /*consume all available output from the encoder before returning. buffers
    are only good through the next vpx_codec call*/
  while( (pkt= vpx_codec_get_cx_data(&ctx->encoder,&iter)) ) {
    switch(pkt->kind) {
    case VPX_CODEC_CX_FRAME_PKT: {
      if(!size) {
        coded_frame_t cx_frame;

        /*avoid storing the frame when the list is empty and we haven't yet
          provided a frame for output*/
        assert(!ctx->coded_frame_list);
        cx_pktcpy(&cx_frame,pkt);
        size = storeframe(avctx,&cx_frame,buf,buf_size,coded_frame);
        if(size<0)
          return -1;
      } else {
        coded_frame_t* const cx_frame = av_malloc(sizeof(coded_frame_t));

        if(!cx_frame) {
          av_log(avctx,AV_LOG_ERROR,"Frame queue element alloc failed\n");
          return AVERROR(ENOMEM);
        }
        cx_pktcpy(cx_frame,pkt);
        cx_frame->buf = av_malloc(cx_frame->sz);

        if(!cx_frame->buf) {
          av_log(avctx,AV_LOG_ERROR,"Data buffer alloc (%zu bytes) failed\n",
            cx_frame->sz);
          return AVERROR(ENOMEM);
        }
        memcpy(cx_frame->buf,pkt->data.frame.buf,pkt->data.frame.sz);
        coded_frame_add(&ctx->coded_frame_list,cx_frame);
      }
      break; }
    case VPX_CODEC_STATS_PKT: {
      vpx_fixed_buf_t* const stats = &ctx->twopass_stats;
      stats->buf = av_realloc(stats->buf,stats->sz+pkt->data.twopass_stats.sz);
      if(!stats->buf) {
        av_log(avctx,AV_LOG_ERROR,"Stat buffer realloc failed\n");
        return AVERROR(ENOMEM);
      }
      memcpy((uint8_t*)stats->buf + stats->sz,
          pkt->data.twopass_stats.buf,
          pkt->data.twopass_stats.sz);
      stats->sz += pkt->data.twopass_stats.sz;
      break; }
    case VPX_CODEC_CUSTOM_PKT:
      //ignore unsupported/unrecognized packet types
      break;
    }
  }

  return size;
}

static int vp8_encode(AVCodecContext *avctx, uint8_t *buf, int buf_size, void *data)
{
  vp8ctx_t* const ctx = avctx->priv_data;
  AVFrame* const frame = data;
  vpx_image_t* rawimg = NULL;
  int coded_size;

  if(frame) {
    rawimg = &ctx->rawimg;
    rawimg->planes[VPX_PLANE_Y] = frame->data[0];
    rawimg->planes[VPX_PLANE_U] = frame->data[1];
    rawimg->planes[VPX_PLANE_V] = frame->data[2];
    rawimg->stride[VPX_PLANE_Y] = frame->linesize[0];
    rawimg->stride[VPX_PLANE_U] = frame->linesize[1];
    rawimg->stride[VPX_PLANE_V] = frame->linesize[2];
  }

  { vpx_codec_err_t res;
  const vpx_codec_pts_t timestamp = frame?frame->pts:0;
  const unsigned long framedur = avctx->ticks_per_frame;
  const vpx_enc_frame_flags_t flags = 0; //VPX_EFLAG_FORCE_KF
  const unsigned long deadline = ctx->deadline;

  //FIXME use vpx_codec_set_cx_data_buf?
  res = vpx_codec_encode(&ctx->encoder,rawimg,timestamp,framedur,flags,deadline);
  if(res!=VPX_CODEC_OK) { log_encoder_error(avctx,"Error encoding frame"); return -1; } }
  coded_size = queue_frames(avctx,buf,buf_size,avctx->coded_frame);

  if(!frame && avctx->flags&CODEC_FLAG_PASS1) {
    const unsigned int b64_size = ((ctx->twopass_stats.sz + 2) / 3) * 4 + 1;

    avctx->stats_out = av_malloc(b64_size);
    if(!avctx->stats_out) {
      av_log(avctx,AV_LOG_ERROR,"Stat buffer alloc (%d bytes) failed\n",b64_size);
      return AVERROR(ENOMEM);
    }
    av_base64_encode(avctx->stats_out, b64_size, ctx->twopass_stats.buf, ctx->twopass_stats.sz);
  }
  return coded_size;
}

static av_cold int vp8_free(AVCodecContext *avctx)
{
  vp8ctx_t* const ctx = avctx->priv_data;

  vpx_codec_destroy(&ctx->encoder);
  av_freep(&ctx->twopass_stats.buf);
  av_freep(&avctx->coded_frame);
  av_freep(&avctx->stats_out);
  free_frame_list(ctx->coded_frame_list);
  return 0;
}

static av_cold void free_coded_frame(coded_frame_t* cx_frame)
{
  av_freep(&cx_frame->buf);
  av_freep(&cx_frame);
}

static av_cold void free_frame_list(coded_frame_t* list)
{
  coded_frame_t* p = list;

  while(p) {
    list = list->next;
    free_coded_frame(p);
    p = list;
  }
}

static void coded_frame_add(void* list, coded_frame_t* cx_frame)
{
  coded_frame_t** p = list;

  while(*p!=NULL) p= &(*p)->next;
  *p = cx_frame;
  cx_frame->next = NULL;
}

static av_cold void log_encoder_error(AVCodecContext *avctx, const char* desc)
{
  vp8ctx_t* const ctx = avctx->priv_data;
  const char* error = vpx_codec_error(&ctx->encoder);
  const char* detail = vpx_codec_error_detail(&ctx->encoder);

  av_log(avctx,AV_LOG_ERROR,"%s: %s\n",desc,error);
  if(detail) av_log(avctx,AV_LOG_ERROR,"  Additional information: %s\n",detail);
}

static av_cold void dump_enc_cfg(AVCodecContext *avctx, const vpx_codec_enc_cfg_t* cfg)
{
  const int width = -30;
  const int level = AV_LOG_DEBUG;

  av_log(avctx,level,"vpx_codec_enc_cfg\n");
  av_log(avctx,level," %*s%u\n",      width, " g_usage:", cfg->g_usage);
  av_log(avctx,level," %*s%u\n",      width, " g_threads:", cfg->g_threads);
  av_log(avctx,level," %*s%u\n",      width, " g_profile:", cfg->g_profile);
  av_log(avctx,level," %*s%u\n",      width, " g_w:", cfg->g_w);
  av_log(avctx,level," %*s%u\n",      width, " g_h:", cfg->g_h);
  av_log(avctx,level," %*s{%u/%u}\n", width, " g_timebase:", cfg->g_timebase.num, cfg->g_timebase.den);
  av_log(avctx,level," %*s%u\n",      width, " g_error_resilient:", cfg->g_error_resilient);
  av_log(avctx,level," %*s%d\n",      width, " g_pass:", cfg->g_pass);
  av_log(avctx,level," %*s%u\n",      width, " g_lag_in_frames:", cfg->g_lag_in_frames);
  av_log(avctx,level," %*s%u\n",      width, " rc_dropframe_thresh:", cfg->rc_dropframe_thresh);
  av_log(avctx,level," %*s%u\n",      width, " rc_resize_allowed:", cfg->rc_resize_allowed);
  av_log(avctx,level," %*s%u\n",      width, " rc_resize_up_thresh:", cfg->rc_resize_up_thresh);
  av_log(avctx,level," %*s%u\n",      width, " rc_resize_down_thresh:", cfg->rc_resize_down_thresh);
  av_log(avctx,level," %*s%d\n",      width, " rc_end_usage:", cfg->rc_end_usage);
  av_log(avctx,level," %*s%p(%zu)\n", width, " rc_twopass_stats_in:", cfg->rc_twopass_stats_in.buf, cfg->rc_twopass_stats_in.sz);
  av_log(avctx,level," %*s%u\n",      width, " rc_target_bitrate:", cfg->rc_target_bitrate);
  av_log(avctx,level," %*s%u\n",      width, " rc_min_quantizer:", cfg->rc_min_quantizer);
  av_log(avctx,level," %*s%u\n",      width, " rc_max_quantizer:", cfg->rc_max_quantizer);
  av_log(avctx,level," %*s%u\n",      width, " rc_undershoot_pct:", cfg->rc_undershoot_pct);
  av_log(avctx,level," %*s%u\n",      width, " rc_overshoot_pct:", cfg->rc_overshoot_pct);
  av_log(avctx,level," %*s%u\n",      width, " rc_buf_sz:", cfg->rc_buf_sz);
  av_log(avctx,level," %*s%u\n",      width, " rc_buf_initial_sz:", cfg->rc_buf_initial_sz);
  av_log(avctx,level," %*s%u\n",      width, " rc_buf_optimal_sz:", cfg->rc_buf_optimal_sz);
  av_log(avctx,level," %*s%u\n",      width, " rc_2pass_vbr_bias_pct:", cfg->rc_2pass_vbr_bias_pct);
  av_log(avctx,level," %*s%u\n",      width, " rc_2pass_vbr_minsection_pct:", cfg->rc_2pass_vbr_minsection_pct);
  av_log(avctx,level," %*s%u\n",      width, " rc_2pass_vbr_maxsection_pct:", cfg->rc_2pass_vbr_maxsection_pct);
  av_log(avctx,level," %*s%d\n",      width, " kf_mode:", cfg->kf_mode);
  av_log(avctx,level," %*s%u\n",      width, " kf_min_dist:", cfg->kf_min_dist);
  av_log(avctx,level," %*s%u\n",      width, " kf_max_dist:", cfg->kf_max_dist);
  av_log(avctx,level,"\n");
}

AVCodec libvpx_encoder = {
  "libvpx",
  AVMEDIA_TYPE_VIDEO,
  CODEC_ID_VP8,
  sizeof(vp8ctx_t),
  vp8_init,
  vp8_encode,
  vp8_free,
  NULL,
  CODEC_CAP_DELAY,
  .pix_fmts  = (const enum PixelFormat[]) { PIX_FMT_YUV420P, PIX_FMT_NONE },
  .long_name = NULL_IF_CONFIG_SMALL("libvpx VP8"),
};
