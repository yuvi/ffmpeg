/*
 * copyright (c) 2010 David Conrad
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

#include <ass/ass.h>
#include "avfilter.h"
#include "libavutil/pixdesc.h"

typedef struct {
    ASS_Library *lib;
    ASS_Track *track;
    ASS_Renderer *render;

    int w;
    int h;
    int hsub;
    int vsub;
} ASSContext;

static void ass_log(int level, const char *fmt, va_list args, void *p)
{
    // MSGL_* is private so eh
    static const int level_map[8] = {
        AV_LOG_FATAL,
        AV_LOG_ERROR,
        AV_LOG_WARNING,
        AV_LOG_WARNING, // not defined / unused
        AV_LOG_INFO,
        AV_LOG_INFO,    // not defined / unused
        AV_LOG_VERBOSE,
        AV_LOG_DEBUG
    };

    if (level < 0 || level > 7)
        return;

    av_vlog(p, level_map[level], fmt, args);
}

static av_cold int init(AVFilterContext *ctx, const char *args, void *opaque)
{
    ASSContext *ass = ctx->priv;

    ass->lib = ass_library_init();
    ass_set_message_cb(ass->lib, ass_log, ctx);

    ass->render = ass_renderer_init(ass->lib);
    if (args)
        ass->track = ass_read_file(ass->lib, args, NULL);
    else
        ass->track = ass_read_file(ass->lib, "/sns.ass", NULL);
    return !(ass->track && ass->render && ass->lib);
}

static av_cold void uninit(AVFilterContext *ctx)
{
    ASSContext *ass = ctx->priv;

    ass_free_track(ass->track);
    ass_renderer_done(ass->render);
    ass_library_done(ass->lib);

    ass->lib = ass->track = ass->renderer = NULL;
}

static int query_formats(AVFilterContext *ctx)
{
    static const enum PixelFormat pix_fmts[] = {
        PIX_FMT_YUV420P,
        PIX_FMT_YUVJ420P,
        PIX_FMT_NONE
    };

    avfilter_set_common_formats(ctx, avfilter_make_format_list(pix_fmts));
    return 0;
}

static int config_input(AVFilterLink *link)
{
    AVFilterContext *ctx = link->dst;
    ASSContext *ass = ctx->priv;
    avcodec_get_chroma_sub_sample(link->format, &ass->hsub, &ass->vsub);
    return 0;
}

static void end_frame(AVFilterLink *link)
{
    ASSContext *ass = link->dst->priv;
    AVFilterLink *output = link->dst->outputs[0];
    AVFilterPicRef *pic = link->cur_pic;
    int64_t time_ms = pic->pts * 1000/AV_TIME_BASE;
    ASS_Image *imgs;

    if (ass->w != pic->w || ass->h != pic->h) {
        ass->w = pic->w;
        ass->h = pic->h;
        ass_set_frame_size(ass->render, ass->w, ass->h);
    }

    imgs = ass_render_frame(ass->render, ass->track, time_ms, NULL);

    avfilter_draw_slice(output, 0, pic->h, 1);
    avfilter_end_frame(output);
}

AVFilter avfilter_vf_ass = {
    .name      = "ass",
    .description = "Add subtitles to a video stream.",

    .init      = init,
    .uninit    = uninit,

    .query_formats = query_formats,

    .priv_size = sizeof(ASSContext),

    .inputs    = (AVFilterPad[]) {{ .name             = "default",
                                    .type             = AVMEDIA_TYPE_VIDEO,
                                    // .start_frame      = start_frame,
                                    .end_frame        = end_frame,
                                    .min_perms        = AV_PERM_WRITE | AV_PERM_READ,
                                    .rej_perms        = AV_PERM_REUSE | AV_PERM_REUSE2,
                                    .config_props     = config_input, },
                                  { .name = NULL }},
    .outputs   = (AVFilterPad[]) {{ .name             = "default",
                                    .type             = AVMEDIA_TYPE_VIDEO, },
                                  { .name = NULL }},
};

static int query_formats_src(AVFilterContext *ctx)
{
    static const enum PixelFormat pix_fmts[] = {
        PIX_FMT_BGR32,
        PIX_FMT_NONE
    };

    avfilter_set_common_formats(ctx, avfilter_make_format_list(pix_fmts));
    return 0;
}

AVFilter avfilter_vsrc_ass = {
    .name      = "ass_src",
    .description = "Generate a set of RGBA images from a subtitle stream.",

    .init      = init,
    .uninit    = uninit,

    .query_formats = query_formats,

    .priv_size = sizeof(ASSContext),

    .inputs    = (AVFilterPad[]) {{ .name = NULL }},
    .outputs   = (AVFilterPad[]) {{ .name             = "default",
                                    .type             = AVMEDIA_TYPE_VIDEO, },
                                  { .name = NULL }},
};
