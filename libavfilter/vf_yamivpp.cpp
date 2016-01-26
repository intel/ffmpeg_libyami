/*
 * Intel Yet Another Media Infrastructure video post process filter
 *
 * Copyright (c) 2016 Jun Zhao
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
 * @file
 * Yet Another Media Infrastructure video post process filter
 *
 * @see https://github.com/01org/libyami
 */
#include <pthread.h>
#include <unistd.h>
#include <assert.h>
#include <float.h>
#include <deque>

extern "C" {
#include "libavutil/avassert.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"
}
#include "VideoPostProcessHost.h"

using namespace YamiMediaCodec;

typedef struct {
    const AVClass *cls;
    int out_width;
    int out_height;

    int dpic;            // destination picture structure
                         // -1 = unkown
                         // 0 = interlaced top field first
                         // 1 = progressive
                         // 2 = interlaced bottom field first

    int deinterlace;     // deinterlace mode : 0=off, 1=bob, 2=advanced
    int denoise;         // enable denoise algo. level is the optional value from the interval [0; 100]

    int cur_out_idx;     // current surface in index

    int frame_number;

    int use_frc;          // use frame rate conversion
    AVRational framerate; // target frame rate
} YamivppContext;

#define OFFSET(x) offsetof(YamivppContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
static const AVOption yamivpp_options[] = {
    {"deinterlace", "deinterlace mode: 0=off, 1=bob, 2=advanced", OFFSET(deinterlace), AV_OPT_TYPE_INT, {.i64=0}, 0, 2, .flags = FLAGS},
    {"denoise", "denoise level [0, 100]", OFFSET(denoise), AV_OPT_TYPE_INT, {.i64=0}, 0, 100, .flags = FLAGS},
    {"w", "Output video width", OFFSET(out_width), AV_OPT_TYPE_INT, {.i64=0}, 0, 4096, .flags = FLAGS},
    {"width", "Output video width", OFFSET(out_width), AV_OPT_TYPE_INT, {.i64=0}, 0, 4096, .flags = FLAGS},
    {"h", "Output video height", OFFSET(out_height), AV_OPT_TYPE_INT, {.i64=0}, 0, 2304, .flags = FLAGS},
    {"height", "Output video height", OFFSET(out_height), AV_OPT_TYPE_INT, {.i64=0}, 0, 2304, .flags = FLAGS},
    {"framerate", "output frame rate", OFFSET(framerate), AV_OPT_TYPE_RATIONAL, { .dbl = 25.0 },0, DBL_MAX, .flags = FLAGS},
    { NULL }
};

static const AVClass yamivpp_class = {
    .class_name                = "yamivpp",
    .item_name                 = av_default_item_name,
    .option                    = yamivpp_options,
    .version                   = LIBAVUTIL_VERSION_INT,
    .log_level_offset_offset   = 0,
    .parent_log_context_offset = 0,
    .child_next                = NULL,
    .child_class_next          = NULL,
    .category                  = AV_CLASS_CATEGORY_FILTER,
    .get_category              = NULL,
    .query_ranges              = NULL,
};

static av_cold int yamivpp_init(AVFilterContext *ctx)
{
    YamivppContext *yamivpp = (YamivppContext *)ctx->priv;

    return 0;
}

static int yamivpp_query_formats(AVFilterContext *ctx)
{
    const YamivppContext *yamivpp = (YamivppContext *)ctx->priv;
    static const enum AVPixelFormat pix_fmts[] = {
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_NV12,
        AV_PIX_FMT_YUYV422,
        AV_PIX_FMT_RGB32 ,
        AV_PIX_FMT_NONE
    };

    //ff_set_common_formats(ctx, ff_make_format_list(pix_fmts));
    return 0;
}

static int config_props(AVFilterLink *inlink)
{
    int p;
    AVFilterContext *ctx = (AVFilterContext *)inlink->dst;
    YamivppContext *yamivpp =  (YamivppContext *)ctx->priv;

    return 0;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = (AVFilterContext *)inlink->dst;
    YamivppContext *yamivpp = (YamivppContext *)ctx->priv;
    AVFilterLink *outlink = (AVFilterLink *)ctx->outputs[0];
    int p, direct = 0;
    AVFrame *out;
    return ff_filter_frame(outlink, out);
}

static av_cold void yamivpp_uninit(AVFilterContext *ctx)
{
    int p;
    YamivppContext *yamivpp = (YamivppContext *)ctx->priv;

    return;
}

static const AVFilterPad yamivpp_inputs[] = {
    {
        .name             = "default",     // name
        .type             = AVMEDIA_TYPE_VIDEO, // type
        .get_video_buffer = NULL,          // get_video_buffer
        .get_audio_buffer = NULL,          // get_audio_buffer
        .filter_frame     = filter_frame,  // filter_frame
        .poll_frame       = NULL,          // poll_frame
        .request_frame    = NULL,          // request_frame
        .config_props     = config_props,  // config_props
        .needs_fifo       = 0,             // needs_fifo
        .needs_writable   = 0,             // needs_writable
    },
    { NULL }
};

static const AVFilterPad yamivpp_outputs[] = {
    {
        .name             = "default",
        .type             = AVMEDIA_TYPE_VIDEO,
        .get_video_buffer = NULL,
        .get_audio_buffer = NULL,
        .filter_frame     = NULL,
        .poll_frame       = NULL,
        .request_frame    = NULL,
        .config_props     = NULL,
        .needs_fifo       = 0,
        .needs_writable   = 0,
    },
    { NULL }
};

AVFilter ff_vf_yamivpp = {
    .name            = "yamivpp",
    .description     = NULL_IF_CONFIG_SMALL("Libyami Video VPP."),
    .inputs          = yamivpp_inputs,
    .outputs         = yamivpp_outputs,
    .priv_class      = &yamivpp_class,
    .flags           = AVFILTER_FLAG_SUPPORT_TIMELINE_GENERIC,
    .init            = yamivpp_init,
    .init_dict       = NULL,
    .uninit          = yamivpp_uninit,
    .query_formats   = yamivpp_query_formats,
    .priv_size       = sizeof(YamivppContext),
    .next            = NULL,
    .process_command = NULL,
    .init_opaque     = NULL,
};
