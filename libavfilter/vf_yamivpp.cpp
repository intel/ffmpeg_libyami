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
 * Yet Another Media Infrastructure video post processing filter
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
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "avfilter.h"
#include "formats.h"
#include "internal.h"
#include "video.h"
}
#include "VideoPostProcessHost.h"
#include "libavcodec/libyami.h"

using namespace YamiMediaCodec;

typedef struct {
    const AVClass *cls;

    IVideoPostProcess *scaler;
    SharedPtr < VideoFrame > src;
    SharedPtr < VideoFrame > dest;

    int out_width;
    int out_height;

    int dpic;            // destination picture structure
                         // -1 = unkown
                         // 0 = interlaced top field first
                         // 1 = progressive
                         // 2 = interlaced bottom field first

    int deinterlace;     // deinterlace mode : 0=off, 1=bob, 2=advanced
    int denoise;         // enable denoise algo. level is the optional value from the interval [0; 100]
    int sharpness;       // enable sharpness. level is the optional value from the interval [0; 100]

    int cur_out_idx;     // current surface in index

    int frame_number;

    int use_frc;         // use frame rate conversion

    int pipeline;        // is vpp in HW pipeline?
    AVRational framerate;// target frame rate
} YamivppContext;

#include <fcntl.h>
#include <unistd.h>

#include <va/va_drm.h>
#if HAVE_VAAPI_X11
#include <va/va_x11.h>
#endif
#define HAVE_VAAPI_DRM 1

#if HAVE_VAAPI_X11
#include <X11/Xlib.h>
#endif

#define OFFSET(x) offsetof(YamivppContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
static const AVOption yamivpp_options[] = {
    {"w",           "Output video width",                         OFFSET(out_width),   AV_OPT_TYPE_INT, {.i64=0}, 0, 4096, .flags = FLAGS},
    {"width",       "Output video width",                         OFFSET(out_width),   AV_OPT_TYPE_INT, {.i64=0}, 0, 4096, .flags = FLAGS},
    {"h",           "Output video height",                        OFFSET(out_height),  AV_OPT_TYPE_INT, {.i64=0}, 0, 2304, .flags = FLAGS},
    {"height",      "Output video height",                        OFFSET(out_height),  AV_OPT_TYPE_INT, {.i64=0}, 0, 2304, .flags = FLAGS},
    {"deinterlace", "setting deinterlace mode: 0=off, 1=bob, 2=advanced", OFFSET(deinterlace), AV_OPT_TYPE_INT, {.i64=0}, 0, 2, .flags = FLAGS, .unit = "deinterlace"},
        { "off",    "no deinterlacing",                        0, AV_OPT_TYPE_CONST, {.i64=0}, 0, 0, .flags=FLAGS, .unit="deinterlace"},
        { "bob",    "bob deinterlacing(linear deinterlacing)", 0, AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, .flags=FLAGS, .unit="deinterlace"},
        { "advanced","advanced deinterlacing",                 0, AV_OPT_TYPE_CONST, {.i64=2}, 0, 0, .flags=FLAGS, .unit="deinterlace"},
    {"denoise",     "denoise level [-1, 100]",                    OFFSET(denoise),     AV_OPT_TYPE_INT, {.i64=DENOISE_LEVEL_NONE}, -1, 100, .flags = FLAGS},
    {"sharpness",   "sharpness level [-1, 100]",                  OFFSET(sharpness),   AV_OPT_TYPE_INT, {.i64=SHARPENING_LEVEL_NONE}, -1, 100, .flags = FLAGS},
    {"framerate",   "output frame rate",                          OFFSET(framerate),   AV_OPT_TYPE_RATIONAL, {.dbl=0.0},0, DBL_MAX, .flags = FLAGS},
    {"pipeline",    "yamivpp in hw pipeline: 0=off, 1=on",        OFFSET(pipeline),    AV_OPT_TYPE_INT, {.i64=0}, 0, 1, .flags = FLAGS, .unit = "pipeline"},
        { "off",    "don't put yamivpp in hw pipeline",        0, AV_OPT_TYPE_CONST, {.i64=0}, 0, 0, .flags=FLAGS, .unit="pipeline"},
        { "on",     "put yamivpp in hw pipeline",             0, AV_OPT_TYPE_CONST, {.i64=1}, 0, 0, .flags=FLAGS, .unit="pipeline"},
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

    av_log(ctx, AV_LOG_VERBOSE, "yamivpp_init\n");
    yamivpp->scaler = createVideoPostProcess(YAMI_VPP_SCALER);
    if (!yamivpp->scaler) {
        av_log(ctx, AV_LOG_ERROR, "fail to create libyami vpp scaler\n");
        return -1;
    }

    av_log(yamivpp, AV_LOG_VERBOSE, "w:%d, h:%d, deinterlace:%d, denoise:%d, "
           "sharpness:%d, framerate:%d/%d, pipeline:%d\n",
           yamivpp->out_width, yamivpp->out_height, yamivpp->deinterlace,
           yamivpp->denoise, yamivpp->sharpness,yamivpp->framerate.num, yamivpp->framerate.den,
           yamivpp->pipeline);

    return 0;
}

static int yamivpp_query_formats(AVFilterContext *ctx)
{
    static const int pix_fmts[] = {
        AV_PIX_FMT_YAMI,
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_P010,
        AV_PIX_FMT_NV12,
        AV_PIX_FMT_NONE
    };

    AVFilterFormats *fmts_list = ff_make_format_list(pix_fmts);
    if (!fmts_list)
        return AVERROR(ENOMEM);
    return ff_set_common_formats(ctx, fmts_list);
}

static int config_props(AVFilterLink *inlink)
{
    AVFilterContext *ctx = (AVFilterContext *)inlink->dst;
    YamivppContext *yamivpp = (YamivppContext *)ctx->priv;
    AVFilterLink *outlink = inlink->dst->outputs[0];

    /* if out_width or out_heigh are zero, used input w/h */
    outlink->w = (yamivpp->out_width > 0) ? yamivpp->out_width : inlink->w;
    outlink->h = (yamivpp->out_height > 0) ? yamivpp->out_height : inlink->h;

    if (yamivpp->pipeline)
        outlink->format = AV_PIX_FMT_YAMI;
    else if (inlink->format == AV_PIX_FMT_P010)
        outlink->format = AV_PIX_FMT_P010;
    else
        outlink->format = AV_PIX_FMT_NV12;

    av_log(yamivpp, AV_LOG_VERBOSE, "out w:%d, h:%d, deinterlace:%d,"
           "denoise:%d, sharpness %d, framerate:%d/%d, pipeline:%d\n",
           yamivpp->out_width, yamivpp->out_height,
           yamivpp->deinterlace, yamivpp->denoise,yamivpp->sharpness,
           yamivpp->framerate.num, yamivpp->framerate.den, yamivpp->pipeline);


    return 0;
}

static int map_fmt_to_fourcc(int fmt)
{
    int fourcc = VA_FOURCC_I420;
    switch (fmt) {
    case AV_PIX_FMT_YUV420P:
        fourcc =  VA_FOURCC_I420;
        break;

    case AV_PIX_FMT_NV12:
    case AV_PIX_FMT_YAMI:
        fourcc =  VA_FOURCC_NV12;
        break;
    case AV_PIX_FMT_P010:
        fourcc = VA_FOURCC_P010;
        break;

    case AV_PIX_FMT_YUYV422:
    case AV_PIX_FMT_RGB32:
    case AV_PIX_FMT_NONE:
        av_log(NULL, AV_LOG_WARNING, "don't support this format now.\n");
        break;

    default:
        av_log(NULL, AV_LOG_WARNING, "don't support the format %d\n", fmt);
        break;
    };

    return fourcc;
}

static SharedPtr<VideoFrame>
ff_vaapi_create_nopipeline_surface(int fmt, uint32_t w, uint32_t h)
{
    SharedPtr<VideoFrame> src;
    int fourcc = map_fmt_to_fourcc(fmt);

    src = ff_vaapi_create_surface(VA_RT_FORMAT_YUV420, fourcc, w, h);

    return src;
}

static SharedPtr<VideoFrame>
ff_vaapi_create_pipeline_src_surface(int fmt, uint32_t w, uint32_t h, AVFrame *frame)
{
    YamiImage *yami_image = NULL;
    yami_image = (YamiImage *)frame->data[3];

    return yami_image->output_frame;
}

static SharedPtr<VideoFrame>
ff_vaapi_create_pipeline_dest_surface(int fmt, uint32_t w, uint32_t h, AVFrame *frame)
{
    SharedPtr<VideoFrame> dest;
    YamiImage *yami_image = (YamiImage *)frame->data[3];
    int fourcc = yami_image->output_frame->fourcc;

    dest =  ff_vaapi_create_surface(VA_RT_FORMAT_YUV420, fourcc, w, h);

    return dest;
}

static void av_recycle_surface(void *opaque, uint8_t *data)
{
    if (!data)
        return;
    YamiImage *yami_image = (YamiImage *)data;
    av_log(NULL, AV_LOG_DEBUG, "free %p in yamivpp\n", data);

    ff_vaapi_destory_surface(yami_image->output_frame);
    yami_image->output_frame.reset();
    av_free(yami_image);

    return;
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = (AVFilterContext *)inlink->dst;
    YamivppContext *yamivpp = (YamivppContext *)ctx->priv;
    AVFilterLink *outlink = (AVFilterLink *)ctx->outputs[0];
    int direct = 0;
    AVFrame *out;
    VADisplay m_display;

    if (in->width == outlink->w
        && in->height == outlink->h
        && in->format == outlink->format
        && yamivpp->denoise != -1
        && yamivpp->sharpness != -1)
         return ff_filter_frame(outlink, in);

    if (in->format != AV_PIX_FMT_YAMI && yamivpp->pipeline == 0) {
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }

        av_frame_copy_props(out, in);

        YamiStatus  status;
        if (yamivpp->frame_number == 0) {
            NativeDisplay native_display;
            native_display.type = NATIVE_DISPLAY_VA;
            m_display = ff_vaapi_create_display();
            native_display.handle = (intptr_t)m_display;
            yamivpp->scaler->setNativeDisplay(native_display);

            VPPDenoiseParameters denoise;
            memset(&denoise, 0, sizeof(denoise));
            denoise.size = sizeof(denoise);
            denoise.level = yamivpp->denoise;
            if (yamivpp->scaler->setParameters(VppParamTypeDenoise,
                                               &denoise) != YAMI_SUCCESS) {
                av_log(ctx, AV_LOG_ERROR, "denoise level should in range "
                       "[%d, %d] or %d for none",
                       DENOISE_LEVEL_MIN, DENOISE_LEVEL_MAX, DENOISE_LEVEL_NONE);
                return -1;
            }

            VPPSharpeningParameters sharpening;
            memset(&sharpening, 0, sizeof(sharpening));
            sharpening.size = sizeof(sharpening);
            sharpening.level = yamivpp->sharpness;
            if (yamivpp->scaler->setParameters(VppParamTypeSharpening,
                                               &sharpening) != YAMI_SUCCESS) {
                av_log(ctx, AV_LOG_ERROR, "sharpening level should in range "
                       "[%d, %d] or %d for none",
                       SHARPENING_LEVEL_MIN, SHARPENING_LEVEL_MAX, SHARPENING_LEVEL_NONE);
                return -1;
            }

            switch (yamivpp->deinterlace) {
            case 0:
                /* Do nothing */
                break;
            case 1:
                VPPDeinterlaceParameters deinterlace;
                memset(&deinterlace, 0, sizeof(deinterlace));
                deinterlace.size = sizeof(deinterlace);
                deinterlace.mode = DEINTERLACE_MODE_BOB;

                if (yamivpp->scaler->setParameters(VppParamTypeDeinterlace,
                                                   &deinterlace) != YAMI_SUCCESS) {
                    av_log(ctx, AV_LOG_ERROR, "deinterlace failed for mode %d",
                           yamivpp->deinterlace);
                    return -1;
                }
                break;
            case 2:
            default:
                av_log(ctx, AV_LOG_WARNING, "Using the deinterlace mode %d, "
                       "but not support.\n", yamivpp->deinterlace);
                break;
            }

            /* create src/dest surface, then load yuv to src surface and get
           yuv from dest surfcace */
            yamivpp->src  = ff_vaapi_create_nopipeline_surface(in->format, in->width, in->height);
            yamivpp->dest = ff_vaapi_create_nopipeline_surface(out->format, outlink->w, outlink->h);
        }
        ff_vaapi_load_image(yamivpp->src, in);
        status = yamivpp->scaler->process(yamivpp->src, yamivpp->dest);
        if (status != YAMI_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "vpp process failed, status = %d\n", status);
        }
        /* get output frame from dest surface */
        ff_vaapi_get_image(yamivpp->dest, out);

        yamivpp->frame_number++;

        if (!direct)
            av_frame_free(&in);

        return ff_filter_frame(outlink, out);
    } else if (in->format != AV_PIX_FMT_YAMI && yamivpp->pipeline == 1) {
        av_log(ctx, AV_LOG_WARNING, "Enable vpp pipeline in mix HW/SW decoder/encoder data path\n");
        return 0;
    } else {
        YamiStatus  status;

        out = av_frame_alloc();
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
        av_frame_copy_props(out, in);
        YamiImage *yami_image = NULL;
        yami_image = (YamiImage *)av_mallocz(sizeof(YamiImage));
        out->width = outlink->w;
        out->height = outlink->h;
        out->format = AV_PIX_FMT_YAMI;

        YamiImage *in_buffer = NULL;
        in_buffer = (YamiImage *)in->data[3];
        if (yamivpp->frame_number == 0) {
            /* used the same display handle in pipeline if it's YAMI format */
            if (in->format == AV_PIX_FMT_YAMI) {
                m_display = (VADisplay)in_buffer->va_display;
            } else {
                m_display = ff_vaapi_create_display();
            }
            NativeDisplay native_display;
            native_display.type = NATIVE_DISPLAY_VA;
            native_display.handle = (intptr_t)m_display;
            yamivpp->scaler->setNativeDisplay(native_display);

                       VPPDenoiseParameters denoise;
            memset(&denoise, 0, sizeof(denoise));
            denoise.size = sizeof(denoise);
            denoise.level = yamivpp->denoise;
            if (yamivpp->scaler->setParameters(VppParamTypeDenoise,
                                               &denoise) != YAMI_SUCCESS) {
                av_log(ctx, AV_LOG_ERROR, "denoise level should in range "
                       "[%d, %d] or %d for none",
                       DENOISE_LEVEL_MIN, DENOISE_LEVEL_MAX, DENOISE_LEVEL_NONE);
                return -1;
            }

            VPPSharpeningParameters sharpening;
            memset(&sharpening, 0, sizeof(sharpening));
            sharpening.size = sizeof(sharpening);
            sharpening.level = yamivpp->sharpness;
            if (yamivpp->scaler->setParameters(VppParamTypeSharpening,
                                               &sharpening) != YAMI_SUCCESS) {
                av_log(ctx, AV_LOG_ERROR, "sharpening level should in range "
                       "[%d, %d] or %d for none",
                       SHARPENING_LEVEL_MIN, SHARPENING_LEVEL_MAX, SHARPENING_LEVEL_NONE);
                return -1;
            }

            switch (yamivpp->deinterlace) {
            case 0:
                /* Do nothing */
                break;
            case 1:
                VPPDeinterlaceParameters deinterlace;
                memset(&deinterlace, 0, sizeof(deinterlace));
                deinterlace.size = sizeof(deinterlace);
                deinterlace.mode = DEINTERLACE_MODE_BOB;

                if (yamivpp->scaler->setParameters(VppParamTypeDeinterlace,
                                                   &deinterlace) != YAMI_SUCCESS) {
                    av_log(ctx, AV_LOG_ERROR, "deinterlace failed for mode %d",
                           yamivpp->deinterlace);
                    return -1;
                }
                break;
            case 2:
            default:
                av_log(ctx, AV_LOG_WARNING, "Using the deinterlace mode %d, "
                       "but not support.\n", yamivpp->deinterlace);
                break;
            }

        }

        if (in->format == AV_PIX_FMT_YAMI) {
            yamivpp->src  = ff_vaapi_create_pipeline_src_surface(in->format, in->width, in->height, in);
        } else {
            yamivpp->src  = ff_vaapi_create_nopipeline_surface(in->format, in->width, in->height);
        }
        yamivpp->dest = ff_vaapi_create_pipeline_dest_surface(in->format, outlink->w, outlink->h, in);

        /* update the out surface to out avframe */
        yami_image->output_frame = yamivpp->dest;
        yami_image->va_display = m_display;
        out->data[3] = reinterpret_cast<uint8_t *>(yami_image);
        out->buf[0] = av_buffer_create((uint8_t *)out->data[3],
                                       sizeof(YamiImage),
                                       av_recycle_surface, NULL, 0);

        status = yamivpp->scaler->process(yamivpp->src, yamivpp->dest);
        if (status != YAMI_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "vpp process failed, status = %d\n", status);
        }

        yamivpp->frame_number++;

        if (!direct)
            av_frame_free(&in);

        return ff_filter_frame(outlink, out);
    }
}

static av_cold void yamivpp_uninit(AVFilterContext *ctx)
{
    return;
}

static const AVFilterPad yamivpp_inputs[] = {
    {
        .name             = "default",          // name
        .type             = AVMEDIA_TYPE_VIDEO, // type
        .get_video_buffer = NULL,               // get_video_buffer
        .get_audio_buffer = NULL,               // get_audio_buffer
        .filter_frame     = filter_frame,       // filter_frame
        .poll_frame       = NULL,               // poll_frame
        .request_frame    = NULL,               // request_frame
        .config_props     = config_props,       // config_props
        .needs_fifo       = 0,                  // needs_fifo
        .needs_writable   = 0,                  // needs_writable
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
    .description     = NULL_IF_CONFIG_SMALL("libyami video post processing"),
    .inputs          = yamivpp_inputs,
    .outputs         = yamivpp_outputs,
    .priv_class      = &yamivpp_class,
    .flags           = 0,
    .init            = yamivpp_init,
    .init_dict       = NULL,
    .uninit          = yamivpp_uninit,
    .query_formats   = yamivpp_query_formats,
    .priv_size       = sizeof(YamivppContext),
    .next            = NULL,
    .process_command = NULL,
    .init_opaque     = NULL,
};
