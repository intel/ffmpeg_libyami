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

using namespace YamiMediaCodec;

#ifndef VA_FOURCC_I420
#define VA_FOURCC_I420 VA_FOURCC('I','4','2','0')
#endif

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

    int cur_out_idx;     // current surface in index

    int frame_number;

    int use_frc;         // use frame rate conversion

    int pipeline;        // is vpp in HW pipeline?
    AVRational framerate;// target frame rate
} YamivppContext;

#include <fcntl.h>
#include <unistd.h>

#include <va/va_drm.h>
#include <va/va_x11.h>
#define HAVE_VAAPI_DRM 1

#if HAVE_VAAPI_X11
#include <X11/Xlib.h>
#endif

static VADisplay ff_vaapi_create_display(void)
{
    static VADisplay display = NULL;
    
    if (!display) {
#if !HAVE_VAAPI_DRM
        const char *device = NULL;/*FIXME*/
	// Try to open the device as an X11 display.
        Display *x11_display = XOpenDisplay(device);
        if (!x11_display) {
            return NULL;
        } else {
            display = vaGetDisplay(x11_display);
            if (!display) {
                XCloseDisplay(x11_display);
            } 
        }
#else
        const char *device = "/dev/dri/card0"; /* FIXME */
        // Try to open the device as a DRM path.
        int drm_fd = open(device, O_RDWR);
        if (drm_fd < 0) {
            return NULL;
        } else {
            display = vaGetDisplayDRM(drm_fd);
            if(!display) 
                close(drm_fd);
        }    
#endif
        if (!display)
            return NULL;
        int majorVersion, minorVersion;
        VAStatus vaStatus = vaInitialize(display, &majorVersion, &minorVersion);
        if (vaStatus != VA_STATUS_SUCCESS) {
#if HAVE_VAAPI_DRM
            close(drm_fd);
#endif
            display = NULL;
            return NULL;
        }
        return display;
    } else {
        return display;
    }
}

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
    {"denoise",     "denoise level [0, 100]",                     OFFSET(denoise),     AV_OPT_TYPE_INT, {.i64=0}, 0, 100, .flags = FLAGS},
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

    av_log(yamivpp, AV_LOG_VERBOSE, "w:%d, h:%d, deinterlace:%d, denoise:%d, framerate:%d/%d, pipeline:%d\n",
           yamivpp->out_width, yamivpp->out_height, yamivpp->deinterlace, yamivpp->denoise, yamivpp->framerate.num, yamivpp->framerate.den, yamivpp->pipeline);

    return 0;
}

static int yamivpp_query_formats(AVFilterContext *ctx)
{
    static const int pix_fmts[] = {
        AV_PIX_FMT_YAMI,
        AV_PIX_FMT_YUV420P,
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
    else
        outlink->format = AV_PIX_FMT_NV12;

    av_log(yamivpp, AV_LOG_VERBOSE, "out w:%d, h:%d, deinterlace:%d, denoise:%d, framerate:%d/%d, pipeline:%d\n",
           yamivpp->out_width, yamivpp->out_height, yamivpp->deinterlace, yamivpp->denoise, yamivpp->framerate.num, yamivpp->framerate.den, yamivpp->pipeline);


    return 0;
}

static inline bool ff_check_vaapi_status(VAStatus status, const char *msg)
{
    if (status != VA_STATUS_SUCCESS) {
        av_log(NULL, AV_LOG_ERROR, "%s: %s", msg, vaErrorStr(status));
        return false;
    }
    return true;
}

static SharedPtr<VideoFrame> ff_vaapi_create_surface(uint32_t rt_fmt, int pix_fmt, uint32_t w, uint32_t h)
{
    SharedPtr<VideoFrame> frame;
    VAStatus status;
    VASurfaceID id;
    VASurfaceAttrib attrib;

    VADisplay m_vaDisplay = ff_vaapi_create_display();

    attrib.type =  VASurfaceAttribPixelFormat;
    attrib.flags = VA_SURFACE_ATTRIB_SETTABLE;
    attrib.value.type = VAGenericValueTypeInteger;
    attrib.value.value.i = pix_fmt;

    status = vaCreateSurfaces(m_vaDisplay, rt_fmt, w, h, &id, 1, &attrib, 1);
    if (!ff_check_vaapi_status(status, "vaCreateSurfaces"))
        return frame;
    frame.reset(new VideoFrame);
    frame->surface = (intptr_t)id;
    frame->crop.x = frame->crop.y = 0;
    frame->crop.width = w;
    frame->crop.height = h;

    return frame;
}

static bool ff_vaapi_load_image(SharedPtr<VideoFrame>& frame, AVFrame *in)
{
    VASurfaceID surface = (VASurfaceID)frame->surface;
    VAImage image;

    uint32_t dest_linesize[4] = {0};
    const uint8_t *src_data[4];
    uint8_t *dest_data[4];

    VADisplay m_vaDisplay = ff_vaapi_create_display();

    VAStatus status = vaDeriveImage(m_vaDisplay, surface, &image);
    if (!ff_check_vaapi_status(status, "vaDeriveImage"))
        return false;

    uint8_t *buf = NULL;
    status = vaMapBuffer(m_vaDisplay, image.buf, (void**)&buf);
    if (!ff_check_vaapi_status(status, "vaMapBuffer")) {
        vaDestroyImage(m_vaDisplay, image.image_id);
        return false;
    }

    src_data[0] = in->data[0];
    src_data[1] = in->data[1];
    src_data[2] = in->data[2];

    dest_data[0] = buf + image.offsets[0];
    dest_data[1] = buf + image.offsets[1];
    dest_data[2] = buf + image.offsets[2];

    if (in->format == AV_PIX_FMT_YUV420P) {
        dest_linesize[0] = image.pitches[0];
        dest_linesize[1] = image.pitches[1];
        dest_linesize[2] = image.pitches[2];
    } else if (in->format == AV_PIX_FMT_NV12) {
        dest_linesize[0] = image.pitches[0];
        dest_linesize[1] = image.pitches[1];
        dest_linesize[2] = image.pitches[2];
    }

    av_image_copy(dest_data, (int *)dest_linesize, src_data,
                  (int *)in->linesize, (AVPixelFormat)in->format,
                  in->width, in->height);

    ff_check_vaapi_status(vaUnmapBuffer(m_vaDisplay, image.buf), "vaUnmapBuffer");
    ff_check_vaapi_status(vaDestroyImage(m_vaDisplay, image.image_id), "vaDestroyImage");
    return true;
}

static bool ff_vaapi_get_image(SharedPtr<VideoFrame>& frame, AVFrame *out)
{
    VASurfaceID surface = (VASurfaceID)frame->surface;
    VAImage image;

    uint32_t src_linesize[4] = { 0 };
    uint32_t dest_linesize[4] = { 0 };
    const uint8_t *src_data[4];
    uint8_t *dest_data[4];

    VADisplay m_vaDisplay = ff_vaapi_create_display();

    VAStatus status = vaDeriveImage(m_vaDisplay, surface, &image);
    if (!ff_check_vaapi_status(status, "vaDeriveImage"))
        return false;

    uint8_t *buf = NULL;
    status = vaMapBuffer(m_vaDisplay, image.buf, (void**)&buf);
    if (!ff_check_vaapi_status(status, "vaMapBuffer")) {
        vaDestroyImage(m_vaDisplay, image.image_id);
        return false;
    }

    dest_data[0] = out->data[0];
    dest_data[1] = out->data[1];
    dest_data[2] = out->data[2];

    src_data[0] = buf + image.offsets[0];
    src_data[1] = buf + image.offsets[1];
    src_data[2] = buf + image.offsets[2];

    if (out->format == AV_PIX_FMT_YUV420P) {
        dest_linesize[0] = out->linesize[0];
        dest_linesize[1] = out->linesize[1];
        dest_linesize[2] = out->linesize[2];

        src_linesize[0] = image.pitches[0];
        src_linesize[1] = image.pitches[1];
        src_linesize[2] = image.pitches[2];
    } else if (out->format == AV_PIX_FMT_NV12) {
        dest_linesize[0] = out->linesize[0];
        dest_linesize[1] = out->linesize[1];
        dest_linesize[2] = out->linesize[2];

        src_linesize[0] = image.pitches[0];
        src_linesize[1] = image.pitches[1];
        src_linesize[2] = image.pitches[2];
    }

    /* 
     * TODO:
     * Need to improving performance of data copies from Uncacheable 
     * Speculative Write Combining(USWC) memory to ordinary Write Back(WB) 
     * system memory
     * https://software.intel.com/en-us/articles/copying-accelerated-video-decode-frame-buffers/
     */
    av_image_copy(dest_data, (int *)dest_linesize, src_data,
                  (int *)src_linesize, (AVPixelFormat)out->format,
                  out->width, out->height);

    ff_check_vaapi_status(vaUnmapBuffer(m_vaDisplay, image.buf), "vaUnmapBuffer");
    ff_check_vaapi_status(vaDestroyImage(m_vaDisplay, image.image_id), "vaDestroyImage");
    return true;
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

static SharedPtr<VideoFrame> ff_vaapi_create_nopipeline_surface(int fmt, uint32_t w, uint32_t h)
{
    SharedPtr<VideoFrame> src;
    int fourcc = map_fmt_to_fourcc(fmt);

    src = ff_vaapi_create_surface(VA_RT_FORMAT_YUV420, fourcc, w, h);
    src->fourcc = fourcc;
    return src;
}

static SharedPtr<VideoFrame> ff_vaapi_create_pipeline_src_surface(int fmt, uint32_t w, uint32_t h, AVFrame *frame)
{
    SharedPtr<VideoFrame> src;
    int fourcc = map_fmt_to_fourcc(fmt);

    VideoFrameRawData *in_buffer = NULL;
    in_buffer = (VideoFrameRawData *)frame->data[3];

    if (frame) {
        src.reset(new VideoFrame);
        src->surface = (intptr_t)in_buffer->internalID; /* XXX: get decoded surface */
        src->timeStamp = in_buffer->timeStamp;
        src->crop.x = 0;
        src->crop.y = 0;
        src->crop.width = w;
        src->crop.height = h;
        src->flags = 0;
        src->fourcc = fourcc;
    }

    return src;
}

static SharedPtr<VideoFrame> ff_vaapi_create_pipeline_dest_surface(int fmt, uint32_t w, uint32_t h, AVFrame *frame)
{
    SharedPtr<VideoFrame> dest;
    int fourcc = map_fmt_to_fourcc(fmt);

    VideoFrameRawData *in_buffer = NULL;
    in_buffer = (VideoFrameRawData *)frame->data[3];
    VAStatus status;
    VASurfaceID id;
    VASurfaceAttrib attrib;

    VADisplay m_vaDisplay = (VADisplay)in_buffer->handle;

    attrib.type =  VASurfaceAttribPixelFormat;
    attrib.flags = VA_SURFACE_ATTRIB_SETTABLE;
    attrib.value.type = VAGenericValueTypeInteger;
    attrib.value.value.i = fourcc;

    status = vaCreateSurfaces(m_vaDisplay, VA_RT_FORMAT_YUV420, w, h, &id, 1, &attrib, 1);
    if (!ff_check_vaapi_status(status, "vaCreateSurfaces"))
        return dest;
    dest.reset(new VideoFrame);
    dest->surface = (intptr_t)id;
    dest->crop.x = dest->crop.y = 0;
    dest->crop.width = w;
    dest->crop.height = h;
    dest->fourcc = fourcc;

    return dest;
}

static void av_recycle_surface(void *opaque, uint8_t *data)
{
    if (!data)
        return;
    VideoFrameRawData *yami_frame = (VideoFrameRawData *)data;
    av_log(NULL, AV_LOG_DEBUG, "free %p in yamivpp\n", data);

    VASurfaceID id = reinterpret_cast<VASurfaceID>(yami_frame->internalID);
    vaDestroySurfaces((VADisplay)yami_frame->handle, &id, 1);
    av_free(yami_frame);

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
        VideoFrameRawData *out_buffer = NULL;
        out_buffer = (VideoFrameRawData *)av_malloc(sizeof(VideoFrameRawData));
        out->width = out_buffer->width =  outlink->w;
        out->height = out_buffer->height = outlink->h;
        out->format = AV_PIX_FMT_YAMI;
        out->data[3] = reinterpret_cast<uint8_t *>(out_buffer);
        out->buf[0] = av_buffer_create((uint8_t *)out->data[3],
                                       sizeof(VideoFrameRawData),
                                       av_recycle_surface, NULL, 0);
        VideoFrameRawData *in_buffer = NULL;
        in_buffer = (VideoFrameRawData *)in->data[3];
        if (yamivpp->frame_number == 0) {
            /* used the same display handle in pipeline if it's YAMI format */
            if (in->format == AV_PIX_FMT_YAMI) {
                m_display = (VADisplay)in_buffer->handle;
            } else {
                m_display = ff_vaapi_create_display();
            }
            NativeDisplay native_display;
            native_display.type = NATIVE_DISPLAY_VA;
            native_display.handle = (intptr_t)m_display;
            yamivpp->scaler->setNativeDisplay(native_display);
        }

        if (in->format == AV_PIX_FMT_YAMI) {
            yamivpp->src  = ff_vaapi_create_pipeline_src_surface(in->format, in->width, in->height, in);
        } else {
            yamivpp->src  = ff_vaapi_create_nopipeline_surface(in->format, in->width, in->height);
        }
        yamivpp->dest = ff_vaapi_create_pipeline_dest_surface(in->format, outlink->w, outlink->h, in);

        /* update the out surface to out avframe */
        out_buffer->handle = in_buffer->handle;
        out_buffer->internalID = yamivpp->dest->surface;

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
