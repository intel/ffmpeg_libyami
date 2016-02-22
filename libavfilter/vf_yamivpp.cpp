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

#define YAMIVPP_TRACE(format, ...)  av_log(yamivpp, AV_LOG_VERBOSE, "## yami vpp ## line:%4d " format, __LINE__, ##__VA_ARGS__)

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

static VADisplay createVADisplay(void)
{
    static VADisplay vadisplay = NULL;

    if (!vadisplay) {
        int fd = open("/dev/dri/card0", O_RDWR);
        if (fd < 0) {
            av_log(NULL, AV_LOG_ERROR, "open card0 failed");
            return NULL;
        }

        vadisplay = vaGetDisplayDRM(fd);
        int majorVersion, minorVersion;
        VAStatus vaStatus = vaInitialize(vadisplay, &majorVersion, &minorVersion);
        if (vaStatus != VA_STATUS_SUCCESS) {
            av_log(NULL, AV_LOG_ERROR, "va init failed, status =  %d", vaStatus);
            close(fd);
            return NULL;
        }
        return vadisplay;
    } else {
        return vadisplay;
    }
}

#define OFFSET(x) offsetof(YamivppContext, x)
#define FLAGS AV_OPT_FLAG_FILTERING_PARAM|AV_OPT_FLAG_VIDEO_PARAM
static const AVOption yamivpp_options[] = {
    {"w",           "Output video width",                         OFFSET(out_width),   AV_OPT_TYPE_INT, {.i64=0}, 0, 4096, .flags = FLAGS},
    {"width",       "Output video width",                         OFFSET(out_width),   AV_OPT_TYPE_INT, {.i64=0}, 0, 4096, .flags = FLAGS},
    {"h",           "Output video height",                        OFFSET(out_height),  AV_OPT_TYPE_INT, {.i64=0}, 0, 2304, .flags = FLAGS},
    {"height",      "Output video height",                        OFFSET(out_height),  AV_OPT_TYPE_INT, {.i64=0}, 0, 2304, .flags = FLAGS},
    {"deinterlace", "deinterlace mode: 0=off, 1=bob, 2=advanced", OFFSET(deinterlace), AV_OPT_TYPE_INT, {.i64=0}, 0, 2, .flags = FLAGS},
    {"denoise",     "denoise level [0, 100]",                     OFFSET(denoise),     AV_OPT_TYPE_INT, {.i64=0}, 0, 100, .flags = FLAGS},
    {"framerate",   "output frame rate",                          OFFSET(framerate),   AV_OPT_TYPE_RATIONAL, {.dbl = 25.0},0, DBL_MAX, .flags = FLAGS},
    {"pipeline",    "yamivpp in hw pipeline: 0=off, 1=on",        OFFSET(pipeline),    AV_OPT_TYPE_INT, {.i64=0}, 0, 1, .flags = FLAGS},
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

    #if 0
    NativeDisplay native_display;
    native_display.type = NATIVE_DISPLAY_VA;
    VADisplay m_display = createVADisplay();
    native_display.handle = (intptr_t)m_display;
    yamivpp->scaler->setNativeDisplay(native_display);
    #endif

#define CHECK_UNSET_OPT(opt)                                       \
    if (s->opt == -1) {                                            \
        av_log(s, AV_LOG_INFO, "Option %s was not set.\n", #opt);  \
    }

    yamivpp->frame_number = 0;

    av_log(yamivpp, AV_LOG_VERBOSE, "w:%d, h:%d, deinterlace:%d, denoise:%d, framerate:%d/%d, pipeline:%d\n",
           yamivpp->out_width, yamivpp->out_height, yamivpp->deinterlace, yamivpp->denoise, yamivpp->framerate.num, yamivpp->framerate.den, yamivpp->pipeline);

    return 0;
}

static int yamivpp_query_formats(AVFilterContext *ctx)
{
    const YamivppContext *yamivpp = (YamivppContext *)ctx->priv;
    static const int pix_fmts[] = {
        AV_PIX_FMT_YAMI,
        AV_PIX_FMT_YUV420P,
        AV_PIX_FMT_NV12,
        AV_PIX_FMT_YUYV422,
        AV_PIX_FMT_RGB32,
        AV_PIX_FMT_NONE
    };

    ff_set_common_formats(ctx, ff_make_format_list(pix_fmts));
    return 0;
}

static int config_props(AVFilterLink *inlink)
{
    AVFilterContext *ctx = (AVFilterContext *)inlink->dst;
    YamivppContext *yamivpp = (YamivppContext *)ctx->priv;
    AVFilterLink *outlink = inlink->dst->outputs[0];

    /*
    yamivpp->out_width  = FFALIGN(yamivpp->out_width, 16);
    yamivpp->out_height = (yamivpp->dpic == 2) ?
                          FFALIGN(yamivpp->out_height, 16):
                          FFALIGN(yamivpp->out_height, 32); */ // force 32 if unkown

    /* if out_width or out_heigh are zero, used input w/h */
    outlink->w = (yamivpp->out_width > 0) ? yamivpp->out_width : inlink->w;
    outlink->h = (yamivpp->out_height > 0) ? yamivpp->out_height : inlink->h;

    outlink->frame_rate = yamivpp->framerate;
    outlink->time_base  = av_inv_q(yamivpp->framerate);

    if (yamivpp->pipeline)
        outlink->format = AV_PIX_FMT_YAMI;
    else
        outlink->format = AV_PIX_FMT_NV12;

    av_log(yamivpp, AV_LOG_VERBOSE, "out w:%d, h:%d, deinterlace:%d, denoise:%d, framerate:%d/%d, pipeline:%d\n",
           yamivpp->out_width, yamivpp->out_height, yamivpp->deinterlace, yamivpp->denoise, yamivpp->framerate.num, yamivpp->framerate.den, yamivpp->pipeline);


    return 0;
}

static inline bool checkVaapiStatus(VAStatus status, const char *msg)
{
    if (status != VA_STATUS_SUCCESS) {
        av_log(NULL, AV_LOG_ERROR, "%s: %s", msg, vaErrorStr(status));
        return false;
    }
    return true;
}

SharedPtr<VideoFrame> createSurface(uint32_t rtFormat, int pixelFormat, uint32_t width, uint32_t height)
{
    SharedPtr<VideoFrame> frame;
    VAStatus status;
    VASurfaceID id;
    VASurfaceAttrib attrib;

    VADisplay m_vaDisplay = createVADisplay();

    attrib.type =  VASurfaceAttribPixelFormat;
    attrib.flags = VA_SURFACE_ATTRIB_SETTABLE;
    attrib.value.type = VAGenericValueTypeInteger;
    attrib.value.value.i = pixelFormat;

    status = vaCreateSurfaces(m_vaDisplay, rtFormat, width, height, &id, 1, &attrib, 1);
    if (!checkVaapiStatus(status, "vaCreateSurfaces"))
        return frame;
    frame.reset(new VideoFrame);
    frame->surface = (intptr_t)id;
    frame->crop.x = frame->crop.y = 0;
    frame->crop.width = width;
    frame->crop.height = height;

    return frame;
}

bool loadSurfaceImage(SharedPtr<VideoFrame>& frame , AVFrame *in)
{
    VASurfaceID surface = (VASurfaceID)frame->surface;
    VAImage image;

    uint32_t src_linesize[4] = {0};
    uint32_t dest_linesize[4] = {0};
    const uint8_t *src_data[4];
    uint8_t *dest_data[4];

    VADisplay m_vaDisplay = createVADisplay();

    VAStatus status = vaDeriveImage(m_vaDisplay, surface, &image);
    if (!checkVaapiStatus(status, "vaDeriveImage"))
        return false;

    uint8_t *buf = NULL;
    status = vaMapBuffer(m_vaDisplay, image.buf, (void**)&buf);
    if (!checkVaapiStatus(status, "vaMapBuffer")) {
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
        dest_linesize[0] = src_linesize[0] = in->width;
        dest_linesize[1] = src_linesize[1] = in->width / 2;
        dest_linesize[2] = src_linesize[2] = in->width / 2;
    } else if (in->format == AV_PIX_FMT_NV12) {
        dest_linesize[0] = src_linesize[0] = in->width;
        dest_linesize[1] = src_linesize[1] = in->width;
        dest_linesize[2] = src_linesize[2] = 0;
    }

    av_image_copy(dest_data, (int *)dest_linesize, src_data,
                  (int *)in->linesize, (AVPixelFormat)in->format,
                  in->width, in->height);

    checkVaapiStatus(vaUnmapBuffer(m_vaDisplay, image.buf), "vaUnmapBuffer");
    checkVaapiStatus(vaDestroyImage(m_vaDisplay, image.image_id), "vaDestroyImage");
    return true;
}

bool getSurfaceImage(SharedPtr<VideoFrame>& frame, AVFrame *out)
{
    VASurfaceID surface = (VASurfaceID)frame->surface;
    VAImage image;

    uint32_t src_linesize[4] = { 0 };
    uint32_t dest_linesize[4] = { 0 };
    const uint8_t *src_data[4];
    uint8_t *dest_data[4];

    VADisplay m_vaDisplay = createVADisplay();

    VAStatus status = vaDeriveImage(m_vaDisplay, surface, &image);
    if (!checkVaapiStatus(status, "vaDeriveImage"))
        return false;

    uint8_t *buf = NULL;
    status = vaMapBuffer(m_vaDisplay, image.buf, (void**)&buf);
    if (!checkVaapiStatus(status, "vaMapBuffer")) {
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

    av_image_copy(dest_data, (int *)dest_linesize, src_data,
                  (int *)src_linesize, (AVPixelFormat)out->format,
                  out->width, out->height);

    checkVaapiStatus(vaUnmapBuffer(m_vaDisplay, image.buf), "vaUnmapBuffer");
    checkVaapiStatus(vaDestroyImage(m_vaDisplay, image.image_id), "vaDestroyImage");
    return true;
}

static SharedPtr<VideoFrame> createSrcSurface(int fmt, uint32_t targetWidth, uint32_t targetHeight)
{
    SharedPtr<VideoFrame> src;
    int fourcc = VA_FOURCC_I420;
    switch (fmt) {
    case AV_PIX_FMT_YUV420P:
        fourcc =  VA_FOURCC_I420;
        break;

    case AV_PIX_FMT_NV12:
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

    src = createSurface(VA_RT_FORMAT_YUV420, fourcc, targetWidth, targetHeight);
    src->fourcc = fourcc;
    return src;
}

static SharedPtr<VideoFrame> createDestSurface(int fmt, uint32_t targetWidth, uint32_t targetHeight)
{
    SharedPtr<VideoFrame> dest;
    int fourcc = VA_FOURCC_I420;
    switch (fmt) {
    case AV_PIX_FMT_YUV420P:
        fourcc =  VA_FOURCC_I420;
        break;

    case AV_PIX_FMT_NV12:
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

    dest = createSurface(VA_RT_FORMAT_YUV420, fourcc, targetWidth, targetHeight);
    dest->fourcc = fourcc;
    return dest;
}

static SharedPtr<VideoFrame> createPipelineSrcSurface(int fmt, uint32_t targetWidth, uint32_t targetHeight, AVFrame *frame)
{
    SharedPtr<VideoFrame> src;
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

    VideoFrameRawData *in_buffer = NULL;
    in_buffer = (VideoFrameRawData *)frame->data[3];

    if (frame) {
        src.reset(new VideoFrame);
        src->surface = (intptr_t)in_buffer->internalID; /* XXX: get decoded surface */
        src->timeStamp = in_buffer->timeStamp;
        src->crop.x = 0;
        src->crop.y = 0;
        src->crop.width = targetWidth;
        src->crop.height = targetHeight;
        src->flags = 0;
        src->fourcc = fourcc;
    }

    return src;
}

static SharedPtr<VideoFrame> createPipelineDestSurface(int fmt, uint32_t targetWidth, uint32_t targetHeight, AVFrame *frame)
{
    SharedPtr<VideoFrame> dest;
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

    status = vaCreateSurfaces(m_vaDisplay, VA_RT_FORMAT_YUV420, targetWidth, targetHeight, &id, 1, &attrib, 1);
    if (!checkVaapiStatus(status, "vaCreateSurfaces"))
        return dest;
    dest.reset(new VideoFrame);
    dest->surface = (intptr_t)id;
    dest->crop.x = dest->crop.y = 0;
    dest->crop.width = targetWidth;
    dest->crop.height = targetHeight;
    dest->fourcc = fourcc;

    return dest;
}

void av_recyle_free(void *opaque, uint8_t *data)
{
    if (!data)
        return;
    VideoFrameRawData *yami_frame = (VideoFrameRawData *)data;
    av_log(NULL, AV_LOG_DEBUG, "free %d in yamivpp\n", data);

    VASurfaceID id = reinterpret_cast<VASurfaceID>(yami_frame->internalID);
    vaDestroySurfaces((VADisplay)yami_frame->handle, &id, 1);
    av_free(yami_frame);
}

static int filter_frame(AVFilterLink *inlink, AVFrame *in)
{
    AVFilterContext *ctx = (AVFilterContext *)inlink->dst;
    YamivppContext *yamivpp = (YamivppContext *)ctx->priv;
    AVFilterLink *outlink = (AVFilterLink *)ctx->outputs[0];
    int p, direct = 0;
    AVFrame *out;
    uint32_t fourcc;

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
            VADisplay m_display = createVADisplay();
            native_display.handle = (intptr_t)m_display;
            yamivpp->scaler->setNativeDisplay(native_display);

            /* create source surface and load data to the surface */
            yamivpp->src  = createSrcSurface(in->format, in->width, in->height);
            yamivpp->dest = createDestSurface(out->format, outlink->w, outlink->h);
        }
        loadSurfaceImage(yamivpp->src, in);
        status = yamivpp->scaler->process(yamivpp->src, yamivpp->dest);
        if (status != YAMI_SUCCESS) {
            av_log(ctx, AV_LOG_ERROR, "vpp process failed, status = %d\n", status);
        }
        /* get output frame from dest surface */
        getSurfaceImage(yamivpp->dest, out);

        yamivpp->frame_number++;

        if (!direct)
            av_frame_free(&in);

        return ff_filter_frame(outlink, out);
    } else {
        YamiStatus  status;

#if 0
        out = ff_get_video_buffer(outlink, outlink->w, outlink->h);
        if (!out) {
            av_frame_free(&in);
            return AVERROR(ENOMEM);
        }
#endif
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
                                         av_recyle_free, NULL, 0);
        VideoFrameRawData *in_buffer = NULL;
        in_buffer = (VideoFrameRawData *)in->data[3];
        if (yamivpp->frame_number == 0) {
            /* used the same display handle in pipeline */
            VADisplay m_vaDisplay = (VADisplay)in_buffer->handle;
            NativeDisplay native_display;
            native_display.type = NATIVE_DISPLAY_VA;
            native_display.handle = (intptr_t)m_vaDisplay;
            yamivpp->scaler->setNativeDisplay(native_display);
        }
        yamivpp->src  = createPipelineSrcSurface(in->format, in->width, in->height, in);
        yamivpp->dest = createPipelineDestSurface(in->format, outlink->w, outlink->h, in);

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
    YamivppContext *yamivpp = (YamivppContext *)ctx->priv;

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
