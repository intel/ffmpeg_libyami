/*
 * libyami_enc.cpp
 *
 *  Created on: 2016.2.19
 *      Author: yunzhoux
 */


#include "libyami.h"
#include "libyami_utils.h"

#define ENCODE_TRACE(format, ...)  av_log(avctx, AV_LOG_VERBOSE, "< encode > line:%4d " format, __LINE__, ##__VA_ARGS__)


static bool
getPlaneResolution(uint32_t fourcc, uint32_t pixelWidth, uint32_t pixelHeight, uint32_t byteWidth[3], uint32_t byteHeight[3],  uint32_t& planes)
{
    int w = pixelWidth;
    int h = pixelHeight;
    uint32_t *width = byteWidth;
    uint32_t *height = byteHeight;

    switch (fourcc) {
    case VA_FOURCC_NV12:
    case VA_FOURCC_I420:
    case VA_FOURCC_YV12:
        width[0] = w;
        height[0] = h;
        if (fourcc == VA_FOURCC_NV12) {
            width[1]  = w + (w & 1);
            height[1] = (h + 1) >> 1;
            planes = 2;
        } else {
            width[1] = width[2] = (w + 1) >> 1;
            height[1] = height[2] = (h + 1) >> 1;
            planes = 3;
        }
        break;
    case VA_FOURCC_YUY2:
    case VA_FOURCC_UYVY:
        width[0] = w * 2;
        height[0] = h;
        planes = 1;
        break;
    case VA_FOURCC_RGBX:
    case VA_FOURCC_RGBA:
    case VA_FOURCC_BGRX:
    case VA_FOURCC_BGRA:
        width[0] = w * 4;
        height[0] = h;
        planes = 1;
        break;
    default:
        assert(0 && "do not support this format");
        planes = 0;
        return false;
    }
    return true;
}

static bool
fillFrameRawData(VideoFrameRawData *frame, uint32_t fourcc, uint32_t width, uint32_t height, uint8_t *data)
{
    memset(frame, 0, sizeof(*frame));
    uint32_t planes;
    uint32_t w[3], h[3];

    if (!getPlaneResolution(fourcc, width, height, w, h, planes))
        return false;
    frame->fourcc = fourcc;
    frame->width  = width;
    frame->height = height;
    frame->handle = reinterpret_cast<intptr_t>(data);

    frame->memoryType = VIDEO_DATA_MEMORY_TYPE_RAW_POINTER;
    uint32_t offset = 0;
    for (uint32_t i = 0; i < planes; i++) {
        frame->pitch[i] = w[i];
        frame->offset[i] = offset;//reinterpret_cast<intptr_t>(data->data[i]);
        offset += w[i] * h[i];
    }
    return true;
}

static void *encodeThread(void *arg)
{
    AVCodecContext *avctx = (AVCodecContext *)arg;
    YamiEncContext *s = (YamiEncContext *)avctx->priv_data;

    while (1) {
        AVFrame *frame;
        VideoFrameRawData *in_buffer = NULL;
        // deque one input buffer
        ENCODE_TRACE("encode thread runs one cycle start ... \n");
        pthread_mutex_lock(&s->in_mutex);
        if (s->in_queue->empty()) {
            if (s->encode_status == ENCODE_THREAD_GOT_EOS) {
                pthread_mutex_unlock(&s->in_mutex);
                break;
            } else {
                ENCODE_TRACE("encode thread wait because s->in_queue is empty\n");
                pthread_cond_wait(&s->in_cond, &s->in_mutex); // wait if no todo frame is available
            }
        }

        if (s->in_queue->empty()) { // may wake up from EOS/Close
            pthread_mutex_unlock(&s->in_mutex);
            continue;
        }

        ENCODE_TRACE("s->in_queue->size()=%ld\n", s->in_queue->size());
        frame = s->in_queue->front();
        pthread_mutex_unlock(&s->in_mutex);

        // encode one input in_buffer
        Decode_Status status;
        if (frame->format != AV_PIX_FMT_YAMI) { /* non zero-copy mode */
            uint32_t src_linesize[4];
            const uint8_t *src_data[4];

            uint8_t *dst_data[4];

            in_buffer = (VideoFrameRawData *)av_malloc(sizeof(VideoFrameRawData));

            in_buffer->width = avctx->width;
            in_buffer->height = avctx->height;
#if 0
            /* FIXME there is risk here, I need another yami interface */
            if (avctx->pix_fmt == AV_PIX_FMT_YUV420P){
                in_buffer->pitch[0] = frame->linesize[0];
                in_buffer->pitch[1] = frame->linesize[1];
                in_buffer->pitch[2] = frame->linesize[2];

                in_buffer->handle = reinterpret_cast<intptr_t>(frame->data[2]);
                in_buffer->offset[0] = reinterpret_cast<intptr_t>(frame->data[0]) - in_buffer->handle;
                in_buffer->offset[1] = reinterpret_cast<intptr_t>(frame->data[1]) - in_buffer->handle;
                in_buffer->offset[2] = reinterpret_cast<intptr_t>(frame->data[2]) - in_buffer->handle;
                in_buffer->fourcc = VA_FOURCC_I420;
            } else {
                src_linesize[0] = in_buffer->pitch[0] = frame->linesize[0];
                src_linesize[1] = in_buffer->pitch[1] = frame->linesize[1];
                in_buffer->handle = reinterpret_cast<intptr_t>(frame->data[0]);
                in_buffer->offset[0] = reinterpret_cast<intptr_t>(frame->data[0]) - in_buffer->handle;
                in_buffer->offset[1] = reinterpret_cast<intptr_t>(frame->data[1]) - in_buffer->handle;
                in_buffer->fourcc = VA_FOURCC_NV12;

            }
#else
            src_linesize[0] = in_buffer->pitch[0] = frame->linesize[0];
            src_linesize[1] = in_buffer->pitch[1] = frame->linesize[1];
            src_linesize[2] = in_buffer->pitch[2] = frame->linesize[2];
            uint8_t *yamidata = reinterpret_cast<uint8_t *>(s->m_buffer);

            dst_data[0] = yamidata;
            dst_data[1] = yamidata + avctx->width * avctx->height;
            dst_data[2] = dst_data[1] + avctx->width * avctx->height / 4;

            src_data[0] = frame->data[0];
            src_data[1] = frame->data[1];
            src_data[2] = frame->data[2];

            av_image_copy(dst_data, (int *)in_buffer->pitch, src_data,
                          (int *)src_linesize, avctx->pix_fmt, avctx->width,
                          avctx->height);

            if (avctx->pix_fmt == AV_PIX_FMT_YUV420P)
                fillFrameRawData(in_buffer, VA_FOURCC_I420, avctx->width,
                                 avctx->height, s->m_buffer);
            else if (avctx->pix_fmt == AV_PIX_FMT_NV12)
                fillFrameRawData(in_buffer, VA_FOURCC_NV12, avctx->width,
                                 avctx->height, s->m_buffer);
#endif
            /* handle decoder busy case */
            do {
                 status = s->encoder->encode(in_buffer);
            } while (status == ENCODE_IS_BUSY);

            ENCODE_TRACE("encode() status=%d, encode_count_yami=%d\n", status, s->encode_count_yami);
            av_free(in_buffer);
        } else { /* zero-copy mode */
            SharedPtr < VideoFrame > yami_frame;
            in_buffer = (VideoFrameRawData *)frame->data[3];

            if (frame) {
                yami_frame.reset(new VideoFrame);
                yami_frame->surface = (intptr_t)in_buffer->internalID; /* XXX: get decoded surface */
                yami_frame->timeStamp = in_buffer->timeStamp;
                yami_frame->crop.x = 0;
                yami_frame->crop.y = 0;
                yami_frame->crop.width = avctx->width;
                yami_frame->crop.height = avctx->height;
                yami_frame->flags = 0;
            }

            /* handle decoder busy case */
            do {
                 status = s->encoder->encode(yami_frame);
            } while (status == ENCODE_IS_BUSY);

            ENCODE_TRACE("encode() status=%d, encode_count_yami=%d\n", status, s->encode_count_yami);
        }

        if (status < 0) {
            av_log(avctx, AV_LOG_ERROR,
                   "encode error %d frame %d\n", status , s->encode_count_yami);
        }
        s->encode_count_yami++;
        s->in_queue->pop_front();
        av_frame_free(&frame);
    }

    ENCODE_TRACE("encode thread exit\n");
    pthread_mutex_lock(&s->mutex_);
    s->encode_status = ENCODE_THREAD_EXIT;
    pthread_mutex_unlock(&s->mutex_);

    return NULL;
}

static bool
createOutputBuffer(VideoEncOutputBuffer *outputBuffer, int maxOutSize)
{
    outputBuffer->data = static_cast<uint8_t *>(malloc(maxOutSize));
    if (!outputBuffer->data)
        return false;
    outputBuffer->bufferSize = maxOutSize;
    outputBuffer->format = OUTPUT_EVERYTHING;

    return true;
}

static bool destroyOutputBuffer(VideoEncOutputBuffer *outputBuffer)
{
    if (outputBuffer->data)
        free(outputBuffer->data);

    return true;
}

int yami_enc_init(AVCodecContext *avctx)
{
    YamiEncContext *s = (YamiEncContext *) avctx->priv_data;
    Decode_Status status;

    enum AVPixelFormat pix_fmts[4] =
        {
            AV_PIX_FMT_YAMI,
            AV_PIX_FMT_NV12,
            AV_PIX_FMT_YUV420P,
            AV_PIX_FMT_NONE
        };

    if (avctx->pix_fmt == AV_PIX_FMT_NONE) {
        int ret = ff_get_format(avctx, pix_fmts);
        if (ret < 0)
            return ret;

        avctx->pix_fmt      = (AVPixelFormat)ret;
    }

    av_log(avctx, AV_LOG_VERBOSE, "yami_init h264 encoder\n");
    s->encoder = createVideoEncoder(YAMI_MIME_H264);
    if (!s->encoder) {
        av_log(avctx, AV_LOG_ERROR, "fail to create libyami h264 encoder\n");
        return -1;
    }

    NativeDisplay native_display;
    native_display.type = NATIVE_DISPLAY_VA;
    VADisplay m_display = createVADisplay();
    native_display.handle = (intptr_t)m_display;
    s->encoder->setNativeDisplay(&native_display);

    // configure encoding parameters
    VideoParamsCommon encVideoParams;
    encVideoParams.size = sizeof(VideoParamsCommon);
    s->encoder->getParameters(VideoParamsTypeCommon, &encVideoParams);

    encVideoParams.resolution.width  = avctx->width;
    encVideoParams.resolution.height = avctx->height;

    // frame rate parameters.
    if (avctx->framerate.den > 0 && avctx->framerate.num > 0) {
        encVideoParams.frameRate.frameRateDenom = avctx->framerate.den;
        encVideoParams.frameRate.frameRateNum = avctx->framerate.num;
    } else {
        encVideoParams.frameRate.frameRateNum = avctx->time_base.den;
        encVideoParams.frameRate.frameRateDenom = avctx->time_base.num;
    }

    // picture type and bitrate
    encVideoParams.intraPeriod = av_clip(avctx->gop_size, 1, 250);
    encVideoParams.ipPeriod = !avctx->max_b_frames ? 1 : 3;
    if (s->rcmod){
        if (!strcmp(s->rcmod, "CQP"))
            encVideoParams.rcMode = RATE_CONTROL_CQP;
        else if (!strcmp(s->rcmod, "VBR")){
            encVideoParams.rcMode = RATE_CONTROL_VBR;
            encVideoParams.rcParams.bitRate = avctx->bit_rate;
        } else {
            encVideoParams.rcMode = RATE_CONTROL_CBR;
            encVideoParams.rcParams.bitRate = avctx->bit_rate;
        }
    } else {
        encVideoParams.rcMode = RATE_CONTROL_CQP;
    }

    encVideoParams.rcParams.initQP = av_clip(s->cqp,1,52);

    if (s->level){
        encVideoParams.level = atoi(s->level);
    } else {
        encVideoParams.level = 40;
    }
    /*libyami only support h264 main now*/
//    if (s->profile){
//        if (!strcmp(s->profile , "high"))
//            encVideoParams.profile = VAProfileH264High;
//        else if(!strcmp(s->profile , "main")){
//            encVideoParams.profile = VAProfileH264Main;
//        } else {
//            encVideoParams.profile = VAProfileH264Baseline;
//        }
//    } else {
//        encVideoParams.profile = VAProfileH264High;
//    }
    // s->encoder->setEncoderParameters(&encVideoParams);
    encVideoParams.size = sizeof(VideoParamsCommon);
    s->encoder->setParameters(VideoParamsTypeCommon, &encVideoParams);

    VideoConfigAVCStreamFormat streamFormat;
    streamFormat.size = sizeof(VideoConfigAVCStreamFormat);
    streamFormat.streamFormat = AVC_STREAM_FORMAT_ANNEXB;
    s->encoder->setParameters(VideoConfigTypeAVCStreamFormat, &streamFormat);

    status = s->encoder->start();
    assert(status == ENCODE_SUCCESS);

    // init output buffer
    s->encoder->getMaxOutSize(&(s->maxOutSize));

    if (!createOutputBuffer(&s->outputBuffer, s->maxOutSize)) {
        av_log(avctx, AV_LOG_ERROR,  "fail to create output\n");
        return -1;
    }
    s->m_frameSize = FFALIGN(avctx->width, 32) * FFALIGN(avctx->height, 32) * 3;
    s->m_buffer = static_cast<uint8_t *>(av_mallocz(s->m_frameSize));

    s->in_queue = new std::deque<AVFrame *>;
    pthread_mutex_init(&s->mutex_, NULL);
    pthread_mutex_init(&s->in_mutex, NULL);
    pthread_cond_init(&s->in_cond, NULL);
    s->encode_status = ENCODE_THREAD_NOT_INIT;

    s->encode_count = 0;
    s->encode_count_yami = 0;
    s->render_count = 0;
    av_log(avctx, AV_LOG_INFO, "yami_enc_init\n");
    return 0;
}

int yami_enc_frame(AVCodecContext *avctx, AVPacket *pkt,
                   const AVFrame *frame, int *got_packet)
{
    YamiEncContext *s = (YamiEncContext *)avctx->priv_data;
    Encode_Status status;
    int ret;

    if(!s->encoder)
        return -1;
    if (frame) {
        AVFrame *qframe = av_frame_alloc();
        if (!qframe) {
            return AVERROR(ENOMEM);
        }

        /* av_frame_ref the src frame and av_frame_unref in encode thread */
        ret = av_frame_ref(qframe, frame);
        if (ret < 0)
            return ret;

        while (s->encode_status < ENCODE_THREAD_GOT_EOS) { // we need enque eos buffer more than once
            pthread_mutex_lock(&s->in_mutex);
            if (s->in_queue->size() < ENCODE_QUEUE_SIZE) {
                s->in_queue->push_back(qframe);
                av_log(avctx, AV_LOG_VERBOSE, "wakeup encode thread ...\n");
                pthread_cond_signal(&s->in_cond);
                pthread_mutex_unlock(&s->in_mutex);
                break;
            }
            pthread_mutex_unlock(&s->in_mutex);

            av_log(avctx, AV_LOG_DEBUG,
                   "s->in_queue->size()=%ld, s->encode_count=%d, s->encode_count_yami=%d, too many buffer are under encoding, wait ...\n",
                   s->in_queue->size(), s->encode_count, s->encode_count_yami);
            usleep(1000);
        };
        s->encode_count++;
    }

    // decode thread status update
    pthread_mutex_lock(&s->mutex_);
    switch (s->encode_status) {
    case ENCODE_THREAD_NOT_INIT:
    case ENCODE_THREAD_EXIT:
        if (frame) {
            s->encode_status = ENCODE_THREAD_RUNING;
            pthread_create(&s->encode_thread_id, NULL, &encodeThread, avctx);
        }
        break;
    case ENCODE_THREAD_RUNING:
        if (!frame) {
            s->encode_status = ENCODE_THREAD_GOT_EOS; // call releaseLock for seek
        }
        break;
    case ENCODE_THREAD_GOT_EOS:
        if (s->in_queue->empty())
            s->encode_status = ENCODE_THREAD_NOT_INIT;
        break;
    default:
        break;
    }
    pthread_mutex_unlock(&s->mutex_);

    do {
        status = s->encoder->getOutput(&s->outputBuffer, true);
    } while (!frame && status != ENCODE_SUCCESS && s->in_queue->size() > 0);
    if (status != ENCODE_SUCCESS)
        return 0;

    s->render_count++;

    if ((ret = ff_alloc_packet2(avctx, pkt, s->outputBuffer.dataSize, 0)) < 0)
        return ret;
    void *p = pkt->data;
    memcpy(p, s->outputBuffer.data, s->outputBuffer.dataSize);
    *got_packet = 1;

    return 0;
}

int yami_enc_close(AVCodecContext *avctx)
{
    YamiEncContext *s = (YamiEncContext *)avctx->priv_data;

    destroyOutputBuffer(&s->outputBuffer);
    pthread_mutex_lock(&s->mutex_);
    while (s->encode_status == ENCODE_THREAD_RUNING) {
        // potential race condition on s->encode_status
        s->encode_status = ENCODE_THREAD_GOT_EOS;
        pthread_mutex_unlock(&s->mutex_);
        pthread_cond_signal(&s->in_cond);
        usleep(10000);
        pthread_mutex_lock(&s->mutex_);
    }
    pthread_mutex_unlock(&s->mutex_);

    if (s->encoder) {
        s->encoder->stop();
        releaseVideoEncoder(s->encoder);
        s->encoder = NULL;
    }
    pthread_mutex_destroy(&s->in_mutex);
    pthread_cond_destroy(&s->in_cond);
    while (!s->in_queue->empty()) {
        AVFrame *in_buffer = s->in_queue->front();
        s->in_queue->pop_front();
        av_frame_free(&in_buffer);
    }
    delete s->in_queue;

    av_free(s->m_buffer);
    s->m_frameSize = 0;

    av_log(avctx, AV_LOG_INFO, "yami_enc_close\n");

    return 0;
}

