/*
 * libyami_dec.h
 *
 *  Created on: 2016.2.19
 *      Author: yunzhoux
 */

#ifndef LIBAVCODEC_LIBYAMI_DEC_H_
#define LIBAVCODEC_LIBYAMI_DEC_H_


using namespace YamiMediaCodec;

#define DECODE_QUEUE_SIZE 8

typedef enum {
    DECODE_THREAD_NOT_INIT = 0,
    DECODE_THREAD_RUNING,
    DECODE_THREAD_GOT_EOS,
    DECODE_THREAD_EXIT,
} DecodeThreadStatus;

struct YamiDecContext {
    AVCodecContext *avctx;
    pthread_mutex_t mutex_; // mutex for decoder->getOutput() and YamiContext itself update (decode_status, etc)

    IVideoDecoder *decoder;
    VideoDataMemoryType output_type;
    const VideoFormatInfo *format_info;
    pthread_t decode_thread_id;
    std::deque<VideoDecodeBuffer *> *in_queue;
    pthread_mutex_t in_mutex; // mutex for in_queue
    pthread_cond_t in_cond;   // decode thread condition wait
    DecodeThreadStatus decode_status;

    // debug use
    int decode_count;
    int decode_count_yami;
    int render_count;
};



#endif /* LIBAVCODEC_LIBYAMI_DEC_H_ */
