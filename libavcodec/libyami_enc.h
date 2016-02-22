/*
 * libyami_enc.h
 *
 *  Created on: 2016.2.19
 *      Author: yunzhoux
 */

#ifndef LIBAVCODEC_LIBYAMI_ENC_H_
#define LIBAVCODEC_LIBYAMI_ENC_H_

using namespace YamiMediaCodec;

#define ENCODE_QUEUE_SIZE 8

typedef enum {
    ENCODE_THREAD_NOT_INIT = 0,
    ENCODE_THREAD_RUNING,
    ENCODE_THREAD_GOT_EOS,
    ENCODE_THREAD_EXIT,
} EncodeThreadStatus;


struct YamiEncContext {
    AVCodecContext *avctx;

    pthread_mutex_t mutex_; // mutex for encoder->getOutput() and YamiEncContext itself update (decode_status, etc)
    IVideoEncoder *encoder;
    VideoEncOutputBuffer outputBuffer;

    pthread_t encode_thread_id;
    std::deque<AVFrame *> *in_queue;
    pthread_mutex_t in_mutex; // mutex for in_queue
    pthread_cond_t in_cond;   // decode thread condition wait
    EncodeThreadStatus encode_status;

    uint8_t *m_buffer;
    uint32_t m_frameSize;
    /***video commom param*****/
    uint32_t cqp;           //qp value 0-52
    uint32_t frame_rate;    //frame rate trasfer the time stamp
    char *rcmod;            //rate control mode CQP|CBR|VBR
    uint32_t gop;           //group of picture 1-250
    char *level;            //level 40|41|50|51
    char *profile;          //profile main|baseline|high
    /*******************/

    uint32_t maxOutSize;

    // debug use
    int encode_count;
    int encode_count_yami;
    int render_count;
};



#endif /* LIBAVCODEC_LIBYAMI_ENC_H_ */
