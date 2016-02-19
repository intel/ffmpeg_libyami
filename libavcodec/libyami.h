/*
 * libyami.h
 *
 *  Created on: 2016.2.19
 *      Author: yunzhoux
 */

#ifndef LIBAVCODEC_LIBYAMI_H_
#define LIBAVCODEC_LIBYAMI_H_

#include <pthread.h>
#include <unistd.h>
#include <assert.h>
#include <deque>

extern "C" {
#include "avcodec.h"
#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "internal.h"
}

#include "VideoDecoderHost.h"
#include "VideoEncoderHost.h"

#include "libyami_dec.h"
#include "libyami_enc.h"


//using namespace YamiMediaCodec;
#ifndef VA_FOURCC_I420
#define VA_FOURCC_I420 VA_FOURCC('I','4','2','0')
#endif

#ifndef VA_FOURCC_NV12
#define VA_FOURCC_NV12 VA_FOURCC('N','V','1','2')
#endif


int yami_dec_init(AVCodecContext *avctx);

int yami_dec_frame(AVCodecContext *avctx, void *data,
                          int *got_frame, AVPacket *avpkt);

int yami_dec_close(AVCodecContext *avctx);

int yami_enc_init(AVCodecContext *avctx);

int yami_enc_frame(AVCodecContext *avctx, AVPacket *pkt,
                          const AVFrame *frame, int *got_packet);

int yami_enc_close(AVCodecContext *avctx);


#endif /* LIBAVCODEC_LIBYAMI_H_ */
