/*
 * Intel Yet Another Media Infrastructure video decoder/encoder
 *
 * Copyright (c) 2016 Intel Corporation
 *     Zhou Yun(yunx.z.zhou@intel.com)
 *     Jun Zhao(jun.zhao@intel.com)
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


#ifndef LIBYAMI_THR_H_
#define LIBYAMI_THR_H_

#include <pthread.h>
#include <unistd.h>
#include <deque>

typedef enum {
    YAMI_THREAD_NOT_INIT = 0,
    YAMI_THREAD_RUNING,
    YAMI_THREAD_GOT_EOS,
    YAMI_THREAD_EXIT,
} YamiThreadStatus;

typedef void (*yami_process_data_func)(void *handle, void *data);
typedef void (*yami_flush_func)();


#include <deque>
template <typename T>
struct YamiThreadContext {
    pthread_t thread_id;
    yami_process_data_func process_data_cb;
    yami_flush_func flush_cb;
    YamiThreadStatus status;
    void *priv;
    pthread_mutex_t priv_lock;
    pthread_mutex_t in_queue_lock;
    pthread_cond_t in_cond;
    std::deque<T> *in_queue;
    std::deque<T> *out_queue;
    pthread_mutex_t out_queue_lock;
    int max_queue_size;
};

/*
 * ff_yami_thread run in backround come up with in data and out data
 * */
template <typename T>
void * ff_yami_thread(void *args)
{
    struct YamiThreadContext<T> *ctx = (struct YamiThreadContext<T> *)args;
    if (!ctx)
        return NULL;
    while (1) {
        if (!ctx->process_data_cb)
            break;
        pthread_mutex_lock(&ctx->in_queue_lock);
        if (ctx->in_queue->empty()) {
            if (ctx->status == YAMI_THREAD_GOT_EOS) {
                /* flush the decode buffer with NULL when get EOS */
                if (ctx->flush_cb)
                    ctx->flush_cb();
                pthread_mutex_unlock(&ctx->in_queue_lock);
                break;
            }

            pthread_cond_wait(&ctx->in_cond, &ctx->in_queue_lock); // wait the packet to decode
            pthread_mutex_unlock(&ctx->in_queue_lock);
            usleep(100);
            continue;
        }
        void *t = ctx->in_queue->front();
        pthread_mutex_unlock(&ctx->in_queue_lock);
        ctx->process_data_cb (ctx, t);
        pthread_mutex_lock(&ctx->in_queue_lock);
        ctx->in_queue->pop_front();
        pthread_mutex_unlock(&ctx->in_queue_lock);

        if (ctx->status == YAMI_THREAD_EXIT)
            break;
    }
    pthread_mutex_lock(&ctx->priv_lock);
    ctx->status = YAMI_THREAD_EXIT;
    pthread_mutex_unlock(&ctx->priv_lock);
    return NULL;
}


/*
 * user can use ff_yami_push_data to push their in data to ff_yami_thread use this function
 * */

template <typename T>
int ff_yami_push_data (YamiThreadContext<T> *ctx, T t)
{
    if (!ctx || !ctx->in_queue)
        return -1;
    while (ctx->status < YAMI_THREAD_GOT_EOS) {
        /* need enque eos buffer more than once */
        pthread_mutex_lock(&ctx->in_queue_lock);
        if (ctx->in_queue->size() < ctx->max_queue_size) {
            ctx->in_queue->push_back(t);
            pthread_cond_signal(&ctx->in_cond);
            pthread_mutex_unlock(&ctx->in_queue_lock);
            break;
        }
        pthread_mutex_unlock(&ctx->in_queue_lock);
        usleep(1000);
    };
    return 0;
}

/*
 * user can use ff_yami_pop_outdata to get the ff_yami_thread output data
 * */

template <typename T>
T ff_yami_pop_outdata (YamiThreadContext<T> *ctx)
{
    if (!ctx || !ctx->out_queue)
        return NULL;
    pthread_mutex_lock(&ctx->out_queue_lock);
    if (ctx->out_queue->empty()) {
        pthread_mutex_unlock(&ctx->out_queue_lock);
        return NULL;
    }
    T t = ctx->out_queue->front();
    ctx->out_queue->pop_front();
    pthread_mutex_unlock(&ctx->out_queue_lock);
    return t;
}

/*
 * user can use ff_yami_push_outdata in yami_thr_process_data_func to push the ff_yami_thread output data
 * */

template <typename T>
int ff_yami_push_outdata (YamiThreadContext<T> *ctx, T t)
{
    if (!ctx || !ctx->out_queue)
        return -1;
    pthread_mutex_lock(&ctx->out_queue_lock);
    ctx->out_queue->push_back(t);
    pthread_mutex_unlock(&ctx->out_queue_lock);
    return 0;
}

/*
 * user should init the YamiThreadContext<T> struct at the start of code
 * */

template <typename T>
int ff_yami_thread_init (YamiThreadContext<T> *ctx)
{
    int ret = 0;
    if (!ctx)
        return -1;
    if ((ret = pthread_mutex_init(&ctx->priv_lock, NULL)) < 0)
        return ret;
    if ((ret = pthread_mutex_init(&ctx->in_queue_lock, NULL)) < 0)
        return ret;
    if ((ret = pthread_mutex_init(&ctx->out_queue_lock, NULL)) < 0)
            return ret;
    if ((ret = pthread_cond_init(&ctx->in_cond, NULL)) < 0)
        return ret;
    ctx->status = YAMI_THREAD_NOT_INIT;
    ctx->in_queue = new std::deque<T>;
    ctx->out_queue = new std::deque<T>;
    return 0;
}

/*
 * user can use ff_yami_thread_create create the thread
 * */

template <typename T>
int ff_yami_thread_create (YamiThreadContext<T> *ctx)
{
    if (!ctx)
        return -1;
    pthread_mutex_lock(&ctx->priv_lock);
    switch (ctx->status) {
    case YAMI_THREAD_NOT_INIT:
        ctx->status = YAMI_THREAD_RUNING;
        pthread_create(&ctx->thread_id, NULL, &ff_yami_thread<T>, ctx);
        break;
    case YAMI_THREAD_RUNING:
        break;
    case YAMI_THREAD_GOT_EOS:
        pthread_cond_signal(&ctx->in_cond);
        break;
    case YAMI_THREAD_EXIT:
        break;
    default:
        break;
    }
    pthread_mutex_unlock(&ctx->priv_lock);
    return 0;
}

/*
 * user can use ff_yami_read_status read thread status
 * */

template <typename T>
YamiThreadStatus ff_yami_read_thread_status (YamiThreadContext<T> *ctx)
{
    if (!ctx)
        return YAMI_THREAD_NOT_INIT;
    YamiThreadStatus status;
    pthread_mutex_lock (&ctx->priv_lock);
    status = ctx->status;
    pthread_mutex_unlock (&ctx->priv_lock);
    return status;
}

/*
 * user can use ff_yami_set_stream_eof tell thread the end of stream
 * */

template <typename T>
int ff_yami_set_stream_eof (YamiThreadContext<T> *ctx)
{
    if (!ctx)
        return -1;
    YamiThreadStatus status;
    pthread_mutex_lock (&ctx->priv_lock);
    ctx->status = YAMI_THREAD_GOT_EOS;
    pthread_mutex_unlock (&ctx->priv_lock);
    return 0;
}

/*
 * user should call ff_yami_thread_close function thread the end of code
 * */

template <typename T>
int ff_yami_thread_close (YamiThreadContext<T> *ctx)
{
    if (!ctx)
        return -1;
    pthread_mutex_lock(&ctx->priv_lock);
    while (ctx->status != YAMI_THREAD_EXIT
           && ctx->status != YAMI_THREAD_NOT_INIT) { // if decode thread do not create do not loop
        // potential race condition on ctx->status
        ctx->status = YAMI_THREAD_GOT_EOS;
        pthread_mutex_unlock(&ctx->priv_lock);
        pthread_cond_signal(&ctx->in_cond);
        usleep(10000);
        pthread_mutex_lock(&ctx->priv_lock);
    }
    pthread_mutex_unlock(&ctx->priv_lock);
    pthread_mutex_destroy(&ctx->in_queue_lock);
    pthread_mutex_destroy(&ctx->out_queue_lock);
    pthread_cond_destroy(&ctx->in_cond);
    delete ctx->in_queue;
    delete ctx->out_queue;
    return 0;
}


#endif /* LIBYAMI_THR_H_ */
