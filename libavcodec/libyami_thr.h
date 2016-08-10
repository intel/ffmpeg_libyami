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

typedef void (*yami_thr_process_data_func)(void *handle, void *data);
typedef void (*yami_thr_flush_func)();


#include <deque>
template <typename T>
struct YamiThreadContext {
    pthread_t yami_thread_id;
    yami_thr_process_data_func yami_thread_process_data;
    yami_thr_flush_func yami_thread_flush;
    YamiThreadStatus thread_status;
    pthread_mutex_t user_data_lock;
    pthread_mutex_t in_queue_lock;
    pthread_mutex_t out_queue_lock;
    pthread_cond_t in_cond;
    std::deque<T> *in_queue;
    std::deque<T> *out_queue;
    int max_queue_size;
    int has_out_queue;
    void *user_data;
};

/*
 * ff_yami_thread run in backround come up with in data and out data
 * */
template <typename T>
void * ff_yami_thread(void *args)
{
    struct YamiThreadContext<T> *ytc = (struct YamiThreadContext<T> *)args;
    if (!ytc)
        return NULL;
    while (1)
    {
        if (!ytc->yami_thread_process_data)
            break;
        pthread_mutex_lock(&ytc->in_queue_lock);
        if (ytc->in_queue->empty()) {
            if (ytc->thread_status == YAMI_THREAD_GOT_EOS) {
                /* flush the decode buffer with NULL when get EOS */
                if (ytc->yami_thread_flush)
                    ytc->yami_thread_flush();
                pthread_mutex_unlock(&ytc->in_queue_lock);
                break;
            }

            pthread_cond_wait(&ytc->in_cond, &ytc->in_queue_lock); // wait the packet to decode
            pthread_mutex_unlock(&ytc->in_queue_lock);
            usleep(100);
            continue;
        }
        void *t = ytc->in_queue->front();
        pthread_mutex_unlock(&ytc->in_queue_lock);
        ytc->yami_thread_process_data (ytc, t);
        pthread_mutex_lock(&ytc->in_queue_lock);
        ytc->in_queue->pop_front();
        pthread_mutex_unlock(&ytc->in_queue_lock);

        if (ytc->thread_status == YAMI_THREAD_EXIT)
            break;
    }
    pthread_mutex_lock(&ytc->user_data_lock);
    ytc->thread_status = YAMI_THREAD_EXIT;
    pthread_mutex_unlock(&ytc->user_data_lock);
    return NULL;
}


/*
 * user can use ff_yami_push_data to push their in data to ff_yami_thread use this function
 * */

template <typename T>
int ff_yami_push_data (YamiThreadContext<T> *ytc, T t)
{
    if (!ytc || !ytc->in_queue)
        return -1;
    while (ytc->thread_status < YAMI_THREAD_GOT_EOS) {
        /* need enque eos buffer more than once */
        pthread_mutex_lock(&ytc->in_queue_lock);
        if (ytc->in_queue->size() < ytc->max_queue_size) {
            ytc->in_queue->push_back(t);
            pthread_cond_signal(&ytc->in_cond);
            pthread_mutex_unlock(&ytc->in_queue_lock);
            break;
        }
        pthread_mutex_unlock(&ytc->in_queue_lock);
        usleep(1000);
    };
    return 0;
}

/*
 * user can use ff_yami_pop_outdata to get the ff_yami_thread output data
 * */

template <typename T>
T ff_yami_pop_outdata (YamiThreadContext<T> *ytc)
{
    if (!ytc || !ytc->out_queue)
        return NULL;
    pthread_mutex_lock(&ytc->out_queue_lock);
    if (ytc->out_queue->empty()) {
        pthread_mutex_unlock(&ytc->out_queue_lock);
        return NULL;
    }
    T t = ytc->out_queue->front();
    ytc->out_queue->pop_front();
    pthread_mutex_unlock(&ytc->out_queue_lock);
    return t;
}

/*
 * user can use ff_yami_push_outdata in yami_thr_process_data_func to push the ff_yami_thread output data
 * */

template <typename T>
int ff_yami_push_outdata (YamiThreadContext<T> *ytc, T t)
{
    if (!ytc || !ytc->out_queue)
        return -1;
    pthread_mutex_lock(&ytc->out_queue_lock);
    ytc->out_queue->push_back(t);
    pthread_mutex_unlock(&ytc->out_queue_lock);
    return 0;
}

/*
 * user should init the YamiThreadContext<T> struct at the start of code
 * */

template <typename T>
int ff_yami_thread_init (YamiThreadContext<T> *ytc)
{
    int ret = 0;
    if (!ytc)
        return -1;
    if ((ret = pthread_mutex_init(&ytc->user_data_lock, NULL)) < 0)
        return ret;
    if ((ret = pthread_mutex_init(&ytc->in_queue_lock, NULL)) < 0)
        return ret;
    if ((ret = pthread_mutex_init(&ytc->out_queue_lock, NULL)) < 0)
            return ret;
    if ((ret = pthread_cond_init(&ytc->in_cond, NULL)) < 0)
        return ret;
    ytc->thread_status = YAMI_THREAD_NOT_INIT;
    ytc->in_queue = new std::deque<T>;
    ytc->out_queue = new std::deque<T>;
    return 0;
}

/*
 * user can use ff_yami_thread_create create the thread
 * */

template <typename T>
int ff_yami_thread_create (YamiThreadContext<T> *ytc)
{
    if (!ytc)
        return -1;
    pthread_mutex_lock(&ytc->user_data_lock);
    switch (ytc->thread_status) {
    case YAMI_THREAD_NOT_INIT:
        ytc->thread_status = YAMI_THREAD_RUNING;
        pthread_create(&ytc->yami_thread_id, NULL, &ff_yami_thread<T>, ytc);
        break;
    case YAMI_THREAD_RUNING:
        break;
    case YAMI_THREAD_GOT_EOS:
        pthread_cond_signal(&ytc->in_cond);
        break;
    case YAMI_THREAD_EXIT:
        break;
    default:
        break;
    }
    pthread_mutex_unlock(&ytc->user_data_lock);

    return 0;
}

/*
 * user can use ff_yami_read_thread_status read thread status
 * */

template <typename T>
YamiThreadStatus ff_yami_read_thread_status (YamiThreadContext<T> *ytc)
{
    YamiThreadStatus thread_status;
    pthread_mutex_lock (&ytc->user_data_lock);
    thread_status = ytc->thread_status;
    pthread_mutex_unlock (&ytc->user_data_lock);
    return thread_status;
}

/*
 * user can use ff_yami_set_stream_eof tell thread the end of stream
 * */

template <typename T>
int ff_yami_set_stream_eof (YamiThreadContext<T> *ytc)
{
    YamiThreadStatus thread_status;
    pthread_mutex_lock (&ytc->user_data_lock);
    ytc->thread_status = YAMI_THREAD_GOT_EOS;
    pthread_mutex_unlock (&ytc->user_data_lock);
    return 0;
}

/*
 * user should call ff_yami_thread_close function thread the end of code
 * */

template <typename T>
int ff_yami_thread_close (YamiThreadContext<T> *ytc)
{
    if (!ytc)
        return -1;
    pthread_mutex_lock(&ytc->user_data_lock);
    while (ytc->thread_status != YAMI_THREAD_EXIT
           && ytc->thread_status != YAMI_THREAD_NOT_INIT) { // if decode thread do not create do not loop
        // potential race condition on s->decode_status
        ytc->thread_status = YAMI_THREAD_GOT_EOS;
        pthread_mutex_unlock(&ytc->user_data_lock);
        pthread_cond_signal(&ytc->in_cond);
        usleep(10000);
        pthread_mutex_lock(&ytc->user_data_lock);
    }
    pthread_mutex_unlock(&ytc->user_data_lock);
    pthread_mutex_destroy(&ytc->in_queue_lock);
    pthread_mutex_destroy(&ytc->out_queue_lock);
    pthread_cond_destroy(&ytc->in_cond);
    delete ytc->in_queue;
    delete ytc->out_queue;
    return 0;
}


#endif /* LIBYAMI_THR_H_ */
