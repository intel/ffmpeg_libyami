/*
 * libyami_utils.cpp
 *
 *  Created on: 2016.2.19
 *      Author: yunzhoux
 */

#include "libyami_utils.h"

static VADisplay getVADisplay(void)
{
    static VADisplay vadisplay = NULL;


    if (!vadisplay) {
        int fd = open("/dev/dri/card0", O_RDWR);
        if (fd < 0) {
//            av_log(NULL, AV_LOG_ERROR, "open card0 failed");
            return NULL;
        }
        vadisplay = vaGetDisplayDRM(fd);
        int majorVersion, minorVersion;
        VAStatus vaStatus = vaInitialize(vadisplay, &majorVersion, &minorVersion);
        if (vaStatus != VA_STATUS_SUCCESS) {
//            av_log(NULL, AV_LOG_ERROR, "va init failed, status =  %d", vaStatus);
            close(fd);
            vadisplay = NULL;
            return NULL;
        }
        return vadisplay;
    } else {
        return vadisplay;
    }
}

VADisplay createVADisplay(void)
{
    return getVADisplay();
}
