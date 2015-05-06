/*
 * timer.cpp
 *
 *  Created on: 2015年5月6日
 *      Author: Winder
 */

#include "header.h"

timer_t fade_in_timer;

int timerInit()
{

    struct sigevent evp;
    memset(&evp, 0, sizeof(evp));

    evp.sigev_value.sival_ptr = &evp;   //这里传一个参数进去，在timer的callback回调函数里面可以获得它
    evp.sigev_notify = SIGEV_THREAD;    //定时器到期后内核创建一个线程执行sigev_notify_function函数
    evp.sigev_notify_function = controlUpdate; //这个就是指定回调函数

    int ret = 0;
    ret = timer_create(CLOCK_REALTIME, &evp, &fade_in_timer);

    if(ret < 0)
    {
        printf("timer_create() fail, ret:%d", ret);
        return ret;
    }

    struct itimerspec ts;
    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = SAMPLE_DT; //4ms
    ts.it_value.tv_sec = 0;
    ts.it_value.tv_nsec = SAMPLE_DT; //4ms
    ret = timer_settime(fade_in_timer, TIMER_ABSTIME, &ts, NULL);

    printf("%d",SAMPLE_DT);
    if(ret < 0)
    {
        printf("timer_settime() fail, ret:%d", ret);
        timer_delete(fade_in_timer);
        //timer_created = false;
        return ret;
    }
}


