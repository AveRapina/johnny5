/*
 * timer.cpp
 *
 *  Created on: 2015��5��6��
 *      Author: Winder
 */

#include "header.h"

timer_t fade_in_timer;

int count = 0;

int timerInit()
{

    struct sigevent evp;
    memset(&evp, 0, sizeof(evp));

    evp.sigev_value.sival_ptr = &evp;   //���ﴫһ��������ȥ����timer��callback�ص�����������Ի����
    evp.sigev_notify = SIGEV_THREAD;    //��ʱ�����ں��ں˴���һ���߳�ִ��sigev_notify_function����
    evp.sigev_notify_function = imuUpdate; //�������ָ���ص�����

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

/*static*/
void imuUpdate(union sigval v)
{
	if(count)
    {
        count = 0;
        gpioWrite(PIN_SIG, 1);
    }
    else
    {
        count = 1;
        gpioWrite(PIN_SIG, 0);
    }

    //v.sival_ptr ���Ǵ���timerʱ��������ָ�룬����ں��ʵĵط�ɾ��һ��timer
    //myclass *ptr = (myclass*)v.sival_ptr;
    //timer_delete(audiotrack->fade_in_timer);
    //printf("call back func\r\n");
    //timer_delete(&fade_in_timer);

	imu9Read(&gyro, &acc, &magnetometer);

#if 0
	printf("gyro x:%10f y:%10f z:%12f\t", gyro.x,gyro.y,gyro.z);
	printf("acc x:%10f y:%10f z:%12f\t", acc.x,acc.y,acc.z);
	printf("mag x:%10f y:%10f z:%12f\r", mag.x, mag.y, mag.z);
#endif

	// 250HZ
	sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,	FUSION_UPDATE_DT);
	sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);

	accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);
	accMAG = (acc.x * acc.x) + (acc.y * acc.y) + (acc.z * acc.z);

	pidControl();

	pitchDesired = eulerPitchActual * pidPitch.kp + gyro.x * pidPitch.kd;

	motorControl();

	/*
	 controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
	 eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
	 &rollRateDesired, &pitchRateDesired, &yawRateDesired);
	 */

	printf("RPY R:%12f P:%12f Y:%12f PWM:%10d Pref:%12f Error:%12f output:%12f\r\n", eulerRollActual, eulerPitchActual, eulerYawActual, pwmOutput, eulerPitchDesired, pidPitch.error, pidPitch.output);
}


