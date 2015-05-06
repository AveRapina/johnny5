/*
 * control.cpp
 *
 *  Created on: 2015��5��6��
 *      Author: Winder
 */

#include "header.h"

PidObject pidPitch;
PidObject pidSpeed;

int32_t pwmMin = 160;


float speedFeedback=0;
float speedDesired=0;

#if 0
float eulerRollActual;
float eulerPitchActual;
float eulerYawActual;
#endif

float eulerPitchDesired;
float pitchDesired;

float speedOutput=0;
int32_t pwmOutput;

int count = 0;

static inline int32_t saturateSignedInt16(float in);

void pidInit()
{
	pidReset(&pidPitch);
	pidInit(&pidPitch, 0, 200, 2, PID_PITCH_KD, IMU_UPDATE_DT/2);
	pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);

	pidReset(&pidSpeed);
	pidInit(&pidSpeed, -10000, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, IMU_UPDATE_DT/2);
	pidSetIntegralLimit(&pidSpeed, PID_PITCH_INTEGRATION_LIMIT);
}

void pidControl()
{
	speedFeedback = (pwmOutput/16/4) * MATH_PI * 0.114;

	  pidSetDesired(&pidSpeed, speedDesired);
	  speedOutput = pidUpdate(&pidPitch, speedFeedback, true);

	// Update PID for pitch axis
	  pidSetDesired(&pidPitch, eulerPitchDesired);
	  //pidSetDesired(&pidPitch, speedOutput);
	  pitchDesired = pidUpdate(&pidPitch, imu.euler.pitch, true);
}

void motorControl()
{
	pwmOutput = saturateSignedInt16(pitchDesired);

#if 1
	if(pwmOutput > 0)
	{
		if(pwmOutput < pwmMin)
		{
			pwmOutput = 0;
		}
		else if (pwmOutput < pwmMin*2)
		{
			pwmOutput = pwmMin*2;
		}
	}
	else
	{
		if(pwmOutput > -pwmMin)
		{
			pwmOutput = 0;
		}
		else if (pwmOutput > -pwmMin*2)
		{
			pwmOutput = -pwmMin*2;
		}
	}
#endif

	motorSetSpeed(&motorL, pwmOutput);
	motorSetSpeed(&motorR, pwmOutput);
}


/*static*/
void controlUpdate(union sigval v)
{
#if 0	//����ʾ�����۲ⶨʱ��
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
#endif

	imuUpdate();
	printImuData();

	pidControl();

	pitchDesired = imu.euler.pitch * pidPitch.kp + imu.gyro.x * pidPitch.kd;

	motorControl();

	/*
	 controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
	 eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
	 &rollRateDesired, &pitchRateDesired, &yawRateDesired);
	 */

	printImuData();
	printf("\r");
}


static inline int32_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT32_MAX)
    return INT32_MAX;
  else if (in < -INT32_MAX)
    return -INT32_MAX;
  else
    return (int32_t)in;
}



