/*
 * control.cpp
 *
 *  Created on: 2015年5月6日
 *      Author: Winder
 */

#include "header.h"


BALANCE_CONTROL balanceControl;

int count = 0;

static inline int32_t saturateSignedInt16(float in);

void pidInit()
{
	pidReset(&balanceControl.pidPitch);
	pidInit(&balanceControl.pidPitch, 0, 1, 0, PID_PITCH_KD, IMU_UPDATE_DT/2);
	pidSetIntegralLimit(&balanceControl.pidPitch, PID_PITCH_INTEGRATION_LIMIT);

	pidReset(&balanceControl.pidSpeed);
	pidInit(&balanceControl.pidSpeed, 0, 1, 0, PID_PITCH_KD, IMU_UPDATE_DT/2);
	pidSetIntegralLimit(&balanceControl.pidSpeed, PID_PITCH_INTEGRATION_LIMIT);
}

float error=0, lastError=0, sum=0, pTerm=0, iTerm=0, dTerm=0;
float error2=0, lastError2=0, sum2=0, pTerm2=0, iTerm2=0, dTerm2=0;
float setAngle=0;
float angleOffset=0;

void pidControl()
{
#if 0
	float out;

	pidSetDesired(&balanceControl.pidSpeed, 0);	//设定目标值
	out = pidUpdate(&balanceControl.pidPitch, balanceControl.PwmLeft, true)/16000;				//计算PID

	pidSetDesired(&balanceControl.pidPitch, out);	//设定目标值
	balanceControl.PwmLeft = pidUpdate(&balanceControl.pidPitch, imu.euler.pitch/45, true)*16000;	//计算PID


	//balanceControl.PwmLeft = out - balanceControl.Kd * imu.gyro.y;

	//balanceControl.PwmLeft = balanceControl.Kp * imu.euler.pitch + balanceControl.Kd * imu.gyro.y;

	//printf("Pitch:%12f Gyro x:%12f y:%12f z:%12f PWM:%d ",imu.euler.pitch, imu.gyro.x, imu.gyro.y, imu.gyro.z, balanceControl.PwmLeft);
	printf("Pitch:%12f Gyro y:%12f PWM:%d  ",imu.euler.pitch, imu.gyro.y, balanceControl.PwmLeft);
	printf("Desired:%12f Error:%12f Kp:%12f Kd:%12f\r", balanceControl.pidPitch.desired, balanceControl.pidPitch.error, balanceControl.pidPitch.kp, balanceControl.Kd);
#endif

#define MAX (32000)
#define MAX2 (32000)

	error2 = balanceControl.PwmLeft;

	pTerm2 = balanceControl.Kp * error2;

	sum2 += error2;
	if(sum2> MAX2){ sum2 = MAX2; }
	if(sum2< -MAX2){ sum2 = -MAX2; }

	iTerm2 = balanceControl.Ki * sum2 * IMU_UPDATE_DT/2;

	dTerm2 = balanceControl.Kd * (error2 - lastError2) / IMU_UPDATE_DT/2;

	setAngle = pTerm2 + iTerm2 + dTerm2;

	if(setAngle> MAX2){ setAngle = MAX2; }
	if(setAngle< -MAX2){ setAngle = -MAX2; }

	lastError2 = error2;



	error = (setAngle+angleOffset) - imu.euler.pitch;

	pTerm = balanceControl.Kp * error;

	sum += error;
	if(sum> MAX2){ sum = MAX2; }
	if(sum< -MAX2){ sum = -MAX2; }

	iTerm = balanceControl.Ki * sum * IMU_UPDATE_DT/2;

	dTerm = balanceControl.Kd * (error - lastError) / IMU_UPDATE_DT/2;

	balanceControl.PwmLeft = pTerm + iTerm + dTerm;

	if(balanceControl.PwmLeft > MAX){ balanceControl.PwmLeft = MAX; }
	if(balanceControl.PwmLeft < -MAX){ balanceControl.PwmLeft = -MAX; }

	lastError = error;

	printf("pitch:%12f PWM:%d setAngle:%12f | sum1:%12f lastError:%12f | sum2:%12f lastError2:%12f | Kp:%4f Ki:%4f Kd:%4f\r", imu.euler.pitch, balanceControl.PwmLeft, setAngle, sum, lastError, sum2, lastError2, balanceControl.Kp, balanceControl.Ki, balanceControl.Kd);
}

void motorControl()
{
	motorSetSpeed(&motorL, -balanceControl.PwmLeft);
	motorSetSpeed(&motorR, balanceControl.PwmLeft);
}


/*static*/
void controlUpdate(union sigval v)
{
#if 0	//用于示波器观测定时器
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
#endif

	imuUpdate();
	//printImuData();

	pidControl();


	motorControl();
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



