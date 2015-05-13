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
float angleOffset=0;

void initPidControl()
{
	pidInit(&balanceControl.pidPitch);
	pidInit(&balanceControl.pidSpeed);

#if 0
	balanceControl.pidPitch.outMax = 64000;
	balanceControl.pidPitch.outMin = -64000;

	balanceControl.pidPitch.iMax = 640000;
	balanceControl.pidPitch.iMin = -640000;
#endif
}

static float currendSpeed=0;
static float position=0;
static int flag = 0;
static float angleFiltered=0;
static float gyroFiltered=0;

void pidControl()
{

#if 0
	//balanceControl.pidPitch.desired  = pidUpdate(&balanceControl.pidSpeed, balanceControl.PwmLeft) + angleOffset;
	balanceControl.PwmLeft = pidUpdate(&balanceControl.pidPitch, imu.euler.pitch);
	currendSpeed = (currendSpeed + balanceControl.PwmLeft * 0.004) * 0.999;
	balanceControl.PwmLeft += currendSpeed;
#endif
#if 0
	balanceControl.pidPitch.desired  = pidUpdate(&balanceControl.pidSpeed, balanceControl.PwmLeft) + angleOffset;
	balanceControl.PwmLeft = pidUpdate(&balanceControl.pidPitch, imu.euler.pitch);
#endif
#if 0
	//Kp:5 Kd:1 offset:-3   20/10/-6  0.5/0.2/06
	balanceControl.PwmLeft = -(1000*(balanceControl.pidPitch.Kp * ((imu.euler.pitch+angleOffset) /90)) + balanceControl.pidPitch.Ki * imu.gyro.y);
#endif
#if 0
	currendSpeed *= 0.7;
	currendSpeed = currendSpeed + balanceControl.PwmLeft * 0.3;
	position += currendSpeed;

	if(position<-60000) position = -60000;
	if(position> 60000) position =  60000;	
	
	balanceControl.PwmLeft = balanceControl.pidPitch.Kp*(imu.euler.pitch - angleOffset)
							-balanceControl.pidPitch.Ki*position
							-balanceControl.pidPitch.Kd*currendSpeed;

	balanceControl.PwmLeft = -balanceControl.PwmLeft;

	if(balanceControl.PwmLeft<-60000) balanceControl.PwmLeft = -60000;
	if(balanceControl.PwmLeft> 60000) balanceControl.PwmLeft =  60000;	
#endif
	float gap = abs(balanceControl.pidPitch.desired - imu.euler.pitch);
	if(gap > 2)
	{
		if(flag == 0)
		{
			flag = 1;
			balanceControl.pidPitch.Kp *= 300;
			balanceControl.pidPitch.Ki *= 100;
			balanceControl.pidPitch.Kd *= 100;
			balanceControl.Kd *= 2;
		}
	}
	else
	{
		if(flag == 1)
		{
			flag = 0;
			balanceControl.pidPitch.Kp /= 300;
			balanceControl.pidPitch.Ki /= 100;
			balanceControl.pidPitch.Kd /= 100;
			balanceControl.Kd /= 2;
		}
	}

	gyroFiltered = 0.5 * imu.gyro.y + gyroFiltered * 0.5;
	angleFiltered = 0.2 * imu.euler.pitch + angleFiltered * 0.8;
	balanceControl.PwmLeft = pidUpdate(&balanceControl.pidPitch, angleFiltered) - gyroFiltered * balanceControl.Kd;
	//balanceControl.PwmLeft = pidUpdate(&balanceControl.pidPitch, imu.euler.pitch);

	printf("pitch:%6.4f | Kp:%4.2f Ki:%4.2f Kd:%4.2f Ref:%4.2f | error:%6.4f sumerror:%6.4f | PWM:%d\r", 
		imu.euler.pitch, balanceControl.pidPitch.Kp, balanceControl.pidPitch.Ki, balanceControl.Kd, balanceControl.pidPitch.desired,
		balanceControl.pidPitch.error, balanceControl.pidPitch.sumError, 
		balanceControl.PwmLeft);
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



