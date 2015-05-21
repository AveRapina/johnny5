/*
 * control.cpp
 *
 *  Created on: 2015年5月6日
 *      Author: Winder
 */

#include "header.h"

#define JOYSTICK_AXIS_MAX (0x7FFF)
#define JOYSTICK_AXIS_MIN (-0x7FFF)

#define LPF_K1 (0.95)
#define LPF_K2 (0.05)

BALANCE_CONTROL balanceControl;
Kalman kalmanPitch;

int count = 0;

static inline int32_t saturateSignedInt16(float in);
void joystickControl();

float angleOffset=0;
extern Joystick* joystick;

float LPF(float lastValue, float currentValue);
float smonthSpeed = 0;
float smonthSpinSpeed = 0;

void initPidControl()
{
	pidInit(&balanceControl.pidPitch);
	pidInit(&balanceControl.pidSpeed);

#if 1
	//balanceControl.pidPitch.outMax = 64000;
	//balanceControl.pidPitch.outMin = -64000;

	balanceControl.pidPitch.iMax = 1000;
	balanceControl.pidPitch.iMin = -1000;
#endif
	balanceControl.pidSpeed.iMax = 1000;
	balanceControl.pidSpeed.iMin = -1000;

	kalmanPitch.setAngle(imu.euler.pitch);
	balanceControl.factorL = 1;
	balanceControl.factorR = 1;
	balanceControl.spinSpeed = 0;
}

static float currendSpeed=0;
static float position=0;
float speed_need = 0;
static int flag = 0;
static float angleFiltered=0;
static float gyroFiltered=0;

void pidControl()
{
	angleFiltered = kalmanPitch.getAngle(imu.euler.pitch, imu.gyro.y/131.0, (double)1/FREQ);
	gyroFiltered = LPF(gyroFiltered, imu.gyro.y);
	//kalman_filter(imu.euler.pitch, imu.gyro.y/131.0, &angleFiltered, &gyroFiltered);
#if 0
	balanceControl.pidPitch.desired  = pidUpdate(&balanceControl.pidSpeed, balanceControl.PwmLeft) + angleOffset;
	balanceControl.PwmLeft = pidUpdate(&balanceControl.pidPitch, angleFiltered);
	currendSpeed = (currendSpeed + balanceControl.PwmLeft * 0.004) * 0.999;
	balanceControl.PwmLeft += currendSpeed;
#endif
#if 0
	balanceControl.pidPitch.desired  = pidUpdate(&balanceControl.pidSpeed, balanceControl.PwmLeft) + angleOffset;
	balanceControl.PwmLeft = pidUpdate(&balanceControl.pidPitch, angleFiltered);
#endif
#if 0
	//Kp:5 Kd:1 offset:-3   20/10/-6  0.5/0.2/06
	balanceControl.PwmLeft = -(1000*(balanceControl.pidPitch.Kp * ((angleFiltered + angleOffset) /90)) + balanceControl.Kd * imu.gyro.y);
#endif
#if 0
	currendSpeed *= 0.7;
	currendSpeed = currendSpeed + balanceControl.PwmLeft * 0.3;
	position += currendSpeed;
	position -= speed_need;

	if(position<-6000000) position = -6000000;
	if(position> 6000000) position =  6000000;	
	
	balanceControl.PwmLeft = balanceControl.pidPitch.Kp * (angleOffset - angleFiltered)
							+balanceControl.Kd * gyroFiltered
							-balanceControl.pidSpeed.Ki * position
							-balanceControl.pidSpeed.Kd * currendSpeed;

	balanceControl.PwmLeft = -balanceControl.PwmLeft;

	if(balanceControl.PwmLeft<-60000) balanceControl.PwmLeft = -60000;
	if(balanceControl.PwmLeft> 60000) balanceControl.PwmLeft =  60000;	

	if(balanceControl.PwmLeft > - 100 && balanceControl.PwmLeft < 100) { balanceControl.PwmLeft = 0; }

	balanceControl.PwmRight = balanceControl.PwmLeft;
#endif
#if 0
	float gap = abs(balanceControl.pidPitch.desired - imu.euler.pitch);
	if(gap > 150)
	{
		if(flag == 0)
		{
			flag = 1;
			balanceControl.pidPitch.Kp *= 10;
			balanceControl.pidPitch.Ki *= 10;
			balanceControl.pidPitch.Kd *= 10;
			//balanceControl.Kd *= 2;
		}
	}
	else
	{
		if(flag == 1)
		{
			flag = 0;
			balanceControl.pidPitch.Kp /= 10;
			balanceControl.pidPitch.Ki /= 10;
			balanceControl.pidPitch.Kd /= 10;
			//balanceControl.Kd /= 2;
		}
	}
#endif
#if 0 //works
	balanceControl.speed = balanceControl.speed * 0.05 + pidUpdate(&balanceControl.pidSpeed, balanceControl.speed) * 0.95;

	gyroFiltered = 0.05 * imu.gyro.y + gyroFiltered * 0.95;
	//angleFiltered = 0.2 * imu.euler.pitch + angleFiltered * 0.8;
	//angleFiltered = kalmanPitch.getAngle(imu.euler.pitch, imu.gyro.y/131.0, (double)1/250);
	//angleFiltered = kalman(imu.euler.pitch, imu.gyro.y, (double)1/FREQ);

	balanceControl.pidPitch.desired = balanceControl.speed + angleOffset;
	balanceControl.speed = pidUpdate(&balanceControl.pidPitch, angleFiltered) + gyroFiltered * balanceControl.Kd;
	//balanceControl.speed += pidUpdate(&balanceControl.pidPitch, angleFiltered) + gyroFiltered * balanceControl.Kd;
	//balanceControl.PwmLeft = pidUpdate(&balanceControl.pidPitch, imu.euler.pitch);

	if(balanceControl.speed < 0.100 && balanceControl.speed > -0.100) { balanceControl.speed = 0; }  //dead-band of PWM

#endif
#if 1
	balanceControl.speed = LPF((float)(balanceControl.PwmLeft + balanceControl.PwmRight)/5600, balanceControl.speed);
	balanceControl.speed = pidUpdate(&balanceControl.pidSpeed, balanceControl.speed);
	balanceControl.pidPitch.desired = balanceControl.speed + angleOffset;
	balanceControl.speed = pidUpdate(&balanceControl.pidPitch, angleFiltered) + gyroFiltered * balanceControl.Kd;

	if(balanceControl.speed < 0.050 && balanceControl.speed > -0.050) { balanceControl.speed = 0; }  //dead-band of PWM
#endif

	//printf("dev:%6.4f ", balanceControl.pidPitch.derivative);
	printf("AngleSet:%4.2f AngleRef:%4.2f Angle:%4.2f error:%6.4f ", angleOffset, balanceControl.pidPitch.desired, angleFiltered, balanceControl.pidPitch.error);
	printf("\t| Kp:%3.2f Kd:%3.3f sumerror:%6.2f", balanceControl.pidPitch.Kp, balanceControl.Kd, balanceControl.pidPitch.sumError);
	printf("\t| Kp:%3.2f Ki:%3.3f sumerror:%6.2f", balanceControl.pidSpeed.Kp, balanceControl.pidSpeed.Ki, balanceControl.pidSpeed.sumError);
	printf("\t| speedref:%6.2f speed%6.2f error:%6.2f\r\n", balanceControl.pidSpeed.desired, balanceControl.speed, balanceControl.pidSpeed.error);

	/*
	printf("\t| Kp:%4.2f Ki:%4.2f Kd:%4.2f Speed:%4.2f angle:%4.2f |\t error:%6.4f sumerror:%6.4f Iterm:%6.4f | PWM:%6.3f\r\n", 
		balanceControl.pidSpeed.Kp, balanceControl.pidSpeed.Ki, balanceControl.pidSpeed.Kd, 
		balanceControl.pidSpeed.desired, balanceControl.pidPitch.desired,
		balanceControl.pidSpeed.error, balanceControl.pidSpeed.sumError, balanceControl.pidSpeed.intergal, 
		balanceControl.speed);
	*/

	balanceControl.PwmLeft  = 2800 * (balanceControl.speed * balanceControl.factorL - balanceControl.spinSpeed);
	balanceControl.PwmRight = 2800 * (balanceControl.speed * balanceControl.factorR + balanceControl.spinSpeed);
}

void motorControl()
{

	motorSetSpeed(&motorL, -balanceControl.PwmLeft);
	motorSetSpeed(&motorR, balanceControl.PwmRight);
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
#if 1
	imuUpdate();
	//printImuData();

	pidControl();


	motorControl();

	joystickControl();
#endif
	//control2();
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

void joystickControl()
{
	// Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick->sample(&event))
    {
      if (event.isButton())
      {
        printf("Button %u is %s\n",
          event.number,
          event.value == 0 ? "up" : "down");
      }
      else if (event.isAxis())
      {
        printf("Axis %u is at position %d\n", event.number, event.value);
		switch(event.number)
		{
			case 0:
				//balanceControl.spinSpeed =(float)event.value/JOYSTICK_AXIS_MAX;
				smonthSpinSpeed = (float)event.value/JOYSTICK_AXIS_MAX;
				//balanceControl.factorL = event.value/JOYSTICK_AXIS_MAX;
				//balanceControl.factorR = 1-balanceControl.factorL;
			break;
			case 1:
			break;
			case 2:		//右手左右
			break;
			case 3:		//右手前后
				//angle = axisToAngle(event.value);
				//printf("angle %f\r\n", angle);
				//servoSetAngle(&servoH, axisToAngle(event.value));
				smonthSpeed = -(float)event.value/JOYSTICK_AXIS_MAX * 10;
				
				//balanceControl.pidSpeed.desired = -(float)event.value/JOYSTICK_AXIS_MAX * 5;
			break;
		}
      }
  	}

  	balanceControl.pidSpeed.desired = LPF(balanceControl.pidSpeed.desired, smonthSpeed);
  	balanceControl.spinSpeed = LPF(balanceControl.spinSpeed, smonthSpinSpeed);

  	speed_need = LPF(speed_need, smonthSpeed);
}

float LPF(float lastValue, float currentValue)
{
	return lastValue * LPF_K1 + currentValue * LPF_K2;
}


