/*
 * control.cpp
 *
 *  Created on: 2015Äê5ÔÂ6ÈÕ
 *      Author: Winder
 */

#include "header.h"

PidObject pidPitch;
PidObject pidSpeed;

int32_t pwmMin = 160;


float speedFeedback=0;
float speedDesired=0;

Axis3f gyro; // Gyro axis data in deg/s
Axis3f acc;  // Accelerometer axis data in mG
Axis3f magnetometer;  // Magnetometer axis data in testla

float accWZ     = 0.0;
float accMAG    = 0.0;

float eulerRollActual;
float eulerPitchActual;
float eulerYawActual;

float eulerPitchDesired;
float pitchDesired;

float speedOutput=0;
int32_t pwmOutput;

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
	  pitchDesired = pidUpdate(&pidPitch, eulerPitchActual, true);
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
#if 0
	if(pwmMin > 0)
	{
		if(pwmOutput < 80)
		{
			pwmOutput = 0;
		}
		else if (pwmOutput < 160)
		{
			pwmOutput = 160;
		}
	}
	else
	{
		if(pwmOutput > -80)
		{
			pwmOutput = 0;
		}
		else if (pwmOutput > -160)
		{
			pwmOutput = -160;
		}
	}
#endif
	motorSetSpeed(&motorL, pwmOutput);
	motorSetSpeed(&motorR, pwmOutput);
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



