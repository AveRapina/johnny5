/*
 * control.h
 *
 *  Created on: 2015年5月6日
 *      Author: Winder
 */

#ifndef INCLUDE_CONTROL_H_
#define INCLUDE_CONTROL_H_

#define INT8_MIN (-128)
#define INT16_MIN (-32768)
#define INT32_MIN (-2147483647 - 1)
#define INT64_MIN  (-9223372036854775807LL - 1)

#define INT8_MAX 127
#define INT16_MAX 32767
#define INT32_MAX 2147483647
#define INT64_MAX 9223372036854775807LL

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

#define FREQ (250)
#define SECOND_TO_NS (1000000000)
#define SAMPLE_DT (SECOND_TO_NS/FREQ)

#define MATH_PI (3.1415926535897932384626433832795)

#define PID_PITCH_KP  3.5
#define PID_PITCH_KI  2.0
#define PID_PITCH_KD  0.0
#define PID_PITCH_INTEGRATION_LIMIT   20.0

typedef struct _BALANCE_CONTROL
{
	float Kp;
	float Ki;
	float Kd;

	float pitchRef;
	float pitchFdb;

	float speedRef;
	float speedFdb;

	PID pidPitch;	//平衡控制PID
	PID pidSpeed;	//速度控制PID

	int32_t PwmLeft;
	int32_t PwmRight;

}BALANCE_CONTROL, *pBALANCE_CONTROL;

extern BALANCE_CONTROL balanceControl;

extern void initPidControl();
extern void pidControl();
extern void motorControl();

extern void controlUpdate(union sigval v);

#endif /* INCLUDE_CONTROL_H_ */
