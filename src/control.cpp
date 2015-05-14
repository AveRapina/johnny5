/*
 * control.cpp
 *
 *  Created on: 2015年5月6日
 *      Author: Winder
 */

#include "header.h"


BALANCE_CONTROL balanceControl;
Kalman kalmanPitch;

int count = 0;

/* Kalman filter variables and constants */ 
const float Q_angle = 0.001; // Process noise covariance for the accelerometer - Sw 
const float Q_gyro = 0.003; // Process noise covariance for the gyro - Sw 
const float R_angle = 0.03; // Measurement noise covariance - Sv 

static double angle = 120; // It starts at 180 degrees 
static double bias = 0; 
static double P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0; 
static double dt, y, S; 
static double K_0, K_1; 

double kalman(double newAngle, double newRate, double dtime);

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

	kalmanPitch.setAngle(imu.euler.pitch);
}

static float currendSpeed=0;
static float position=0;
static int flag = 0;
static float angleFiltered=0;
static float gyroFiltered=0;

void pidControl()
{
	angleFiltered = kalmanPitch.getAngle(imu.euler.pitch, imu.gyro.y/131.0, (double)1/FREQ);
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

	if(position<-60000) position = -60000;
	if(position> 60000) position =  60000;	
	
	balanceControl.PwmLeft = balanceControl.pidPitch.Kp * (angleFiltered- angleOffset)
							-balanceControl.pidPitch.Ki * position
							-balanceControl.Kd * currendSpeed;

	balanceControl.PwmLeft = -balanceControl.PwmLeft;

	if(balanceControl.PwmLeft<-60000) balanceControl.PwmLeft = -60000;
	if(balanceControl.PwmLeft> 60000) balanceControl.PwmLeft =  60000;	
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
#if 1
	gyroFiltered = 0.05 * imu.gyro.y + gyroFiltered * 0.95;
	//angleFiltered = 0.2 * imu.euler.pitch + angleFiltered * 0.8;
	//angleFiltered = kalmanPitch.getAngle(imu.euler.pitch, imu.gyro.y/131.0, (double)1/250);
	//angleFiltered = kalman(imu.euler.pitch, imu.gyro.y, (double)1/FREQ);

	balanceControl.PwmLeft = pidUpdate(&balanceControl.pidPitch, angleFiltered) + gyroFiltered * balanceControl.Kd;
	//balanceControl.PwmLeft = pidUpdate(&balanceControl.pidPitch, imu.euler.pitch);

	if(balanceControl.PwmLeft < 100 && balanceControl.PwmLeft > -100) { balanceControl.PwmLeft = 0; }  //dead-band of PWM
#endif
	//printf("dev:%6.4f ", balanceControl.pidPitch.derivative);
	printf("angleF:%6.4f pitch:%6.4f gyroY:%6.4f ", imu.euler.pitch, angleFiltered, imu.gyro.y);
	printf("| Kp:%4.2f Ki:%4.2f Kd:%4.2f Ref:%4.2f | error:%6.4f sumerror:%6.4f | PWM:%d\r\n", 
		balanceControl.pidPitch.Kp, balanceControl.pidPitch.Ki, balanceControl.Kd, balanceControl.pidPitch.desired,
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

double kalman(double newAngle, double newRate, double dtime) 
{
    // KasBot V2  -  Kalman filter module - http://www.arduino.cc/cgi-bin/yabb2/YaBB.pl?num=1284738418 - http://www.x-firm.com/?page_id=145
    // with slightly modifications by Kristian Lauszus
    // See http://academic.csuohio.edu/simond/courses/eec644/kalman.pdf and http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf for more information
    //dt = dtime / 1000000; // Convert from microseconds to seconds
    dt = dtime;
    
    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    angle += dt * (newRate - bias);
    
    printf("angle:%6.4f newAngle:%6.4f newRate:%6.4f dt:%6.4f\r", angle, newAngle, newRate, dt);

    // Update estimation error covariance - Project the error covariance ahead
    P_00 += -dt * (P_10 + P_01) + Q_angle * dt;
    P_01 += -dt * P_11;
    P_10 += -dt * P_11;
    P_11 += +Q_gyro * dt;
    
    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    S = P_00 + R_angle;
    K_0 = P_00 / S;
    K_1 = P_10 / S;
    
    // Calculate angle and resting rate - Update estimate with measurement zk
    y = newAngle - angle;
    angle += K_0 * y;
    bias += K_1 * y;
    
    // Calculate estimation error covariance - Update the error covariance
    P_00 -= K_0 * P_00;
    P_01 -= K_0 * P_01;
    P_10 -= K_1 * P_00;
    P_11 -= K_1 * P_01;
    
    return angle;
}


