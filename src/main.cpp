/*
 * main.cpp
 *
 *  Created on: 2015年4月21日
 *      Author: Winder
 */

/*包含头------------------------------------------------------------------*/

#include "header.h"


#if 0


static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;
static float rollRateDesired;
static float pitchRateDesired;
static float yawRateDesired;

#endif

using namespace std;

extern float error, lastError, sum, pTerm, iTerm, dTerm;
extern float error2, lastError2, sum2, pTerm2, iTerm2, dTerm2;
extern float setAngle, angleOffset;

int main()
{
	gpioInitialise();	//初始化 PiGpio
    gpioSetMode(PIN_SIG, PI_OUTPUT);	/*设置信号管脚 用于示波器测量*/
	motorInit();		//初始化电机控制管脚及PWM模块

	pidInit();			//初始化PID

	imu6Init();			//初始化传感器
	imu6Test();			//传感器自检

    float a;
    cin >> a;

    motorEnable(&motorL);	//左电机使能
    motorEnable(&motorR);	//右电机使能

    timerInit();		//初始化计时器

    balanceControl.Kp = 1;
    balanceControl.Ki = 0;
    balanceControl.Kd = 0;

    while(a != 200)
    {
    	cout<<endl;
    	cin >> a;
    	balanceControl.Kp = a;
    	//pidSetKp(&balanceControl.pidPitch, a);
    	//pidSetKp(&balanceControl.pidSpeed, a);

    	cin >> a;
    	balanceControl.Kd = a;

    	cin >> a;
    	//balanceControl.Ki = a;
    	angleOffset = a;
    	//pidSetKi(&balanceControl.pidPitch, a);
    	//pidSetKi(&balanceControl.pidSpeed, a);

    	sum=0;
    	sum2=0;

    	lastError = 0;
    	lastError2=0;
    }
    motorTerminate();
    //gpioTerminate();

    return 0;
}




