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

int main()
{
	gpioInitialise();	//初始化 PiGpio
    gpioSetMode(PIN_SIG, PI_OUTPUT);	/*设置信号管脚 用于示波器测量*/
	motorInit();		//初始化电机控制管脚及PWM模块

	pidInit();			//初始化PID

	imu6Init();			//初始化传感器
	imu6Test();			//传感器自检

    int a;
    cin >> a;

    motorEnable(&motorL);	//左电机使能
    motorEnable(&motorR);	//右电机使能

    timerInit();		//初始化计时器

    //cin >> a;

    while(a>0)
    {
    	int Kp,Ki,Kd,angle;

    	cout<<"Kp:\r\n";
    	cin>>Kp;
    	cout<<"Ki:\r\n";
    	cin>>Ki;
    	//cout<<"Kd:\r\n";
    	//cin>>Kd;

    	//cout<<"pwmMin:\r\n";
    	//cin>>pwmMin;

    	cout<<"Angle:\r\n";
    	cin>>angle;

    	pidReset(&pidPitch);
    	pidInit(&pidPitch, angle, Kp, Ki, Kd, IMU_UPDATE_DT/2);

    	cout<<"Continue?:\r\n";
    	cin>>a;
    }

    motorTerminate();
    //gpioTerminate();

    return 0;
}




