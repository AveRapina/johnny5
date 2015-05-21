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

extern float set_point;
extern float proportion;
extern float integral2;
extern float derivative;
extern float derivative2;
extern float speed_need;

Joystick* joystick;

void initJoystick()
{
  // Create an instance of Joystick
  joystick  = new Joystick("/dev/input/js2");

  // Ensure that it was found and that we can use it
  if (!joystick->isFound())
  {
    printf("open failed.\n");
    exit(1);
  }
  else
  {
    printf("Joystick OK.\n");
  }
}

int main()
{
	gpioInitialise();	//初始化 PiGpio
    gpioSetMode(PIN_SIG, PI_OUTPUT);	/*设置信号管脚 用于示波器测量*/
	motorInit();		//初始化电机控制管脚及PWM模块

    imu6Init();			//初始化传感器
	imu6Test();			//传感器自检

    initJoystick();

    initPidControl();   //初始化PID

    float a;
    cin >> a;

    motorEnable(&motorL);	//左电机使能
    motorEnable(&motorR);	//右电机使能

    timerInit();		//初始化计时器
#if 0
    balanceControl.pidPitch.Kp = 1500;
    balanceControl.pidPitch.Ki = 1000;
    balanceControl.Kd = 5;
    balanceControl.pidPitch.desired = 5;
#endif
#if 1
    balanceControl.pidPitch.Kp = 1000;
    balanceControl.pidPitch.Ki = 0;
    balanceControl.Kd = 8;
    //balanceControl.pidPitch.desired = 5;
    angleOffset = 5;

    balanceControl.pidSpeed.Ki = 0.005;
    balanceControl.pidSpeed.Kd = 0.5;
#endif

    while(a != 200)
    {
    	cout<<endl;

        //cin >> a;
        //balanceControl.pidSpeed.desired = a;
        cout<<"Kp["<<balanceControl.pidPitch.Kp<<"]:";
    	cin >> a;
        //proportion = a;     
        pidSetKp(&balanceControl.pidPitch, a);

        cout<<"Kd["<<balanceControl.Kd<<"]:";
        cin >> a;
        balanceControl.Kd = a;

    	//pidSetKp(&balanceControl.pidPitch, a);
    	//pidSetKp(&balanceControl.pidSpeed, a);
        cout<<"Kpos["<<balanceControl.pidSpeed.Ki<<"]:";
        cin >> a;
        balanceControl.pidSpeed.Ki = a;        

        //pidSetKi(&balanceControl.pidPitch, a);
        //pidSetKi(&balanceControl.pidSpeed, a);
        cout<<"Kspeed["<<balanceControl.pidSpeed.Kd<<"]:";
        cin >> a;
        balanceControl.pidSpeed.Kd = a;

        //balanceControl.pidPitch.Kd = a;
        //balanceControl.Kd = a;
        //pidSetKd(&balanceControl.pidPitch, a);
        //pidSetKd(&balanceControl.pidSpeed, a);

    	cin >> a;
        //set_point;
    	angleOffset = a;
        balanceControl.pidPitch.desired = a;

        //cin>>a;
        //speed_need = a;

        pidReset(&balanceControl.pidPitch);
        pidReset(&balanceControl.pidSpeed);
    }
    motorTerminate();
    //gpioTerminate();

    return 0;
}




