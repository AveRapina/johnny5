/*
 * main.cpp
 *
 *  Created on: 2015��4��21��
 *      Author: Winder
 */

/*����ͷ------------------------------------------------------------------*/

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
	gpioInitialise();	//��ʼ�� PiGpio
    gpioSetMode(PIN_SIG, PI_OUTPUT);	/*�����źŹܽ� ����ʾ��������*/
	motorInit();		//��ʼ��������ƹܽż�PWMģ��

	pidInit();			//��ʼ��PID

	imu6Init();			//��ʼ��������
	imu6Test();			//�������Լ�

    int a;
    cin >> a;

    motorEnable(&motorL);	//����ʹ��
    motorEnable(&motorR);	//�ҵ��ʹ��

    timerInit();		//��ʼ����ʱ��

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




