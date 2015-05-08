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

extern float error, lastError, sum, pTerm, iTerm, dTerm;
extern float error2, lastError2, sum2, pTerm2, iTerm2, dTerm2;
extern float setAngle, angleOffset;

int main()
{
	gpioInitialise();	//��ʼ�� PiGpio
    gpioSetMode(PIN_SIG, PI_OUTPUT);	/*�����źŹܽ� ����ʾ��������*/
	motorInit();		//��ʼ��������ƹܽż�PWMģ��

	pidInit();			//��ʼ��PID

	imu6Init();			//��ʼ��������
	imu6Test();			//�������Լ�

    float a;
    cin >> a;

    motorEnable(&motorL);	//����ʹ��
    motorEnable(&motorR);	//�ҵ��ʹ��

    timerInit();		//��ʼ����ʱ��

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




