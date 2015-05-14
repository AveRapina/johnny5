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

    imu6Init();			//��ʼ��������
	imu6Test();			//�������Լ�

    initPidControl();   //��ʼ��PID

    float a;
    cin >> a;

    motorEnable(&motorL);	//����ʹ��
    motorEnable(&motorR);	//�ҵ��ʹ��

    timerInit();		//��ʼ����ʱ��
#if 1
    balanceControl.pidPitch.Kp = 1500;
    balanceControl.pidPitch.Ki = 1000;
    balanceControl.Kd = 5;
    balanceControl.pidPitch.desired = 5;
#endif
    while(a != 200)
    {
    	cout<<endl;
    	cin >> a;
    	//balanceControl.pidPitch.Kp = a;
    	pidSetKp(&balanceControl.pidPitch, a);
    	//pidSetKp(&balanceControl.pidSpeed, a);

        cin >> a;
        //balanceControl.pidPitch.Ki = a;
        pidSetKi(&balanceControl.pidPitch, a);
        //pidSetKi(&balanceControl.pidSpeed, a);

        cin >> a;
        //balanceControl.pidPitch.Kd = a;
        balanceControl.Kd = a;
        //pidSetKd(&balanceControl.pidPitch, a);
        //pidSetKd(&balanceControl.pidSpeed, a);

    	cin >> a;
    	angleOffset = a;
        balanceControl.pidPitch.desired = a;

        pidReset(&balanceControl.pidPitch);
        pidReset(&balanceControl.pidSpeed);
    }
    motorTerminate();
    //gpioTerminate();

    return 0;
}




