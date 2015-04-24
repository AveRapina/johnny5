
#include "header.h"

using namespace std;

MOTOR motorL;
MOTOR motorR;

void pwmInit(pMOTOR p)
{
	gpioCfgClock(5,1,0);	//sample rate 5ms, clock source from PCM, reserved

	gpioSetMode(p->gpioENA, PI_OUTPUT);
	gpioSetMode(p->gpioDIR, PI_OUTPUT);
	gpioSetMode(p->gpioPWM, PI_OUTPUT);

	gpioWrite(p->gpioENA, 0);
	gpioWrite(p->gpioDIR, 0);
	gpioHardwarePWM(p->gpioPWM, 0, 0);
}

void motorInit()
{
	memset(&motorL, 0, sizeof(MOTOR));
	memset(&motorR, 0, sizeof(MOTOR));

	motorL.gpioENA = PIN_MOTOR_L_ENA;
	motorL.gpioDIR = PIN_MOTOR_L_DIR;
	motorL.gpioPWM = PIN_MOTOR_L_PWM;

	motorR.gpioENA = PIN_MOTOR_R_ENA;
	motorR.gpioDIR = PIN_MOTOR_R_DIR;
	motorR.gpioPWM = PIN_MOTOR_R_PWM;

	motorL.dutyCycle = PWM_DUTY_CYCLE_SET;
	motorR.dutyCycle = PWM_DUTY_CYCLE_SET;

	//gpioInitialise();	//Initialise pigpio

	pwmInit(&motorL);
	pwmInit(&motorR);
}

void motorEnable(pMOTOR p)
{
	gpioWrite(p->gpioENA, MOTOR_ENABLE);
}

void motorDisable(pMOTOR p)
{
	gpioWrite(p->gpioENA, MOTOR_DISABLE);
}

void motorSetDirectionPostive(pMOTOR p)
{
	gpioWrite(p->gpioDIR, MOTOR_POS);
}

void motorSetDirectionNegtive(pMOTOR p)
{
	gpioWrite(p->gpioDIR, MOTOR_NEG);
}

void motorSetDutyCycle(pMOTOR p, float dutyCycle)
{
	p->dutyCycle = PWM_RANGE * dutyCycle;
}

void motorSetSpeed(pMOTOR p, int32_t speed)
{
	if(speed > 0)
	{
		gpioWrite(p->gpioDIR, MOTOR_POS);
		cout<< p->gpioPWM<< "\t"<< speed << "\t" << PWM_RANGE * p->dutyCycle <<endl;
		gpioHardwarePWM(p->gpioPWM, speed, p->dutyCycle);
	}
	else
	{
		gpioWrite(p->gpioDIR, MOTOR_NEG);
		cout<< p->gpioPWM<< "\t"<< -speed << "\t" << PWM_RANGE * p->dutyCycle <<endl;
		gpioHardwarePWM(p->gpioPWM, -speed, p->dutyCycle);
	}
}

void motorSpeed(int gpio, int speed, float dc)
{
	gpioHardwarePWM(gpio, speed, PWM_RANGE*dc);
}

void motorTerminate()
{
	gpioHardwarePWM(motorL.gpioPWM, 0, 0);
	gpioHardwarePWM(motorR.gpioPWM, 0, 0);
	gpioTerminate();
}

#if 0
int main()
{
	int speed;
	
	motorInit();
	
	cout<<"Input speed of left motor:";
	cin>>speed;

	motorSetSpeed(&motorL, speed);

	cout<<"Input speed of right motor:";
	cin>>speed;

	motorSetSpeed(&motorR, speed);
	motorEnable(&motorR);

	cout<<"Input speed of right motor:";
	cin>>speed;

	motorSetSpeed(&motorR, speed);

#if 0
	cout<<"Input speed of left motor:";
	cin>>speed;

	motorSetSpeed(motorL.gpioPWM, speed, 0.25);

	cout<<"Input speed of right motor:";
	cin>>speed;

	motorSetSpeed(motorR.gpioPWM, speed, 0.25);
	motorEnable(motorR.gpioENA, 1);

	cout<<"Input any number to invert direction:";
	cin>>speed;

	motorSetDirecton(motorR.gpioDIR, 1);
#endif

	cout<<"Input any number to terminate program:";
	cin>>speed;

	motorTerminate();
	
	return 0;
}
#endif
