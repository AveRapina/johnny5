
#include <iostream>
#include <pigpio.h>
#include <string.h>

#define PIN_MOTOR_L_ENA (05)
#define PIN_MOTOR_L_DIR (06)
#define PIN_MOTOR_L_PWM (12)

#define PIN_MOTOR_R_ENA (19)
#define PIN_MOTOR_R_DIR (16)
#define PIN_MOTOR_R_PWM (13)

#define PWM_FREQ_L (10000)
#define PWM_FREQ_R (10000)
#define PWM_RANGE (1000000)
#define PWM_DUTY_CYCLE (0.25)
#define PWM_DUTY_CYCLE_SET (PWM_DUTY_CYCLE * PWM_RANGE)

#define MOTOR_POS	(0)
#define MOTOR_NEG	(1)

#define MOTOR_ENABLE	(1)
#define MOTOR_DISABLE	(0)

#define ABS(x) ((x)>0?(x):-(x))

using namespace std;

typedef struct _MOTOR
{
	//gpio
	int gpioENA;
	int gpioDIR;
	int gpioPWM;

	//control
	int enable;
	int direction;
	int speed;		//frequency of PWM
	int dutyCycle;	//duty cycle of PWM
}MOTOR, *pMOTOR;

MOTOR motorL;
MOTOR motorR;
//MOTOR motorL = {PIN_MOTOR_L_ENA, PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, 0, 0, 0, 0};
//MOTOR motorR = {PIN_MOTOR_R_ENA, PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, 0, 0, 0, 0};

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

	gpioInitialise();	//Initialise pigpio

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

void motorSetDirecton(int gpio, int direction)
{
	gpioWrite(gpio, direction);
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
