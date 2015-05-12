
#include <iostream>
#include <pigpio.h>
#include <string.h>

#define PIN_HEAD_H (05)
#define PIN_HEAD_V (06)

#define PWM_FREQ (50)
#define PWM_RANGE (10000)

#define PWM_DC_MIN (0.05)
#define PWM_DC_MAX (0.1)

#define SERVO_MAX_ANGLE (120)

#define ABS(x) ((x)>0?(x):-(x))

using namespace std;

typedef struct _SERVO
{
	//gpio
	int gpioPWM;

	//control
	float Angle;
	uint32_t dutyCycle;
	
}SERVO, *pSERVO;

SERVO servoH, servoV;

//MOTOR motorL = {PIN_MOTOR_L_ENA, PIN_MOTOR_L_DIR, PIN_MOTOR_L_PWM, 0, 0, 0, 0};
//MOTOR motorR = {PIN_MOTOR_R_ENA, PIN_MOTOR_R_DIR, PIN_MOTOR_R_PWM, 0, 0, 0, 0};

void pwmInit(pSERVO p)
{
	gpioSetMode(p->gpioPWM, PI_OUTPUT);
	gpioSetPWMrange(p->gpioPWM, PWM_RANGE);
	gpioSetPWMfrequency(p->gpioPWM, PWM_FREQ);
	gpioHardwarePWM(p->gpioPWM, 0, 0);
}

void servoInit()
{
	memset(&servoH, 0, sizeof(SERVO));
	memset(&servoV, 0, sizeof(SERVO));

	servoH.gpioPWM = PIN_HEAD_H;
	servoV.gpioPWM = PIN_HEAD_V;

	servoH.dutyCycle = PWM_RANGE * PWM_DC_MIN;
	servoV.dutyCycle = PWM_RANGE * PWM_DC_MIN;

	gpioInitialise();	//Initialise pigpio

	pwmInit(&servoH);
	pwmInit(&servoV);
}

void servoSetAngle(pSERVO p, float angle)
{
	if(angle < 0) { angle = 0; }
	if(angle > SERVO_MAX_ANGLE) { angle = SERVO_MAX_ANGLE; }
	
	cout<<"angle:"<<angle<<" ";
	
	p->dutyCycle = ((float)(angle/SERVO_MAX_ANGLE)+1) * PWM_DC_MIN * PWM_RANGE;		/*Angle to duty cycle. control signal is PPM*/
	
	cout<<"GPIO"<<p->gpioPWM<<" PWM duty cycle:"<<p->dutyCycle<<endl;
	
	gpioPWM(p->gpioPWM, p->dutyCycle);
}

void servoEnable(pSERVO p)
{
	servoSetAngle(p, 0);
}

void servoDisable(pSERVO p)
{
	gpioPWM(p->gpioPWM, 0);
}

void servoTerminate()
{
	gpioPWM(servoH.gpioPWM, 0);
	gpioPWM(servoV.gpioPWM, 0);
	gpioTerminate();
}

int main()
{
	float angle;
	
	servoInit();
	
	cout<<"Input angle of H servo:";
	cin>>angle;

	servoEnable(&servoH);
	servoSetAngle(&servoH, angle);

	cout<<"Input angle of V servo:";
	cin>>angle;

	servoEnable(&servoV);
	servoSetAngle(&servoV, angle);

	cout<<"Input any number to terminate program:";
	cin>>angle;

	servoTerminate();
	
	return 0;
}