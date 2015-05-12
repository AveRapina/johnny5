
#include <iostream>
#include <pigpio.h>
#include <string.h>
#include <unistd.h>
#include "joystick.h"

#define PIN_HEAD_H (05)
#define PIN_HEAD_V (06)

#define PWM_FREQ (50)
#define PWM_RANGE (10000)

#define PWM_DC_MIN (0.05)
#define PWM_DC_MAX (0.1)

#define SERVO_MAX_ANGLE (120.0)

#define JOYSTICK_AXIS_MAX (0x7FFF)
#define JOYSTICK_AXIS_MIN (-0x7FFF)

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

float axisToAngle(int value)
{
	if(value < 0)
	{
		return (1-(float)((float)(abs(value))/JOYSTICK_AXIS_MAX))*(SERVO_MAX_ANGLE/2);
	}
	else
	{
		return (1+(float)((float)value/JOYSTICK_AXIS_MAX))*(SERVO_MAX_ANGLE/2);
	}
}

int main()
{
	#if 0
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
#endif

  // Create an instance of Joystick
  Joystick joystick("/dev/input/js2");

	servoInit();

  // Ensure that it was found and that we can use it
  if (!joystick.isFound())
  {
    printf("open failed.\n");
    exit(1);
  }

  while (true)
  {
		float angle = 0;
    // Restrict rate
    usleep(1000);

    // Attempt to sample an event from the joystick
    JoystickEvent event;
    if (joystick.sample(&event))
    {
      if (event.isButton())
      {
        printf("Button %u is %s\n",
          event.number,
          event.value == 0 ? "up" : "down");
      }
      else if (event.isAxis())
      {
        printf("Axis %u is at position %d\n", event.number, event.value);
				switch(event.number)
				{
					case 3:		//×óÓÒ
						angle = axisToAngle(event.value);
						printf("angle %f\r\n", angle);
						servoSetAngle(&servoH, axisToAngle(event.value));
					break;
					case 2:		//ÉÏÏÂ
						angle = axisToAngle(event.value);
						printf("angle %f\r\n", angle);
						servoSetAngle(&servoV, axisToAngle(event.value));
					break;
				}
      }
    }
  }
	servoTerminate();
	return 0;
}