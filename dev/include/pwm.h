/*
 * pwm.h
 *
 *  Created on: 2015Äê4ÔÂ23ÈÕ
 *      Author: Winder
 */

#ifndef DEV_INCLUDE_PWM_H_
#define DEV_INCLUDE_PWM_H_


#define PIN_MOTOR_L_ENA (26)
#define PIN_MOTOR_L_DIR (19)
#define PIN_MOTOR_L_PWM (13)

#define PIN_MOTOR_R_ENA (20)
#define PIN_MOTOR_R_DIR (16)
#define PIN_MOTOR_R_PWM (12)

#define PWM_FREQ_L (10000)
#define PWM_FREQ_R (10000)
#define PWM_RANGE (1000000)
#define PWM_DUTY_CYCLE (0.25)
#define PWM_DUTY_CYCLE_SET (PWM_DUTY_CYCLE * PWM_RANGE)

#define MOTOR_POS	(0)
#define MOTOR_NEG	(1)

#define MOTOR_ENABLE	(1)
#define MOTOR_DISABLE	(0)

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

extern MOTOR motorL;
extern MOTOR motorR;

extern void motorInit();
extern void motorEnable(pMOTOR p);
extern void motorDisable(pMOTOR p);
extern void motorSetSpeed(pMOTOR p, int32_t speed);	//Set motor speed

extern void motorSetDirectionPostive(pMOTOR p);
extern void motorSetDirectionNegtive(pMOTOR p);
extern void motorSetDutyCycle(pMOTOR p, float dutyCycle);
extern void motorTerminate();

#endif /* DEV_INCLUDE_PWM_H_ */
