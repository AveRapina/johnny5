/*
 * timer.h
 *
 *  Created on: 2015��5��6��
 *      Author: Winder
 */

#ifndef INCLUDE_TIMER_H_
#define INCLUDE_TIMER_H_

#define PIN_SIG (18)

extern int timerInit();
extern void imuUpdate(union sigval v);

#endif /* INCLUDE_TIMER_H_ */
