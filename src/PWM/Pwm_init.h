/*
 * Pwm_init.h
 *
 *  Created on: 6 mars 2017
 *      Author: JulienCombattelli
 */

#ifndef PWM_PWM_INIT_H_
#define PWM_PWM_INIT_H_

#include "PWM/Pwm_driver.h"

#ifdef __cplusplus
extern "C"{
#endif

void MainMotorPWM_init(pwm_t* pwm);

void TailMotorPWM_init(pwm_t* pwm);

#ifdef __cplusplus
}
#endif

#endif /* PWM_PWM_INIT_H_ */
