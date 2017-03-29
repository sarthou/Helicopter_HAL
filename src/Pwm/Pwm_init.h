/*
 * Pwm_init.h
 *
 *  Created on: 6 mars 2017
 *      Author: JulienCombattelli
 */

#ifndef PWM_PWM_INIT_H_
#define PWM_PWM_INIT_H_

#include <Pwm/Pwm_driver.h>

#ifdef __cplusplus
extern "C"{
#endif

void MainMotorPWM_init(DRV_PWM_TypeDef* pwm);

void TailMotorPWM_init(DRV_PWM_TypeDef* pwm);

#ifdef __cplusplus
}
#endif

#endif /* PWM_PWM_INIT_H_ */
