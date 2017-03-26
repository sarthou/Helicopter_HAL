/*
 * PwmDriver.h
 *
 *  Created on: 5 mars 2017
 *      Author: JulienCombattelli
 */

#ifndef PWMDRIVER_H_
#define PWMDRIVER_H_

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C" {
#endif // __cplusplus

#define TIM_COUNTER_CLOCK_MHZ (uint32_t)(32000000)

typedef struct pwm_s
{
	TIM_HandleTypeDef htim;
	uint32_t channel;
	uint32_t period_tick;
	float dutyCycle;

} pwm_t;

void DRV_PWM_init(pwm_t* pwm, TIM_TypeDef* tim, uint32_t channel);
void DRV_PWM_setPeriod(pwm_t* pwm, uint32_t period_us);
void DRV_PWM_setDutyCycle(pwm_t* pwm, float dutyCycle);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif /* PWMDRIVER_H_ */
