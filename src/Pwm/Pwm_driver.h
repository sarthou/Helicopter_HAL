/*
 * PwmDriver.h
 *
 *  Created on: 26 mars 2017
 *      Author: JulienCombattelli
 */

#ifndef PWMDRIVER_H_
#define PWMDRIVER_H_

#include <stm32f3xx_hal.h>
#include <stdint.h>

typedef struct
{
	TIM_TypeDef* pwm;
    uint32_t prescaler;
    uint32_t period;
    uint32_t pulse;
    uint8_t channel;
    uint8_t inverted;

} DRV_PWM_TypeDef;

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

void DRV_PWM_init(DRV_PWM_TypeDef* obj, TIM_TypeDef* tim, uint32_t channel);
void DRV_PWM_free(DRV_PWM_TypeDef* obj);
void DRV_PWM_setDutyCycle(DRV_PWM_TypeDef* obj, float value);
float DRV_PWM_getDutyCycle(DRV_PWM_TypeDef* obj);
void DRV_PWM_setPeriod(DRV_PWM_TypeDef* obj, int us);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* PWMDRIVER_H_ */
