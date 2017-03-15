/*
 * PwmDriver.cpp
 *
 *  Created on: 5 mars 2017
 *      Author: JulienCombattelli
 */

#include "Error_handler/Error_handler.h"
#include "PWM/Pwm_driver.h"

void DRV_PWM_init(pwm_t* pwm, TIM_TypeDef* tim, uint32_t channel)
{
	pwm->htim.Instance = tim;
	pwm->channel = channel;
	pwm->period_tick = 0;
	pwm->dutyCycle = 0;

	if(HAL_TIM_PWM_DeInit(&pwm->htim) != HAL_OK)
		Error_Handler();

	DRV_PWM_setPeriod(pwm, 0);
	DRV_PWM_setDutyCycle(pwm, 0);
}

void DRV_PWM_setPeriod(pwm_t* pwm, uint32_t period_us)
{
	__HAL_TIM_DISABLE(&pwm->htim);

	uint32_t prescaler = (HAL_RCC_GetSysClockFreq() / TIM_COUNTER_CLOCK_MHZ) - 1;
	pwm->period_tick = (TIM_COUNTER_CLOCK_MHZ / (1000000/period_us)) - 1;

	pwm->htim.Init.Prescaler         = prescaler;
	pwm->htim.Init.Period            = pwm->period_tick;
	pwm->htim.Init.ClockDivision     = 0;
	pwm->htim.Init.CounterMode       = TIM_COUNTERMODE_UP;
	pwm->htim.Init.RepetitionCounter = 0;
	if(HAL_TIM_PWM_Init(&pwm->htim) != HAL_OK)
		Error_Handler();

	__HAL_TIM_ENABLE(&pwm->htim);
}

void DRV_PWM_setDutyCycle(pwm_t* pwm, float dutyCycle)
{
	TIM_OC_InitTypeDef sConfig;

	sConfig.OCMode       = TIM_OCMODE_PWM1;
	sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
	sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
	sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
	sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
	sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

	sConfig.Pulse = (uint32_t)((pwm->period_tick + 1) * dutyCycle);
	if(HAL_TIM_PWM_ConfigChannel(&pwm->htim, &sConfig, pwm->channel) != HAL_OK)
		Error_Handler();

	if(HAL_TIM_PWM_Start(&pwm->htim, pwm->channel) != HAL_OK)
		Error_Handler();

	pwm->dutyCycle = dutyCycle;
}


