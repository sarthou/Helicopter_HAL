/**
  ******************************************************************************
  * @file    Pwm1_Init.c
  * @author  MCD Application Team
  * @version V1.7.0
  * @date    16-December-2016
  * @brief
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#include "stm32f3xx_hal.h"
#include "PWM/Pwm_driver.h"

/*
 * Main motor : PA_8
 * Tail motor : PA_11
 */

#define MOTOR_PWM_PERIOD		(int)(100)

#define MOTOR_MAIN_PIN 			GPIO_PIN_8
#define MOTOR_MAIN_PORT 		GPIOA
#define MOTOR_MAIN_CLK_PORT		__HAL_RCC_GPIOA_CLK_ENABLE
#define MOTOR_MAIN_TIMx			TIM1
#define MOTOR_MAIN_PWM_CHANNEL	TIM_CHANNEL_1
#define MOTOR_MAIN_GPIO_AF		GPIO_AF6_TIM1
#define MOTOR_MAIN_CLK_TIMx		__HAL_RCC_TIM1_CLK_ENABLE

#define MOTOR_TAIL_PIN 			GPIO_PIN_11
#define MOTOR_TAIL_PORT 		GPIOA
#define MOTOR_TAIL_CLK_PORT		__HAL_RCC_GPIOA_CLK_ENABLE
#define MOTOR_TAIL_TIMx			TIM1
#define MOTOR_TAIL_PWM_CHANNEL	TIM_CHANNEL_4
#define MOTOR_TAIL_GPIO_AF		GPIO_AF11_TIM1
#define MOTOR_TAIL_CLK_TIMx		__HAL_RCC_TIM1_CLK_ENABLE

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim)
{
	GPIO_InitTypeDef   GPIO_InitStruct;

	// Enable main motor PWM
	MOTOR_MAIN_CLK_TIMx();
	MOTOR_MAIN_CLK_PORT();

	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Alternate = MOTOR_MAIN_GPIO_AF;
	GPIO_InitStruct.Pin = MOTOR_MAIN_PIN;
	HAL_GPIO_Init(MOTOR_MAIN_PORT, &GPIO_InitStruct);

	// Enable tail motor PWM
	MOTOR_TAIL_CLK_TIMx();
	MOTOR_TAIL_CLK_PORT();

	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;

	GPIO_InitStruct.Alternate = MOTOR_TAIL_GPIO_AF;
	GPIO_InitStruct.Pin = MOTOR_TAIL_PIN;
	HAL_GPIO_Init(MOTOR_TAIL_PORT, &GPIO_InitStruct);
}

void MainMotorPWM_init(DRV_PWM_TypeDef* pwm)
{
	DRV_PWM_init(pwm, MOTOR_MAIN_TIMx, MOTOR_MAIN_PWM_CHANNEL);
	DRV_PWM_setPeriod(pwm, 100);
	DRV_PWM_setDutyCycle(pwm, 100.f);
}

void TailMotorPWM_init(DRV_PWM_TypeDef* pwm)
{
	DRV_PWM_init(pwm, MOTOR_TAIL_TIMx, MOTOR_TAIL_PWM_CHANNEL);
	DRV_PWM_setPeriod(pwm, 100);
	DRV_PWM_setDutyCycle(pwm, 100.f);
}
