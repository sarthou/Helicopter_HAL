/*
 * PwmDriver.c
 *
 *  Created on: 26 mars 2017
 *      Author: JulienCombattelli
 */

#include <Pwm/Pwm_driver.h>
#include <Error_handler/Error_handler.h>

static TIM_HandleTypeDef TimHandle;

void DRV_PWM_init(DRV_PWM_TypeDef* obj, TIM_TypeDef* tim, uint32_t channel)
{
	obj->pwm = tim;
    obj->channel = channel;
    obj->period = 0;
    obj->pulse = 0;
    obj->prescaler = 1;
    obj->inverted = 0;

    DRV_PWM_setPeriod(obj, 20000); // 20 ms per default
}

void DRV_PWM_free(DRV_PWM_TypeDef* obj)
{

}

void DRV_PWM_setDutyCycle(DRV_PWM_TypeDef* obj, float value)
{
    TIM_OC_InitTypeDef sConfig;

    TimHandle.Instance = (TIM_TypeDef *)(obj->pwm);

    if (value < (float)0.0) {
        value = 0.0;
    } else if (value > (float)1.0) {
        value = 1.0;
    }

    obj->pulse = (uint32_t)((float)obj->period * value);

    // Configure channels
    sConfig.OCMode       = TIM_OCMODE_PWM1;
    sConfig.Pulse        = obj->pulse / obj->prescaler;
    sConfig.OCPolarity   = TIM_OCPOLARITY_HIGH;
    sConfig.OCNPolarity  = TIM_OCNPOLARITY_HIGH;
    sConfig.OCFastMode   = TIM_OCFAST_DISABLE;
    sConfig.OCIdleState  = TIM_OCIDLESTATE_RESET;
    sConfig.OCNIdleState = TIM_OCNIDLESTATE_RESET;

    if (HAL_TIM_PWM_ConfigChannel(&TimHandle, &sConfig, obj->channel) != HAL_OK) {
        Error_Handler(); //("Cannot initialize PWM");
    }

    if (obj->inverted) {
        HAL_TIMEx_PWMN_Start(&TimHandle, obj->channel);
    } else {
        HAL_TIM_PWM_Start(&TimHandle, obj->channel);
    }
}

float DRV_PWM_getDutyCycle(DRV_PWM_TypeDef* obj)
{
    float value = 0;
    if (obj->period > 0) {
        value = (float)(obj->pulse) / (float)(obj->period);
    }
    return ((value > (float)1.0) ? (float)(1.0) : (value));
}

void DRV_PWM_setPeriod(DRV_PWM_TypeDef* obj, int us)
{
    TimHandle.Instance = (TIM_TypeDef *)(obj->pwm);

    float dc = DRV_PWM_getDutyCycle(obj);

    __HAL_TIM_DISABLE(&TimHandle);

    // Update the SystemCoreClock variable
    SystemCoreClockUpdate();

    /* To make it simple, we use to possible prescaler values which lead to:
     * pwm unit = 1us, period/pulse can be from 1us to 65535us
     * or
     * pwm unit = 500us, period/pulse can be from 500us to ~32.76sec
     * Be careful that all the channels of a PWM shares the same prescaler
     */
    if (us >  0xFFFF) {
        obj->prescaler = 500;
    } else {
        obj->prescaler = 1;
    }
    TimHandle.Init.Prescaler     = ((SystemCoreClock / 1000000) * obj->prescaler) - 1;

    if (TimHandle.Init.Prescaler > 0xFFFF)
    	Error_Handler(); //("PWM: out of range prescaler");

    TimHandle.Init.Period        = (us - 1) / obj->prescaler;
    if (TimHandle.Init.Period > 0xFFFF)
    	Error_Handler(); //("PWM: out of range period");

    TimHandle.Init.ClockDivision = 0;
    TimHandle.Init.CounterMode   = TIM_COUNTERMODE_UP;

    if (HAL_TIM_PWM_Init(&TimHandle) != HAL_OK) {
    	Error_Handler(); //("Cannot initialize PWM");
    }

    // Save for future use
    obj->period = us;

    // Set duty cycle again
    DRV_PWM_setDutyCycle(obj, dc);

    __HAL_TIM_ENABLE(&TimHandle);
}

