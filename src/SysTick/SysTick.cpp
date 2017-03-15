/*
 * SysTick.c
 *
 *  Created on: 8 mars 2017
 *      Author: JulienCombattelli
 */

#include "stm32f3xx_hal.h"
#include "Helicopter/Helicopter.h"

static Helicopter* instance = 0;

void SysTick_setInstance(Helicopter* helicopter)
{
	instance = helicopter;
}

void HAL_SYSTICK_Callback(void)
{
	instance->process();
}
