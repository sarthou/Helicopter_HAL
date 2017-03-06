#ifndef ADC_INIT_H
#define ADC_INIT_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C"{
#endif

void POT1_init(ADC_HandleTypeDef* AdcHandle);
void POT2_init(ADC_HandleTypeDef* AdcHandle);
void AIN1_init(ADC_HandleTypeDef* AdcHandle);
void AIN2_init(ADC_HandleTypeDef* AdcHandle);

#ifdef __cplusplus
}
#endif

#endif
