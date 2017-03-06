#ifndef ADC_INIT_H
#define ADC_INIT_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f3xx.h"
#include "stm32f3xx_nucleo_32.h"

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc);

void POT1_init(ADC_HandleTypeDef* AdcHandle);
void POT2_init(ADC_HandleTypeDef* AdcHandle);
void AIN1_init(ADC_HandleTypeDef* AdcHandle);
void AIN2_init(ADC_HandleTypeDef* AdcHandle);

#ifdef __cplusplus
}
#endif

#endif
