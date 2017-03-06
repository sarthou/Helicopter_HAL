#ifndef ADC_H
#define ADC_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C"{
#endif

void DRV_ADC_init(ADC_HandleTypeDef* AdcHandle, ADC_TypeDef* instance, uint32_t channel);

uint32_t DRV_ADC_getValue(ADC_HandleTypeDef* AdcHandle);

#ifdef __cplusplus
}
#endif

#endif
