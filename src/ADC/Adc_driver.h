#ifndef ADC_H
#define ADC_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f3xx.h"
#include "stm32f3xx_nucleo_32.h"

uint32_t ADC_get_value(ADC_HandleTypeDef* AdcHandle);


#ifdef __cplusplus
}
#endif

#endif
