#include "Adc_driver.h"

uint32_t ADC_get_value(ADC_HandleTypeDef* AdcHandle)
{
	HAL_ADC_Start(AdcHandle);
	HAL_ADC_PollForConversion(AdcHandle, 1);
	return HAL_ADC_GetValue(AdcHandle);
}
