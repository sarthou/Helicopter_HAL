#include "Adc_driver.h"
#include "Error_handler/Error_handler.h"

void DRV_ADC_init(ADC_HandleTypeDef* AdcHandle, ADC_TypeDef* instance, uint32_t channel)
{
	AdcHandle->Instance          = instance;
	if (HAL_ADC_DeInit(AdcHandle) != HAL_OK)
		Error_Handler();

	AdcHandle->Init.ClockPrescaler        = ADC_CLOCK_SYNC_PCLK_DIV2;
	AdcHandle->Init.Resolution            = ADC_RESOLUTION_12B;
	AdcHandle->Init.DataAlign             = ADC_DATAALIGN_RIGHT;
	AdcHandle->Init.ScanConvMode          = DISABLE;
	AdcHandle->Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
	AdcHandle->Init.LowPowerAutoWait      = DISABLE;
	AdcHandle->Init.ContinuousConvMode    = ENABLE;
	AdcHandle->Init.ExternalTrigConv      = ADC_SOFTWARE_START;
	AdcHandle->Init.DMAContinuousRequests = DISABLE;
	AdcHandle->Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;

	if (HAL_ADC_Init(AdcHandle) != HAL_OK)
		Error_Handler();

	if (HAL_ADCEx_Calibration_Start(AdcHandle, ADC_SINGLE_ENDED) !=  HAL_OK)
		Error_Handler();

	ADC_ChannelConfTypeDef  sConfig;
	sConfig.Channel      = channel;
	sConfig.Rank         = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
	sConfig.SingleDiff   = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	if (HAL_ADC_ConfigChannel(AdcHandle, &sConfig) != HAL_OK)
		Error_Handler();
}

uint32_t DRV_ADC_getValue(ADC_HandleTypeDef* AdcHandle)
{
	HAL_ADC_Start(AdcHandle);
	HAL_ADC_PollForConversion(AdcHandle, 1);
	return HAL_ADC_GetValue(AdcHandle);
}
