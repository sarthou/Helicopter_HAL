#include "ADC_init.h"

/*POT1*/
#define POT1_CHANNEL_PIN		GPIO_PIN_7
#define POT1_CHANNEL_GPIO_PORT	GPIOA
#define POT1_ADC                ADC2
#define POT1_CHANNEL            ADC_CHANNEL_4

/*POT2*/
#define POT2_CHANNEL_PIN		GPIO_PIN_0
#define POT2_CHANNEL_GPIO_PORT	GPIOB
#define POT2_ADC                ADC1
#define POT2_CHANNEL            ADC_CHANNEL_11

/*AIN1*/
#define AIN1_CHANNEL_PIN		GPIO_PIN_0
#define AIN1_CHANNEL_GPIO_PORT	GPIOA
#define AIN1_ADC                ADC1
#define AIN1_CHANNEL            ADC_CHANNEL_1

/*AIN2*/
#define AIN2_CHANNEL_PIN		GPIO_PIN_1
#define AIN2_CHANNEL_GPIO_PORT	GPIOA
#define AIN2_ADC                ADC1
#define AIN2_CHANNEL            ADC_CHANNEL_2

void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc)
{
	GPIO_InitTypeDef        GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_ADC12_CLK_ENABLE();

	//POT1
	GPIO_InitStruct.Pin = POT1_CHANNEL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(POT1_CHANNEL_GPIO_PORT, &GPIO_InitStruct);

	//POT2
	GPIO_InitStruct.Pin = POT2_CHANNEL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(POT2_CHANNEL_GPIO_PORT, &GPIO_InitStruct);

	//AIN1
	GPIO_InitStruct.Pin = AIN1_CHANNEL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(AIN1_CHANNEL_GPIO_PORT, &GPIO_InitStruct);

	//AIN2
	GPIO_InitStruct.Pin = AIN2_CHANNEL_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(AIN2_CHANNEL_GPIO_PORT, &GPIO_InitStruct);
}

void init_adc(ADC_HandleTypeDef* AdcHandle, ADC_TypeDef* instance, uint32_t channel)
{
	AdcHandle->Instance          = instance;
	if (HAL_ADC_DeInit(AdcHandle) != HAL_OK)
		while(1);//Error_Handler();

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
		while(1);//Error_Handler();

	if (HAL_ADCEx_Calibration_Start(AdcHandle, ADC_SINGLE_ENDED) !=  HAL_OK)
		while(1);//Error_Handler();

	ADC_ChannelConfTypeDef  sConfig;
	sConfig.Channel      = channel;
	sConfig.Rank         = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_61CYCLES_5;
	sConfig.SingleDiff   = ADC_SINGLE_ENDED;
	sConfig.OffsetNumber = ADC_OFFSET_NONE;
	if (HAL_ADC_ConfigChannel(AdcHandle, &sConfig) != HAL_OK)
		while(1);//Error_Handler();
}

void POT1_init(ADC_HandleTypeDef* AdcHandle)
{
	init_adc(AdcHandle, POT1_ADC, POT1_CHANNEL);
}

void POT2_init(ADC_HandleTypeDef* AdcHandle)
{
	init_adc(AdcHandle, POT2_ADC, POT2_CHANNEL);
}

void AIN1_init(ADC_HandleTypeDef* AdcHandle)
{
	init_adc(AdcHandle, AIN1_ADC, AIN1_CHANNEL);
}

void AIN2_init(ADC_HandleTypeDef* AdcHandle)
{
	init_adc(AdcHandle, AIN2_ADC, AIN2_CHANNEL);
}
