#include "Adc_init.h"
#include "Adc_driver.h"

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

void POT1_init(ADC_HandleTypeDef* AdcHandle)
{
	DRV_ADC_init(AdcHandle, POT1_ADC, POT1_CHANNEL);
}

void POT2_init(ADC_HandleTypeDef* AdcHandle)
{
	DRV_ADC_init(AdcHandle, POT2_ADC, POT2_CHANNEL);
}

void AIN1_init(ADC_HandleTypeDef* AdcHandle)
{
	DRV_ADC_init(AdcHandle, AIN1_ADC, AIN1_CHANNEL);
}

void AIN2_init(ADC_HandleTypeDef* AdcHandle)
{
	DRV_ADC_init(AdcHandle, AIN2_ADC, AIN2_CHANNEL);
}
