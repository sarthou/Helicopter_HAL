/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f3xx.h"
#include "stm32f3xx_nucleo_32.h"

#include "ADC/Adc_driver.h"
#include "ADC/ADC_init.h"

#include "UART/UART_init.h"
#include "UART/Uart_driver.h"

void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* HSI Oscillator already ON after system reset, activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1);
  }
}

int main()
{
	HAL_Init();
	SystemClock_Config();

	ADC_HandleTypeDef handler_pot1;
	ADC_HandleTypeDef handler_pot2;
	ADC_HandleTypeDef handler_ain1;
	ADC_HandleTypeDef handler_ain2;

	UART_HandleTypeDef uart;

	POT1_init(&handler_pot1);
	POT2_init(&handler_pot2);
	AIN2_init(&handler_ain2);
	AIN1_init(&handler_ain1);


	UART_init(&uart);

	UART_printf(uart,"hello word\n\r");

	while(1)
	{
		uint32_t value;
		value = ADC_get_value(&handler_pot1);
		UART_printf(uart,"pot1 : %d ", value);
		value = ADC_get_value(&handler_pot2);
		UART_printf(uart,"pot2 : %d ", value);
		value = ADC_get_value(&handler_ain1);
		UART_printf(uart,"ain1 : %d ", value);
		value = ADC_get_value(&handler_ain2);
		UART_printf(uart,"ain2 : %d\n\r", value);
	}

	return 0;
}
