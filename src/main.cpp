/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "ADC/Adc_init.h"
#include "UART/Uart_init.h"
#include "PWM/Pwm_init.h"

#include "ADC/Adc_driver.h"
#include "UART/Uart_driver.h"
#include "PWM/Pwm_driver.h"

#include "Error_handler/Error_handler.h"

#include "Helicopter/Helicopter.h"

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
	  Error_Handler();

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
	  Error_Handler();
}

/*int main()
{
	HAL_Init();
	SystemClock_Config();

	ADC_HandleTypeDef handler_pot1;
	ADC_HandleTypeDef handler_pot2;
	ADC_HandleTypeDef handler_ain1;
	ADC_HandleTypeDef handler_ain2;

	UART_HandleTypeDef uart;

	pwm_t motorMain;
	pwm_t motorTail;

	POT1_init(&handler_pot1);
	POT2_init(&handler_pot2);
	AIN2_init(&handler_ain2);
	AIN1_init(&handler_ain1);

	USB_UART_init(&uart);

	MainMotorPWM_init(&motorMain);
	TailMotorPWM_init(&motorTail);

	DRV_PWM_setPeriod(&motorMain, 10);
	DRV_PWM_setDutyCycle(&motorMain, 0.2);

	DRV_PWM_setPeriod(&motorTail, 1);
	DRV_PWM_setDutyCycle(&motorTail, 0.5);

	DRV_UART_printf(uart,"hello word\n\r");

	while(1)
	{
		uint32_t value;
		value = DRV_ADC_getValue(&handler_pot1);
		DRV_UART_printf(uart,"pot1 : %d ", value);
		value = DRV_ADC_getValue(&handler_pot2);
		DRV_UART_printf(uart,"pot2 : %d ", value);
		value = DRV_ADC_getValue(&handler_ain1);
		DRV_UART_printf(uart,"ain1 : %d ", value);
		value = DRV_ADC_getValue(&handler_ain2);
		DRV_UART_printf(uart,"ain2 : %d\n\r", value);
	}

	return 0;
}*/

int main(void)
{
	HAL_Init();
	SystemClock_Config();

	Helicopter helico;
	helico.run();

	return 0;
}
