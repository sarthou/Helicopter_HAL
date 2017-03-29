/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include <stm32f3xx.h>
#include <Error_handler/Error_handler.h>
#include <Pwm/Pwm_driver.h>
#include <Pwm/Pwm_init.h>
#include <Uart/Uart_driver.h>
#include <Uart/Uart_init.h>
#include <Helicopter/Helicopter.h>
			
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  // HSI Oscillator already ON after system reset, activate PLL with HSI as source
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_NONE;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct)!= HAL_OK)
	  Error_Handler();

  // Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  //   clocks dividers
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2)!= HAL_OK)
	  Error_Handler();
}

int main(void)
{
	SystemClock_Config();

	Helicopter helico;
	helico.run();

	for(;;);
}

/*int main(void)
{
	SystemClock_Config();

	DRV_PWM_TypeDef pwm1, pwm2;
	DRV_UART_TypeDef uart;

	DRV_UART_init(&uart, USART2);
	int baud = 9600;
	DRV_UART_baud(&uart, 9600);
	DRV_UART_printf(&uart, "\r\n\r\ntest ctor\r\nbaud = %d\r\n", baud);

	MainMotorPWM_init(&pwm1);
	DRV_PWM_setDutyCycle(&pwm1, 0.7);

	TailMotorPWM_init(&pwm2);
	DRV_PWM_setDutyCycle(&pwm2, 0.1);*/

	/*for(;;)
	{
		if(DRV_UART_readable(&uart))
		{
			int c1 = DRV_UART_getc(&uart);
			int c2 = DRV_UART_getc(&uart);
			int c3 = c1+c2;
			DRV_UART_putc(&uart, c3/2);
		}
	}
}*/
