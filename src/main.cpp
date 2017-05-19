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
#include <Uart/Uart_driver.h>
#include <Uart/Uart_init.h>
#include <Helicopter/Helicopter.h>

#include <I2C/I2c_driver.h>
#include <I2C/I2c_init.h>
			
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
//I2C_HandleTypeDef i2c;
int main(void)
{
	SystemClock_Config();
	HAL_Init();

	Helicopter helico;
	helico.run();


	/*MPU9250_I2C_init(&i2c);

	while(1)
	{
		DRV_I2C_read_byte(&i2c, 0x75);
	}*/

	for(;;);
}
