/**
  ******************************************************************************
  * @file    stm32f3xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include <stm32f3xx_hal.h>
#include <stm32f3xx.h>
#ifdef USE_RTOS_SYSTICK
#include <cmsis_os.h>
#endif
#include <stm32f3xx_it.h>
#include <Error_handler/Error_handler.h>

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

void NMI_Handler(void)
{
	Error_Handler();
}

void HardFault_Handler(void)
{
	Error_Handler();
}

void MemManage_Handler(void)
{
	Error_Handler();
}

void BusFault_Handler(void)
{
	Error_Handler();
}

void UsageFault_Handler(void)
{
	Error_Handler();
}

void SVC_Handler(void)
{
	Error_Handler();
}

void DebugMon_Handler(void)
{
	Error_Handler();
}

void PendSV_Handler(void)
{
	Error_Handler();
}

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}
