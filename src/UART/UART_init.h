#ifndef UART_INIT_H
#define UART_INIT_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f3xx.h"
#include "stm32f3xx_nucleo_32.h"

void HAL_UART_MspInit(UART_HandleTypeDef *huart);

void UART_init(UART_HandleTypeDef* UartHandle);


#ifdef __cplusplus
}
#endif

#endif
