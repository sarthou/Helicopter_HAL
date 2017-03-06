#ifndef UART_INIT_H
#define UART_INIT_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C"{
#endif

void USB_UART_init(UART_HandleTypeDef* UartHandle);

#ifdef __cplusplus
}
#endif

#endif
