#ifndef UART_INIT_H
#define UART_INIT_H

#include <stm32f3xx_hal.h>

#ifdef __cplusplus
extern "C"{
#endif // __cplusplus

void USB_UART_init(DRV_UART_TypeDef* uart);

void BLE_UART_init(DRV_UART_TypeDef* uart);

#ifdef __cplusplus
}
#endif // __cplusplus

#endif
