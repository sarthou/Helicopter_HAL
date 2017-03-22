#ifndef UART_INIT_H
#define UART_INIT_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C"{
#endif

#define SD_CS_PIN                                 GPIO_PIN_4
#define SD_CS_GPIO_PORT                           GPIOA
#define SD_SPI                               	  SPI1
#define SD_FREQUENCY							  (16000000u)

#ifdef __cplusplus
}
#endif

#endif
