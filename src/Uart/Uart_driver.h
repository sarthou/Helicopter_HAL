/*
 * UartDriver.h
 *
 *  Created on: 26 mars 2017
 *      Author: JulienCombattelli
 */

#ifndef UARTDRIVER_H_
#define UARTDRIVER_H_

#include <stm32f3xx_hal.h>
#include <stdint.h>

#define DRV_UART_USE_PRINTF

typedef enum
{
	ParityNone = 0,
	ParityOdd = 1,
	ParityEven = 2,
	ParityForced1 = 3,
	ParityForced0 = 4
} DRV_UART_parity;

/*typedef enum
{
	RxIrq,
	TxIrq
} UartIrq;

typedef void (*DRV_UART_irqHandler)(uint32_t id, SerialIrq event);*/

typedef struct
{
	USART_TypeDef* uart;
	int index;
	uint32_t baudrate;
	uint32_t databits;
	uint32_t stopbits;
	uint32_t parity;
} DRV_UART_TypeDef;

#ifdef __cplusplus
extern "C" {
#endif //__cplusplus

void DRV_UART_init(DRV_UART_TypeDef *obj, USART_TypeDef* uart);
void DRV_UART_free(DRV_UART_TypeDef *obj);
void DRV_UART_baud(DRV_UART_TypeDef *obj, int baudrate);
void DRV_UART_format(DRV_UART_TypeDef *obj, int data_bits, DRV_UART_parity parity, int stop_bits);
//void DRV_UART_irq_handler(DRV_UART_TypeDef *obj, DRV_UART_irqHandler handler, uint32_t id);
//void DRV_UART_irq_set(DRV_UART_TypeDef *obj, SerialIrq irq, uint32_t enable);
uint8_t DRV_UART_getc(DRV_UART_TypeDef *obj);
void DRV_UART_read(DRV_UART_TypeDef *obj, uint8_t *buffer, size_t nbytes);
void DRV_UART_putc(DRV_UART_TypeDef *obj, int c);
void DRV_UART_puts(DRV_UART_TypeDef *obj, const char* str);
void DRV_UART_write(DRV_UART_TypeDef *obj, uint8_t *buffer, size_t nbytes);
void DRV_UART_printf(DRV_UART_TypeDef *obj, const char* str, ...);
int DRV_UART_readable(DRV_UART_TypeDef *obj);
int DRV_UART_writable(DRV_UART_TypeDef *obj);
void DRV_UART_clear(DRV_UART_TypeDef *obj);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* UARTDRIVER_H_ */
