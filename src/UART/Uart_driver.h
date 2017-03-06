#ifndef UART_DRIVER_H
#define UART_DRIVER_H

#ifdef __cplusplus
extern "C"{
#endif

#include "stm32f3xx.h"
#include "stm32f3xx_nucleo_32.h"

void UART_putchar(UART_HandleTypeDef UartHandle, uint8_t str);
void UART_transmit(UART_HandleTypeDef UartHandle, uint8_t str[]);
uint8_t UART_getchar(UART_HandleTypeDef UartHandle);
void UART_receive(UART_HandleTypeDef UartHandle, uint8_t str[]);

int UART_printf(UART_HandleTypeDef UartHandle, const char *format, ...);
int UART_sprintf(UART_HandleTypeDef UartHandle, char *out, const char *format, ...);

/*
	UART_init(&uart);

	char *ptr = "Hello world!";
	char *np = 0;
	int i = 5;
	unsigned int bs = sizeof(int)*8;
	int mi;
	char buf[80];

	mi = (1 << (bs-1)) + 1;
	UART_printf(uart,"%s\n\r", ptr);
	UART_printf(uart,"printf test\n\r");
	UART_printf(uart,"%s is null pointer\n\r", np);
	UART_printf(uart,"%d = 5\n\r", i);
	UART_printf(uart,"%d = - max int\n\r", mi);
	UART_printf(uart,"char %c = 'a'\n\r", 'a');
	UART_printf(uart,"hex %x = ff\n\r", 0xff);
	UART_printf(uart,"hex %02x = 00\n\r", 0);
	UART_printf(uart,"signed %d = unsigned %u = hex %x\n\r", -3, -3, -3);
	UART_printf(uart,"%d %s(s)%", 0, "message");
	UART_printf(uart,"\n\r");
	UART_printf(uart,"%d %s(s) with %%\n\r", 0, "message");
	UART_sprintf(uart,buf, "justif: \"%-10s\"\n\r", "left"); UART_printf(uart,"%s", buf);
	UART_sprintf(uart,buf, "justif: \"%10s\"\n\r", "right"); UART_printf(uart,"%s", buf);
	UART_sprintf(uart,buf, " 3: %04d zero padded\n\r", 3); UART_printf(uart,"%s", buf);
	UART_sprintf(uart,buf, " 3: %-4d left justif.\n\r", 3); UART_printf(uart,"%s", buf);
	UART_sprintf(uart,buf, " 3: %4d right justif.\n\r", 3); UART_printf(uart,"%s", buf);
	UART_sprintf(uart,buf, "-3: %04d zero padded\n\r", -3); UART_printf(uart,"%s", buf);
	UART_sprintf(uart,buf, "-3: %-4d left justif.\n\r", -3); UART_printf(uart,"%s", buf);
	UART_sprintf(uart,buf, "-3: %4d right justif.\n\r", -3); UART_printf(uart,"%s", buf);
*/

#ifdef __cplusplus
}
#endif

#endif
