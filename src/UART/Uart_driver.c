#include "Uart_driver.h"
#include "Error_handler/Error_handler.h"

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

void DRV_UART_init(UART_HandleTypeDef* UartHandle, USART_TypeDef* instance, uint32_t baudrate)
{
	UartHandle->Instance          = instance;

	UartHandle->Init.BaudRate     = baudrate;
	UartHandle->Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle->Init.StopBits     = UART_STOPBITS_1;
	UartHandle->Init.Parity       = UART_PARITY_NONE;
	UartHandle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle->Init.Mode         = UART_MODE_TX_RX;
	UartHandle->Init.OverSampling = UART_OVERSAMPLING_16;
	UartHandle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if(HAL_UART_DeInit(UartHandle) != HAL_OK)
		Error_Handler();

	if(HAL_UART_Init(UartHandle) != HAL_OK)
		Error_Handler();
}

void DRV_UART_putchar(UART_HandleTypeDef* UartHandle, uint8_t str)
{
	while(HAL_UART_GetState(UartHandle) != HAL_UART_STATE_READY);
	HAL_UART_Transmit(UartHandle, &str, 1, 0xFFFF);
}

void DRV_UART_transmit(UART_HandleTypeDef* UartHandle, uint8_t str[])
{
	while(HAL_UART_GetState(UartHandle) != HAL_UART_STATE_READY);
	HAL_UART_Transmit(UartHandle, str, (strlen((char*)str) + 1 ) * sizeof(char), 0xFFFF);
}

uint8_t DRV_UART_readable(UART_HandleTypeDef* UartHandle)
{
	if(__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_RXNE) != RESET)
		return 1;
	else
		return 0;
}

uint8_t DRV_UART_getchar(UART_HandleTypeDef* UartHandle)
{
	uint8_t car = 0x00;
	if(__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_RXNE) != RESET)
		HAL_UART_Receive(UartHandle, &car, 1, 0xFFFF);

	return car;
}

void DRV_UART_receive(UART_HandleTypeDef* UartHandle, uint8_t str[])
{
	if(__HAL_UART_GET_FLAG(UartHandle, UART_FLAG_RXNE) != RESET)
		HAL_UART_Receive(UartHandle, str, 1, 0xFFFF);
}

static void printchar(UART_HandleTypeDef* UartHandle, char **str, int c)
{
	//extern int putchar(int c);

	if (str) {
		**str = c;
		++(*str);
	}
	else (void)DRV_UART_putchar(UartHandle, c);
}

#define PAD_RIGHT 1
#define PAD_ZERO 2

static int prints(UART_HandleTypeDef* UartHandle, char **out, const char *string, int width, int pad)
{
	register int pc = 0, padchar = ' ';

	if (width > 0) {
		register int len = 0;
		register const char *ptr;
		for (ptr = string; *ptr; ++ptr) ++len;
		if (len >= width) width = 0;
		else width -= len;
		if (pad & PAD_ZERO) padchar = '0';
	}
	if (!(pad & PAD_RIGHT)) {
		for ( ; width > 0; --width) {
			printchar (UartHandle, out, padchar);
			++pc;
		}
	}
	for ( ; *string ; ++string) {
		printchar (UartHandle, out, *string);
		++pc;
	}
	for ( ; width > 0; --width) {
		printchar (UartHandle, out, padchar);
		++pc;
	}

	return pc;
}

/* the following should be enough for 32 bit int */
#define PRINT_BUF_LEN 12

static int printi(UART_HandleTypeDef* UartHandle, char **out, int i, int b, int sg, int width, int pad, int letbase)
{
	char print_buf[PRINT_BUF_LEN];
	register char *s;
	register int t, neg = 0, pc = 0;
	register unsigned int u = i;

	if (i == 0) {
		print_buf[0] = '0';
		print_buf[1] = '\0';
		return prints (UartHandle, out, print_buf, width, pad);
	}

	if (sg && b == 10 && i < 0) {
		neg = 1;
		u = -i;
	}

	s = print_buf + PRINT_BUF_LEN-1;
	*s = '\0';

	while (u) {
		t = u % b;
		if( t >= 10 )
			t += letbase - '0' - 10;
		*--s = t + '0';
		u /= b;
	}

	if (neg) {
		if( width && (pad & PAD_ZERO) ) {
			printchar (UartHandle, out, '-');
			++pc;
			--width;
		}
		else {
			*--s = '-';
		}
	}

	return pc + prints (UartHandle, out, s, width, pad);
}

static int print(UART_HandleTypeDef* UartHandle, char **out, const char *format, va_list args )
{
	register int width, pad;
	register int pc = 0;
	char scr[2];

	for (; *format != 0; ++format) {
		if (*format == '%') {
			++format;
			width = pad = 0;
			if (*format == '\0') break;
			if (*format == '%') goto out;
			if (*format == '-') {
				++format;
				pad = PAD_RIGHT;
			}
			while (*format == '0') {
				++format;
				pad |= PAD_ZERO;
			}
			for ( ; *format >= '0' && *format <= '9'; ++format) {
				width *= 10;
				width += *format - '0';
			}
			if( *format == 's' ) {
				register char *s = (char *)va_arg( args, int );
				pc += prints (UartHandle, out, s?s:"(null)", width, pad);
				continue;
			}
			if( *format == 'd' ) {
				pc += printi (UartHandle, out, va_arg( args, int ), 10, 1, width, pad, 'a');
				continue;
			}
			if( *format == 'x' ) {
				pc += printi (UartHandle, out, va_arg( args, int ), 16, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'X' ) {
				pc += printi (UartHandle, out, va_arg( args, int ), 16, 0, width, pad, 'A');
				continue;
			}
			if( *format == 'u' ) {
				pc += printi (UartHandle, out, va_arg( args, int ), 10, 0, width, pad, 'a');
				continue;
			}
			if( *format == 'c' ) {
				/* char are converted to int then pushed on the stack */
				scr[0] = (char)va_arg( args, int );
				scr[1] = '\0';
				pc += prints (UartHandle, out, scr, width, pad);
				continue;
			}
		}
		else {
		out:
			printchar (UartHandle, out, *format);
			++pc;
		}
	}
	if (out) **out = '\0';
	va_end( args );
	return pc;
}

int DRV_UART_printf(UART_HandleTypeDef* UartHandle, const char *format, ...)
{
        va_list args;

        va_start( args, format );
        return print(UartHandle, 0, format, args );
}

int DRV_UART_sprintf(UART_HandleTypeDef* UartHandle, char *out, const char *format, ...)
{
        va_list args;

        va_start( args, format );
        return print(UartHandle, &out, format, args );
}
