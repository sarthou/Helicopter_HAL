/*
 * UartDriver.c
 *
 *  Created on: 26 mars 2017
 *      Author: JulienCombattelli
 */

#include <Uart/Uart_driver.h>
#include <Error_handler/Error_handler.h>

#include <stdbool.h>
#ifdef DRV_UART_USE_PRINTF
#include <stdarg.h>
#include <stdio.h>
#endif

#define UART_NUM (3)

//static uint32_t serial_irq_ids[UART_NUM] = {0};
static UART_HandleTypeDef uart_handlers[UART_NUM];

//static DRV_UART_irqHandler irq_handler;

static void init_uart(DRV_UART_TypeDef *obj)
{
	UART_HandleTypeDef *huart = &uart_handlers[obj->index];
	huart->Instance = (USART_TypeDef *)(obj->uart);

	huart->Init.BaudRate = obj->baudrate;
	huart->Init.WordLength = obj->databits;
	huart->Init.StopBits = obj->stopbits;
	huart->Init.Parity = obj->parity;
	huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;

	huart->TxXferCount = 0;
	huart->TxXferSize = 0;
	huart->RxXferCount = 0;
	huart->RxXferSize = 0;

	huart->Init.Mode = UART_MODE_TX_RX;

	if(HAL_UART_Init(huart) != HAL_OK)
		Error_Handler();//("Cannot initialize UART\n");
}

void DRV_UART_init(DRV_UART_TypeDef *obj, USART_TypeDef* uart)
{
	obj->uart = uart;
	// Enable USART clock + switch to SystemClock
	if(obj->uart == USART1)
	{
		__USART1_FORCE_RESET();
		__USART1_RELEASE_RESET();
		__USART1_CLK_ENABLE();
#if defined(RCC_USART1CLKSOURCE_SYSCLK)
		__HAL_RCC_USART1_CONFIG(RCC_USART1CLKSOURCE_SYSCLK);
#endif
		obj->index = 0;
	}
	if(obj->uart == USART2)
	{
		__USART2_FORCE_RESET();
		__USART2_RELEASE_RESET();
		__USART2_CLK_ENABLE();
#if defined(RCC_USART2CLKSOURCE_SYSCLK)
		__HAL_RCC_USART2_CONFIG(RCC_USART2CLKSOURCE_SYSCLK);
#endif
		obj->index = 1;
	}
	if(obj->uart == USART3)
	{
		__USART3_FORCE_RESET();
		__USART3_RELEASE_RESET();
		__USART3_CLK_ENABLE();
#if defined(RCC_USART3CLKSOURCE_SYSCLK)
		__HAL_RCC_USART3_CONFIG(RCC_USART3CLKSOURCE_SYSCLK);
#endif
		obj->index = 2;
	}

	// Configure UART
	obj->baudrate = 9600;
	obj->databits = UART_WORDLENGTH_8B;
	obj->stopbits = UART_STOPBITS_1;
	obj->parity = UART_PARITY_NONE;

	init_uart(obj);
}

void DRV_UART_free(DRV_UART_TypeDef *obj)
{
	// Reset UART and disable clock
	if(obj->uart == USART1)
	{
		__USART1_FORCE_RESET();
		__USART1_RELEASE_RESET();
		__USART1_CLK_DISABLE();
	}
	if(obj->uart == USART2)
	{
		__USART2_FORCE_RESET();
		__USART2_RELEASE_RESET();
		__USART2_CLK_DISABLE();
	}
	if(obj->uart == USART3)
	{
		__USART3_FORCE_RESET();
		__USART3_RELEASE_RESET();
		__USART3_CLK_DISABLE();
	}

	//serial_irq_ids[obj->index] = 0;
}

void DRV_UART_baud(DRV_UART_TypeDef *obj, int baudrate)
{
	obj->baudrate = baudrate;
	init_uart(obj);
}

void DRV_UART_format(DRV_UART_TypeDef *obj, int data_bits, DRV_UART_parity parity, int stop_bits)
{
	if(data_bits == 9)
		obj->databits = UART_WORDLENGTH_9B;
	else
		obj->databits = UART_WORDLENGTH_8B;

	switch(parity)
	{
		case ParityOdd:
			obj->parity = UART_PARITY_ODD;
			break;
		case ParityEven:
			obj->parity = UART_PARITY_EVEN;
			break;
		default: // ParityNone
		case ParityForced0: // unsupported!
		case ParityForced1: // unsupported!
			obj->parity = UART_PARITY_NONE;
			break;
	}

	if(stop_bits == 2)
		obj->stopbits = UART_STOPBITS_2;
	else
		obj->stopbits = UART_STOPBITS_1;

	init_uart(obj);
}

/*static void uart_irq(int id)
{
	UART_HandleTypeDef * huart = &uart_handlers[id];

	if(serial_irq_ids[id] != 0)
	{
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_TC) != RESET)
		{
			if(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_TC) != RESET)
			{
				irq_handler(serial_irq_ids[id],TxIrq);
				__HAL_UART_CLEAR_FLAG(huart,UART_CLEAR_TCF);
			}
		}
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET)
		{
			if(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_RXNE) != RESET)
			{
				irq_handler(serial_irq_ids[id],RxIrq);
				volatile uint32_t tmpval = huart->Instance->RDR; // Clear RXNE flag
				UNUSED(tmpval);
			}
		}
		if(__HAL_UART_GET_FLAG(huart, UART_FLAG_ORE) != RESET)
		{
			if(__HAL_UART_GET_IT_SOURCE(huart, UART_IT_ORE) != RESET)
			{
				__HAL_UART_CLEAR_FLAG(huart,UART_CLEAR_OREF);
			}
		}
	}
}

static void uart1_irq(void)
{
	uart_irq(0);
}

static void uart2_irq(void)
{
	uart_irq(1);
}

static void uart3_irq(void)
{
	uart_irq(2);
}

void DRV_UART_irq_handler(DRV_UART_TypeDef *obj, DRV_UART_irqHandler handler, uint32_t id)
{
	irq_handler = handler;
	serial_irq_ids[obj->index] = id;
}

void DRV_UART_irq_set(DRV_UART_TypeDef *obj, SerialIrq irq, uint32_t enable)
{
	UART_HandleTypeDef *huart = &uart_handlers[obj->index];
	IRQn_Type irq_n = (IRQn_Type)0;
	uint32_t vector = 0;

	if(obj->uart == USART1)
	{
		irq_n = USART1_IRQn;
		vector = (uint32_t)&uart1_irq;
	}

	if(obj->uart == USART2)
	{
		irq_n = USART2_IRQn;
		vector = (uint32_t)&uart2_irq;
	}

	if(obj->uart == USART3)
	{
		irq_n = USART3_IRQn;
		vector = (uint32_t)&uart3_irq;
	}

	if(enable)
	{
		if(irq == RxIrq)
			__HAL_UART_ENABLE_IT(huart,UART_IT_RXNE);
		else
			// TxIrq
			__HAL_UART_ENABLE_IT(huart,UART_IT_TC);

		extern void NVIC_SetVector(IRQn_Type,uint32_t); // remove implicit declaration warning
		NVIC_SetVector(irq_n,vector);
		NVIC_EnableIRQ(irq_n);

	}
	else // disable
	{
		int all_disabled = 0;
		if(irq == RxIrq)
		{
			__HAL_UART_DISABLE_IT(huart,UART_IT_RXNE);
			// Check if TxIrq is disabled too
			if((huart->Instance->CR1 & USART_CR1_TXEIE) == 0)
				all_disabled = 1;
		}
		else // TxIrq
		{
			__HAL_UART_DISABLE_IT(huart,UART_IT_TC);
			// Check if RxIrq is disabled too
			if((huart->Instance->CR1 & USART_CR1_RXNEIE) == 0)
				all_disabled = 1;
		}

		if(all_disabled)
			NVIC_DisableIRQ(irq_n);
	}
}*/

uint8_t DRV_UART_getc(DRV_UART_TypeDef *obj)
{
	UART_HandleTypeDef *huart = &uart_handlers[obj->index];

	while(!DRV_UART_readable(obj));

	if(obj->databits == UART_WORDLENGTH_8B)
		return (uint8_t)(huart->Instance->RDR & (uint8_t)0xFF);
	else
		return (uint8_t)(huart->Instance->RDR & (uint16_t)0x1FF);
}

void DRV_UART_read(DRV_UART_TypeDef *obj, uint8_t *buffer, size_t nbytes)
{
	for(int i = 0 ; i < nbytes ; i++)
		buffer[i] = DRV_UART_getc(obj);
}

void DRV_UART_putc(DRV_UART_TypeDef *obj, int c)
{
	UART_HandleTypeDef *huart = &uart_handlers[obj->index];

	while(!DRV_UART_writable(obj));

	if(obj->databits == UART_WORDLENGTH_8B)
		huart->Instance->TDR = (uint8_t)(c & (uint8_t)0xFF);
	else
		huart->Instance->TDR = (uint16_t)(c & (uint16_t)0x1FF);
}

void DRV_UART_puts(DRV_UART_TypeDef *obj, const char* str)
{
	while (*str)
		DRV_UART_putc(obj, *str ++);
}

void DRV_UART_write(DRV_UART_TypeDef *obj, uint8_t *buffer, size_t nbytes)
{
	for(int i = 0 ; i < nbytes ; i++)
		DRV_UART_putc(obj, buffer[i]);
}

void DRV_UART_printf(DRV_UART_TypeDef *obj, const char* str, ...)
{
#ifdef DRV_UART_USE_PRINTF
	char buffer[50];

	va_list va;
	va_start(va, str);
	vsnprintf(buffer, 50, str, va);
	va_end(va);

	DRV_UART_puts(obj, buffer);
#else
	(void)obj;
	(void)str;
#endif
}

int DRV_UART_readable(DRV_UART_TypeDef *obj)
{
	UART_HandleTypeDef *huart = &uart_handlers[obj->index];

	// Check if data is received
	return (__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) != RESET) ? 1 : 0;
}

int DRV_UART_writable(DRV_UART_TypeDef *obj)
{
	UART_HandleTypeDef *huart = &uart_handlers[obj->index];

	// Check if data is transmitted
	return (__HAL_UART_GET_FLAG(huart, UART_FLAG_TXE) != RESET) ? 1 : 0;
}

void DRV_UART_clear(DRV_UART_TypeDef *obj)
{
	UART_HandleTypeDef *huart = &uart_handlers[obj->index];

	__HAL_UART_CLEAR_FLAG(huart,UART_CLEAR_TCF);
	__HAL_UART_SEND_REQ(huart,UART_RXDATA_FLUSH_REQUEST);
}
