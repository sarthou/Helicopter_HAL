#include "Uart_init.h"
#include "Uart_driver.h"

/*COMMON*/
#define USART_NO                         USART1 //BLE UART1 / USB UART2
#define USART_NO_CLK_ENABLE()            __HAL_RCC_USART1_CLK_ENABLE()//__HAL_RCC_USART2_CLK_ENABLE()
#define USART_AF                     	GPIO_AF7_USART1

/*TX*/
#define USARTx_TX_PIN                    GPIO_PIN_9//BLE 9 / USB 2
#define USARTx_TX_GPIO_PORT              GPIOA

/*RX*/
#define USARTx_RX_PIN                    GPIO_PIN_10//BLE 10 / USB 15
#define USARTx_RX_GPIO_PORT              GPIOA

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	USART_NO_CLK_ENABLE();

	GPIO_InitStruct.Pin       = USARTx_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = USART_AF;

	HAL_GPIO_Init(USARTx_TX_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = USARTx_RX_PIN;
	GPIO_InitStruct.Alternate = USART_AF;

	HAL_GPIO_Init(USARTx_RX_GPIO_PORT, &GPIO_InitStruct);
}

void USB_UART_init(UART_HandleTypeDef* UartHandle)
{
	DRV_UART_init(UartHandle, USART_NO, 115200);
}
