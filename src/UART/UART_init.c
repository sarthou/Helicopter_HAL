#include "UART_init.h"

/*COMMON*/
#define USART_NO                         USART2 //BLE UART1
#define USART_NO_CLK_ENABLE()            __HAL_RCC_USART2_CLK_ENABLE()
#define USART_AF                     	GPIO_AF7_USART2

/*TX*/
#define USARTx_TX_PIN                    GPIO_PIN_2//BLE 9
#define USARTx_TX_GPIO_PORT              GPIOA

/*RX*/
#define USARTx_RX_PIN                    GPIO_PIN_15//BLE 10
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

void UART_init(UART_HandleTypeDef* UartHandle)
{
	UartHandle->Instance        = USART_NO;

	UartHandle->Init.BaudRate     = 9600;
	UartHandle->Init.WordLength   = UART_WORDLENGTH_8B;
	UartHandle->Init.StopBits     = UART_STOPBITS_1;
	UartHandle->Init.Parity       = UART_PARITY_NONE;
	UartHandle->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
	UartHandle->Init.Mode         = UART_MODE_TX_RX;
	UartHandle->Init.OverSampling = UART_OVERSAMPLING_16;
	UartHandle->AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;

	if(HAL_UART_DeInit(UartHandle) != HAL_OK)
		while(1);//Error_Handler();

	if(HAL_UART_Init(UartHandle) != HAL_OK)
		while(1);//Error_Handler();
}
