#include <Uart/Uart_driver.h>
#include <Uart/Uart_init.h>

#define USART_USB                       USART2
#define USART_USB_CLK_ENABLE()          __HAL_RCC_USART2_CLK_ENABLE()
#define USART_USB_AF                    GPIO_AF7_USART2
#define USART_USB_TX_PIN                GPIO_PIN_2
#define USART_USB_TX_GPIO_PORT          GPIOA
#define USART_USB_RX_PIN                GPIO_PIN_15
#define USART_USB_RX_GPIO_PORT          GPIOA

#define USART_BLE                       USART1
#define USART_BLE_CLK_ENABLE()          __HAL_RCC_USART1_CLK_ENABLE()
#define USART_BLE_AF                    GPIO_AF7_USART1
#define USART_BLE_TX_PIN                GPIO_PIN_9
#define USART_BLE_TX_GPIO_PORT          GPIOA
#define USART_BLE_RX_PIN                GPIO_PIN_10
#define USART_BLE_RX_GPIO_PORT          GPIOA

void HAL_UART_MspInit(UART_HandleTypeDef *huart)
{
	GPIO_InitTypeDef  GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();

	// Enable USB Uart
	USART_USB_CLK_ENABLE();

	GPIO_InitStruct.Pin       = USART_USB_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = USART_USB_AF;

	HAL_GPIO_Init(USART_USB_TX_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = USART_USB_RX_PIN;
	GPIO_InitStruct.Alternate = USART_USB_AF;

	HAL_GPIO_Init(USART_USB_RX_GPIO_PORT, &GPIO_InitStruct);

	// Enable BLE Uart
	USART_BLE_CLK_ENABLE();

	GPIO_InitStruct.Pin       = USART_BLE_TX_PIN;
	GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull      = GPIO_PULLUP;
	GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = USART_BLE_AF;

	HAL_GPIO_Init(USART_BLE_TX_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = USART_BLE_RX_PIN;
	GPIO_InitStruct.Alternate = USART_BLE_AF;

	HAL_GPIO_Init(USART_BLE_RX_GPIO_PORT, &GPIO_InitStruct);
}

void USB_UART_init(DRV_UART_TypeDef* uart)
{
	DRV_UART_init(uart, USART_USB);
	DRV_UART_baud(uart, 115200);
}

void BLE_UART_init(DRV_UART_TypeDef* uart)
{
	DRV_UART_init(uart, USART_BLE);
	DRV_UART_baud(uart, 115200);
}
