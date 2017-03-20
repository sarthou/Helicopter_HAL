#include "SPI/Spi_driver.h"
#include "SPI/Spi_init.h"

#define NUCLEO_SPIx                               SPI1
#define NUCLEO_SPIx_CLK_ENABLE()                  __HAL_RCC_SPI1_CLK_ENABLE()

#define NUCLEO_SPIx_SCK_AF                        GPIO_AF5_SPI1
#define NUCLEO_SPIx_SCK_GPIO_PORT                 GPIOB
#define NUCLEO_SPIx_SCK_PIN                       GPIO_PIN_3
#define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOA_CLK_DISABLE()

#define NUCLEO_SPIx_MISO_MOSI_AF                  GPIO_AF5_SPI1
#define NUCLEO_SPIx_MISO_MOSI_GPIO_PORT           GPIOB
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOA_CLK_ENABLE()
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOA_CLK_DISABLE()
#define NUCLEO_SPIx_MISO_PIN                      GPIO_PIN_4
#define NUCLEO_SPIx_MOSI_PIN                      GPIO_PIN_5

void HAL_SPI_MspInit(SPI_HandleTypeDef *hspi)
{
	UNUSED(hspi);

	GPIO_InitTypeDef  GPIO_InitStruct;

	/*** Configure the GPIOs ***/
	/* Enable GPIO clock */
	NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE();
	NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE();

	/* Configure SPI SCK */
	GPIO_InitStruct.Pin = NUCLEO_SPIx_SCK_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull  = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	GPIO_InitStruct.Alternate = NUCLEO_SPIx_SCK_AF;
	HAL_GPIO_Init(NUCLEO_SPIx_SCK_GPIO_PORT, &GPIO_InitStruct);

	/* Configure SPI MISO and MOSI */
	GPIO_InitStruct.Pin = NUCLEO_SPIx_MOSI_PIN;
	GPIO_InitStruct.Alternate = NUCLEO_SPIx_MISO_MOSI_AF;
	GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
	HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = NUCLEO_SPIx_MISO_PIN;
	HAL_GPIO_Init(NUCLEO_SPIx_MISO_MOSI_GPIO_PORT, &GPIO_InitStruct);

	/*** Configure the SPI peripheral ***/
	/* Enable SPI clock */
	NUCLEO_SPIx_CLK_ENABLE();
}
