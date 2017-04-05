#include "Error_handler/Error_handler.h"
#include "SDCard/Sd_driver.h"
#include "SDCard/Sd_init.h"
#include "Spi/Spi_driver.h"

/*#define NUCLEO_SPIx                               SPI1
#define NUCLEO_SPIx_CLK_ENABLE()                  __HAL_RCC_SPI1_CLK_ENABLE()

#define NUCLEO_SPIx_SCK_AF                        GPIO_AF5_SPI1
#define NUCLEO_SPIx_SCK_GPIO_PORT                 GPIOB
#define NUCLEO_SPIx_SCK_PIN                       GPIO_PIN_3
#define NUCLEO_SPIx_SCK_GPIO_CLK_ENABLE()         __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_SPIx_SCK_GPIO_CLK_DISABLE()        __HAL_RCC_GPIOB_CLK_DISABLE()

#define NUCLEO_SPIx_MISO_MOSI_AF                  GPIO_AF5_SPI1
#define NUCLEO_SPIx_MISO_MOSI_GPIO_PORT           GPIOB
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_ENABLE()   __HAL_RCC_GPIOB_CLK_ENABLE()
#define NUCLEO_SPIx_MISO_MOSI_GPIO_CLK_DISABLE()  __HAL_RCC_GPIOB_CLK_DISABLE()
#define NUCLEO_SPIx_MISO_PIN                      GPIO_PIN_4
#define NUCLEO_SPIx_MOSI_PIN                      GPIO_PIN_5

#define NUCLEO_SPIx_TIMEOUT_MAX                   1000*/

#define SD_CS_LOW()       HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_RESET)
#define SD_CS_HIGH()      HAL_GPIO_WritePin(SD_CS_GPIO_PORT, SD_CS_PIN, GPIO_PIN_SET)

#define SD_CS_PIN                                 GPIO_PIN_4
#define SD_CS_GPIO_PORT                           GPIOA
#define SD_CS_GPIO_CLK_ENABLE()                 __HAL_RCC_GPIOA_CLK_ENABLE()
#define SD_CS_GPIO_CLK_DISABLE()                __HAL_RCC_GPIOA_CLK_DISABLE()

#define SD_DUMMY_BYTE            0xFF
#define SD_NO_RESPONSE_EXPECTED  0x80

void DRV_SD_MspInit(DRV_SD_HandleTypeDef *hsd)
{
	UNUSED(hsd);

	GPIO_InitTypeDef  GPIO_InitStruct;

	SD_CS_GPIO_CLK_ENABLE();

	GPIO_InitStruct.Pin = SD_CS_PIN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
	HAL_GPIO_Init(SD_CS_GPIO_PORT, &GPIO_InitStruct);
}
