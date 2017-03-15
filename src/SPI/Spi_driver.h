#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "stm32f3xx_hal.h"

#define SPIx_TIMEOUT_MAX 10

#ifdef __cplusplus
extern "C"{
#endif

void DRV_SPI_init(SPI_HandleTypeDef* SpiHandle, SPI_TypeDef* instance, uint32_t frequency_hz);

void DRV_SPI_writeReadData(SPI_HandleTypeDef* SpiHandle, const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth);

uint32_t DRV_SPI_read(SPI_HandleTypeDef* SpiHandle);

void DRV_SPI_write(SPI_HandleTypeDef* SpiHandle, uint8_t Value);

#ifdef __cplusplus
}
#endif

#endif
