#ifndef I2C_DRV_H
#define I2C_DRV_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C"{
#endif

void DRV_I2C_init(I2C_HandleTypeDef* I2cHandle, unsigned int Freq, uint32_t addr, uint8_t nb_addr_bit);

void DRV_I2C_write(I2C_HandleTypeDef* I2cHandle, uint8_t* aTxBuffer, uint16_t size);
void DRV_I2C_read(I2C_HandleTypeDef* I2cHandle, uint8_t* aRxBuffer, uint16_t size);

#ifdef __cplusplus
}
#endif

#endif
