#ifndef I2C_DRV_H
#define I2C_DRV_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C"{
#endif

void DRV_I2C_init(I2C_HandleTypeDef* I2cHandle, unsigned int Freq, uint32_t addr, uint8_t nb_addr_bit);

void DRV_I2C_write(I2C_HandleTypeDef* I2cHandle, uint8_t* aTxBuffer, uint16_t size);
void DRV_I2C_read(I2C_HandleTypeDef* I2cHandle, uint8_t* aRxBuffer, uint16_t size);
void DRV_I2C_write_byte(I2C_HandleTypeDef* I2cHandle, uint8_t regAddr, uint8_t data);
void DRV_I2C_write_word(I2C_HandleTypeDef* I2cHandle, uint8_t regAddr, uint16_t data);
uint8_t DRV_I2C_read_byte(I2C_HandleTypeDef* I2cHandle, uint8_t regAddr);
uint16_t DRV_I2C_read_word(I2C_HandleTypeDef* I2cHandle, uint8_t regAddr);

#ifdef __cplusplus
}
#endif

#endif
