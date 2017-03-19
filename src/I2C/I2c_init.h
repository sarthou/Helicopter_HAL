#ifndef I2C_INIT_H
#define I2C_INIT_H

#include "stm32f3xx_hal.h"

#ifdef __cplusplus
extern "C"{
#endif

void MPU9250_I2C_init(I2C_HandleTypeDef* hi2c);

#ifdef __cplusplus
}
#endif

#endif
