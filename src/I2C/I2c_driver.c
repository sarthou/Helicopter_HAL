#include "I2c_driver.h"
#include "../Error_handler/Error_handler.h"

#define I2C_TIMING_750K        0x00400B27
#define I2C_TIMING_500K      0x00400B67
#define I2C_TIMING_250K      0x00400CE6

uint32_t m_addr = 0x00;

void DRV_I2C_init(I2C_HandleTypeDef* I2cHandle, unsigned int Freq, uint32_t addr, uint8_t nb_addr_bit)
{
	m_addr = addr;
	I2cHandle->Instance             = I2C1;
	switch(Freq)
	{
	case 750: I2cHandle->Init.Timing = I2C_TIMING_750K; break;
	case 500: I2cHandle->Init.Timing = I2C_TIMING_500K; break;
	case 250: I2cHandle->Init.Timing = I2C_TIMING_250K; break;
	default: I2cHandle->Init.Timing = I2C_TIMING_250K; break;
	}
	I2cHandle->Init.OwnAddress1     = m_addr;
	if(nb_addr_bit == 10)
		I2cHandle->Init.AddressingMode  = I2C_ADDRESSINGMODE_10BIT;
	else
		I2cHandle->Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
	I2cHandle->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	I2cHandle->Init.OwnAddress2     = 0xFF;
	I2cHandle->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	I2cHandle->Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;

	if(HAL_I2C_Init(I2cHandle) != HAL_OK)
		Error_Handler();

	HAL_I2CEx_ConfigAnalogFilter(I2cHandle,I2C_ANALOGFILTER_ENABLE);
}

void DRV_I2C_write(I2C_HandleTypeDef* I2cHandle, uint8_t* aTxBuffer, uint16_t size)
{
	while(HAL_I2C_Master_Transmit(I2cHandle, (uint16_t)m_addr, (uint8_t*)aTxBuffer, size, 10000)!= HAL_OK)
	{
		if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
			Error_Handler();
	}
}

void DRV_I2C_read(I2C_HandleTypeDef* I2cHandle, uint8_t* aRxBuffer, uint16_t size)
{
	while(HAL_I2C_Master_Receive(I2cHandle, (uint16_t)m_addr, (uint8_t *)aRxBuffer, size, 10000) != HAL_OK)
	{
		if (HAL_I2C_GetError(I2cHandle) != HAL_I2C_ERROR_AF)
			Error_Handler();
	}
}

void DRV_I2C_write_byte(I2C_HandleTypeDef* I2cHandle, uint8_t regAddr, uint8_t data)
{
	uint8_t aTxBuffer[2];
	aTxBuffer[0] = regAddr;
	aTxBuffer[1] = data;
	DRV_I2C_write(I2cHandle, aTxBuffer, 2);
}

void DRV_I2C_write_word(I2C_HandleTypeDef* I2cHandle, uint8_t regAddr, uint16_t data)
{
	uint8_t aTxBuffer[3];
	aTxBuffer[0] = regAddr;
	aTxBuffer[1] = data;
	aTxBuffer[2] = (data >> 8);
	DRV_I2C_write(I2cHandle, aTxBuffer, 3);
}

uint8_t DRV_I2C_read_byte(I2C_HandleTypeDef* I2cHandle, uint8_t regAddr)
{
	uint8_t aTxBuffer[1];
	aTxBuffer[0] = regAddr;
	DRV_I2C_write(I2cHandle, aTxBuffer, 1);
	DRV_I2C_read(I2cHandle, aTxBuffer, 1);
	return aTxBuffer[0];
}

uint16_t DRV_I2C_read_word(I2C_HandleTypeDef* I2cHandle, uint8_t regAddr)
{
	uint8_t aTxBuffer[2];
	aTxBuffer[0] = regAddr;
	DRV_I2C_write(I2cHandle, aTxBuffer, 1);
	DRV_I2C_read(I2cHandle, aTxBuffer, 2);
	return aTxBuffer[0] & (aTxBuffer[1] << 8);
}
