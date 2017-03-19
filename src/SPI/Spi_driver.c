#include "Error_handler/Error_handler.h"
#include "SPI/Spi_driver.h"

static uint32_t DRV_SPI_getPCLKxFreq(SPI_TypeDef* instance)
{
	uint32_t clkFreq = 0;
    switch ((int)instance)
    {
#if defined SPI1_BASE
		case SPI1_BASE:
			clkFreq = HAL_RCC_GetPCLK2Freq();
			break;
#endif
#if defined SPI2_BASE
		case SPI2_BASE:
#endif
#if defined SPI3_BASE
		case SPI3_BASE:
#endif
#if defined SPI4_BASE
		case SPI4_BASE:
#endif
			clkFreq = HAL_RCC_GetPCLK1Freq();
			break;
		default:
			Error_Handler();
            break;
    }
    return clkFreq;
}

void DRV_SPI_init(SPI_HandleTypeDef* SpiHandle, SPI_TypeDef* instance, uint32_t frequency_hz)
{
	uint32_t clkFreq = DRV_SPI_getPCLKxFreq(instance);
	uint32_t prescaler = 0;


    while(clkFreq/prescaler > frequency_hz)
    	prescaler += 8;

	if(HAL_SPI_GetState(SpiHandle) == HAL_SPI_STATE_RESET)
	{
		SpiHandle->Instance = instance;

		SpiHandle->Init.BaudRatePrescaler = prescaler;
		SpiHandle->Init.Direction = SPI_DIRECTION_2LINES;
		SpiHandle->Init.CLKPhase = SPI_PHASE_2EDGE;
		SpiHandle->Init.CLKPolarity = SPI_POLARITY_HIGH;
		SpiHandle->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
		SpiHandle->Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
		SpiHandle->Init.CRCPolynomial = 7;
		SpiHandle->Init.DataSize = SPI_DATASIZE_8BIT;
		SpiHandle->Init.FirstBit = SPI_FIRSTBIT_MSB;
		SpiHandle->Init.NSS = SPI_NSS_SOFT;
		SpiHandle->Init.TIMode = SPI_TIMODE_DISABLE;
		SpiHandle->Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
		SpiHandle->Init.Mode = SPI_MODE_MASTER;

		HAL_SPI_Init(SpiHandle);
	}
}

void DRV_SPI_writeReadData(SPI_HandleTypeDef* SpiHandle, const uint8_t *DataIn, uint8_t *DataOut, uint16_t DataLegnth)
{
	HAL_StatusTypeDef status = HAL_OK;

	status = HAL_SPI_TransmitReceive(SpiHandle, (uint8_t*) DataIn, DataOut, DataLegnth, SPIx_TIMEOUT_MAX);

	if(status != HAL_OK)
		Error_Handler();
}

uint32_t DRV_SPI_read(SPI_HandleTypeDef* SpiHandle)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint32_t readvalue = 0;
	uint32_t writevalue = 0xFFFFFFFF;

	status = HAL_SPI_TransmitReceive(SpiHandle, (uint8_t*) &writevalue, (uint8_t*) &readvalue, 1, SPIx_TIMEOUT_MAX);

	if(status != HAL_OK)
		Error_Handler();

	return readvalue;
}

void DRV_SPI_write(SPI_HandleTypeDef* SpiHandle, uint8_t Value)
{
	HAL_StatusTypeDef status = HAL_OK;
	uint8_t data;

	status = HAL_SPI_TransmitReceive(SpiHandle, (uint8_t*) &Value, &data, 1, SPIx_TIMEOUT_MAX);

	if(status != HAL_OK)
		Error_Handler();
}
