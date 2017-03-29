/*
 * Helicopter.cpp
 *
 *  Created on: 9 févr. 2017
 *      Author: JulienCombattelli
 */

#include <Helicopter/Helicopter.h>
#include <Helicopter/CommunicationProtocol.h>

#include <Helicopter/StringUtility.h>

//#include "SD/sd_diskio.h"
#include <Error_handler/Error_handler.h>


/*
 * Static Helicopter instance use for SysTick timer interrupt handler
 */
static Helicopter* helicopterInstance = NULL;



/*
 * HAL SysTick overloaded functions
 */
void HAL_SYSTICK_Callback(void)
{
	helicopterInstance->process();
}

HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)
{
	HAL_NVIC_SetPriority((IRQn_Type)SysTick_IRQn, TickPriority ,0U);

	return HAL_OK;
}



/*
 * Helicopter methods definitions
 */
Helicopter::Helicopter() :
	m_Te(0), m_Tsim(0), m_currentTime(0),
	m_waveformMain(NULL),
	m_waveformTail(NULL),
	m_isRunning(false)
{
	MainMotorPWM_init(&m_motorMain);
	DRV_PWM_setDutyCycle(&m_motorMain, 0.7);

	TailMotorPWM_init(&m_motorTail);
	DRV_PWM_setDutyCycle(&m_motorTail, 0.1);

	//MPU9250_I2C_init(&m_i2c);
	USB_UART_init(&m_remotePc);
	DRV_UART_puts(&m_remotePc, "hello \r\n");

	helicopterInstance = this;

	//SD card
	/*UINT byteswritten;

	if(FATFS_LinkDriver(&SD_Driver, m_SDPath) != 0)
		Error_Handler();
	if(f_mount(&m_SDFatFs, (TCHAR const*)m_SDPath, 0) != FR_OK)
		Error_Handler();

	if(f_open(&m_file, "STM32.TXT", FA_OPEN_ALWAYS | FA_WRITE) != FR_OK)
		Error_Handler();

	char text[] = "hello !";
	f_write(&m_file, text, sizeof(text), &byteswritten);

	if (f_close(&m_file) != FR_OK )
		Error_Handler();*/
	//END SD card
}

Helicopter::~Helicopter()
{
	helicopterInstance = NULL;
}

void Helicopter::run()
{
	//uint32_t freq = HAL_RCC_GetHCLKFreq();
	//HAL_SYSTICK_Config((uint32_t)(freq * 1000.f / 1000000.f));
	DRV_UART_puts(&m_remotePc, "run\r\n");
	while(1)
	{
		if(DRV_UART_readable(&m_remotePc))
		{
			char cmd = 0;
			cmd = DRV_UART_getc(&m_remotePc);
			cmd = DRV_UART_getc(&m_remotePc);
			switch(cmd)
			{
			case 0x01:
				if(not m_isRunning)
					motorMainSetSpeed(((float)DRV_UART_getc(&m_remotePc))/100.f);
				break;
			case 0x02:
				if(not m_isRunning)
					motorTailSetSpeed(((float)DRV_UART_getc(&m_remotePc))/100.f);
				break;
			case FrameType_Stop:
				stop();
				break;
			case FrameType_Initialization:
				if(not m_isRunning)
					handleInitializationFrame();
				break;
			case FrameType_SignalRotorMain:
				if(not m_isRunning)
					handleSignalRotorMainFrame();
				break;
			case FrameType_SignalRotorTail:
				if(not m_isRunning)
					handleSignalRotorTailFrame();
				break;
			case FrameType_Start:
				if(not m_isRunning)
					handleStartFrame();
				break;
			};
		}
	}
}

void Helicopter::stop()
{
	//m_ticker.detach();
	motorMainSetSpeed(0);
	motorTailSetSpeed(0);
	m_isRunning = false;
	m_currentTime = 0;
}

void Helicopter::motorMainSetSpeed(float speed)
{
	DRV_PWM_setDutyCycle(&m_motorMain, 1.0f - speed);
}

void Helicopter::motorMainIncreaseSpeed(float speed)
{
	DRV_PWM_setDutyCycle(&m_motorMain, DRV_PWM_getDutyCycle(&m_motorMain) - speed);
}

void Helicopter::motorMainDecreaseSpeed(float speed)
{
	DRV_PWM_setDutyCycle(&m_motorMain, DRV_PWM_getDutyCycle(&m_motorMain) + speed);
}

int Helicopter::motorMainGetSpeed()
{
	return 100 - (int(DRV_PWM_getDutyCycle(&m_motorMain) * 100.f));
}

void Helicopter::motorTailSetSpeed(float speed)
{
	DRV_PWM_setDutyCycle(&m_motorTail, 1.0f - speed);
}

void Helicopter::motorTailIncreaseSpeed(float speed)
{
	DRV_PWM_setDutyCycle(&m_motorTail, DRV_PWM_getDutyCycle(&m_motorTail) - speed);
}

void Helicopter::motorTailDecreaseSpeed(float speed)
{
	DRV_PWM_setDutyCycle(&m_motorTail, DRV_PWM_getDutyCycle(&m_motorTail) + speed);
}

int Helicopter::motorTailGetSpeed()
{
	return 100 - (int(DRV_PWM_getDutyCycle(&m_motorTail) * 100.f));
}

void Helicopter::setAnalog1(float value)
{
	//m_dac1.write(value);
}

void Helicopter::setAnalog2(float value)
{
	//m_dac2.write(value);
}

float Helicopter::getAnalog1()
{
	return 0;//m_adc1.read();
}

float Helicopter::getAnalog2()
{
	return 0;//m_adc2.read();
}

void Helicopter::handleInitializationFrame()
{
	uint8_t buffer[4];

	receive(buffer, 2);
	m_Te = to_uint16(buffer);

	receive(buffer,4);
	m_Tsim = to_uint32(buffer);
}

void Helicopter::handleSignalRotorMainFrame()
{
	uint8_t buffer[4];

	receive(buffer, 4);
	uint32_t Tstart = to_uint32(buffer);

	uint8_t waveform = DRV_UART_getc(&m_remotePc);
	switch(waveform)
	{
		case WaveformType_Step:
		{
			receive(buffer, 4);
			uint32_t finalValue = to_uint32(buffer);
			m_waveformMain = new StepWaveform(Tstart, finalValue);
			break;
		}
		case WaveformType_Ramp:
		{
			receive(buffer, 4);
			uint32_t slope = to_uint32(buffer);
			m_waveformMain = new RampWaveform(Tstart, slope);
			break;
		}
	}
}

void Helicopter::handleSignalRotorTailFrame()
{
	uint8_t buffer[4];

	receive(buffer, 4);
	uint32_t Tstart = to_uint32(buffer);

	uint8_t waveform = DRV_UART_getc(&m_remotePc);
	switch(waveform)
	{
		case WaveformType_Step:
		{
			receive(buffer, 4);
			uint32_t finalValue = to_uint32(buffer);
			m_waveformTail = new StepWaveform(Tstart, finalValue);
			break;
		}
		case WaveformType_Ramp:
		{
			receive(buffer, 4);
			uint32_t slope = to_uint32(buffer);
			m_waveformTail = new RampWaveform(Tstart, slope);
			break;
		}
	}
}

void Helicopter::handleStartFrame()
{
	m_isRunning = true;
	HAL_SYSTICK_Config((uint32_t)(HAL_RCC_GetHCLKFreq() * m_Te / 1000000.f));
}

void Helicopter::process()
{
	float commandRotorMain = m_waveformMain->generate(m_currentTime)/100000000.f;
	motorMainSetSpeed(commandRotorMain);

	float commandRotorTail = m_waveformTail->generate(m_currentTime)/100000000.f;
	motorTailSetSpeed(commandRotorTail);

	m_currentTime++;
	if(m_currentTime > m_Tsim)
		stop();
}

