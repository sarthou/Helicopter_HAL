/*
 * Helicopter.cpp
 *
 *  Created on: 9 févr. 2017
 *      Author: JulienCombattelli
 */

#include "Helicopter.h"
#include "CommunicationProtocol.h"
#include "UART/Uart_init.h"
#include "UART/Uart_driver.h"
#include "StringUtility.h"

#include "SysTick/SysTick.h"

#include "SD/sd_diskio.h"
#include "Error_handler/Error_handler.h"

Helicopter::Helicopter() :
	//m_motorMain(PIN_MOT_1), m_motorTail(PIN_MOT_2),
	//m_adc1(PIN_ADC_1), m_adc2(PIN_ADC_2),
	//m_dac1(PIN_DAC_1), m_dac2(PIN_DAC_2)
	m_Te(0), m_Tsim(0), m_currentTime(0),
	m_waveformMain(NULL),
	m_waveformTail(NULL),
	m_isRunning(false)
{
	MainMotorPWM_init(&m_motorMain);
	TailMotorPWM_init(&m_motorTail);
	DRV_PWM_setPeriod(&m_motorMain, 100);
	DRV_PWM_setPeriod(&m_motorTail, 100);
	motorMainSetSpeed(0);
	motorTailSetSpeed(0);

	SysTick_setInstance(this);

	MPU9250_I2C_init(&m_i2c);
	USB_UART_init(&m_remotePC);
	DRV_UART_transmit(&m_remotePC, (uint8_t*)"hello \r\n");


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

void Helicopter::run()
{
	/*uint32_t freq = HAL_RCC_GetHCLKFreq();
	HAL_SYSTICK_Config((uint32_t)(freq * 1000.f / 1000000.f));*/
	DRV_UART_transmit(&m_remotePC, (uint8_t*)"run \r\n");
	while(1)
	{
		if(DRV_UART_readable(&m_remotePC))
		{
			char cmd = 0;
			cmd = DRV_UART_getchar(&m_remotePC);
			cmd = DRV_UART_getchar(&m_remotePC);
			switch(cmd)
			{
			case 0x01:
				if(not m_isRunning)
					motorMainSetSpeed(((float)DRV_UART_getchar(&m_remotePC))/100.f);
				break;
			case 0x02:
				if(not m_isRunning)
					motorTailSetSpeed(((float)DRV_UART_getchar(&m_remotePC))/100.f);
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
	DRV_PWM_setDutyCycle(&m_motorMain, m_motorMain.dutyCycle - speed);
}

void Helicopter::motorMainDecreaseSpeed(float speed)
{
	DRV_PWM_setDutyCycle(&m_motorMain, m_motorMain.dutyCycle + speed);
}

int Helicopter::motorMainGetSpeed()
{
	return 100 - (int(m_motorMain.dutyCycle * 100.f));
}

void Helicopter::motorTailSetSpeed(float speed)
{
	DRV_PWM_setDutyCycle(&m_motorTail, 1.0f - speed);
}

void Helicopter::motorTailIncreaseSpeed(float speed)
{
	DRV_PWM_setDutyCycle(&m_motorTail, m_motorTail.dutyCycle - speed);
}

void Helicopter::motorTailDecreaseSpeed(float speed)
{
	DRV_PWM_setDutyCycle(&m_motorTail, m_motorTail.dutyCycle + speed);
}

int Helicopter::motorTailGetSpeed()
{
	return 100 - (int(m_motorTail.dutyCycle * 100.f));
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

	uint8_t waveform = getchar();
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

	uint8_t waveform = getchar();
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
		//m_ticker.attach_us(callback(this, &Helicopter::process), m_Te);
}

void Helicopter::process()
{
	float commandRotorMain = m_waveformMain->generate(m_currentTime)/100000000.f;
	//printf("main : %d\r\n", int(commandRotorMain*100));
	//printf("main : %lu at time : %lu\r\n", m_waveformMain->generate(m_currentTime), m_currentTime*m_Te);
	motorMainSetSpeed(commandRotorMain);

	float commandRotorTail = m_waveformTail->generate(m_currentTime)/100000000.f;
	//printf("tail : %d\r\n", int(commandRotorTail*100));
	motorTailSetSpeed(commandRotorTail);

	m_currentTime++;
	if(m_currentTime > m_Tsim)
		stop();
}
