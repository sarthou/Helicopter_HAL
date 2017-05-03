/*
 * Helicopter.cpp
 *
 *  Created on: 9 févr. 2017
 *      Author: JulienCombattelli
 */

#include <Helicopter/Helicopter.h>

#include <Error_handler/Error_handler.h>
#include <Helicopter/Hcp.h>
#include <SDCard/sd_diskio.h>

#include <limits>


/*
 * Static Helicopter instance use for SysTick timer interrupt handler
 */
static Helicopter* helicopterInstance = NULL;



/*
 * HAL SysTick overloaded functions
 */
void HAL_SYSTICK_Callback(void)
{
	if(helicopterInstance)
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
	m_isRunning(false),
	m_isTimeToSendData(false)
{
	MainMotorPWM_init(&m_motorMain);
	TailMotorPWM_init(&m_motorTail);

	//MPU9250_I2C_init(&m_i2c);
	BLE_UART_init(&m_remotePc);

	DRV_UART_puts(&m_remotePc, "hello \r\n");

	disableSysTickHandler();

	POT2_init(&m_adc);

	MPU9250_I2C_init(&m_i2c);

	//SD card
	uint32_t byteswritten;

	if(FATFS_LinkDriver(&SD_Driver, m_SDPath) != 0)
		Error_Handler();
	if(f_mount(&m_SDFatFs, (char const*)m_SDPath, 0) != FR_OK)
		Error_Handler();

	if(f_open(&m_file, "STM32.TXT", FA_OPEN_ALWAYS | FA_WRITE) != FR_OK)
		Error_Handler();

	char text[] = "hello !";
	f_write(&m_file, text, sizeof(text), &byteswritten);

	if (f_close(&m_file) != FR_OK )
		Error_Handler();
	//END SD card
}

Helicopter::~Helicopter()
{
	disableSysTickHandler();
}

void Helicopter::run()
{
	while(1)
	{
		if(m_isTimeToSendData)
		{
			uint8_t buffer[4] = {0};
			uint32_t bytesread = 0;
			m_isTimeToSendData = false;

			if(f_open(&m_file, "SAMPLE1.TXT", FA_OPEN_EXISTING | FA_READ) != FR_OK)
				Error_Handler();

			DRV_UART_putc(&m_remotePc, FrameType_StartTransmission);

			f_read(&m_file, buffer, sizeof(uint16_t), &bytesread);
			DRV_UART_write(&m_remotePc, buffer, sizeof(uint16_t));

			f_read(&m_file, buffer, sizeof(uint32_t), &bytesread);
			DRV_UART_write(&m_remotePc, buffer, sizeof(uint32_t));

			f_read(&m_file, buffer, sizeof(uint16_t), &bytesread);
			DRV_UART_write(&m_remotePc, buffer, sizeof(uint16_t));

			int nb = 0;
			while(not f_eof(&m_file))
			{
				f_read(&m_file, buffer, sizeof(float), &bytesread);
				DRV_UART_write(&m_remotePc, buffer, sizeof(float));
				nb++;
			}

			if(f_close(&m_file) != FR_OK )
				Error_Handler();
		}
		if(DRV_UART_readable(&m_remotePc))
		{
			char cmd = 0;
			cmd = DRV_UART_getc(&m_remotePc);
			switch(cmd)
			{
			case FrameType_ManualRotorMain:
				if(not m_isRunning)
					handleManualRotorMainFrame();
				break;
			case FrameType_ManualRotorTail:
				if(not m_isRunning)
					handleManualRotorTailFrame();
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
			case FrameType_SignalSensors:
				if(not m_isRunning)
					handleSignalSensorsFrame();
				break;
			case FrameType_Start:
				if(not m_isRunning)
					handleStartFrame();
				break;
			}
		}
	}
}

void Helicopter::stop()
{
	disableSysTickHandler();
	f_close(&m_file);
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

void Helicopter::setSysTickTimer(uint32_t period_us)
{
	float freq = (float)HAL_RCC_GetHCLKFreq();
	uint32_t ticknumb = (uint32_t)(freq * period_us / 1000000.f);
	HAL_SYSTICK_Config(ticknumb);
}

void Helicopter::enableSysTickHandler()
{
	helicopterInstance = this;
}

void Helicopter::disableSysTickHandler()
{
	helicopterInstance = NULL;
}

void Helicopter::handleManualRotorMainFrame()
{
	uint8_t buffer[2];

	DRV_UART_read(&m_remotePc, buffer, 2);
	int value = HCP_toUint16(buffer);

	motorMainSetSpeed(((float)value)/std::numeric_limits<uint16_t>::max());
}

void Helicopter::handleManualRotorTailFrame()
{
	uint8_t buffer[2];

	DRV_UART_read(&m_remotePc, buffer, 2);
	int value = HCP_toUint16(buffer);

	motorTailSetSpeed(((float)value)/std::numeric_limits<uint16_t>::max());
}

void Helicopter::handleInitializationFrame()
{
	uint8_t buffer[4];

	DRV_UART_read(&m_remotePc, buffer, 2);
	m_Te = HCP_toUint16(buffer);

	DRV_UART_read(&m_remotePc, buffer, 4);
	m_Tsim = HCP_toUint32(buffer);
}

void Helicopter::handleSignalRotorMainFrame()
{
	uint8_t buffer[4];

	uint8_t waveform = DRV_UART_getc(&m_remotePc);
	switch(waveform)
	{
		case WaveformType_Step:
		{
			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t Tstart = HCP_toUint32(buffer);

			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t finalValue = HCP_toUint32(buffer);
			m_waveformMain = new StepWaveform(Tstart, finalValue);
			break;
		}
		case WaveformType_Ramp:
		{
			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t Tstart = HCP_toUint32(buffer);

			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t slope = HCP_toUint32(buffer);
			m_waveformMain = new RampWaveform(Tstart, slope);
			break;
		}
		case WaveformType_PRBS:
		{
			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t min = HCP_toUint32(buffer);

			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t max = HCP_toUint32(buffer);

			DRV_UART_read(&m_remotePc, buffer, 2);
			uint32_t seed = HCP_toUint16(buffer);

			m_waveformMain = new PRBSWaveform(0, min, max, seed);

			break;
		}
	}
}

void Helicopter::handleSignalRotorTailFrame()
{
	uint8_t buffer[4];

	uint8_t waveform = DRV_UART_getc(&m_remotePc);
	switch(waveform)
	{
		case WaveformType_Step:
		{
			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t Tstart = HCP_toUint32(buffer);

			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t finalValue = HCP_toUint32(buffer);
			m_waveformTail = new StepWaveform(Tstart, finalValue);
			break;
		}
		case WaveformType_Ramp:
		{
			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t Tstart = HCP_toUint32(buffer);

			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t slope = HCP_toUint32(buffer);
			m_waveformTail = new RampWaveform(Tstart, slope);
			break;
		}
		case WaveformType_PRBS:
		{
			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t min = HCP_toUint32(buffer);

			DRV_UART_read(&m_remotePc, buffer, 4);
			uint32_t max = HCP_toUint32(buffer);

			DRV_UART_read(&m_remotePc, buffer, 2);
			uint32_t seed = HCP_toUint16(buffer);

			m_waveformTail = new PRBSWaveform(0, min, max, seed);

			break;
		}
	}
}

void Helicopter::handleSignalSensorsFrame()
{
	uint8_t nbSensors = DRV_UART_getc(&m_remotePc);
	sensors.nbSensors = nbSensors;
	sensors.mainMotor = 0;
	sensors.tailMotor = 0;
	sensors.pitchPot = 0;
	sensors.acceleroX = 0;
	sensors.acceleroY = 0;
	sensors.acceleroZ = 0;
	sensors.gyroX = 0;
	sensors.gyroY = 0;
	sensors.gyroZ = 0;

	for(int i = 0; i < nbSensors; i++)
	{
		uint8_t sensorId = DRV_UART_getc(&m_remotePc);
		switch(sensorId)
		{
		case 0x01: sensors.mainMotor = 1; break;
		case 0x02: sensors.tailMotor = 1; break;
		case 0x03: sensors.pitchPot = 1; break;
		case 0x04: sensors.acceleroX = 1; break;
		case 0x05: sensors.acceleroY = 1; break;
		case 0x06: sensors.acceleroZ = 1; break;
		case 0x07: sensors.gyroX = 1; break;
		case 0x08: sensors.gyroY = 1; break;
		case 0x09: sensors.gyroZ = 1; break;
		default : break;
		}
	}
}

void Helicopter::handleStartFrame()
{
	m_isRunning = true;

	uint8_t buffer[5] = {0};
	uint32_t byteswritten = 0;

	if(f_open(&m_file, "SAMPLE1.TXT", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
		Error_Handler();

	buffer[0] = sensors.nbSensors;
	f_write(&m_file, (uint8_t*)&buffer, 2, &byteswritten);

	f_write(&m_file, (uint8_t*)&m_Tsim, sizeof(m_Tsim), &byteswritten);

	f_write(&m_file, (uint8_t*)&m_Te, sizeof(m_Te), &byteswritten);

	setSysTickTimer(m_Te);
	enableSysTickHandler();
}

void Helicopter::process()
{
	if(m_isRunning)
	{
		float commandRotorMain = m_waveformMain->generate(m_currentTime)/100000000.f;
		motorMainSetSpeed(commandRotorMain);

		float commandRotorTail = m_waveformTail->generate(m_currentTime)/100000000.f;
		motorTailSetSpeed(commandRotorTail);

		uint32_t byteswritten = 0;
		if(sensors.mainMotor)
			f_write(&m_file, (uint8_t*)&commandRotorMain, sizeof(commandRotorMain), &byteswritten);
		if(sensors.tailMotor)
			f_write(&m_file, (uint8_t*)&commandRotorTail, sizeof(commandRotorTail), &byteswritten);
		if(sensors.pitchPot)
		{
			float pitchPot = DRV_ADC_getValue(&m_adc);
			f_write(&m_file, (uint8_t*)&pitchPot, sizeof(pitchPot), &byteswritten);
		}

		m_currentTime++;
		if(m_currentTime > m_Tsim)
		{
			stop();
			m_isTimeToSendData = true;
		}
	}
}

