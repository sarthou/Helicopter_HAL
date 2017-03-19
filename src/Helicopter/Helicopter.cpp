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

#include "SysTick/SysTick.h"

Helicopter::Helicopter() :
	//m_motorMain(PIN_MOT_1), m_motorTail(PIN_MOT_2),
	//m_adc1(PIN_ADC_1), m_adc2(PIN_ADC_2),
	//m_dac1(PIN_DAC_1), m_dac2(PIN_DAC_2),
	//m_i2c(PIN_SDA,PIN_SCL)
	//m_remotePC(PA_9, PA_10, 115200),
	m_Te(0), m_Tsim(0), m_currentTime(0),
	m_waveformMain(NULL),
	m_waveformTail(NULL),
	m_isRunning(false)
{
	//m_motorMain = 1.0f;
	//m_motorTail = 1.0f;

	SysTick_setInstance(this);

	USB_UART_init(&m_remotePC);
	DRV_UART_transmit(&m_remotePC, (uint8_t*)"hello \r\n");
}

static int i = 0;

void Helicopter::run()
{
	DRV_UART_transmit(&m_remotePC, (uint8_t*)"run \r\n");
	while(1)
	{
		if(i == 1000)
		{
			i = 0;
			DRV_UART_transmit(&m_remotePC, (uint8_t*)"a");
		}
		/*if(m_remotePC.readable())
		{
			char cmd = 0;
			cmd = m_remotePC.getc();
			switch(cmd)
			{
			case 0x01:
				if(not m_isRunning)
					motorMainSetSpeed(((float)m_remotePC.getc())/100.f);
				break;
			case 0x02:
				if(not m_isRunning)
					motorTailSetSpeed(((float)m_remotePC.getc())/100.f);
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
				{
					m_isRunning = true;
					m_ticker.attach_us(callback(this, &Helicopter::process), m_Te);
				}
				break;
			};
			m_remotePC.printf("abcdefghijklmnopqrst");
		}*/
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
	//m_motorMain = 1.0f - speed;
	DRV_PWM_setDutyCycle(&m_motorMain, 1.0f - speed);
}

void Helicopter::motorMainIncreaseSpeed(float speed)
{
	//m_motorMain = m_motorMain - speed;
	DRV_PWM_setDutyCycle(&m_motorMain, m_motorMain.dutyCycle - speed);
}

void Helicopter::motorMainDecreaseSpeed(float speed)
{
	//m_motorMain = m_motorMain + speed;
	DRV_PWM_setDutyCycle(&m_motorMain, m_motorMain.dutyCycle + speed);
}

int Helicopter::motorMainGetSpeed()
{
	//return 100 - (int(m_motorMain.read() * 100.f));
	return 100 - (int(m_motorMain.dutyCycle * 100.f));
}

void Helicopter::motorTailSetSpeed(float speed)
{
	//m_motorTail = 1.0f - speed;
	DRV_PWM_setDutyCycle(&m_motorTail, 1.0f - speed);
}

void Helicopter::motorTailIncreaseSpeed(float speed)
{
	//m_motorTail = m_motorTail - speed;
	DRV_PWM_setDutyCycle(&m_motorTail, m_motorTail.dutyCycle - speed);
}

void Helicopter::motorTailDecreaseSpeed(float speed)
{
	//m_motorTail = m_motorTail + speed;
	DRV_PWM_setDutyCycle(&m_motorTail, m_motorTail.dutyCycle + speed);
}

int Helicopter::motorTailGetSpeed()
{
	//return 100 - (int(m_motorTail.read() * 100.f));
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

void Helicopter::process()
{
	i++;
	//DRV_UART_putchar(m_remotePC, 'a');
	/*float commandRotorMain = m_waveformMain->generate(m_currentTime)/100000000.f;
	//printf("main : %d\r\n", int(commandRotorMain*100));
	//printf("main : %lu at time : %lu\r\n", m_waveformMain->generate(m_currentTime), m_currentTime*m_Te);
	motorMainSetSpeed(commandRotorMain);

	float commandRotorTail = m_waveformTail->generate(m_currentTime)/100000000.f;
	//printf("tail : %d\r\n", int(commandRotorTail*100));
	motorTailSetSpeed(commandRotorTail);

	m_currentTime++;
	if(m_currentTime > m_Tsim)
		stop();*/
}
