/*
 * Helicopter.h
 *
 *  Created on: 9 févr. 2017
 *      Author: JulienCombattelli
 */

#ifndef HELICOPTER_H_
#define HELICOPTER_H_

#include "CommunicationProtocol.h"
#include "Waveform.h"
#include "FatFs/ff_gen_drv.h"

#include "PWM/Pwm_driver.h"
#include "I2C/I2c_driver.h"

#include "PWM/Pwm_init.h"
#include "I2C/I2c_init.h"

class Helicopter
{
public:

	Helicopter();

	void run();
	void stop();

	void motorMainSetSpeed(float speed);
	void motorMainIncreaseSpeed(float speed);
	void motorMainDecreaseSpeed(float speed);
	int  motorMainGetSpeed();

	void motorTailSetSpeed(float speed);
	void motorTailIncreaseSpeed(float speed);
	void motorTailDecreaseSpeed(float speed);
	int  motorTailGetSpeed();

	void setAnalog1(float value);
	void setAnalog2(float value);
	float getAnalog1();
	float getAnalog2();

private:

	friend void HAL_SYSTICK_Callback(void);

	void handleInitializationFrame();
	void handleSignalRotorMainFrame();
	void handleSignalRotorTailFrame();
	void handleStartFrame();

	void process();

	pwm_t m_motorMain;
	pwm_t m_motorTail;

	/*AnalogIn m_adc1;
	AnalogIn m_adc2;
	AnalogOut m_dac1;
	AnalogOut m_dac2;*/
	I2C_HandleTypeDef m_i2c;
	UART_HandleTypeDef m_remotePC;

	FATFS m_SDFatFs;
	FIL m_file;
	char m_SDPath[4];

	uint16_t m_Te;
	uint32_t m_Tsim;
	uint32_t m_currentTime;

	Waveform* m_waveformMain;
	Waveform* m_waveformTail;

	//Ticker m_ticker;

	bool m_isRunning;
};

#endif /* HELICOPTER_H_ */
