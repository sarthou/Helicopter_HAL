/*
 * Helicopter.h
 *
 *  Created on: 9 févr. 2017
 *      Author: JulienCombattelli
 */

#ifndef HELICOPTER_H_
#define HELICOPTER_H_

#include <Helicopter/Hcp.h>
#include <Helicopter/Waveform.h>
#include <FatFs/ff_gen_drv.h>

#include <Pwm/Pwm_driver.h>
#include <Pwm/Pwm_init.h>

#include <Uart/Uart_driver.h>
#include <Uart/Uart_init.h>

#include <ADC/Adc_driver.h>
#include <ADC/Adc_init.h>

#include <I2C/I2c_driver.h>
#include <I2C/I2c_init.h>

struct sensors_t
{
	uint8_t mainMotor;
	uint8_t tailMotor;
	uint8_t pitchPot;

	uint8_t acceleroX;
	uint8_t acceleroY;
	uint8_t acceleroZ;

	uint8_t gyroX;
	uint8_t gyroY;
	uint8_t gyroZ;


	uint8_t nbSensors;
};

class Helicopter
{
public:

	Helicopter();
	~Helicopter();

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

	void setSysTickTimer(uint32_t period_us);
	void enableSysTickHandler();
	void disableSysTickHandler();

	void handleManualRotorMainFrame();
	void handleManualRotorTailFrame();
	void handleInitializationFrame();
	void handleSignalRotorMainFrame();
	void handleSignalRotorTailFrame();
	void handleSignalSensorsFrame();
	void handleStartFrame();

	void process();

	DRV_PWM_TypeDef m_motorMain;
	DRV_PWM_TypeDef m_motorTail;

	//AnalogIn m_adc1;
	//AnalogIn m_adc2;
	//AnalogOut m_dac1;
	//AnalogOut m_dac2;

	I2C_HandleTypeDef m_i2c;
	DRV_UART_TypeDef m_remotePc;
	ADC_HandleTypeDef m_adc;

	FATFS m_SDFatFs;
	FIL m_file;
	char m_SDPath[4];

	uint16_t m_Te;
	uint32_t m_Tsim;
	uint32_t m_currentTime;

	Waveform* m_waveformMain;
	Waveform* m_waveformTail;

	sensors_t sensors;

	bool m_isRunning;
	bool m_isTimeToSendData;
};

#endif /* HELICOPTER_H_ */
