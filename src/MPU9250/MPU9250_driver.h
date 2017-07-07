/*
 * MPU9250_driver.h
 *
 *  Created on: 15 avr. 2017
 *      Author: Julia
 */

#ifndef SRC_MPU9250_MPU9250_DRIVER_H_
#define SRC_MPU9250_MPU9250_DRIVER_H_

#include "I2C\I2c_driver.h"
#include "I2C\I2c_init.h"

struct MPU_values_t
{
	uint16_t acceleroX;
	uint16_t acceleroY;
	uint16_t acceleroZ;

	uint16_t gyroX;
	uint16_t gyroY;
	uint16_t gyroZ;
};


enum GyroBandwith {
	bw_8800=8,
	bw_3600=7,
	bw_250=0,
	bw_184=1,
	bw_92=2,
	bw_41=3,
	bw_20=4,
	bw_10=5,
	bw_5=6};

enum GyroFullScale {
	fs_250=0,
	fs_500=1,
	fs_1000=2,
	fs_2000=3};

enum AccelBandwith {
	bw_1_046=8,
	bw_218_1=0,
	bw_99=2,
	bw_44_8=3,
	bw_21_2=4,
	bw_10_2=5,
	bw_5_05=6,
	bw_420=7};

enum AccelFullScale {
	fs_2=0,
	fs_4=1,
	fs_8=2,
	fs_16=3};

enum Axis {
	ZG=0,
	YG=1,
	XG=2,
	ZA=3,
	YA=4,
	XA=5};

class MPU9250_driver
{
public:
	MPU9250_driver(I2C_HandleTypeDef* i2c);
	void init();

	// WHO AM I
	uint8_t MPU_getWhoAmI();

	// DISABLE AXIS
	void MPU_disableAxis(Axis ax);

	// GYROSCOPE
	void MPU_setGyroOffset();

	GyroFullScale MPU_getGyroFullScale();
	void MPU_setGyroFullScale(GyroFullScale fs);

	GyroBandwith MPU_getGyroBandwith();

	void MPU_setGyroBandwith(GyroBandwith bw, bool dlpf_on);

	void readGyro();
	uint16_t MPU_getGyroX();
	uint16_t MPU_getGyroY();
	uint16_t MPU_getGyroZ();

	float MPU_getGyroX_f();
	float MPU_getGyroY_f();
	float MPU_getGyroZ_f();


	// ACCELEROMETRE
	void MPU_setAccelOffset();

	AccelFullScale MPU_getAccelFullScale();

	void MPU_setAccelFullScale(AccelFullScale fs);

	AccelBandwith MPU_getAccelBandwith();

	void MPU_setAccelBandwith(AccelBandwith bw, bool dlpf_on);

	void readAccel();
	uint16_t MPU_getAccelX();
	uint16_t MPU_getAccelY();
	uint16_t MPU_getAccelZ();

	float MPU_getAccelX_f();
	float MPU_getAccelY_f();
	float MPU_getAccelZ_f();


	// CONFIGURATION Sample Rate
	uint8_t MPU_getSampleRateDivider();
	void MPU_setSampleRateDivider (uint8_t div);

	// CONFIGURATIOn FIFO
	void MPU_setFifoMode(uint8_t fifo_mode);

	//CONFIGURATION FSYNC
	void MPU_setFsyncSampled(uint8_t ext_sync_set);


	//CONFIGURATION PIN INT
	uint8_t MPU_getINTactl();
	void MPU_setINTactl(uint8_t actl);
	uint8_t MPU_getINTopendrain();
	void MPU_setINTopendrain(uint8_t opendrain);
	uint8_t MPU_getINTlatchIntEn();
	void MPU_setINTlatchIntEn(uint8_t levelHeld);
	uint8_t MPU_getINTanyrd();
	void MPU_setINTanyrd(uint8_t anyread);
	uint8_t MPU_getITRawRdyEn();
	void MPU_setITRawRdyEn(uint8_t enable);
	uint8_t MPU_getITStatusRawDataRdy();

	//CONFIGURATION MASTER I2C
	uint8_t MPU_getWaitForEs();
	void MPU_setWaitForEs(uint8_t delay);

	//CONFIGURATION SIGNAL PATH ET RESET
	uint8_t MPU_getSignalPathGyroReset();
	void MPU_setSignalPathGyroReset(uint8_t reset);
	uint8_t MPU_getSignalPathAcceleroReset();
	void MPU_setSignalPathAcceleroReset(uint8_t reset);

	//CONFIGURATION USER CONTROL
	uint8_t MPU_getUSRCTRLsigCondRst();
	void MPU_setUSRCTRLsigCondRst(uint8_t reset);

	//POWER MANAGEMENT
	void MPU_ResetDefault();
	uint8_t MPU_getSleepMode();
	void MPU_setSleepMode(uint8_t sleep);
	uint8_t MPU_getCycleMode();
	void MPU_setCycleMode(uint8_t cycle);
	uint8_t MPU_getGyroStandby();
	void MPU_setGyroStandby(uint8_t standby);
	uint8_t MPU_getClkSel();
	void MPU_setClkSel(uint8_t code);



private:
    I2C_HandleTypeDef* m_i2c;
    GyroFullScale m_gyroFs;
    AccelFullScale m_acceFs;

    uint8_t readRegister(uint8_t reg_addr);
    uint16_t readWordRegister(uint8_t reg_addr);

    void writeRegister(uint8_t reg_addr, uint8_t data);

    MPU_values_t m_values;

};

#endif /* SRC_MPU9250_MPU9250_DRIVER_H_ */
