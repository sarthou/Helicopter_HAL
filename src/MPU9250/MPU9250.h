// MPU
// MPU.h (define registres)
// MPU.c : fonctions de base pour initialiser (dans le constructeur) et manipulation des valeurs => abstraction des registres

// autre fichier : fonctions haut niveau


/*
FSYNC = Frame synchronization digital input.
INT = Interrupt digital output.

Accel+Gyro=mode 7
*/
#ifndef MPU9250_H
#define MPU9250_H

#include "I2C\I2c_driver.h"
#include "I2C\I2c_init.h"

#define SELF_TEST_X_GYRO        0x00
#define SELF_TEST_Y_GYRO        0x01
#define SELF_TEST_Z_GYRO        0x02

#define SELF_TEST_X_ACCEL       0x0D //13
#define SELF_TEST_Y_ACCEL       0x0E //14
#define SELF_TEST_Z_ACCEL       0x0F //15

/* Registres 19 à 24 : Offset gyro
OffsetLSB=(X/Y/Z)_OFFSET_USER*4/(2^FS_SEL)
OffsetDPS=(X/Y/Z)_OFFSET_USER*4/(2^FS_SEL)/Gyro_sensitivity
Nominal : FS_SEL=0
Conditions : Gyro_Sensitivity=2^16LSB/500dps
A chaque mesure, les offsets sont enlevés à la valeur avant de la stocker.*/
#define XG_OFFSET_H             0x13 //19
#define XG_OFFSET_L             0x14 //20

#define YG_OFFSET_H             0x15
#define YG_OFFSET_L             0x16

#define ZG_OFFSET_H             0x17
#define ZG_OFFSET_L             0x18 //24

/* Sample rate divider
Divides the internal sample rate (see register CONFIG) to generate the sample
rate that controls sensor data output rate, FIFO sample rate.
NOTE: This register is only effective when Fchoice = 2’b11 (fchoice_b register bits
are 2’b00), and (0 < dlpf_cfg < 7), such that the average filter’s output is
selected (see chart below).
This is the update rate of sensor register.
SAMPLE_RATE= Internal_Sample_Rate / (1 + SMPLRT_DIV)
Data should be sampled at or above sample rate; SMPLRT_DIV is only used for1kHz internal sampling. */
#define SMPLRT_DIV              0x19

/*Configuration
Bit[6] : FIFO_MODE
Bit[5:3] : EXT_SYNC_SET
Bit[2:0] : DLPF_CFG
=> selon F_CHOICE (GYRO_CONFIG reg)*/
#define CONFIG                    0x1A //26

/*Gyroscope configuration
Bit[4:3] : GYRO_FS_SEL=gyro full scale select (250,500,1000,2000dps)
Bit[1:0] : Fchoice_b*/
#define GYRO_CONFIG             0x1B

/*Accelero configuration
Bit[4:3] : ACCEL_FS_SEL=accel full scale select (+-2,4,8,16g)*/
#define ACCEL_CONFIG            0x1C
/* Bit[3] : accel_fchoice_b
Bit[2:0] : A_DLPF_CFG=filtre passe-bas*/
#define ACCEL_CONFIG2           0x1D

/*Low Power Accelero ODR Control
Bit[3:0] : Iposc_clksel=frequence des mesures en mode Low Power*/
#define LP_ACCEL_ODR            0x1E

/*Wake_on Motion Threshold
threshold value for the Wake on Motion Interrupt for accelero
x/y/z axes. LSB=4mg, range=0mg-1020mg*/
#define WOM_THR                 0x1F //31

/*FIFO Enable
Bit[6] à Bit[4] : enable les mesures du gyro
Bit[3] : enable les mesures de l'accelero
Bit[2] à Bit[0] : config I2C*/
#define FIFO_EN                 0x23 //35

/*I2C Master Control */
#define I2C_MST_CTRL            0x24

#define I2C_SLV0_ADDR           0x25
#define I2C_SLV0_REG            0x26
#define I2C_SLV0_CTRL           0x27

#define I2C_SLV1_ADDR           0x28
#define I2C_SLV1_REG            0x29 //41
#define I2C_SLV1_CTRL           0x2A //42

#define I2C_SLV2_ADDR           0x2B
#define I2C_SLV2_REG            0x2C
#define I2C_SLV2_CTRL           0x2D

#define I2C_SLV3_ADDR           0x2E
#define I2C_SLV3_REG            0x2F //47
#define I2C_SLV3_CTRL           0x30 //48

#define I2C_SLV4_ADDR           0x31
#define I2C_SLV4_REG            0x32
#define I2C_SLV4_DO             0x33
#define I2C_SLV4_CTRL           0x34
#define I2C_SLV4_DI             0x35

#define I2C_MST_STATUS          0x36

/*INT Pin / Bypass enable configuration
Bit[7] : ACTL=pin INT active low si 1
Bit[6] : OPEN=open drain si 1, push-pull sinon
Bit[5] : LATCH_INT_EN : actif jusqu'au clear de
l'interruption si 1, 50us si 0
Bit[4] : INT_ANYRD_2CLEAR=interruption clear par n'importe
quelle lecture si 1, par lecture de INT_STATUS reg si 0 => a mettre a 0
Bit[3] : ACTL_FSYNC=FSYNC actif bas si 1
Bit[2] : FSYNC_INT_MODE_EN=enable la pin FSYNC pour être utilisée
comme interruption*/
#define INT_PIN_CFG             0x37 //55

/*Interrupt enable
Bit[4] : FIFO_OVERFLOW_EN=enable l'interruption pour l'overflow de
la fifo, sur la pin INT
Bit[3] : FSYNC_INT_EN=enable l'interruption sur Fsync à se propager
sur la pin INT
Bit[0] : RAW_RDY_EN=enable l'interruption RawSensorDataReady à se
propager sur la pin INT
*/
#define INT_ENABLE              0x38 //56

/*Interrupt status
Bit[6] : wake on motion interrupt (à désactiver)
Bit[4] : FIFO_OVERFLOW_INT
Bit[3] : FSYNC_INT
Bit[0] : RAW_DATA_RDY_INT */
#define INT_STATUS              0x3A //58

/*Registres 59 à 64 : mesures accelero */
#define ACCEL_XOUT_H            0x3B
#define ACCEL_XOUT_L            0x3C

#define ACCEL_YOUT_H            0x3D
#define ACCEL_YOUT_L            0x3E

#define ACCEL_ZOUT_H            0x3F //63
#define ACCEL_ZOUT_L            0x40 //64

/*Registres 65-66 : mesures température*/
#define TEMP_OUT_H              0x41
#define TEMP_OUT_L              0x42

/*Registres 67 à 72 : mesures Gyro*/
#define GYRO_XOUT_H             0x43
#define GYRO_XOUT_L             0x44

#define GYRO_YOUT_H             0x45
#define GYRO_YOUT_L             0x46

#define GYRO_ZOUT_H             0x47
#define GYRO_ZOUT_L             0x48

#define EXT_SENS_DATA_00        0x49 //73
#define EXT_SENS_DATA_01        0x4A //74
#define EXT_SENS_DATA_02        0x4B
#define EXT_SENS_DATA_03        0x4C
#define EXT_SENS_DATA_04        0x4D
#define EXT_SENS_DATA_05        0x4E //78
// ....
#define EXT_SENS_DATA_23        0x60 //96

#define I2C_SLV0_DO             0x63 //99
#define I2C_SLV1_DO             0x64
#define I2C_SLV2_DO             0x65
#define I2C_SLV3_DO             0x66
#define I2C_MST_DELAY_CTRL      0x67

/*Signal path reset
Bit[2] : GYRO_RST
Bit[1] : ACCEL_RST
Bit[0] : TEMP_RST*/
#define SIGNAL_PATH_RESET       0x68

/* Accelero interrupt control
=> pour Wake on motion*/
#define MOT_DETECT_CTRL         0x69 // ACCEL_INTEL_CTRL

/*User control
Bit[6] : FIFO_EN
Bit[5],Bit[4], Bit[1] => I2C
Bit[2] : FIFO_RST
Bit[0] : SIG_COND_RST=reset gyro+accelero + temp digital
signal path, clear les registres de tous les capteurs*/
#define USER_CTRL               0x6A

/*Power management 1 :
Bit[7] : H_RESET=reset des registres internes
Bit[6] : SLEEP
Bit[5] : CYCLE=quand CYCLE à 1,SLEEP et STANDBY à 0, alterne SLEEP et
prise de mesure à la fréquence fixée par le registre LP_ACCEL_ODR
Bit[2:0] : CLKSEL*/
#define PWR_MGMT_1              0x6B
/*Power management 2 :
pour désactiver X, Y ou Z de l'accelero ou du gyro*/
#define PWR_MGMT_2              0x6C //108

/*Registres 114 et 115 : FIFO count registers*/
#define FIFO_COUNTH             0x72 //114
#define FIFO_COUNTL             0x73
/*FIFO Read Write*/
#define FIFO_R_W                0x74

/*Who Am I=device ID*/
#define WHO_AM_I                0x75 //117

/*Registres 119,120,122,123,125,126 : accelero offsets */
#define XA_OFFSET_H             0x77 //119
#define XA_OFFSET_L             0x78 //120

#define YA_OFFSET_H             0x7A //122
#define YA_OFFSET_L             0x7B //123

#define ZA_OFFSET_H             0x7D //125
#define ZA_OFFSET_L             0x7E //126

enum GyroBandwith {bw_8800=8,bw_3600=7,bw_250=0,bw_184=1,bw_92=2,bw_41=3,bw_20=4,bw_10=5,bw_5=6};
enum GyroFullScale {fs_250=0,fs_500=1,fs_1000=2,fs_2000=3};
enum AccelBandwith {bw_1_046=8,bw_218_1=0,bw_99=2,bw_44_8=3,bw_21_2=4,bw_10_2=5,bw_5_05=6,bw_420=7};
enum AccelFullScale {fs_2=0,fs_4=1,fs_8=2,fs_16=3};
enum Axis {ZG=0,YG=1,XG=2,ZA=3,YA=4,XA=5};

/* classe MPU */
class MPU9250
{
public:
    MPU9250(I2C_HandleTypeDef* i2c)
    {
    	m_i2c=i2c;
    	MPU9250_I2C_init(m_i2c);
    }

    void MPU_setGyroOffset()
    {
        //offset X_h
        writeRegister(m_i2c,XG_OFFSET_H,readRegister(m_i2c,GYRO_XOUT_H));
        //offset X_l
        writeRegister(m_i2c,XG_OFFSET_L,readRegister(m_i2c,GYRO_XOUT_L));
        //offset Y_h
        writeRegister(m_i2c,YG_OFFSET_H,readRegister(m_i2c,GYRO_YOUT_H));
        //offset Y_l
        writeRegister(m_i2c,YG_OFFSET_L,readRegister(m_i2c,GYRO_YOUT_L));
        //offset Z_h
        writeRegister(m_i2c,ZG_OFFSET_H,readRegister(m_i2c,GYRO_ZOUT_H));
        //offset Z_l
        writeRegister(m_i2c,ZG_OFFSET_L,readRegister(m_i2c,GYRO_ZOUT_L));
    }

    void MPU_setAccelOffset()
    {
        uint16_t data;
        //X
        data=(readRegister(m_i2c,ACCEL_XOUT_H)<<8)|(readRegister(m_i2c,ACCEL_XOUT_L));
        writeRegister(m_i2c,XA_OFFSET_H,data>>7);
        writeRegister(m_i2c,XA_OFFSET_L,(readRegister(m_i2c,XA_OFFSET_L)&~(0x1))|(data<<1));

        data=(readRegister(m_i2c,ACCEL_YOUT_H)<<8)|(readRegister(m_i2c,ACCEL_YOUT_L));
        writeRegister(m_i2c,YA_OFFSET_H,data>>7);
        writeRegister(m_i2c,YA_OFFSET_L,(readRegister(m_i2c,YA_OFFSET_L)&~(0x1))|(data<<1));

        data=(readRegister(m_i2c,ACCEL_ZOUT_H)<<8)|(readRegister(m_i2c,ACCEL_ZOUT_L));
        writeRegister(m_i2c,ZA_OFFSET_H,data>>7);
        writeRegister(m_i2c,ZA_OFFSET_L,(readRegister(m_i2c,ZA_OFFSET_L)&~(0x1))|(data<<1));
    }

	uint8_t MPU_getSampleRateDivider()
	{
		return readRegister(m_i2c,SMPLRT_DIV);
	}

	void MPU_setSampleRateDivider (uint8_t div)
	{
		writeRegister(m_i2c,SMPLRT_DIV,div);
	}

	void MPU_setFifoMode(uint8_t fifo_mode)
	{
		//fifo_mode = 0 or 1
		writeRegister(m_i2c,CONFIG,(readRegister(m_i2c,CONFIG)&~(0x40))|(fifo_mode<<6));
	}
	void MPU_setFsyncSampled(uint8_t ext_sync_set)
	{
		writeRegister(m_i2c,CONFIG,(readRegister(m_i2c,CONFIG)&~(0x38))|(ext_sync_set<<3));
	}

	GyroFullScale MPU_getGyroFullScale()
	{
		uint8_t data=(readRegister(m_i2c,GYRO_CONFIG)&0x18);
		GyroFullScale fs;
		switch (data)
		{
		case 0:
			fs=fs_250;
			break;
		case 1:
			fs=fs_500;
			break;
		case 2:
			fs=fs_1000;
			break;
		case 3:
			fs=fs_2000;
			break;
		}
		return fs;
	}

	void MPU_setGyroFullScale(GyroFullScale fs)
	{
		writeRegister(m_i2c,GYRO_CONFIG,(readRegister(m_i2c,GYRO_CONFIG)&~(0x18))|(fs<<3));
	}

	GyroBandwith MPU_getGyroBandwith()
	{
		uint8_t fChoice_b=(readRegister(m_i2c,GYRO_CONFIG)&0x3);
		uint8_t dlpf=(readRegister(m_i2c,CONFIG)&0x7);
		GyroBandwith res;
		if (fChoice_b==2)
		{
			res=bw_3600;
		}
		else if (fChoice_b>0)
		{
			res=bw_8800;
		}
		else
		{
			res=(GyroBandwith) dlpf;
		}
		return res;
	}
	void MPU_setGyroBandwith(GyroBandwith bw, bool dlpf_on)
	{
		if (dlpf_on==false)
		{
			if (bw==8800)
			{
				writeRegister(m_i2c,GYRO_CONFIG,readRegister(m_i2c,GYRO_CONFIG)|(0x1));
			}
			else
			{
				writeRegister(m_i2c,GYRO_CONFIG,(readRegister(m_i2c,GYRO_CONFIG)&~(0x3))|(0x2));
			}
		}
		else
		{
			writeRegister(m_i2c,CONFIG,(readRegister(m_i2c,CONFIG)&~(0x3))|bw);
		}
	}

	AccelFullScale MPU_getAccelFullScale()
	{
		return (AccelFullScale)(readRegister(m_i2c,ACCEL_CONFIG)&0x18);
	}
	void MPU_setAccelFullScale(AccelFullScale fs)
	{
		writeRegister(m_i2c,ACCEL_CONFIG,(readRegister(m_i2c,ACCEL_CONFIG)&~(0x18))|(fs<<3));
	}

	AccelBandwith MPU_getAccelBandwith()
	{
		uint8_t fChoice_b = (readRegister(m_i2c,ACCEL_CONFIG2)&0x8);
		uint16_t dlpf = (readRegister(m_i2c,ACCEL_CONFIG2)&0x7);
		AccelBandwith res;
		if (fChoice_b==1)
		{
			res=bw_1_046;
		}
		else
		{
			res=(AccelBandwith) dlpf;
		}
		return res;
	}

	void MPU_setAccelBandwith(AccelBandwith bw, bool dlpf_on)
	{
		if ((dlpf_on==false)||(bw==8))
		{
			//f_choice_b=1
			writeRegister(m_i2c,ACCEL_CONFIG2,readRegister(m_i2c,ACCEL_CONFIG2)|(1<<3));
		}
		else
		{
			//f_choice_b=0
			writeRegister(m_i2c,ACCEL_CONFIG2,readRegister(m_i2c,ACCEL_CONFIG2)&~(1<<3));
			writeRegister(m_i2c,ACCEL_CONFIG2,(readRegister(m_i2c,ACCEL_CONFIG2)&~(0x7))|bw);
		}
	}

	uint8_t MPU_getINTactl()
	{
		return (readRegister(m_i2c,INT_PIN_CFG)&0x80);
	}

	void MPU_setINTactl(uint8_t actl)
	{
		//actl=1 => active low
		writeRegister(m_i2c,INT_PIN_CFG,(readRegister(m_i2c,INT_PIN_CFG)&~(0x80))|(actl<<7));
	}

	uint8_t MPU_getINTopendrain()
	{
		return (readRegister(m_i2c,INT_PIN_CFG)&0x40);
	}

	void MPU_setINTopendrain(uint8_t opendrain)
	{
		writeRegister(m_i2c,INT_PIN_CFG,(readRegister(m_i2c,INT_PIN_CFG)&~(0x40))|(opendrain<<6));
	}

	uint8_t MPU_getINTlatchIntEn()
	{
		return (readRegister(m_i2c,INT_PIN_CFG)&0x20);
	}

	void MPU_setINTlatchIntEn(uint8_t levelHeld)
	{
		writeRegister(m_i2c,INT_PIN_CFG,(readRegister(m_i2c,INT_PIN_CFG)&~(0x20))|(levelHeld<<5));
	}

	uint8_t MPU_getINTanyrd()
	{
		return (readRegister(m_i2c,INT_PIN_CFG)&0x10);
	}

	void MPU_setINTanyrd(uint8_t anyread)
	{
		writeRegister(m_i2c,INT_PIN_CFG,(readRegister(m_i2c,INT_PIN_CFG)&~(0x10))|(anyread<<4));
	}

	uint8_t MPU_getITRawRdyEn()
	{
		return (readRegister(m_i2c,INT_ENABLE)&0x1);
	}

	void MPU_setITRawRdyEn(uint8_t enable)
	{
		writeRegister(m_i2c,INT_ENABLE,(readRegister(m_i2c,INT_ENABLE)&~(0x1))|enable);
	}

	uint8_t MPU_getITStatusRawDataRdy()
	{
		return (readRegister(m_i2c,INT_STATUS)&0x1);
	}

	uint8_t MPU_getWaitForEs()
	{
		return (readRegister(m_i2c,I2C_MST_CTRL)&0x40);
	}

	void MPU_setWaitForEs(uint8_t delay)
	{
		writeRegister(m_i2c,I2C_MST_CTRL,(readRegister(m_i2c,I2C_MST_CTRL)&~(0x40))|(delay<<6));
	}

	uint8_t MPU_getSignalPathGyroReset()
	{
		return (readRegister(m_i2c,SIGNAL_PATH_RESET)&0x4);
	}

	void MPU_setSignalPathGyroReset(uint8_t reset)
	{
		writeRegister(m_i2c,SIGNAL_PATH_RESET,(readRegister(m_i2c,SIGNAL_PATH_RESET)&~(0x4))|(reset<<2));
	}

	uint8_t MPU_getSignalPathAcceleroReset()
	{
		return (readRegister(m_i2c,SIGNAL_PATH_RESET)&0x2);
	}

	void MPU_setSignalPathAcceleroReset(uint8_t reset)
	{
		writeRegister(m_i2c,SIGNAL_PATH_RESET,(readRegister(m_i2c,SIGNAL_PATH_RESET)&~(0x2))|(reset<<1));
	}

	uint8_t MPU_getUSRCTRLsigCondRst()
	{
		return (readRegister(m_i2c,USER_CTRL)&0x1);
	}

	void MPU_setUSRCTRLsigCondRst(uint8_t reset)
	{
		writeRegister(m_i2c,USER_CTRL,(readRegister(m_i2c,USER_CTRL)&~(0x1))|reset);
	}

	void MPU_ResetDefault()
	{
		writeRegister(m_i2c,PWR_MGMT_1,readRegister(m_i2c,PWR_MGMT_1)|0x80);
	}

	uint8_t MPU_getSleepMode()
	{
		return (readRegister(m_i2c,PWR_MGMT_1)&0x40);
	}

	void MPU_setSleepMode(uint8_t sleep)
	{
		writeRegister(m_i2c,PWR_MGMT_1,(readRegister(m_i2c,PWR_MGMT_1)&~(0x40))|(sleep<<6));
	}

	uint8_t MPU_getCycleMode()
	{
		return (readRegister(m_i2c,PWR_MGMT_1)&0x20);
	}

	void MPU_setCycleMode(uint8_t cycle)
	{
		writeRegister(m_i2c,PWR_MGMT_1,(readRegister(m_i2c,PWR_MGMT_1)&~(0x20))|(cycle<<5));
	}

	uint8_t MPU_getGyroStandby()
	{
		return (readRegister(m_i2c,PWR_MGMT_1)&0x10);
	}

	void MPU_setGyroStandby(uint8_t standby)
	{
		writeRegister(m_i2c,PWR_MGMT_1,(readRegister(m_i2c,PWR_MGMT_1)&~(0x10))|(standby<<4));
	}

	uint8_t MPU_getClkSel()
	{
		return (readRegister(m_i2c,PWR_MGMT_1)&0x7);
	}

	void MPU_setClkSel(uint8_t code)
	{
		writeRegister(m_i2c,PWR_MGMT_1,(readRegister(m_i2c,PWR_MGMT_1)&~(0x7))|code);
	}

	uint8_t MPU_getWhoAmI()
	{
		return readRegister(m_i2c,WHO_AM_I);
	}

	uint16_t MPU_getAccelX()
	{
		uint16_t dataL=readRegister(m_i2c,ACCEL_XOUT_L);
		uint16_t dataH=readRegister(m_i2c,ACCEL_XOUT_H);
		return ((dataH<<8)|dataL);
	}

	uint16_t MPU_getAccelY()
	{
		uint16_t dataL=readRegister(m_i2c,ACCEL_YOUT_L);
		uint16_t dataH=readRegister(m_i2c,ACCEL_YOUT_H);
		return ((dataH<<8)|dataL);
	}

	uint16_t MPU_getAccelZ()
	{
		uint16_t dataL=readRegister(m_i2c,ACCEL_ZOUT_L);
		uint16_t dataH=readRegister(m_i2c,ACCEL_ZOUT_H);
		return ((dataH<<8)|dataL);
	}

	uint16_t MPU_getGyroX()
	{
		uint16_t dataL=readRegister(m_i2c,GYRO_XOUT_L);
		uint16_t dataH=readRegister(m_i2c,GYRO_XOUT_H);
		return ((dataH<<8)|dataL);
	}

	uint16_t MPU_getGyroY()
	{
		uint16_t dataL=readRegister(m_i2c,GYRO_YOUT_L);
		uint16_t dataH=readRegister(m_i2c,GYRO_YOUT_H);
		return ((dataH<<8)|dataL);
	}

	uint16_t MPU_getGyroZ()
	{
		uint16_t dataL=readRegister(m_i2c,GYRO_ZOUT_L);
		uint16_t dataH=readRegister(m_i2c,GYRO_ZOUT_H);
		return ((dataH<<8)|dataL);
	}

	void MPU_disableAxis(Axis ax)
	{
		writeRegister(m_i2c,PWR_MGMT_2,readRegister(m_i2c,PWR_MGMT_2)|(1<<ax));
	}

private:
    I2C_HandleTypeDef* m_i2c;
    // autres attributs : garder certains registres en mémoire pour ne pas les relire à chaque fois


// fonction read register
    uint8_t readRegister(I2C_HandleTypeDef* i2c, uint8_t reg_addr)
    {
    	return DRV_I2C_read_byte(i2c, reg_addr);

    }
// fonction write register (taille=nb_octets)
    void writeRegister(I2C_HandleTypeDef* i2c,uint8_t reg_addr, uint8_t data)
    {
    	DRV_I2C_write_byte(i2c,reg_addr,data);

    }
};

#endif
