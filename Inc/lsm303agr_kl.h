#ifndef LSM303AGR_KL_H_
#define LSM303AGR_KL_H_

#include <stdint.h>

#define LSM303AGR_MAG_ADDR 0x1E// 0b0011110
#define LSM303AGR_MAG_I2C_TIMEOUT 250

#define LSM303AGR_STATUS_REG_AUX  0x07
#define LSM303AGR_OUT_TEMP_L_A  0x0C
#define LSM303AGR_OUT_TEMP_H_A  0x0D
#define LSM303AGR_INT_COUNTER_REG_A  0x0E
#define LSM303AGR_ WHO_AM_I_A  0x0F
#define LSM303AGR_TEMP_CFG_REG_A  0x1F
#define LSM303AGR_CTRL_REG1_A  0x20
#define LSM303AGR_CTRL_REG2_A  0x21
#define LSM303AGR_CTRL_REG3_A  0x22
#define LSM303AGR_CTRL_REG4_A  0x23
#define LSM303AGR_CTRL_REG5_A  0x24
#define LSM303AGR_CTRL_REG6_A  0x25
#define LSM303AGR_REFERENCE_DATACAPTURE_A  0x26
#define LSM303AGR_STATUS_REG_A  0x27
#define LSM303AGR_OUT_X_L_A  0x28
#define LSM303AGR_OUT_X_H_A  0x29
#define LSM303AGR_OUT_Y_L_A  0x2A
#define LSM303AGR_OUT_Y_H_A  0x2B
#define LSM303AGR_OUT_Z_L_A  0x2C
#define LSM303AGR_OUT_Z_H_A  0x2D
#define LSM303AGR_FIFO_CTRL_REG_A  0x2E
#define LSM303AGR_FIFO_SRC_REG_A  0x2F
#define LSM303AGR_INT1_CFG_A  0x30
#define LSM303AGR_INT1_SRC_A  0x31
#define LSM303AGR_INT1_THS_A  0x32
#define LSM303AGR_INT1_DURATION_A  0x33
#define LSM303AGR_INT2_CFG_A  0x34
#define LSM303AGR_INT2_SRC_A  0x35
#define LSM303AGR_INT2_THS_A  0x36
#define LSM303AGR_INT2_DURATION_A  0x37
#define LSM303AGR_CLICK_CFG_A  0x38
#define LSM303AGR_CLICK_SRC_A  0x39
#define LSM303AGR_CLICK_THS_A  0x3A
#define LSM303AGR_TIME_LIMIT_A  0x3B
#define LSM303AGR_TIME_LATENCY_A  0x3C
#define LSM303AGR_TIME_WINDOW_A  0x3D
#define LSM303AGR_ACT_THS_A  0x3E
#define LSM303AGR_ACT_DUR_A  0x3F
#define LSM303AGR_OFFSET_X_REG_L_M  0x45
#define LSM303AGR_OFFSET_X_REG_H_M  0x46
#define LSM303AGR_OFFSET_Y_REG_L_M  0x47
#define LSM303AGR_OFFSET_Y_REG_H_M  0x48
#define LSM303AGR_OFFSET_Z_REG_L_M  0x49
#define LSM303AGR_OFFSET_Z_REG_H_M  0x4A
#define LSM303AGR_WHO_AM_I_M  0x4F
#define LSM303AGR_CFG_REG_A_M  0x60
#define LSM303AGR_CFG_REG_B_M  0x61
#define LSM303AGR_CFG_REG_C_M  0x62
#define LSM303AGR_INT_CTRL_REG_M  0x63
#define LSM303AGR_INT_SOURCE_REG_M  0x64
#define LSM303AGR_INT_THS_L_REG_M  0x65
#define LSM303AGR_INT_THS_H_REG_M  0x66
#define LSM303AGR_STATUS_REG_M  0x67
#define LSM303AGR_OUTX_L_REG_M  0x68
#define LSM303AGR_OUTX_H_REG_M  0x69
#define LSM303AGR_OUTY_L_REG_M  0x6A
#define LSM303AGR_OUTY_H_REG_M  0x6B
#define LSM303AGR_OUTZ_L_REG_M  0x6C
#define LSM303AGR_OUTZ_H_REG_M  0x6D

typedef enum
{
	// CTRL_REG1_A
	ODR0 = 4,
	LPen = 3,
	Zen = 2,
	Yen = 1,
	Xen = 0,

	// CTRL_REG3_A
	INT1_CLICK = 7,
	INT1_AOI1 = 6,
	INT1_AOI2 = 5,
	INT1_DRDY1 = 4,
	INT1_DRDY2 = 3,
	INT1_WTM = 2,
	INT1_OVERRUN = 1,

	// CTRL_REG4_A
	BDU = 7,
	BLE = 6,
	FS0 = 4,
	HR = 3,
	ST0 = 1,
	SPI_ENABLE = 0,

	// CTRL_REG5_A
	BOOT = 7,
	FIFO_EN = 6,
	LIR_IG1 = 3,
	D4D_IG1 = 2,
	LIR_IG2 = 1,
	D4D_IG2 = 0,

	// CTRL_REG6_A
	INT2_CLICK_EN = 7,
	I2_INT1 = 6,
	I2_INT2 = 5,
	BOOT_I2 = 4,
	P2_ACT = 3,
	H_LACTIVE = 1,

	// TEMP_CFG_REG_A
	TEMP_EN1 = 7,
	TEMP_EN0 = 6,
} RegisterBits;

typedef enum
{
	// CFG_REG_A_M
	MD0 = 0,
	MD1 = 1,
	MagODR0 = 2,
	LP = 4,
	SOFT_RST = 5,
	REBOOT = 6,
	COMP_TEMP_EN = 7,

	// CFG_REG_B_M
	LPF = 0,
	OFF_CANC = 1,
	Set_FREQ = 2,
	INT_on_DataOFF = 3,
	OFF_CANC_ONE_SHOT = 4,

	// CFG_REG_C_M
	INT_MAG = 0,
	Self_test = 1,
	MagBLE = 3,
	MagBDU = 4,
	I2C_DIS = 5,
	INT_MAG_PIN = 6,

	// INT_CTRL_REG_M
	IEN = 0,
	IEL = 1,
	IEA = 2,
	ZIEN = 5,
	YIEN = 6,
	XIEN = 7,

	// INT_SOURCE_REG_M
	INT = 0,
	MROI = 1,
	X_TH_S_Z = 2,
	N_TH_S_Y = 3,
	N_TH_S_X = 4,

	P_TH_S_Z = 5,
	P_TH_S_Y = 6,
	P_TH_S_X = 7,

	// STATUS_REG_M
	xda = 0,
	yda = 1,
	zda = 2,

	Zyxda = 3,

	Magxor = 4,
	Magyor = 5,
	Magzor = 5,

	Zyxor = 6,
} MagRegisterBits;

typedef enum
{
	MagLowPowerMode = 0b0, MagHighResMode = 0b1
} MagnetometerMode;

typedef enum
{
	PowerDown = 0b000,
	HrNormalLowPower1Hz = 0b0001,
	HrNormalLowPower10Hz = 0b0010,
	HrNormalLowPower25Hz = 0b0011,
	HrNormalLowPower50Hz = 0b0100,
	HrNormalLowPower100Hz = 0b0101,
	HrNormalLowPower200Hz = 0b0110,
	HrNormalLowPower400Hz = 0b0111,
	LowPower1k6Hz = 0b1000,
	HrNormal1k344LowPower5k376Hz = 0b1001
} AccelerometerODR;

typedef enum
{
	Hz10 = 0b00, Hz20 = 0b01, Hz50 = 0b10, Hz100 = 0b11,
} MagnetometerODR;

typedef enum
{
	Continuous = 0b00, Single = 0b01, IdleMode = 0b10,
} MagnetometerSystemMode;

typedef enum
{
	NoAxis = 0, X = 0b001, Y = 0b010, Z = 0b100, XY = X | Y, XZ = X | Z, YZ = Y
			| Z, XYZ = X | Y | Z
} Axes;

typedef enum
{
	MagX = 0b100, MagY = 0b010, MagZ = 0b001
} MagAxes;

typedef enum
{
	ZHigh = 0b00100000,
	ZLow = 0b00010000,
	YHigh = 0b00001000,
	YLow = 0b00000100,
	XHigh = 0b00000010,
	XLow = 0b00000001
} AxesEvents;

typedef enum
{
	OrCombination = 0b00000000,
	MovementRecognition = 0b01000000,
	AndCombination = 0b10000000,
	PositionRecognition = 0b11000000
} InterruptMode;

float LSM303AGR_readMagX();
float LSM303AGR_readMagY();
float LSM303AGR_readMagZ();
uint16_t LSM303AGR_readRawMagX();
uint16_t LSM303AGR_readRawMagY();
uint16_t LSM303AGR_readRawMagZ();
float mapDouble(float x, float in_min, float in_max, float out_min, float out_max);
float LSM303AGR_calcMag(int16_t value);
void LSM303AGR_magInit();
void LSM303AGR_setLPF();
//void LSM303AGR_disableMagnetometer();
void LSM303AGR_rebootMagnetometer();
void LSM303AGR_setRegisterBits(uint8_t reg, uint8_t byteValue);
void LSM303AGR_unsetRegisterBits(uint8_t reg, uint8_t byteValue);
//void LSM303AGR_enableMagnetometerInterrupt(uint8_t magAxesEvents, double threshold, bool highOnInterrupt);
void LSM303AGR_disableMagnetometerInterrupt();
uint8_t LSM303AGR_readRegister(uint8_t reg);
uint16_t LSM303AGR_readRegister16Bits(uint8_t reg);
void LSM303AGR_writeRegister(uint8_t reg, uint8_t value);

#endif
