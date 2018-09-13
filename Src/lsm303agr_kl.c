#include <math.h>
#include "lsm303agr_kl.h"

#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;

#define _BV(bit) (1 << (bit))

float mapFloat(float x, float in_min, float in_max, float out_min,
		float out_max)
{
	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

float LSM303AGR_getMagFromScaledValue(int16_t value)
{
	return value * 1.5;
}

uint16_t LSM303AGR_readRawMagX()
{
	return (LSM303AGR_readRegister16Bits(LSM303AGR_OUTX_L_REG_M));
}

uint16_t LSM303AGR_readRawMagY()
{
	return (LSM303AGR_readRegister16Bits(LSM303AGR_OUTY_L_REG_M));
}

uint16_t LSM303AGR_readRawMagZ()
{
	return (LSM303AGR_readRegister16Bits(LSM303AGR_OUTZ_L_REG_M));
}

float LSM303AGR_readMagX()
{
	return LSM303AGR_getMagFromScaledValue(LSM303AGR_readRegister16Bits(LSM303AGR_OUTX_L_REG_M));
}

float LSM303AGR_readMagY()
{
	return LSM303AGR_getMagFromScaledValue(LSM303AGR_readRegister16Bits(LSM303AGR_OUTY_L_REG_M));
}
float LSM303AGR_readMagZ()
{
	return LSM303AGR_getMagFromScaledValue(LSM303AGR_readRegister16Bits(LSM303AGR_OUTZ_L_REG_M));
}

/*bool LSM303AGR_checkWhoAmI()
 {
 return (readAccelRegister(WHO_AM_I_A) == 0b00110011) &&
 (readMagRegister(WHO_AM_I_M) == 0b01000000);
 }*/

void LSM303AGR_magInit()
{
	MagnetometerMode mode = MagLowPowerMode;
	MagnetometerODR odr = Hz10;
	MagnetometerSystemMode systemMode = Continuous;

	// set odr, mode, systemMode
	LSM303AGR_writeRegister(LSM303AGR_CFG_REG_A_M, (odr << MagODR0) | systemMode);

	if (mode == MagLowPowerMode) {
		LSM303AGR_setRegisterBits(LSM303AGR_CFG_REG_A_M, _BV(LP));
	}
	else {
		LSM303AGR_unsetRegisterBits(LSM303AGR_CFG_REG_A_M, _BV(LP));
	}

	// Compensate temp
	LSM303AGR_setRegisterBits(LSM303AGR_CFG_REG_A_M, _BV(COMP_TEMP_EN));

	// disable hard-iron calibration
	LSM303AGR_writeRegister(LSM303AGR_OFFSET_X_REG_L_M, 0);
	LSM303AGR_writeRegister(LSM303AGR_OFFSET_X_REG_H_M, 0);
	LSM303AGR_writeRegister(LSM303AGR_OFFSET_Y_REG_L_M, 0);
	LSM303AGR_writeRegister(LSM303AGR_OFFSET_Y_REG_H_M, 0);
	LSM303AGR_writeRegister(LSM303AGR_OFFSET_Z_REG_L_M, 0);
	LSM303AGR_writeRegister(LSM303AGR_OFFSET_Z_REG_H_M, 0);

	// disable offset cancellation
	LSM303AGR_unsetRegisterBits(LSM303AGR_CFG_REG_B_M, _BV(OFF_CANC));

	LSM303AGR_setLPF();
}

void LSM303AGR_setLPF()
{

	LSM303AGR_setRegisterBits(LSM303AGR_CFG_REG_B_M, _BV(LPF));

}

/*void LSM303AGR_disableMagnetometer()
{
	LSM303AGR_enableMagnetometer(MagLowPowerMode, Hz10, IdleMode, false);
}*/

void LSM303AGR_rebootMagnetometer()
{
	LSM303AGR_writeRegister(LSM303AGR_CFG_REG_A_M, _BV(REBOOT));
}

void LSM303AGR_setRegisterBits(uint8_t reg, uint8_t byteValue)
{
	uint8_t value = LSM303AGR_readRegister(reg);
	value |= byteValue;
	LSM303AGR_writeRegister(reg, value);
}

void LSM303AGR_unsetRegisterBits(uint8_t reg, uint8_t byteValue)
{
	uint8_t value = LSM303AGR_readRegister(reg);
	value &= ~byteValue;
	LSM303AGR_writeRegister(reg, value);
}

/*void LSM303AGR_enableMagnetometerInterrupt(uint8_t magAxesEvents, double threshold, bool highOnInterrupt)
 {
 // threshold needs to be positive, because mag checks interrupts for -threshold and +threshold always
 if (threshold < 0) {
 threshold = -threshold;
 }

 // set axes
 LSM303AGR_writeRegister(INT_CTRL_REG_M, (magAxesEvents << ZIEN));

 // interrupt mode
 if (highOnInterrupt) {
 LSM303AGR_setMagRegisterBits(INT_CTRL_REG_M, _BV(IEA));
 }
 else {
 LSM303AGR_unsetMagRegisterBits(INT_CTRL_REG_M, _BV(IEA));
 }

 // set threshold registers
 int16_t ths = trunc(threshold / 1.5);
 LSM303AGR_writeRegister(INT_THS_L_REG_M, ths & 0x00FF);
 LSM303AGR_writeRegister(INT_THS_H_REG_M, (ths & 0xFF00) >> 8);

 // disable latching
 LSM303AGR_unsetMagRegisterBits(INT_CTRL_REG_M, _BV(IEL));

 // enable mag interrupt
 LSM303AGR_setMagRegisterBits(INT_CTRL_REG_M, _BV(IEN));

 // set mag interrupt to INT_MAG_PIN
 LSM303AGR_setMagRegisterBits(CFG_REG_C_M, _BV(INT_MAG_PIN));
 }*/

void LSM303AGR_disableMagnetometerInterrupt()
{
	// disable mag interrupt
	LSM303AGR_unsetRegisterBits(LSM303AGR_INT_CTRL_REG_M, _BV(IEN));
}

uint8_t LSM303AGR_readRegister(uint8_t reg)
{
	/*_wire.beginTransmission(deviceAddress);
	 _wire.write((uint8_t)reg);
	 _wire.endTransmission();

	 _wire.requestFrom(deviceAddress, 1);

	 return _wire.read();*/
	uint8_t result;

	HAL_I2C_Mem_Read(&hi2c1, (uint16_t)LSM303AGR_MAG_ADDR << 1, reg, 1, &result, 1, LSM303AGR_MAG_I2C_TIMEOUT);

	return result;
}

uint16_t LSM303AGR_readRegister16Bits(uint8_t reg)
{

	uint8_t temp;

	HAL_I2C_Mem_Read(&hi2c1, (uint16_t)LSM303AGR_MAG_ADDR << 1, reg, 1, &temp, 1, LSM303AGR_MAG_I2C_TIMEOUT);
	uint16_t result = temp;
	HAL_I2C_Mem_Read(&hi2c1, (uint16_t)LSM303AGR_MAG_ADDR << 1, reg+1, 1, &temp, 1, LSM303AGR_MAG_I2C_TIMEOUT);
	result |= temp << 8;

	return result;
}

void LSM303AGR_writeRegister(uint8_t reg, uint8_t value)
{
	uint8_t data[2];
	data[0] = reg;
	data[1] = value;
HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)LSM303AGR_MAG_ADDR << 1, data, 2, LSM303AGR_MAG_I2C_TIMEOUT);
}
