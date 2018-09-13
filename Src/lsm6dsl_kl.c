/******************************************************************************
SparkFunLSM6DS3.cpp
LSM6DS3 Arduino and Teensy Driver
Marshall Taylor @ SparkFun Electronics
May 20, 2015
https://github.com/sparkfun/LSM6DS3_Breakout
https://github.com/sparkfun/SparkFun_LSM6DS3_Arduino_Library
Resources:
Uses Wire.h for i2c operation
Uses SPI.h for SPI operation
Either can be omitted if not used
Development environment specifics:
Arduino IDE 1.6.4
Teensy loader 1.23
This code is released under the [MIT License](http://opensource.org/licenses/MIT).
Please review the LICENSE.md file included with this example. If you have any questions
or concerns with licensing, please contact techsupport@sparkfun.com.
Distributed as-is; no warranty is given.
******************************************************************************/

//See SparkFunLSM6DS3.h for additional topology notes.

#include "stdint.h"
#include "stm32f4xx_hal.h"
#include "lsm6dsl_kl.h"

extern I2C_HandleTypeDef hi2c1;

static uint8_t gyroRange, accelRange;
//****************************************************************************//
//
//  ReadRegisterRegion
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//    length -- number of bytes to read
//
//  Note:  Does not know if the target memory space is an array or not, or
//    if there is the array is big enough.  if the variable passed is only
//    two bytes long and 3 bytes are requested, this will over-write some
//    other memory!
//
//****************************************************************************//
status_t LSM6DS3_readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	status_t returnError = IMU_SUCCESS;

    HAL_I2C_Mem_Read(&hi2c1, (uint16_t)LSM6DS3_ADDR << 1, offset, 1, outputPointer, length, LSM6DS3_I2C_TIMEOUT);

	return returnError;
}

//****************************************************************************//
//
//  ReadRegister
//
//  Parameters:
//    *outputPointer -- Pass &variable (address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LSM6DS3_readRegister(uint8_t* outputPointer, uint8_t offset)
{
	uint8_t result;

	HAL_I2C_Mem_Read(&hi2c1, (uint16_t)LSM6DS3_ADDR << 1, offset, 1, &result, 1, LSM6DS3_I2C_TIMEOUT);

	status_t returnError = IMU_SUCCESS;

	*outputPointer = result;
	return returnError;
}

//****************************************************************************//
//
//  readRegisterInt16
//
//  Parameters:
//    *outputPointer -- Pass &variable (base address of) to save read data to
//    offset -- register to read
//
//****************************************************************************//
status_t LSM6DS3_readRegisterInt16( int16_t* outputPointer, uint8_t offset )
{

	uint8_t myBuffer[2];

	status_t returnError = HAL_I2C_Mem_Read(&hi2c1, (uint16_t)LSM6DS3_ADDR << 1, offset, 1, myBuffer, 2, LSM6DS3_I2C_TIMEOUT);

	int16_t output = (int16_t)myBuffer[0] | (int16_t)(myBuffer[1] << 8);

	*outputPointer = output;
	return returnError;
}

//****************************************************************************//
//
//  writeRegister
//
//  Parameters:
//    offset -- register to write
//    dataToWrite -- 8 bit data to write to register
//
//****************************************************************************//
status_t LSM6DS3_writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	status_t returnError = IMU_SUCCESS;

	uint8_t data[2];
	data[0] = offset;
	data[1] = dataToWrite;
	HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)LSM6DS3_ADDR << 1, data, 2, LSM6DS3_I2C_TIMEOUT);

	return returnError;
}

status_t LSM6DS3_embeddedPage( void )
{
	status_t returnError = LSM6DS3_writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x80);

	return returnError;
}

status_t LSM6DS3_basePage( void )
{
	status_t returnError = LSM6DS3_writeRegister(LSM6DS3_ACC_GYRO_RAM_ACCESS, 0x00);

	return returnError;
}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "myIMU.settings.commInterface = SPI_MODE;" or
//  "myIMU.settings.accelEnabled = 1;" to configure before calling .begin();
//
//****************************************************************************//
status_t LSM6DS3_begin()
{
	// Set the device parameters
	accelRange = LSM6DS3_ACC_GYRO_FS_XL_16g;
	uint8_t accelBandWidth = LSM6DS3_ACC_GYRO_BW_XL_50Hz;
	uint8_t accelOdr = LSM6DS3_ACC_GYRO_ODR_XL_13Hz;

	gyroRange = LSM6DS3_ACC_GYRO_FS_G_2000dps;
	uint8_t gyroOdr = LSM6DS3_ACC_GYRO_ODR_G_13Hz;

	uint8_t dataToWrite = 0;  //Temporary variable

	//Begin the inherited core.  This gets the physical wires connected
	status_t returnError = IMU_SUCCESS;
	uint8_t readCheck;
	LSM6DS3_readRegister(&readCheck, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);
	if( readCheck != 0x69 )
	{
		returnError = IMU_HW_ERROR;
	}

	//Setup the accelerometer******************************
	dataToWrite = 0; //Start Fresh!
	dataToWrite |= accelBandWidth;
	dataToWrite |= accelRange;
	dataToWrite |= accelOdr;

	//Now, write the patched together data
	LSM6DS3_writeRegister(LSM6DS3_ACC_GYRO_CTRL1_XL, dataToWrite);

	//Set the ODR bit
	LSM6DS3_readRegister(&dataToWrite, LSM6DS3_ACC_GYRO_CTRL4_C);
	dataToWrite &= ~((uint8_t)LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED);
	dataToWrite |= LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED;
	LSM6DS3_writeRegister(LSM6DS3_ACC_GYRO_CTRL4_C, dataToWrite);

	//Setup the gyroscope**********************************************
	dataToWrite = 0; //Start Fresh!
	dataToWrite |= gyroRange;
	dataToWrite |= gyroOdr;

	//Write the byte
	LSM6DS3_writeRegister(LSM6DS3_ACC_GYRO_CTRL2_G, dataToWrite);

	//Return WHO AM I reg
	uint8_t result;
	LSM6DS3_readRegister(&result, LSM6DS3_ACC_GYRO_WHO_AM_I_REG);

	return returnError;
}

//****************************************************************************//
//
//  Accelerometer section
//
//****************************************************************************//
int16_t LSM6DS3_readRawAccelX( void )
{
	int16_t output;
	status_t errorLevel = LSM6DS3_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_XL );
	/*if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}*/
	return output;
}
float LSM6DS3_readFloatAccelX( void )
{
	float output = LSM6DS3_calcAccel(LSM6DS3_readRawAccelX());
	return output;
}

int16_t LSM6DS3_readRawAccelY( void )
{
	int16_t output;
	status_t errorLevel = LSM6DS3_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_XL );
	/*if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}*/
	return output;
}
float LSM6DS3_readFloatAccelY( void )
{
	float output = LSM6DS3_calcAccel(LSM6DS3_readRawAccelY());
	return output;
}

int16_t LSM6DS3_readRawAccelZ( void )
{
	int16_t output;
	status_t errorLevel = LSM6DS3_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_XL );
	/*if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}*/
	return output;
}
float LSM6DS3_readFloatAccelZ( void )
{
	float output = LSM6DS3_calcAccel(LSM6DS3_readRawAccelZ());
	return output;
}

float LSM6DS3_calcAccel( int16_t input )
{
	uint8_t temp_range;
	if(accelRange == LSM6DS3_ACC_GYRO_FS_XL_16g){
		temp_range = 16;
	}
	float output = (float)input * 0.061 * (temp_range >> 1) / 1000;
	return output;
}

//****************************************************************************//
//
//  Gyroscope section
//
//****************************************************************************//
int16_t LSM6DS3_readRawGyroX( void )
{
	int16_t output;
	status_t errorLevel = LSM6DS3_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTX_L_G );
	/*if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}*/
	return output;
}
float LSM6DS3_readFloatGyroX( void )
{
	float output = LSM6DS3_calcGyro(LSM6DS3_readRawGyroX());
	return output;
}

int16_t LSM6DS3_readRawGyroY( void )
{
	int16_t output;
	status_t errorLevel = LSM6DS3_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTY_L_G );
	/*if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}*/
	return output;
}
float LSM6DS3_readFloatGyroY( void )
{
	float output = LSM6DS3_calcGyro(LSM6DS3_readRawGyroY());
	return output;
}

int16_t LSM6DS3_readRawGyroZ( void )
{
	int16_t output;
	status_t errorLevel = LSM6DS3_readRegisterInt16( &output, LSM6DS3_ACC_GYRO_OUTZ_L_G );
	/*if( errorLevel != IMU_SUCCESS )
	{
		if( errorLevel == IMU_ALL_ONES_WARNING )
		{
			allOnesCounter++;
		}
		else
		{
			nonSuccessCounter++;
		}
	}*/
	return output;
}
float LSM6DS3_readFloatGyroZ( void )
{
	float output = LSM6DS3_calcGyro(LSM6DS3_readRawGyroZ());
	return output;
}

float LSM6DS3_calcGyro( int16_t input )
{
	uint8_t temp_range;
	// TODO: add mode options
	if(gyroRange == LSM6DS3_ACC_GYRO_FS_G_2000dps){
		temp_range = 2000;
	}

	uint8_t gyroRangeDivisor = temp_range / 125;
	if ( temp_range == 245 ) {
			gyroRangeDivisor = 2;
		}

	float output = (float)input * 4.375 * (gyroRangeDivisor) / 1000;
	return output;
}

