#ifndef LSM6DSL_KL_H
#define LSM6DSL_KL_H

#include <stdint.h>


#define LSM6DS3_ADDR	(0x6B) //0x6A
#define LSM6DS3_I2C_TIMEOUT	250
/** registers */
/************** Device Register  *******************/
#define LSM6DS3_ACC_GYRO_TEST_PAGE  			0X00
#define LSM6DS3_ACC_GYRO_RAM_ACCESS  			0X01
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_TIME  		0X04
#define LSM6DS3_ACC_GYRO_SENSOR_SYNC_EN  		0X05
#define LSM6DS3_ACC_GYRO_FIFO_CTRL1  			0X06
#define LSM6DS3_ACC_GYRO_FIFO_CTRL2  			0X07
#define LSM6DS3_ACC_GYRO_FIFO_CTRL3  			0X08
#define LSM6DS3_ACC_GYRO_FIFO_CTRL4  			0X09
#define LSM6DS3_ACC_GYRO_FIFO_CTRL5  			0X0A
#define LSM6DS3_ACC_GYRO_ORIENT_CFG_G  			0X0B
#define LSM6DS3_ACC_GYRO_REFERENCE_G  			0X0C
#define LSM6DS3_ACC_GYRO_INT1_CTRL  			0X0D
#define LSM6DS3_ACC_GYRO_INT2_CTRL  			0X0E
#define LSM6DS3_ACC_GYRO_WHO_AM_I_REG  			0X0F
#define LSM6DS3_ACC_GYRO_CTRL1_XL  			0X10
#define LSM6DS3_ACC_GYRO_CTRL2_G  			0X11
#define LSM6DS3_ACC_GYRO_CTRL3_C  			0X12
#define LSM6DS3_ACC_GYRO_CTRL4_C  			0X13
#define LSM6DS3_ACC_GYRO_CTRL5_C  			0X14
#define LSM6DS3_ACC_GYRO_CTRL6_G  			0X15
#define LSM6DS3_ACC_GYRO_CTRL7_G  			0X16
#define LSM6DS3_ACC_GYRO_CTRL8_XL  			0X17
#define LSM6DS3_ACC_GYRO_CTRL9_XL  			0X18
#define LSM6DS3_ACC_GYRO_CTRL10_C  			0X19
#define LSM6DS3_ACC_GYRO_MASTER_CONFIG  		0X1A
#define LSM6DS3_ACC_GYRO_WAKE_UP_SRC  			0X1B
#define LSM6DS3_ACC_GYRO_TAP_SRC  			0X1C
#define LSM6DS3_ACC_GYRO_D6D_SRC  			0X1D
#define LSM6DS3_ACC_GYRO_STATUS_REG  			0X1E
#define LSM6DS3_ACC_GYRO_OUT_TEMP_L  			0X20
#define LSM6DS3_ACC_GYRO_OUT_TEMP_H  			0X21
#define LSM6DS3_ACC_GYRO_OUTX_L_G  			0X22
#define LSM6DS3_ACC_GYRO_OUTX_H_G  			0X23
#define LSM6DS3_ACC_GYRO_OUTY_L_G  			0X24
#define LSM6DS3_ACC_GYRO_OUTY_H_G  			0X25
#define LSM6DS3_ACC_GYRO_OUTZ_L_G  			0X26
#define LSM6DS3_ACC_GYRO_OUTZ_H_G  			0X27
#define LSM6DS3_ACC_GYRO_OUTX_L_XL  			0X28
#define LSM6DS3_ACC_GYRO_OUTX_H_XL  			0X29
#define LSM6DS3_ACC_GYRO_OUTY_L_XL  			0X2A
#define LSM6DS3_ACC_GYRO_OUTY_H_XL  			0X2B
#define LSM6DS3_ACC_GYRO_OUTZ_L_XL  			0X2C
#define LSM6DS3_ACC_GYRO_OUTZ_H_XL  			0X2D
#define LSM6DS3_ACC_GYRO_SENSORHUB1_REG  		0X2E
#define LSM6DS3_ACC_GYRO_SENSORHUB2_REG  		0X2F
#define LSM6DS3_ACC_GYRO_SENSORHUB3_REG  		0X30
#define LSM6DS3_ACC_GYRO_SENSORHUB4_REG  		0X31
#define LSM6DS3_ACC_GYRO_SENSORHUB5_REG  		0X32
#define LSM6DS3_ACC_GYRO_SENSORHUB6_REG  		0X33
#define LSM6DS3_ACC_GYRO_SENSORHUB7_REG  		0X34
#define LSM6DS3_ACC_GYRO_SENSORHUB8_REG  		0X35
#define LSM6DS3_ACC_GYRO_SENSORHUB9_REG  		0X36
#define LSM6DS3_ACC_GYRO_SENSORHUB10_REG  		0X37
#define LSM6DS3_ACC_GYRO_SENSORHUB11_REG  		0X38
#define LSM6DS3_ACC_GYRO_SENSORHUB12_REG  		0X39
#define LSM6DS3_ACC_GYRO_FIFO_STATUS1  			0X3A
#define LSM6DS3_ACC_GYRO_FIFO_STATUS2  			0X3B
#define LSM6DS3_ACC_GYRO_FIFO_STATUS3  			0X3C
#define LSM6DS3_ACC_GYRO_FIFO_STATUS4  			0X3D
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_L  		0X3E
#define LSM6DS3_ACC_GYRO_FIFO_DATA_OUT_H  		0X3F
#define LSM6DS3_ACC_GYRO_TIMESTAMP0_REG  		0X40
#define LSM6DS3_ACC_GYRO_TIMESTAMP1_REG  		0X41
#define LSM6DS3_ACC_GYRO_TIMESTAMP2_REG  		0X42
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_L  		0X4B
#define LSM6DS3_ACC_GYRO_STEP_COUNTER_H  		0X4C
#define LSM6DS3_ACC_GYRO_FUNC_SRC  			0X53
#define LSM6DS3_ACC_GYRO_TAP_CFG1  			0X58
#define LSM6DS3_ACC_GYRO_TAP_THS_6D  			0X59
#define LSM6DS3_ACC_GYRO_INT_DUR2  			0X5A
#define LSM6DS3_ACC_GYRO_WAKE_UP_THS  			0X5B
#define LSM6DS3_ACC_GYRO_WAKE_UP_DUR  			0X5C
#define LSM6DS3_ACC_GYRO_FREE_FALL  			0X5D
#define LSM6DS3_ACC_GYRO_MD1_CFG  			0X5E
#define LSM6DS3_ACC_GYRO_MD2_CFG  			0X5F

#define  	LSM6DS3_ACC_GYRO_WHO_AM_I_BIT_MASK  	0xFF
#define  	LSM6DS3_ACC_GYRO_WHO_AM_I_BIT_POSITION  	0

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: BW_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS3_ACC_GYRO_BW_XL_400Hz 		 = 0x00,
	LSM6DS3_ACC_GYRO_BW_XL_200Hz 		 = 0x01,
	LSM6DS3_ACC_GYRO_BW_XL_100Hz 		 = 0x02,
	LSM6DS3_ACC_GYRO_BW_XL_50Hz 		 = 0x03,
} LSM6DS3_ACC_GYRO_BW_XL_t;

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: FS_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS3_ACC_GYRO_FS_XL_2g 		 = 0x00,
	LSM6DS3_ACC_GYRO_FS_XL_16g 		 = 0x04,
	LSM6DS3_ACC_GYRO_FS_XL_4g 		 = 0x08,
	LSM6DS3_ACC_GYRO_FS_XL_8g 		 = 0x0C,
} LSM6DS3_ACC_GYRO_FS_XL_t;

/*******************************************************************************
* Register      : CTRL1_XL
* Address       : 0X10
* Bit Group Name: ODR_XL
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS3_ACC_GYRO_ODR_XL_POWER_DOWN 		 = 0x00,
	LSM6DS3_ACC_GYRO_ODR_XL_13Hz 		         = 0x10,
	LSM6DS3_ACC_GYRO_ODR_XL_26Hz 		         = 0x20,
	LSM6DS3_ACC_GYRO_ODR_XL_52Hz 		         = 0x30,
	LSM6DS3_ACC_GYRO_ODR_XL_104Hz 		 = 0x40,
	LSM6DS3_ACC_GYRO_ODR_XL_208Hz 		 = 0x50,
	LSM6DS3_ACC_GYRO_ODR_XL_416Hz 		 = 0x60,
	LSM6DS3_ACC_GYRO_ODR_XL_833Hz 		 = 0x70,
	LSM6DS3_ACC_GYRO_ODR_XL_1660Hz 		 = 0x80,
	LSM6DS3_ACC_GYRO_ODR_XL_3330Hz 		 = 0x90,
	LSM6DS3_ACC_GYRO_ODR_XL_6660Hz 		 = 0xA0,
	LSM6DS3_ACC_GYRO_ODR_XL_13330Hz 		 = 0xB0,
} LSM6DS3_ACC_GYRO_ODR_XL_t;

/*******************************************************************************
* Register      : CTRL4_C
* Address       : 0X13
* Bit Group Name: BW_SCAL_ODR
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS3_ACC_GYRO_BW_SCAL_ODR_DISABLED 		 = 0x00,
	LSM6DS3_ACC_GYRO_BW_SCAL_ODR_ENABLED 		 = 0x80,
} LSM6DS3_ACC_GYRO_BW_SCAL_ODR_t;

/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: FS_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS3_ACC_GYRO_FS_G_245dps 		 = 0x00,
	LSM6DS3_ACC_GYRO_FS_G_500dps 		 = 0x04,
	LSM6DS3_ACC_GYRO_FS_G_1000dps 		 = 0x08,
	LSM6DS3_ACC_GYRO_FS_G_2000dps 		 = 0x0C,
} LSM6DS3_ACC_GYRO_FS_G_t;
/*******************************************************************************
* Register      : CTRL2_G
* Address       : 0X11
* Bit Group Name: ODR_G
* Permission    : RW
*******************************************************************************/
typedef enum {
	LSM6DS3_ACC_GYRO_ODR_G_POWER_DOWN 		 = 0x00,
	LSM6DS3_ACC_GYRO_ODR_G_13Hz 		 = 0x10,
	LSM6DS3_ACC_GYRO_ODR_G_26Hz 		 = 0x20,
	LSM6DS3_ACC_GYRO_ODR_G_52Hz 		 = 0x30,
	LSM6DS3_ACC_GYRO_ODR_G_104Hz 		 = 0x40,
	LSM6DS3_ACC_GYRO_ODR_G_208Hz 		 = 0x50,
	LSM6DS3_ACC_GYRO_ODR_G_416Hz 		 = 0x60,
	LSM6DS3_ACC_GYRO_ODR_G_833Hz 		 = 0x70,
	LSM6DS3_ACC_GYRO_ODR_G_1660Hz 		 = 0x80,
} LSM6DS3_ACC_GYRO_ODR_G_t;
/** enums */
typedef enum {
    LSM6DSL_ACC_GYRO_ODR_XL_POWER_DOWN  = 0x00,
    LSM6DSL_ACC_GYRO_ODR_XL_13Hz        = 0x10,
    LSM6DSL_ACC_GYRO_ODR_XL_26Hz        = 0x20,
    LSM6DSL_ACC_GYRO_ODR_XL_52Hz        = 0x30,
    LSM6DSL_ACC_GYRO_ODR_XL_104Hz 		= 0x40,
    LSM6DSL_ACC_GYRO_ODR_XL_208Hz 		= 0x50,
    LSM6DSL_ACC_GYRO_ODR_XL_416Hz 		= 0x60,
    LSM6DSL_ACC_GYRO_ODR_XL_833Hz 		= 0x70,
    LSM6DSL_ACC_GYRO_ODR_XL_1660Hz 		= 0x80,
    LSM6DSL_ACC_GYRO_ODR_XL_3330Hz 		= 0x90,
    LSM6DSL_ACC_GYRO_ODR_XL_6660Hz 		= 0xA0,
} LSM6DSL_ACC_ODR_t;

typedef enum {
    LSM6DSL_ACC_GYRO_FS_XL_2g       = 0x00,
    LSM6DSL_ACC_GYRO_FS_XL_4g       = 0x08,
    LSM6DSL_ACC_GYRO_FS_XL_8g       = 0x0C,
    LSM6DSL_ACC_GYRO_FS_XL_16g      = 0x04,
} LSM6DSL_ACC_FS_t;

typedef enum {
    LSM6DSL_ACC_GYRO_ODR_G_POWER_DOWN   = 0x00,
    LSM6DSL_ACC_GYRO_ODR_G_13Hz         = 0x10,
    LSM6DSL_ACC_GYRO_ODR_G_26Hz         = 0x20,
    LSM6DSL_ACC_GYRO_ODR_G_52Hz         = 0x30,
    LSM6DSL_ACC_GYRO_ODR_G_104Hz 		= 0x40,
    LSM6DSL_ACC_GYRO_ODR_G_208Hz 		= 0x50,
    LSM6DSL_ACC_GYRO_ODR_G_416Hz 		= 0x60,
    LSM6DSL_ACC_GYRO_ODR_G_833Hz 		= 0x70,
    LSM6DSL_ACC_GYRO_ODR_G_1660Hz 		= 0x80,
    LSM6DSL_ACC_GYRO_ODR_G_3330Hz 		= 0x90,
    LSM6DSL_ACC_GYRO_ODR_G_6660Hz 		= 0xA0,
} LSM6DSL_GYRO_ODR_t;


typedef enum {
    LSM6DSL_ACC_GYRO_FS_G_125dps        = 0x02,
    LSM6DSL_ACC_GYRO_FS_G_245dps        = 0x00,
    LSM6DSL_ACC_GYRO_FS_G_500dps        = 0x04,
    LSM6DSL_ACC_GYRO_FS_G_1000dps       = 0x08,
    LSM6DSL_ACC_GYRO_FS_G_2000dps       = 0x0C,
} LSM6DSL_GYRO_FS_t;

// Return error
typedef enum
{
	IMU_SUCCESS,
	IMU_HW_ERROR,
	IMU_NOT_SUPPORTED,
	IMU_GENERIC_ERROR,
	IMU_OUT_OF_BOUNDS,
	IMU_ALL_ONES_WARNING,
} status_t;

status_t LSM6DS3_readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length);
status_t LSM6DS3_readRegister(uint8_t* outputPointer, uint8_t offset);
status_t LSM6DS3_readRegisterInt16( int16_t* outputPointer, uint8_t offset );
status_t LSM6DS3_writeRegister(uint8_t offset, uint8_t dataToWrite);
status_t LSM6DS3_embeddedPage( void );
status_t LSM6DS3_basePage( void );
status_t LSM6DS3_begin();
int16_t LSM6DS3_readRawAccX( void );
float LSM6DS3_readFloatAccX( void );
int16_t LSM6DS3_readRawAccY( void );
float LSM6DS3_readFloatAccY( void );
int16_t LSM6DS3_readRawAccZ( void );
float LSM6DS3_readFloatAccZ( void );
float LSM6DS3_calcAcc( int16_t input );
int16_t LSM6DS3_readRawGyrX( void );
float LSM6DS3_readFloatGyrX( void );
int16_t LSM6DS3_readRawGyrY( void );
float LSM6DS3_readFloatGyrY( void );
int16_t LSM6DS3_readRawGyrZ( void );
float LSM6DS3_readFloatGyrZ( void );
float LSM6DS3_calcGyr( int16_t input );

#endif
