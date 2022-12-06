#ifndef STM32_MPU9250_H
#define STM32_MPU9250_H

// Includes
#include <stm32f10x_i2c.h>
#include <stm32f10x_gpio.h>
#include <stm32f10x_rcc.h>
#include "stdbool.h"
#include "ssd1306.h"
/**

  MPU9250 power options are configurable by PWM_MGMT_1 register

  Bit 0-2: Clock selection
    000 (0) Internal oscillator
    001 (1) Auto select best one
    ...
    110 (6) Internal oscillator
    111 (7) Stop the clock

  Bit 6: Sleep
    0 Wake up
    1 Sleep

*/

enum MPU9250_ClockConfig
{
    MPU9250_Clk_Internal = 0,
    MPU_Clk_Auto,
    MPU_Clk_Disable = 7,
};

/*

  ACCEL and GYRO range
  Bit 3-4 of ACCEL/GYRO CONFIG register

*/

enum MPU9250_AccelRange
{
    ACCEL_2G = 0,
    ACCEL_4G,
    ACCEL_8G,
    ACCEL_16G,
};

enum MPU9250_GyroRange
{
    GYRO_250DPS = 0,
    GYRO_500DPS,
    GYRO_1000DPS,
    GYRO_2000DPS,
};

// --  Function declaration -- //

// MPU9250 Functions
void MPU9250_init(void);
void MPU9250_readImuData(uint8_t *destination);
void MPU9250_SetClockSource(uint8_t source);
void MPU9250_SetAccelRange(uint8_t range);
void MPU9250_SetGyroRange(uint8_t range);
void MPU9250_WakeMeUp(void);
void MPU9250_enableBypass(void);
void readAccelData(int16_t*);
void readGyroData(int16_t*);
void MPU9250_calibrateOffsets(uint16_t samples);
void MPU9250_setGyroBias(int16_t bias_x, int16_t bias_y, int16_t bias_z);
void MPU9250_setAccelBias(int16_t bias_x, int16_t bias_y, int16_t bias_z);


// I2C Functions
void I2C_init(void);
void I2C_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data);
void I2C_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data);
void I2C_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data);
void I2C_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data);
void I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 writeAddr);
void I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead);



// MPU9250 Pin Assignment

#define MPU9250_I2C                  I2C1
#define MPU9250_I2C_RCC_Periph       RCC_APB1Periph_I2C1
#define MPU9250_I2C_Port             GPIOB
#define MPU9250_I2C_SCL_Pin          GPIO_Pin_6
#define MPU9250_I2C_SDA_Pin          GPIO_Pin_7
#define MPU9250_I2C_RCC_Port         RCC_APB2Periph_GPIOB
#define MPU9250_I2C_Speed            100000 // 100kHz standard mode


/**
 * @brief    MPU9250 and AK8963 refer to IMU and Magnetometer respectively with different I2C slave addresses
 */


// --  MPU9250 Registers address -- //
// Register values are 0x00 at reset except PWR_MGMT_1 (0x01) and WHO_AM_I (0x71)

#define MPU9250_ADDRESS (0x68) << 1

// Configurations
#define MPU9250_CONFIG 0x1A
#define MPU9250_GYRO_CONFIG 0x1B
#define MPU9250_ACCEL_CONFIG 0x1C
#define MPU9250_ACCEL_CONFIG_2 0x1D

// Gyro bias registers
#define MPU9250_XG_OFFSET_H 0x13
#define MPU9250_XG_OFFSET_L 0x14
#define MPU9250_YG_OFFSET_H 0x15
#define MPU9250_YG_OFFSET_L 0x16
#define MPU9250_ZG_OFFSET_H 0x17
#define MPU9250_ZG_OFFSET_L 0x18

// Acceleromater bias registers
#define MPU9250_XA_OFFSET_H 0x77
#define MPU9250_XA_OFFSET_L 0x78
#define MPU9250_YA_OFFSET_H 0x7A
#define MPU9250_YA_OFFSET_L 0x7B
#define MPU9250_ZA_OFFSET_H 0x7C
#define MPU9250_ZA_OFFSET_L 0x7D

// Accelerometer output
#define MPU9250_ACCEL_XOUT_H 0x3B
#define MPU9250_ACCEL_XOUT_L 0x3C
#define MPU9250_ACCEL_YOUT_H 0x3D
#define MPU9250_ACCEL_YOUT_L 0x3E
#define MPU9250_ACCEL_ZOUT_H 0x3F
#define MPU9250_ACCEL_ZOUT_L 0x40

// Temperature output
#define MPU9250_TEMP_OUT_H 0x41
#define MPU9250_TEMP_OUT_L 0x42

// Gyro output
#define MPU9250_GYRO_XOUT_H 0x43
#define MPU9250_GYRO_XOUT_L 0x44
#define MPU9250_GYRO_YOUT_H 0x45
#define MPU9250_GYRO_YOUT_L 0x46
#define MPU9250_GYRO_ZOUT_H 0x47
#define MPU9250_GYRO_ZOUT_L 0x48

// Power Management
#define MPU9250_PWR_MGMT_1 0x6B
#define MPU9250_PWR_MGMT_2 0x6C

// Some other management
#define MPU9250_INT_PIN_CFG 0x37



// -- AK8963 Registers address -- //

#define AK8963_ADDRESS (0x0C) << 1

// Magnetometer output
#define AK8963_HX_L 0x03
#define AK8963_HX_H 0x04
#define AK8963_HY_L 0x05
#define AK8963_HY_H 0x06
#define AK8963_HZ_L 0x07
#define AK8963_HZ_H 0x08

#define AK8963_CNTL1 0x0A

// Sensitivity adjustment values
#define AK8963_ASAX 0x10
#define AK8963_ASAY 0x11
#define AK8963_ASAZ 0x12

enum AK8963_mode
{
    AK8963_powerDown = 0,
    AK8963_singleMeasurement,
    AK8963_continuous1,
    AK8963_continuous2 = 6,
    AK8963_extTrigger = 4,
    AK8963_selfTest = 8,
    AK8963_fuseROM = 15,
};

#define DELTA_THRESHOLD 2
#define FILTER_COEFF 8

void AK8963_init(void);
void AK8963_setMode(uint8_t mode);
void AK8963_enable16bit(void);
void AK8963_readMagData(uint16_t * data, uint8_t * calibration);
void AK8963_calibrate(uint16_t samples);


#endif //STM32_MPU9250_H
