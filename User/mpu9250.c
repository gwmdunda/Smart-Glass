#include "mpu9250.h"

bool I2C_initialized = true;
u8 magSensitivity[3];

u16 magBias[3];
double magScale[3];

// -- MPU9250 functions -- //

void Delay_ms(int duration)
{
	duration *= 1000;
		while(duration--) 
		{
			int i=0x02;				
			while(i--)
			__asm("nop");
		}
}

void LongDelay(u32 nCount)
{
  for(; nCount != 0; nCount--);
}

void MPU9250_init()
{
    if(I2C_initialized)
    {

        MPU9250_SetClockSource(MPU9250_Clk_Internal);
        Delay_ms(100);
				
        // Set the clock source
        MPU9250_SetClockSource(MPU_Clk_Auto);
				
				// Set accel range
        MPU9250_SetAccelRange(ACCEL_2G);
        
        I2C_WriteBits(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG_2, 0, 2, 3);

        // Set gyro range
        MPU9250_SetGyroRange(GYRO_250DPS);

        // Wake up device
        MPU9250_WakeMeUp();

        Delay_ms(100);

        // Calibrate gyro and accel bias
        MPU9250_calibrateOffsets(30);
/*

        // Enable I2C bypass so AK8963 can be controllled as I2C slave
        MPU9250_enableBypass();
				*/
        Delay_ms(100);
    }
    else
    {
        I2C_init();
        MPU9250_init();
    }

}

void MPU9250_calibrateOffsets(uint16_t samples)
{
    int16_t accelBias[] = {0,0,0};
    int16_t gyroBias[] = {0,0,0};
		int i, j;
    int16_t gyroSensitivity = 131; // 131 raw value/deg/sec cuz 32767 / 250
    int16_t accelSensitivity = 16384; // 16384 raw value/g cuz 32767 / 2

		int16_t accelTemp[3] = {0,0,0};
		int16_t gyroTemp[3] = {0,0,0};
	
    for (i = 0; i < samples; ++i)
    {
        readAccelData(accelTemp);
        readGyroData(gyroTemp);

        for (j = 0; j < 3; ++j)
        {
            accelBias[j] += accelTemp[j];
            gyroBias[j] += gyroTemp[j];
        }
        Delay_ms(10);
    }
    for (i = 0; i < 3; ++i)
    {
        accelBias[i] /= samples;
        gyroBias[i] /= samples;
    }

    // Remove gravity from z axis accelerometer
    if(accelBias[2] > 0)
        accelBias[2] -= accelSensitivity;
    else
        accelBias[2] += accelSensitivity;

    // Write biases to offset gyro and accel registers
    MPU9250_setGyroBias(gyroBias[0], gyroBias[1], gyroBias[2]);
    MPU9250_setAccelBias(0, 0, accelBias[2]);
}

void MPU9250_setAccelBias(int16_t bias_x, int16_t bias_y, int16_t bias_z)
{
    int16_t accel_trim [3];
    uint8_t data[6];
    uint8_t temp_mask = ~(0x01);
		int i;
	
    I2C_BufferRead(MPU9250_ADDRESS, data, MPU9250_XA_OFFSET_H, 2);
    I2C_BufferRead(MPU9250_ADDRESS, &data[2], MPU9250_YA_OFFSET_H, 4);

    for(i = 0; i < 3; i++)
    {
        accel_trim[i] = (int16_t)data[2 * i] << 8 | data[2 * i + 1];
    }

    accel_trim[0] -= bias_x/8;
    accel_trim[1] -= bias_y/8;
    accel_trim[2] -= bias_z/8;

    for(i = 0; i < 3; i++)
    {
        data[2 * i] = (accel_trim[i] >> 8) & 0xff;

        // need to mask the LSB for temperature compensation -> refer to datasheet
        data[2 * i +1] = (accel_trim[i]) & temp_mask;

    }

    I2C_ByteWrite(MPU9250_ADDRESS, &data[0], MPU9250_XA_OFFSET_H);
    I2C_ByteWrite(MPU9250_ADDRESS, &data[1], MPU9250_XA_OFFSET_L);
    I2C_ByteWrite(MPU9250_ADDRESS, &data[2], MPU9250_YA_OFFSET_H);
    I2C_ByteWrite(MPU9250_ADDRESS, &data[3], MPU9250_YA_OFFSET_L);
    I2C_ByteWrite(MPU9250_ADDRESS, &data[4], MPU9250_ZA_OFFSET_H);
    I2C_ByteWrite(MPU9250_ADDRESS, &data[5], MPU9250_ZA_OFFSET_L);

}

void MPU9250_setGyroBias(int16_t bias_x, int16_t bias_y, int16_t bias_z)
{
    uint8_t biases[6];

    biases[0] = (-bias_x/4 >> 8)  & 0xff;
    biases[1] = (-bias_x/4)       & 0xff;
    biases[2] = (-bias_y/4 >> 8)  & 0xff;
    biases[3] = (-bias_y/4)       & 0xff;
    biases[4] = (-bias_z/4 >> 8)  & 0xff;
    biases[5] = (-bias_z/4)       & 0xff;

    I2C_ByteWrite(MPU9250_ADDRESS, &biases[0], MPU9250_XG_OFFSET_H);
    I2C_ByteWrite(MPU9250_ADDRESS, &biases[1], MPU9250_XG_OFFSET_L);
    I2C_ByteWrite(MPU9250_ADDRESS, &biases[2], MPU9250_YG_OFFSET_H);
    I2C_ByteWrite(MPU9250_ADDRESS, &biases[3], MPU9250_YG_OFFSET_L);
    I2C_ByteWrite(MPU9250_ADDRESS, &biases[4], MPU9250_ZG_OFFSET_H);
    I2C_ByteWrite(MPU9250_ADDRESS, &biases[5], MPU9250_ZG_OFFSET_L);
}

void MPU9250_readImuData(uint8_t *destination)
{
    I2C_BufferRead(MPU9250_ADDRESS, destination, MPU9250_ACCEL_XOUT_H, 14);
}

void MPU9250_SetClockSource(uint8_t source)
{
    I2C_WriteBits(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 0, 3, source);
}

void MPU9250_SetAccelRange(uint8_t range)
{
    I2C_WriteBits(MPU9250_ADDRESS, MPU9250_ACCEL_CONFIG, 3, 2, range);
}

void MPU9250_SetGyroRange(uint8_t range)
{
    I2C_WriteBits(MPU9250_ADDRESS, MPU9250_GYRO_CONFIG, 3, 2, range);
}

void MPU9250_WakeMeUp()
{
    I2C_WriteBit(MPU9250_ADDRESS, MPU9250_PWR_MGMT_1, 6, 0);
}

void MPU9250_enableBypass()
{
    I2C_WriteBit(MPU9250_ADDRESS, MPU9250_INT_PIN_CFG, 1, 1);
}

void AK8963_init()
{
    if(I2C_initialized)
    {
        AK8963_setMode(AK8963_powerDown);
        Delay_ms(100);

        AK8963_setMode(AK8963_fuseROM);
        Delay_ms(100);

        I2C_BufferRead(AK8963_ADDRESS, magSensitivity, AK8963_ASAX, 3);

        AK8963_setMode(AK8963_powerDown);
        Delay_ms(100);

        // Set bit 4 in CNTL1 register to 1 to enable 16 bit output
        // continuous mode 2 for 100Hz sample rate
        // refer to p.51 in register map
        AK8963_setMode(AK8963_continuous2 | (0x01 << 4));
        Delay_ms(100);

        AK8963_calibrate(1000); // might be too much
        Delay_ms(100);

    }
    else
    {
        I2C_init();
        AK8963_init();
    }
}

void readAccelData(int16_t *accelData)
{
    uint8_t rawData[6];
    I2C_BufferRead(MPU9250_ADDRESS, rawData, MPU9250_ACCEL_XOUT_H, 6);

    // Join the low bits and high bits
    // Accel is in big endian
    accelData[0] = ((int16_t)rawData[0] << 8) | rawData[1];
    accelData[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    accelData[2] = ((int16_t)rawData[4] << 8) | rawData[5];

}

void readGyroData(int16_t *gyroData)
{
    uint8_t rawData[6];
    I2C_BufferRead(MPU9250_ADDRESS, rawData, MPU9250_GYRO_XOUT_H, 6);

    // Join the low bits and high bits
    // Gyro is in big endian
    gyroData[0] = ((int16_t)rawData[0] << 8) | rawData[1];
    gyroData[1] = ((int16_t)rawData[2] << 8) | rawData[3];
    gyroData[2] = ((int16_t)rawData[4] << 8) | rawData[5];

}

void readMagData(int16_t *magData)
{
    uint8_t rawData[6];
    I2C_BufferRead(AK8963_ADDRESS, rawData, AK8963_HX_L, 6);

    // Join the low bits and high bits
    // Magnetometer is in little endian
    magData[0] = ((int16_t)rawData[1] << 8) | rawData[0];
    magData[1] = ((int16_t)rawData[3] << 8) | rawData[2];
    magData[2] = ((int16_t)rawData[5] << 8) | rawData[4];

    magData[0] = (((int16_t)(magSensitivity[0]) + 128) * magData[0]) / 256;
    magData[1] = (((int16_t)(magSensitivity[1]) + 128) * magData[1]) / 256;
    magData[2] = (((int16_t)(magSensitivity[2]) + 128) * magData[2]) / 256;

}
void AK8963_calibrate(uint16_t samples)
{
    int16_t max[3], min[3], filter[3];
    int16_t delta = 0;
		int16_t frame_delta = 0;
		int count = 0;
		int16_t magTemp[3];
		double avg;
		int i;
	
    readMagData(magTemp);


    for(i = 0; i < 3; ++i){
        max[i] = magTemp[i];
        min[i] = magTemp[i];
        filter[i] = 0;
    }

    for(count = 0; count < samples;) {
        readMagData(magTemp);
        for(i = 0; i < 3; ++i){
            filter[i] = ((FILTER_COEFF - 1) * filter[i] + magTemp[i]) / FILTER_COEFF;
        }

    
        for(i = 0; i < 3; ++i) {
            if(filter[i] > max[i]) {
                delta = filter[i] - max[i];
                max[i] = filter[i];
            }
            if (delta > frame_delta) {
                frame_delta = delta;
            }

            if(filter[i] < min[i]) {
                delta = min[i] - filter[i];
                min[i] = filter[i];
            }
            if (delta > frame_delta) {
                frame_delta = delta;
            }
        }

        if (frame_delta > DELTA_THRESHOLD)
            count = 0;
        else
            ++count;
    
        Delay_ms(20);
    }

    for(i = 0; i < 3; ++i) {
        magBias[i] = (max[i] + min[i]) / 2;
        magScale[i] = (max[i] - min[i]) / 2;
    }

    avg = (magScale[0] + magScale[1] + magScale[2]) / 3;
    for(i = 0; i < 3; ++i)
        magScale[i] = avg / magScale[i];

}

void AK8963_readMagData(uint16_t* data, uint8_t* calibration)
{
		int i;
		readMagData(data);
    for(i = 0; i < 3; ++i)
        calibration[i] = magSensitivity[i];
}

void AK8963_setMode(uint8_t mode)
{
    I2C_ByteWrite(AK8963_ADDRESS, &mode,AK8963_CNTL1);
}

void AK8963_enable16bit()
{
    I2C_WriteBit(AK8963_ADDRESS, AK8963_CNTL1, 4, 1);
}


//-- I2C related functions from John's library --//

void I2C_init()
{

    I2C_InitTypeDef I2C_InitStruct;
    GPIO_InitTypeDef GPIO_InitStruct;

    //Enable peripheral bus
    RCC_APB1PeriphClockCmd(MPU9250_I2C_RCC_Periph, ENABLE);
    RCC_APB2PeriphClockCmd(MPU9250_I2C_RCC_Port, ENABLE);

    //GPIO Init
    GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_InitStruct.GPIO_Pin = MPU9250_I2C_SCL_Pin | MPU9250_I2C_SDA_Pin;
    GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(MPU9250_I2C_Port, &GPIO_InitStruct);

    //I2C Init
    I2C_InitStruct.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStruct.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStruct.I2C_OwnAddress1 = 0x00; // not relevant in master mode ;)
    I2C_InitStruct.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStruct.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStruct.I2C_ClockSpeed = MPU9250_I2C_Speed;

    I2C_Init(MPU9250_I2C, &I2C_InitStruct);

    I2C_Cmd(MPU9250_I2C, ENABLE);

    I2C_initialized = true;

}

void I2C_WriteBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data)
{
    //      010 value to write
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    // 00011100 mask byte
    // 10101111 original value (sample)
    // 10100011 original & ~mask
    // 10101011 masked | value
    uint8_t tmp;
    uint8_t mask;
	
		I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    data <<= (bitStart - length + 1); // shift data into correct position
    data &= mask; // zero all non-important bits in data
    tmp &= ~(mask); // zero all important bits in existing byte
    tmp |= data; // combine data with existing byte
    I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}




void I2C_WriteBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data)
{
    uint8_t tmp;
    I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    tmp = (data != 0) ? (tmp | (1 << bitNum)) : (tmp & ~(1 << bitNum));
    I2C_ByteWrite(slaveAddr, &tmp, regAddr);
}

void I2C_ReadBits(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data)
{
    // 01101001 read byte
    // 76543210 bit numbers
    //    xxx   args: bitStart=4, length=3
    //    010   masked
    //   -> 010 shifted
    uint8_t tmp;
		uint8_t mask;  
		I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    mask = ((1 << length) - 1) << (bitStart - length + 1);
    tmp &= mask;
    tmp >>= (bitStart - length + 1);
    *data = tmp;
}

void I2C_ReadBit(uint8_t slaveAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data)
{
    uint8_t tmp;
    I2C_BufferRead(slaveAddr, &tmp, regAddr, 1);
    *data = tmp & (1 << bitNum);
}


void I2C_ByteWrite(u8 slaveAddr, u8* pBuffer, u8 writeAddr)
{
    // ENTR_CRT_SECTION();

    /* Send START condition */
    I2C_GenerateSTART(MPU9250_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU6050 address for write */
    I2C_Send7bitAddress(MPU9250_I2C, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Send the MPU6050's internal address to write to */
    I2C_SendData(MPU9250_I2C, writeAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send the byte to be written */
    I2C_SendData(MPU9250_I2C, *pBuffer);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STOP condition */
    I2C_GenerateSTOP(MPU9250_I2C, ENABLE);

  /* Delay */
  LongDelay(0x50);
    // EXT_CRT_SECTION();
}




void I2C_BufferRead(u8 slaveAddr, u8* pBuffer, u8 readAddr, u16 NumByteToRead)
{
    // ENTR_CRT_SECTION();
    /* While the bus is busy */
    while (I2C_GetFlagStatus(MPU9250_I2C, I2C_FLAG_BUSY));

    /* Send START condition */
    I2C_GenerateSTART(MPU9250_I2C, ENABLE);


    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU9250 address for write */
    I2C_Send7bitAddress(MPU9250_I2C, slaveAddr, I2C_Direction_Transmitter);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    /* Clear EV6 by setting again the PE bit */
    I2C_Cmd(MPU9250_I2C, ENABLE);

    /* Send the MPU9250's internal address to write to */
    I2C_SendData(MPU9250_I2C, readAddr);

    /* Test on EV8 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    /* Send STRAT condition a second time */
    I2C_GenerateSTART(MPU9250_I2C, ENABLE);

    /* Test on EV5 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_MODE_SELECT));

    /* Send MPU9250 address for read */
    I2C_Send7bitAddress(MPU9250_I2C, slaveAddr, I2C_Direction_Receiver);

    /* Test on EV6 and clear it */
    while (!I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

    /* While there is data to be read */
    while (NumByteToRead)
    {
        if (NumByteToRead == 1)
        {
            /* Disable Acknowledgement */
            I2C_AcknowledgeConfig(MPU9250_I2C, DISABLE);

            /* Send STOP Condition */
            I2C_GenerateSTOP(MPU9250_I2C, ENABLE);
        }

        /* Test on EV7 and clear it */
        if (I2C_CheckEvent(MPU9250_I2C, I2C_EVENT_MASTER_BYTE_RECEIVED))
        {
            /* Read a byte from the MPU9250 */
            *pBuffer = I2C_ReceiveData(MPU9250_I2C);

            /* Point to the next location where the byte read will be saved */
            pBuffer++;

            /* Decrement the read bytes counter */
            NumByteToRead--;
        }
    }


    /* Enable Acknowledgement to be ready for another reception */
    I2C_AcknowledgeConfig(MPU9250_I2C, ENABLE);
    // EXT_CRT_SECTION();
			
		/* Delay */
		LongDelay(0x4000);

		
}
