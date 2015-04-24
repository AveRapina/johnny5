
#include "header.h"

MPU6050 mpu6050;

void mpu6050_init(void);			//模块初始化
void mpu6050_read_gyro(void);	//读取陀螺仪数据
void mpu6050_read_accel(void);	//读取加速度数据
//void mpu6050_check_id(void);		//读取模块信息
void mpu6050TestConnection();

//模块初始化
void mpu6050_init(void)
{
	printf("Init MPU6050...\r\n");

	memset(&mpu6050, 0, sizeof(MPU6050));

	mpu6050.handle = i2cOpen(1, MPU6050_ADDR, 0);	//打开I2C

	if (mpu6050.handle >= 0)	//若I2C打开成功
	{
		mpu6050TestConnection();

		//初始化陀螺仪
		i2cWriteByteData(mpu6050.handle, PWR_MGMT_1, 0x00);		//解除休眠状态

		i2cWriteByteData(mpu6050.handle, SMPLRT_DIV, 0x07);   	//陀螺仪采样率
		i2cWriteByteData(mpu6050.handle, CONFIG, 0x06);       	//5Hz

		i2cWriteByteData(mpu6050.handle, INT_PIN_CFG, 0x42);  	//使能旁路I2C
		i2cWriteByteData(mpu6050.handle, USER_CTRL, 0x40);    	//使能旁路I2C

		i2cWriteByteData(mpu6050.handle, GYRO_CONFIG, 0x18);
		i2cWriteByteData(mpu6050.handle, ACCEL_CONFIG, 0x01);

	}
	else
	{
		printf("I2C error open failed.\r\n");
	}
}

void mpu6050TestConnection()
{
	uint8_t dev = 0;

	//Get device ID
	if (dev = i2cReadByteData(mpu6050.handle, WHO_AM_I), dev == MPU6050_ADDR)
	{
		printf("MPU6050 I2C connection [OK], ID = 0x%x\r\n", dev);
	}
	else
	{
		printf("MPU6050 I2C connection [FAIL], ID = 0x%x\r\n", dev);
	}
}

void mpu6050GetMotion6()
{
	char buffer[14];
	memset(&buffer, 0, sizeof(buffer));

	i2cReadI2CBlockData(mpu6050.handle, ACCEL_XOUT_H, buffer, 14);
	  mpu6050.AccelSrc.x = (((int16_t) buffer[0]) << 8) | buffer[1];
	  mpu6050.AccelSrc.y = (((int16_t) buffer[2]) << 8) | buffer[3];
	  mpu6050.AccelSrc.z = (((int16_t) buffer[4]) << 8) | buffer[5];
	  mpu6050.GyroSrc.x = (((int16_t) buffer[8]) << 8) | buffer[9];
	  mpu6050.GyroSrc.y = (((int16_t) buffer[10]) << 8) | buffer[11];
	  mpu6050.GyroSrc.z = (((int16_t) buffer[12]) << 8) | buffer[13];
}
	
//读取陀螺仪数据
void mpu6050_read_gyro(void)
{
	//读取计算X轴数据
	mpu6050.Buff[0] = i2cReadByteData(mpu6050.handle, GYRO_XOUT_L);
	mpu6050.Buff[1] = i2cReadByteData(mpu6050.handle, GYRO_XOUT_H);
	mpu6050.Gyro.x = (mpu6050.Buff[1] << 8) | mpu6050.Buff[0];
	mpu6050.Gyro.x /= 16.4;

	//读取计算Y轴数据
	mpu6050.Buff[2] = i2cReadByteData(mpu6050.handle, GYRO_YOUT_L);
	mpu6050.Buff[3] = i2cReadByteData(mpu6050.handle, GYRO_YOUT_H);
	mpu6050.Gyro.y = (mpu6050.Buff[3] << 8) | mpu6050.Buff[2];
	mpu6050.Gyro.y /= 16.4;

	//读取计算Z轴数据
	mpu6050.Buff[4] = i2cReadByteData(mpu6050.handle, GYRO_ZOUT_L);
	mpu6050.Buff[5] = i2cReadByteData(mpu6050.handle, GYRO_ZOUT_H);
	mpu6050.Gyro.z = (mpu6050.Buff[5] << 8) | mpu6050.Buff[4];
	mpu6050.Gyro.z /= 16.4;

	
	// mpu6050.Buff[6]=i2cReadByteData(mpu6050.handle,TEMP_OUT_L);
	// mpu6050.Buff[7]=i2cReadByteData(mpu6050.handle,TEMP_OUT_H);
	// T_T=(mpu6050.Buff[7]<<8)|mpu6050.Buff[6];
	// T_T = 35+ ((double) (T_T + 13200)) / 280;// 读取计算出温度
}

//读取加速度数据
void mpu6050_read_accel(void)
{
	//读取计算X轴数据
	mpu6050.Buff[0] = i2cReadByteData(mpu6050.handle, ACCEL_XOUT_L);
	mpu6050.Buff[1] = i2cReadByteData(mpu6050.handle, ACCEL_XOUT_H);
	mpu6050.Accel.x = (mpu6050.Buff[1] << 8) | mpu6050.Buff[0];
	mpu6050.Accel.x = (float) ((float) mpu6050.Accel.x / 16384) * 100; //扩大100倍

	//读取计算Y轴数据
	mpu6050.Buff[2] = i2cReadByteData(mpu6050.handle, ACCEL_YOUT_L);
	mpu6050.Buff[3] = i2cReadByteData(mpu6050.handle, ACCEL_YOUT_H);
	mpu6050.Accel.y = (mpu6050.Buff[3] << 8) | mpu6050.Buff[2];
	mpu6050.Accel.y = (float) ((float) mpu6050.Accel.y / 16384) * 100;

	//读取计算Z轴数据
	mpu6050.Buff[4] = i2cReadByteData(mpu6050.handle, ACCEL_ZOUT_L);
	mpu6050.Buff[5] = i2cReadByteData(mpu6050.handle, ACCEL_ZOUT_H);
	mpu6050.Accel.z = (mpu6050.Buff[5] << 8) | mpu6050.Buff[4];
	mpu6050.Accel.z = (float) ((float) mpu6050.Accel.z / 16384) * 100;

	// 读取计算温度
	mpu6050.Buff[6] = i2cReadByteData(mpu6050.handle, TEMP_OUT_L);
	mpu6050.Buff[7] = i2cReadByteData(mpu6050.handle, TEMP_OUT_H);
	mpu6050.Temperature = (mpu6050.Buff[7] << 8) | mpu6050.Buff[6];
	mpu6050.Temperature = (((double) mpu6050.Temperature) / 340 + 36.53) * 10; //+36.53;
	//mpu6050.Temperature = 35+ ((double) (T_T + 13200)) / 280;
}
