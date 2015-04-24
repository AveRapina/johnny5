#ifndef _MP6050_H_
#define _MP6050_H_

/*宏定义------------------------------------------------------------------*/
#define	MPU6050_ADDR   0x68	  //定义器件在IIC总线中的从地址,根据ALT  ADDRESS地址引脚不同修改

// 定义MPU6050 寄存器地址
#define	SMPLRT_DIV		0x19	//陀螺仪采样率，典型值：0x07(125Hz)
#define	CONFIG			0x1A	//低通滤波频率，典型值：0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//陀螺仪自检及测量范围，典型值：0x18(不自检，2000deg/s)
#define	ACCEL_CONFIG	0x1C	//加速计自检测量范围及高通滤波频率，典型值：0x01(不自检，2G，5Hz)

#define 	INT_PIN_CFG     0x37    //设置旁路有效 打开值：0x42 AUX_DA的辅助I2C
#define 	USER_CTRL       0x6A    //用户配置寄存器 打开值：0x40  AUX_DA的辅助I2C

#if 0
	#define	ACCEL_X			0x3B
	#define	ACCEL_Y			0x3D
	#define	ACCEL_Z			0x3F

	#define	TEMP			0x41

	#define	GYRO_X			0x43
	#define	GYRO_Y			0x45
	#define	GYRO_Z			0x47
#else
	#define	ACCEL_XOUT_H	0x3B
	#define	ACCEL_XOUT_L	0x3C
	#define	ACCEL_YOUT_H	0x3D
	#define	ACCEL_YOUT_L	0x3E
	#define	ACCEL_ZOUT_H	0x3F
	#define	ACCEL_ZOUT_L	0x40

	#define	TEMP_OUT_H		0x41
	#define	TEMP_OUT_L		0x42

	#define	GYRO_XOUT_H		0x43
	#define	GYRO_XOUT_L		0x44
	#define	GYRO_YOUT_H		0x45
	#define	GYRO_YOUT_L		0x46
	#define	GYRO_ZOUT_H		0x47
	#define	GYRO_ZOUT_L		0x48
#endif

#define	PWR_MGMT_1		0x6B	//电源管理，典型值：0x00(正常启用)
#define	WHO_AM_I		0x75	//IIC地址寄存器(默认数值0x68，只读)

typedef struct _THREE_AXIS_INT16
{
	uint16_t x;
	uint16_t y;
	uint16_t z;
}THREE_AXIS_INT16,*pTHREE_AXIS_INT16;

typedef struct _THREE_AXIS_FLOAT
{
	float x;
	float y;
	float z;
}THREE_AXIS_FLOAT,*pTHREE_AXIS_FLOAT;

typedef struct _MPU6050
{
	int handle;				//I2C句柄
	uint8_t Data[4];  		//显示数据缓存区
	uint8_t Buff[10];     	//接收数据缓存区

	THREE_AXIS_INT16 GyroSrc;	//陀螺仪采样原始值
	THREE_AXIS_INT16 AccelSrc;	//加速度采样原始值

	THREE_AXIS_FLOAT Gyro;		//陀螺仪数据
	THREE_AXIS_FLOAT Accel;		//加速度数据

	uint16_t Temperature;	//温度
}MPU6050,pMPU6050;

/*变量声明----------------------------------------------------------------*/

extern MPU6050 mpu6050;

/*函数声明----------------------------------------------------------------*/
extern void mpu6050_init(void);			//模块初始化
extern void mpu6050_read_gyro(void);	//读取陀螺仪数据
extern void mpu6050_read_accel(void);	//读取加速度数据
extern void mpu6050_check_id(void);		//模块检测

extern void mpu6050GetMotion6();

#endif
