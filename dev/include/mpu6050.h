#ifndef _MP6050_H_
#define _MP6050_H_

/*�궨��------------------------------------------------------------------*/
#define	MPU6050_ADDR   0x68	  //����������IIC�����еĴӵ�ַ,����ALT  ADDRESS��ַ���Ų�ͬ�޸�

// ����MPU6050 �Ĵ�����ַ
#define	SMPLRT_DIV		0x19	//�����ǲ����ʣ�����ֵ��0x07(125Hz)
#define	CONFIG			0x1A	//��ͨ�˲�Ƶ�ʣ�����ֵ��0x06(5Hz)
#define	GYRO_CONFIG		0x1B	//�������Լ켰������Χ������ֵ��0x18(���Լ죬2000deg/s)
#define	ACCEL_CONFIG	0x1C	//���ټ��Լ������Χ����ͨ�˲�Ƶ�ʣ�����ֵ��0x01(���Լ죬2G��5Hz)

#define 	INT_PIN_CFG     0x37    //������·��Ч ��ֵ��0x42 AUX_DA�ĸ���I2C
#define 	USER_CTRL       0x6A    //�û����üĴ��� ��ֵ��0x40  AUX_DA�ĸ���I2C

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

#define	PWR_MGMT_1		0x6B	//��Դ��������ֵ��0x00(��������)
#define	WHO_AM_I		0x75	//IIC��ַ�Ĵ���(Ĭ����ֵ0x68��ֻ��)

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
	int handle;				//I2C���
	uint8_t Data[4];  		//��ʾ���ݻ�����
	uint8_t Buff[10];     	//�������ݻ�����

	THREE_AXIS_INT16 GyroSrc;	//�����ǲ���ԭʼֵ
	THREE_AXIS_INT16 AccelSrc;	//���ٶȲ���ԭʼֵ

	THREE_AXIS_FLOAT Gyro;		//����������
	THREE_AXIS_FLOAT Accel;		//���ٶ�����

	uint16_t Temperature;	//�¶�
}MPU6050,pMPU6050;

/*��������----------------------------------------------------------------*/

extern MPU6050 mpu6050;

/*��������----------------------------------------------------------------*/
extern void mpu6050_init(void);			//ģ���ʼ��
extern void mpu6050_read_gyro(void);	//��ȡ����������
extern void mpu6050_read_accel(void);	//��ȡ���ٶ�����
extern void mpu6050_check_id(void);		//ģ����

extern void mpu6050GetMotion6();

#endif
