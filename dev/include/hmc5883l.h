#ifndef _HMC5883L_H_
#define _HMC5883L_H_

/*�궨��------------------------------------------------------------------*/
#define ABS(x) ((x)>=0?(x):(-(x)))

#define HMC5883L_ADDR  		0x1E	//�ų�������������ַ

#define HMC5883L_CFG_REG_A		0x00
#define HMC5883L_CFG_REG_B		0x01

#define HMC5883L_MODE_REG		0x02

#define HMC5883L_MAG_X_MSB		0x03
#define HMC5883L_MAG_X_LSB 	0x04
#define HMC5883L_MAG_Z_MSB		0x05
#define HMC5883L_MAG_Z_LSB 	0x06
#define HMC5883L_MAG_Y_MSB		0x07
#define HMC5883L_MAG_Y_LSB		0x08

#define HMC5883L_STATUS_REG	0x09

#define HMC5883L_ID_A			0x0A
#define HMC5883L_ID_B			0x0B
#define HMC5883L_ID_C			0x0C

#define HMC5883L_OFFSET_X  	(9)
#define HMC5883L_OFFSET_Y   	(149)

#define CalThreshold 			(0)

typedef struct _THREE_AXIS_INT
{
	int x;
	int y;
	int z;
}THREE_AXIS_INT,*pTHREE_AXIS_INT;

#if 0
typedef struct _THREE_AXIS_FLOAT
{
	float x;
	float y;
	float z;
}THREE_AXIS_FLOAT,*pTHREE_AXIS_FLOAT;
#endif

typedef struct _HMC5883L
{
	int handle;

	THREE_AXIS_INT Magnetic;
	THREE_AXIS_INT MagMax;
	THREE_AXIS_INT MagMin;

	float Angle;   //5883 ��ָ����

	THREE_AXIS_FLOAT Offset;
	THREE_AXIS_FLOAT Scale;

	THREE_AXIS_INT Max;
	THREE_AXIS_INT Min;
	THREE_AXIS_INT MagUserCalOffset;
}HMC5883L, *pHMC5883L;

/*��������----------------------------------------------------------------*/

extern HMC5883L hmc5883l;

extern void hmc5883l_init(void);				//��ʼ���豸
extern void hmc5883l_identify(void);		//ʶ���豸
extern void hmc5883l_read_magnetic(void);	//��ȡ
extern void hmc5883l_self_test(void);		//��ȡ

#endif
