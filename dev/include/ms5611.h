#ifndef _MS5611_H_
#define _MS5611_H_

/*�궨��------------------------------------------------------------------*/

//MS5611 I2C��ַ
#define MS5611_ADDR   0x77   //CBR=0 0x77 I2C address when CSB is connected to LOW (GND)
//#define MS5611_ADDR  0x76   //CBR=1 0x76 I2C address when CSB is connected to HIGH (VCC)

//MS5611�ڲ��Ĵ��ַ
#define MS5611_D1		 0x40
#define MS5611_D2		 0x50
#define MS5611_RESET	 0x1E

// D1 and D2 result size (bytes)
#define MS5611_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256 		0x00
#define MS5611_OSR_512 		0x02
#define MS5611_OSR_1024 		0x04
#define MS5611_OSR_2048 		0x06
#define MS5611_OSR_4096 		0x08
#define  MS5611_D1_OSR_256 	0x40
#define  MS5611_D1_OSR_512 	0x42
#define  MS5611_D1_OSR_1024 	0x44
#define  MS5611_D1_OSR_2048 	0x46
#define  MS5611_D1_OSR_4096 	0x48

#define  MS5611_D2_OSR_256 	0x50
#define  MS5611_D2_OSR_512 	0x52
#define  MS5611_D2_OSR_1024 	0x54
#define  MS5611_D2_OSR_2048 	0x56
#define  MS5611_D2_OSR_4096 	0x58

#define MS5611_PROM_BASE_ADDR 	0xA0	// by adding ints from 0 to 6 we can read all the prom configuration values.
										// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS5611_PROM_REG_COUNT 	6		// number of registers in the PROM
#define MS5611_PROM_REG_SIZE 	2		// size in bytes of a prom registry.

//MS5611���ݽṹ��
typedef struct _MS5611
{
	int handle;			//I2C���
	uint16_t Data[7];	//���ڴ��PROM�е�6������
	uint32_t D1_Pres;	//ѹ������ֵ
	uint32_t D2_Temp;	//�¶Ȳ���ֵ

	float Pressure;		//�¶Ȳ�������ѹ
	float dT;			//ʵ�ʺͲο��¶�֮��Ĳ���
	float Temperature;	//ʵ���¶�
	float Temperature2;	//�¶��м�ֵ

	double OFF;			//ʵ���¶ȵ���
	double SENS; 		//ʵ���¶�������

	float Aux;
	float OFF2;
	float SENS2;  		//�¶�У��ֵ
}MS5611, *pMS5611;

/*��������----------------------------------------------------------------*/
extern MS5611 ms5611;

/*��������----------------------------------------------------------------*/
extern void ms5611_reset(void);
extern void ms5611_read_prom(void);
extern void ms5611_init(void);

extern uint32_t ms5611_do_conversion(uint8_t command);
extern void ms5611_get_temperature(uint8_t OSR_Temp);
extern void ms5611_get_pressure(uint8_t OSR_Pres);
//void SampleANDExchange(void);

#endif

