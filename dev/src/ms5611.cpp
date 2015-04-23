/*包含头------------------------------------------------------------------*/
#include "header.h"

MS5611 ms5611;

//uint32_t ex_Pressure;			//串口读数转换值
//uint8_t exchange_num[8];

/*函数声明----------------------------------------------------------------*/
void ms5611_reset(void);
void ms5611_read_prom(void);
void ms5611_init(void);

uint32_t ms5611_do_conversion(uint8_t command);
void ms5611_get_temperature(uint8_t OSR_Temp);
void ms5611_get_pressure(uint8_t OSR_Pres);
//void SampleANDExchange(void);

/*----------------------------------------------------------------*/

//复位
void ms5611_reset(void)
{
	i2cWriteByte(MS5611_ADDR, MS5611_RESET);
}

//从PROM读取出厂校准数据
void ms5611_read_prom(void)
{
	//uint16_t value = 0;
	//uint8_t temp1[2] =	{ 0 };
	int i;

	printf("reading PROM of MS5611:\r\n");
	for (i = 0; i <= MS5611_PROM_REG_COUNT; i++)
	{
		// I2C_Read_MultiBytes(MS5611_ADDR,MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE),2,temp1);
		//value=temp1[0]<<8|temp1[1];
		//ms5611.Data[i]=value;

		ms5611.Data[i] = i2cReadWordData(MS5611_ADDR, MS5611_PROM_BASE_ADDR + (i * MS5611_PROM_REG_SIZE));
		printf("C%d = %d ", i, ms5611.Data[i]);
	}
	printf("\r\n");
}

uint32_t ms5611_do_conversion(uint8_t command)
{
	uint32_t conversion;

	i2cWriteByte(ms5611.handle, command);

	//delay_ms(10);		//延时,去掉数据错误

	conversion = I2C_Read_3Bytes(MS5611_ADDR, 0);

	return conversion;

}

//读取数字温度
void ms5611_get_temperature(uint8_t OSR_Temp)
{

	ms5611.D2_Temp = ms5611_do_conversion(OSR_Temp);
	delay_ms(100);

	ms5611.dT = ms5611.D2_Temp - (((uint32_t) ms5611.Data[5]) << 8);
	ms5611.Temperature = 2000 + ms5611.dT * ((uint32_t) ms5611.Data[6]) / 8388608;//算出温度值的100倍，2001表示20.01°

}

//读取数字气压
void ms5611_get_pressure(uint8_t OSR_Pres)
{
	ms5611.D1_Pres = ms5611_do_conversion(OSR_Pres);

	delay_ms(100);

	ms5611.OFF  = (uint32_t) (ms5611.Data[2] << 16) + ((uint32_t) ms5611.Data[4] * ms5611.dT) / 128.0;
	ms5611.SENS = (uint32_t) (ms5611.Data[1] << 15) + ((uint32_t) ms5611.Data[3] * ms5611.dT) / 256.0;

	//温度补偿
	if (ms5611.Temperature < 2000)// second order temperature compensation when under 20 degrees C
	{
		ms5611.Temperature2 = (ms5611.dT * ms5611.dT) / 0x80000000;
		ms5611.Aux = (ms5611.Temperature - 2000) * (ms5611.Temperature - 2000);
		ms5611.OFF2 = 2.5 * ms5611.Aux;
		ms5611.SENS2 = 1.25 * ms5611.Aux;
		if (ms5611.Temperature < -1500)
		{
			ms5611.Aux = (ms5611.Temperature + 1500) * (ms5611.Temperature + 1500);
			ms5611.OFF2 = ms5611.OFF2 + 7 * ms5611.Aux;
			ms5611.SENS2 = ms5611.SENS + 5.5 * ms5611.Aux;
		}
	}
	else  //(Temperature > 2000)
	{
		ms5611.Temperature2 = 0;
		ms5611.OFF2 = 0;
		ms5611.SENS2 = 0;
	}

	ms5611.Temperature = ms5611.Temperature - ms5611.Temperature2;
	ms5611.OFF = ms5611.OFF - ms5611.OFF2;
	ms5611.SENS = ms5611.SENS - ms5611.SENS2;

	ms5611.Pressure = (ms5611.D1_Pres * ms5611.SENS / 2097152.0 - ms5611.OFF) / 32768.0;
}

//MS5611初始化
void ms5611_init(void)
{
	memset(&ms5611, 0, sizeof(MS5611));

	ms5611.handle = i2cOpen(1, MS5611_ADDR, 0);	//打开I2C

	if (ms5611.handle >= 0)		//若I2C打开成功
	{
		ms5611_reset();
		//delay_ms(100);
		ms5611_read_prom();
		//delay_ms(100);
	}
	else
	{
		printf("I2C error open failed.\r\n");
	}
}

#if 0
//读取数据并转换串口发送
void SampleANDExchange(void)
{
	uint8_t i = 0;
	MS5611_GetTemperature(MS5611_D2_OSR_4096);  //0x58
	MS5611_GetPressure(MS5611_D1_OSR_4096);		//0x48
	ex_Pressure = (long) (Pressure);

	if (Pressure < 0)
	{
		ex_Pressure = -ex_Pressure;
		exchange_num[0] = '-';
	}
	else
		exchange_num[0] = ' ';

	exchange_num[1] = ex_Pressure / 100000 + 0x30;
	ex_Pressure = ex_Pressure % 100000;

	exchange_num[2] = ex_Pressure / 10000 + 0x30;
	ex_Pressure = ex_Pressure % 10000;

	exchange_num[3] = ex_Pressure / 1000 + 0x30;
	ex_Pressure = ex_Pressure % 1000;

	exchange_num[4] = ex_Pressure / 100 + 0x30;
	ex_Pressure = ex_Pressure % 100;

	exchange_num[5] = '.';

	exchange_num[6] = ex_Pressure / 10 + 0x30;
	ex_Pressure = ex_Pressure % 10;

	exchange_num[7] = ex_Pressure + 0x30;
	printf("Press:");
	for (i = 0; i < 8; i++)
	{
		printf("%c", exchange_num[i]);
		delay_ms(20);
	}
	printf("mbar   ");
	printf("Temp:%4.3f°C  ", Temperature);

}
#endif
