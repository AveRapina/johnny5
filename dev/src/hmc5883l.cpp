/*包含头------------------------------------------------------------------*/

#include "header.h"

HMC5883L hmc5883l;

/*函数声明----------------------------------------------------------------*/
void hmc5883l_identify(void);	//设备识别
void hmc5883l_read_magnetic(void);
void hmc5883l_self_test(void);

void Initialize_Cal_Variables(int MagX, int MagY, int  MagZ); 
void Calibrate(int MagX, int MagY, int  MagZ);
void Compute_and_Save(void);
void Hard_Iron_Correction(int MagX, int MagY, int  MagZ);

//设备初始化
void hmc5883l_init()
{
	printf("Init HMC5883L...\r\n");

	memset(&hmc5883l, 0, sizeof(HMC5883L));

	hmc5883l.handle = i2cOpen(1, HMC5883L_ADDR, 0);	//打开I2C

	if (hmc5883l.handle >= 0)	//若I2C打开成功
	{
		hmc5883l_identify();
		hmc5883l_self_test();
	}
	else
	{
		printf("I2C error open failed.\r\n");
	}
}

//设备识别
void hmc5883l_identify(void)
{
	uint8_t ID_A, ID_B, ID_C;
	ID_A = i2cReadByteData(hmc5883l.handle, HMC5883L_ID_A);
	ID_B = i2cReadByteData(hmc5883l.handle, HMC5883L_ID_B);
	ID_C = i2cReadByteData(hmc5883l.handle, HMC5883L_ID_C);

	if (ID_A == 'H' && ID_B == '4' && ID_C == '3')
	{
		printf("HMC5883L OK!\r\n");
	}else
	{
		printf("HMC5773L Error!\r\n");
	}
}

//计算X,Y,Z轴偏移,被HMC5883L_Self_Test（）调用
void HMC58X3_Offset(void)
{
	//计算零偏
	hmc5883l.Offset.x = (hmc5883l.MagMax.x + hmc5883l.MagMin.x) / 2;
	hmc5883l.Offset.y = (hmc5883l.MagMax.y + hmc5883l.MagMin.y) / 2;
	hmc5883l.Offset.z = (hmc5883l.MagMax.z + hmc5883l.MagMin.z) / 2;
}

//计算X,Y,Z轴最值,被HMC5883L_Self_Test（）调用
void Initialize_Cal_Variables(int MagX, int MagY, int  MagZ)
{
	// set Max and Min values of the mag output to the current values
	hmc5883l.Max.x = MagX;
	hmc5883l.Max.y = MagY;
	hmc5883l.Max.z = MagZ;
	hmc5883l.Min.x = MagX;
	hmc5883l.Min.y = MagY;
	hmc5883l.Min.z = MagZ;
}

void Calibrate(int MagX, int MagY, int  MagZ)
{
	//  this routine will capture the max and min values of the mag X, Y, and Z data while the
	//  compass is being rotated 360 degrees through the level plane and the upright plane.
	//  i.e. horizontal and vertical circles.
	// This function should be invoked while making continuous measurements
	//on the magnetometers

	int MagXreading, MagYreading, MagZreading;

	MagXreading = MagX;  // just for clarification...  can remove these lines
	MagYreading = MagY;
	MagZreading = MagZ;

	if (MagXreading > hmc5883l.Max.x)
		hmc5883l.Max.x = MagXreading;
	if (MagXreading < hmc5883l.Min.x)
		hmc5883l.Min.x = MagXreading;
	if (MagYreading > hmc5883l.Max.y)
		hmc5883l.Max.y = MagYreading;
	if (MagYreading < hmc5883l.Min.y)
		hmc5883l.Min.y = MagYreading;
	if (MagZreading > hmc5883l.Max.z)
		hmc5883l.Max.z = MagZreading;
	if (MagZreading < hmc5883l.Min.z)
		hmc5883l.Min.z = MagZreading;
}

void Compute_and_Save(void)
{
	if (ABS(hmc5883l.Max.x - hmc5883l.Min.x) > CalThreshold)
	{

		hmc5883l.MagUserCalOffset.x = (hmc5883l.Max.x + hmc5883l.Min.x) / 2;
		// Save parameters in EE
	}

	if (ABS(hmc5883l.Max.y - hmc5883l.Min.y) > CalThreshold)
	{

		hmc5883l.MagUserCalOffset.y = (hmc5883l.Max.y + hmc5883l.Min.y) / 2;
		//Save parameters in EE
	}

	if (ABS(hmc5883l.Max.z - hmc5883l.Min.z) > CalThreshold)
	{
		hmc5883l.MagUserCalOffset.z = (hmc5883l.Max.z + hmc5883l.Min.z) / 2;
		// Save parameters in EE
	}

}

void Hard_Iron_Correction( int MagX, int MagY, int  MagZ)   // call this function for correction
{
	MagX -= hmc5883l.MagUserCalOffset.x;
	MagY -= hmc5883l.MagUserCalOffset.y;
	MagZ -= hmc5883l.MagUserCalOffset.z;
}

//自测磁场强度求比例因子
void hmc5883l_self_test(void)
{
	uint8_t Buff[7] = { 0 };

	i2cWriteByteData(hmc5883l.handle, HMC5883L_CFG_REG_A, 0x15); 	//配置寄存器A：采样平均数1 输出速率30Hz 自测模式
	//i2cWriteByteData(hmc5883l.handle,HMC5883L_CFG_REG_B,0x20);		//配置寄存器B：增益控制
	i2cWriteByteData(hmc5883l.handle, HMC5883L_MODE_REG, 0x01);		//模式寄存器：单一测量模式

	//delay_ms(5);

	Buff[1] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_X_MSB);   //OUT_X_H
	Buff[2] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_X_LSB);   //OUT_X_L

	Buff[3] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_Y_MSB);   //OUT_Y_L_A
	Buff[4] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_Y_LSB);   //OUT_Y_H_A

	Buff[5] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_Z_MSB);   //OUT_Z_L_A
	Buff[6] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_Z_LSB);   //OUT_Z_H_A

	hmc5883l.Magnetic.x = (Buff[1] << 8) | Buff[2];	//Combine MSB and LSB of X Data output register
	hmc5883l.Magnetic.y = (Buff[3] << 8) | Buff[4]; //Combine MSB and LSB of Z Data output register
	hmc5883l.Magnetic.z = (Buff[5] << 8) | Buff[6]; //Combine MSB and LSB of Z Data output register

	if (hmc5883l.Magnetic.x > 0x7fff) {
		hmc5883l.Magnetic.x -= 0xffff;
	}
	if (hmc5883l.Magnetic.y > 0x7fff) {
		hmc5883l.Magnetic.y -= 0xffff;
	}
	if (hmc5883l.Magnetic.z > 0x7fff) {
		hmc5883l.Magnetic.z -= 0xffff;
	}

	// hmc5883l.Angle in degrees
	hmc5883l.Angle = atan2((double) hmc5883l.Magnetic.y, (double) hmc5883l.Magnetic.x) * (180 / 3.14159265) + 180;

	printf("Magnetic Values：x=%d,y=%d,z=%d \r\n", hmc5883l.Magnetic.x, hmc5883l.Magnetic.y, hmc5883l.Magnetic.z);
	printf("Angle=%f \r\n", hmc5883l.Angle);
}

//读取磁场强度
void hmc5883l_read_magnetic(void)
{
	uint8_t Buff[7] = { 0 };
	i2cWriteByteData(hmc5883l.handle, HMC5883L_CFG_REG_A, 0x14);	//配置寄存器A：采样平均数1 输出速率30Hz 正常测量
	i2cWriteByteData(hmc5883l.handle, HMC5883L_CFG_REG_B, 0x20);	//配置寄存器B：增益控制
	i2cWriteByteData(hmc5883l.handle, HMC5883L_MODE_REG, 0x00);   	//模式寄存器：连续测量模式

	//delay_ms(5);

	Buff[1] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_X_MSB);   //OUT_X_H
	Buff[2] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_X_LSB);   //OUT_X_L

	Buff[3] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_Y_MSB);   //OUT_Y_L
	Buff[4] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_Y_LSB);   //OUT_Y_H

	Buff[5] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_Z_MSB);   //OUT_Z_L
	Buff[6] = i2cReadByteData(hmc5883l.handle, HMC5883L_MAG_Z_LSB);   //OUT_Z_H

	hmc5883l.Magnetic.x = (Buff[1] << 8) | Buff[2]; //Combine MSB and LSB of X Data output register
	hmc5883l.Magnetic.y = (Buff[3] << 8) | Buff[4]; //Combine MSB and LSB of Z Data output register
	hmc5883l.Magnetic.z = (Buff[5] << 8) | Buff[6]; //Combine MSB and LSB of Z Data output register

	//	hmc5883l.Magnetic.x=hmc5883l.Magnetic.x-X_Offset;
	//	hmc5883l.Magnetic.y=hmc5883l.Magnetic.y-Y_Offset;
	//	hmc5883l.Magnetic.z=hmc5883l.Magnetic.z-Z_Offset;
	//  hmc5883l.Magnetic.x=hmc5883l.Magnetic.x;
	//	hmc5883l.Magnetic.y=hmc5883l.Magnetic.y;
	//hmc5883l.Magnetic.y=(hmc5883l.Magnetic.y*HMC5883L_GAIN_Y)/10000;

	if (hmc5883l.Magnetic.x > 0x7fff)
	{
		hmc5883l.Magnetic.x -= 0xffff;
	}
	if (hmc5883l.Magnetic.y > 0x7fff)
	{
		hmc5883l.Magnetic.y -= 0xffff;
	}
	if (hmc5883l.Magnetic.z > 0x7fff)
	{
		hmc5883l.Magnetic.z -= 0xffff;
	}

	// hmc5883l.Angle in degrees
	hmc5883l.Angle = atan2((double) hmc5883l.Magnetic.y, (double) hmc5883l.Magnetic.x) * (180 / 3.14159265) + 180;
	hmc5883l.Angle += 100;

	if (hmc5883l.Angle > 360)
	{
		hmc5883l.Angle = hmc5883l.Angle - 360;
	}
}
