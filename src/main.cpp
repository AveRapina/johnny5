/*
 * main.cpp
 *
 *  Created on: 2015年4月21日
 *      Author: Winder
 */

/*包含头------------------------------------------------------------------*/

#include "header.h"

using namespace std;

int main(void)
{
	gpioInitialise();
	
	mpu6050_init();

	hmc5883l_init();

	//ms5611_init();

#if 0
	int speed;

	motorInit();

	cout<<"Input speed of left motor:";
	cin>>speed;

	motorSetSpeed(&motorL, speed);

	cout<<"Input speed of right motor:";
	cin>>speed;

	motorSetSpeed(&motorR, speed);
	motorEnable(&motorR);

	cout<<"Input speed of right motor:";
	cin>>speed;

	motorSetSpeed(&motorR, speed);

	cout<<"Input any number to terminate program:";
	cin>>speed;
#endif
	while (1)
	//for(int i=0; i<100;i++)
	{
		mpu6050GetMotion6();
		//mpu6050_read_gyro();
		cout << "Gyro";
		cout << "\tX:" << (float)mpu6050.GyroSrc.x;
		cout << "\tY:" << (float)mpu6050.GyroSrc.y;
		cout << "\tZ:" << (float)mpu6050.GyroSrc.z;

		//mpu6050_read_accel();
		cout << "\tAccel";
		cout << "\tX:" << (float)mpu6050.AccelSrc.x;
		cout << "\tY:" << (float)mpu6050.AccelSrc.y;
		cout << "\tZ:" << (float)mpu6050.AccelSrc.z;

		//delay(500);
		//cout << "Temperature:" << mpu6050.Temperature << endl;

		/*
		cout<< "Pressure:";
		ms5611_get_pressure(MS5611_D1_OSR_4096);
		ms5611_get_temperature(MS5611_D2_OSR_4096);
		*/
		
	    cout<<"\tMag:";
	    hmc5883l_read_magnetic();
		cout << "\tX:" << (float)hmc5883l.Magnetic.x/100;
		cout << "\tY:" << (float)hmc5883l.Magnetic.y/100;
		cout << "\tZ:" << (float)hmc5883l.Magnetic.z/100;
		cout << "\tAngle:" << (float)hmc5883l.Angle/100<<"\r";
	}

	//motorTerminate();
	return 0;
}






