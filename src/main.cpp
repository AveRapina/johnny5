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

	motorTerminate();

	while (1)
	{
		mpu6050_read_gyro();
		cout << "Gyro";
		cout << "\tX:" << (float)mpu6050.Gyro.x/100;
		cout << "\tY:" << (float)mpu6050.Gyro.y/100;
		cout << "\tZ:" << (float)mpu6050.Gyro.z/100;

		mpu6050_read_accel();
		cout << "\tAccel";
		cout << "\tX:" << (float)mpu6050.Accel.x/100;
		cout << "\tY:" << (float)mpu6050.Accel.y/100;
		cout << "\tZ:" << (float)mpu6050.Accel.z/100<<"\r";

		//delay(500);
		//cout << "Temperature:" << mpu6050.Temperature << endl;

		/*
		cout<< "Pressure:";
		ms5611_get_pressure(MS5611_D1_OSR_4096);
		ms5611_get_temperature(MS5611_D2_OSR_4096);
		*/
		
	    cout<<"Magnetic:";
	    hmc5883l_read_magnetic();
		//cout<<endl;
		//delayms(500);
	}

}






