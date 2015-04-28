/*
 * main.cpp
 *
 *  Created on: 2015年4月21日
 *      Author: Winder
 */

/*包含头------------------------------------------------------------------*/

#include "header.h"

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

static Axis3f gyro; // Gyro axis data in deg/s
static Axis3f acc;  // Accelerometer axis data in mG
static Axis3f mag;  // Magnetometer axis data in testla

static float accWZ     = 0.0;
static float accMAG    = 0.0;

static float eulerRollActual;
static float eulerPitchActual;
static float eulerYawActual;
static float eulerRollDesired;
static float eulerPitchDesired;
static float eulerYawDesired;
static float rollRateDesired;
static float pitchRateDesired;
static float yawRateDesired;

using namespace std;

int main(void)
{
	  uint32_t attitudeCounter = 0;
	  uint32_t altHoldCounter = 0;
	  uint32_t lastWakeTime;

	gpioInitialise();
	motorInit();

	imu6Init();
	imu6Test();

	printf("=====================================\r\n");
	while(1)
	  {
		usleep(IMU_UPDATE_FREQ * 20); // 500Hz
		gpioTrigger(motorL.gpioPWM,100,1);

		// Magnetometer not yet used more then for logging.
		imu9Read(&gyro, &acc, &mag);
		printf("gyro x:%10f y:%10f z:%12f\t", gyro.x,gyro.y,gyro.z);
		printf("acc x:%10f y:%10f z:%12f\t", acc.x,acc.y,acc.z);
		printf("mag x:%10f y:%10f z:%12f\r", mag.x, mag.y, mag.z);

		if (1)//imu6IsCalibrated())
		{
			//commanderGetRPY(&eulerRollDesired, &eulerPitchDesired, &eulerYawDesired);
			//commanderGetRPYType(&rollType, &pitchType, &yawType);

			// 250HZ
			if (++attitudeCounter >= ATTITUDE_UPDATE_RATE_DIVIDER)
			{
				sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,
						FUSION_UPDATE_DT);
				sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual,
						&eulerYawActual);

				accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);
				accMAG = (acc.x * acc.x) + (acc.y * acc.y) + (acc.z * acc.z);

				/*
				 controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
				 eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
				 &rollRateDesired, &pitchRateDesired, &yawRateDesired);
				 */
				attitudeCounter = 0;

				//printf("RPY R:%12f P:%12f Y:%12f\r",eulerRollActual,eulerPitchActual,eulerYawActual);
			}

			attitudeCounter++;
		}
		else
		{
			printf("imu not calibrate\r\n");
		}
	}

#if 0
	mpu6050_init();

	hmc5883l_init();

	ms5611_init();

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
#endif

	//motorTerminate();
	return 0;
}






