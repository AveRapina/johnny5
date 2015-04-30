/*
 * main.cpp
 *
 *  Created on: 2015年4月21日
 *      Author: Winder
 */

/*包含头------------------------------------------------------------------*/

#include "header.h"
#include "stdint.h"

#define INT8_MIN (-128)
#define INT16_MIN (-32768)
#define INT32_MIN (-2147483647 - 1)
#define INT64_MIN  (-9223372036854775807LL - 1)

#define INT8_MAX 127
#define INT16_MAX 32767
#define INT32_MAX 2147483647
#define INT64_MAX 9223372036854775807LL

/**
 * Defines in what divided update rate should the attitude
 * control loop run relative the rate control loop.
 */
#define ATTITUDE_UPDATE_RATE_DIVIDER  2
#define FUSION_UPDATE_DT  (float)(1.0 / (IMU_UPDATE_FREQ / ATTITUDE_UPDATE_RATE_DIVIDER)) // 250hz

#define FREQ (250)
#define SECOND_TO_NS (1000000000)
#define SAMPLE_DT (SECOND_TO_NS/FREQ)

#define PIN_SIG (18)

#define MATH_PI (3.1415926535897932384626433832795)

#define PID_PITCH_KP  3.5
#define PID_PITCH_KI  2.0
#define PID_PITCH_KD  0.0
#define PID_PITCH_INTEGRATION_LIMIT   20.0

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

static float pitchDesired;

PidObject pidPitch;
PidObject pidSpeed;

float speedOutput=0;
int32_t pwmOutput;

float speedFeedback=0;
float speedDesired=0;
int32_t pwmMin = 160;

using namespace std;

#if 0
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
#endif



timer_t fade_in_timer;

int count=0;


static inline int32_t saturateSignedInt16(float in)
{
  // don't use INT16_MIN, because later we may negate it, which won't work for that value.
  if (in > INT32_MAX)
    return INT32_MAX;
  else if (in < -INT32_MAX)
    return -INT32_MAX;
  else
    return (int32_t)in;
}

void pidInit()
{
	pidReset(&pidPitch);
	pidInit(&pidPitch, 0, 200, 2, PID_PITCH_KD, IMU_UPDATE_DT/2);
	pidSetIntegralLimit(&pidPitch, PID_PITCH_INTEGRATION_LIMIT);

	pidReset(&pidSpeed);
	pidInit(&pidSpeed, -10000, PID_PITCH_KP, PID_PITCH_KI, PID_PITCH_KD, IMU_UPDATE_DT/2);
	pidSetIntegralLimit(&pidSpeed, PID_PITCH_INTEGRATION_LIMIT);
}

void pidControl()
{
	speedFeedback = (pwmOutput/16/4) * MATH_PI * 0.114;

	  pidSetDesired(&pidSpeed, speedDesired);
	  speedOutput = pidUpdate(&pidPitch, speedFeedback, true);

	// Update PID for pitch axis
	  pidSetDesired(&pidPitch, eulerPitchDesired);
	  //pidSetDesired(&pidPitch, speedOutput);
	  pitchDesired = pidUpdate(&pidPitch, eulerPitchActual, true);
}

void motorControl()
{
	pwmOutput = saturateSignedInt16(pitchDesired);

#if 1
	if(pwmOutput > 0)
	{
		if(pwmOutput < pwmMin)
		{
			pwmOutput = 0;
		}
		else if (pwmOutput < pwmMin*2)
		{
			pwmOutput = pwmMin*2;
		}
	}
	else
	{
		if(pwmOutput > -pwmMin)
		{
			pwmOutput = 0;
		}
		else if (pwmOutput > -pwmMin*2)
		{
			pwmOutput = -pwmMin*2;
		}
	}
#endif
#if 0
	if(pwmMin > 0)
	{
		if(pwmOutput < 80)
		{
			pwmOutput = 0;
		}
		else if (pwmOutput < 160)
		{
			pwmOutput = 160;
		}
	}
	else
	{
		if(pwmOutput > -80)
		{
			pwmOutput = 0;
		}
		else if (pwmOutput > -160)
		{
			pwmOutput = -160;
		}
	}
#endif
	motorSetSpeed(&motorL, pwmOutput);
	motorSetSpeed(&motorR, pwmOutput);
}

/*static*/void imuUpdate(union sigval v)
{
	if(count)
    {
        count = 0;
        gpioWrite(PIN_SIG, 1);
    }
    else
    {
        count = 1;
        gpioWrite(PIN_SIG, 0);
    }

    //v.sival_ptr 就是创建timer时传进来的指针，最后在合适的地方删除一下timer
    //myclass *ptr = (myclass*)v.sival_ptr;
    //timer_delete(audiotrack->fade_in_timer);
    //printf("call back func\r\n");
    //timer_delete(&fade_in_timer);

	imu9Read(&gyro, &acc, &mag);

#if 0
	printf("gyro x:%10f y:%10f z:%12f\t", gyro.x,gyro.y,gyro.z);
	printf("acc x:%10f y:%10f z:%12f\t", acc.x,acc.y,acc.z);
	printf("mag x:%10f y:%10f z:%12f\r", mag.x, mag.y, mag.z);
#endif

	// 250HZ
	sensfusion6UpdateQ(gyro.x, gyro.y, gyro.z, acc.x, acc.y, acc.z,	FUSION_UPDATE_DT);
	sensfusion6GetEulerRPY(&eulerRollActual, &eulerPitchActual, &eulerYawActual);

	accWZ = sensfusion6GetAccZWithoutGravity(acc.x, acc.y, acc.z);
	accMAG = (acc.x * acc.x) + (acc.y * acc.y) + (acc.z * acc.z);

	pidControl();

	motorControl();

	/*
	 controllerCorrectAttitudePID(eulerRollActual, eulerPitchActual, eulerYawActual,
	 eulerRollDesired, eulerPitchDesired, -eulerYawDesired,
	 &rollRateDesired, &pitchRateDesired, &yawRateDesired);
	 */

	printf("RPY R:%12f P:%12f Y:%12f PWM:%10d Pref:%12f Error:%12f output:%12f\r\n", eulerRollActual, eulerPitchActual, eulerYawActual, pwmOutput, eulerPitchDesired, pidPitch.error, pidPitch.output);
}

int timerInit()
{

    struct sigevent evp;
    memset(&evp, 0, sizeof(evp));

    evp.sigev_value.sival_ptr = &evp;   //这里传一个参数进去，在timer的callback回调函数里面可以获得它
    evp.sigev_notify = SIGEV_THREAD;    //定时器到期后内核创建一个线程执行sigev_notify_function函数
    evp.sigev_notify_function = imuUpdate; //这个就是指定回调函数

    int ret = 0;
    ret = timer_create(CLOCK_REALTIME, &evp, &fade_in_timer);

    if(ret < 0)
    {
        printf("timer_create() fail, ret:%d", ret);
        return ret;
    }

    struct itimerspec ts;
    ts.it_interval.tv_sec = 0;
    ts.it_interval.tv_nsec = SAMPLE_DT; //4ms
    ts.it_value.tv_sec = 0;
    ts.it_value.tv_nsec = SAMPLE_DT; //4ms
    ret = timer_settime(fade_in_timer, TIMER_ABSTIME, &ts, NULL);

    printf("%d",SAMPLE_DT);
    if(ret < 0)
    {
        printf("timer_settime() fail, ret:%d", ret);
        timer_delete(fade_in_timer);
        //timer_created = false;
        return ret;
    }
}

int main()
{
	gpioInitialise();
    gpioSetMode(PIN_SIG, PI_OUTPUT);
	motorInit();

	pidInit();

	imu6Init();
	imu6Test();

    int a;
    cin >> a;

    motorEnable(&motorL);
    motorEnable(&motorR);

    timerInit();

    //cin >> a;
    eulerPitchDesired =1.5;
    while(a>0)
    {
    	int Kp,Ki,Kd,angle;

    	cout<<"Kp:\r\n";
    	cin>>Kp;
    	cout<<"Ki:\r\n";
    	cin>>Ki;
    	//cout<<"Kd:\r\n";
    	//cin>>Kd;

    	cout<<"pwmMin:\r\n";
    	cin>>pwmMin;

    	cout<<"Angle:\r\n";
    	cin>>angle;

    	pidReset(&pidPitch);
    	pidInit(&pidPitch, angle, Kp, Ki, Kd, IMU_UPDATE_DT/2);
    	eulerPitchDesired = -angle;

    	cout<<"Continue?:\r\n";
    	cin>>a;
    }

    motorTerminate();
    //gpioTerminate();

    return 0;
}




