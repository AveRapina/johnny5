/**
 *    ||          ____  _ __                           
 * +------+      / __ )(_) /_______________ _____  ___ 
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 *
 * imu.c - inertial measurement unit
 */
#define DEBUG_MODULE "IMU"

#include "header.h"

static BiasObj    gyroBias;
static BiasObj    accelBias;
static int32_t    varianceSampleTime;
static Axis3i16   gyroMpu;
static Axis3i16   accelMpu;
static Axis3i16   accelLPF;
static Axis3i16   accelLPFAligned;
static Axis3i16   mag;
static Axis3i32   accelStoredFilterValues;
static uint8_t    imuAccLpfAttFactor;
static bool       isHmc5883lPresent;
static bool       isMs5611Present;

static bool isMpu6050TestPassed;
static bool isHmc5883lTestPassed;
static bool isMs5611TestPassed;

// Pre-calculated values for accelerometer alignment
static float cosPitch;
static float sinPitch;
static float cosRoll;
static float sinRoll;

/**
 * MPU6050 selt test function. If the chip is moved to much during the self test
 * it will cause the test to fail.
 */
static void imuBiasInit(BiasObj* bias);
static void imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut);
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut);
static bool imuFindBiasValue(BiasObj* bias);
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal);
static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out,
                              Axis3i32* storedValues, int32_t attenuation);
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out);

// TODO: Fix __errno linker error with math lib
int __attribute__((used)) __errno;

static bool isInit;

IMU imu;		//传感器数据汇总

void imu6Init(void)
{
  if(isInit)
    return;

 isHmc5883lPresent = FALSE;
 isMs5611Present = FALSE;

  // Wait for sensors to startup
  while (gpioTick() < IMU_STARTUP_TIME_MS);

  //i2cdevInit();
  mpu6050Init();
  if (mpu6050TestConnection() == TRUE)
  {
    DEBUG_PRINT("MPU6050 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("MPU6050 I2C connection [FAIL].\n");
  }

  mpu6050Reset();
  usleep(50000);
  // Activate MPU6050
  printf("\r\nSet sleep enable \r\n");
  //mpu6050SetSleepEnabled(FALSE);
  mpu6050SetSleepEnabled(0);
  // Enable temp sensor
  printf("\r\nSet temperature sensor enable \r\n");
  mpu6050SetTempSensorEnabled(TRUE);
  // Disable interrupts
  printf("\r\ndisable interrupts \r\n");
  mpu6050SetIntEnabled(FALSE);
  // Connect the HMC5883L to the main I2C bus
  printf("\r\nset i2c bypass enable \r\n");
  mpu6050SetI2CBypassEnabled(TRUE);
  // Set x-axis gyro as clock source
  printf("\r\nset gyro clock source \r\n");
  mpu6050SetClockSource(MPU6050_CLOCK_INTERNAL);
  // Set gyro full scale range
  printf("\r\nset gyro full scale range \r\n");
  mpu6050SetFullScaleGyroRange(IMU_GYRO_FS_CFG);
  // Set accelerometer full scale range
  printf("\r\nSet accelerometer full scale range \r\n");
  mpu6050SetFullScaleAccelRange(IMU_ACCEL_FS_CFG);

#ifdef IMU_MPU6050_DLPF_256HZ
  // 256Hz digital low-pass filter only works with little vibrations
  // Set output rate (15): 8000 / (1 + 15) = 500Hz
  mpu6050SetRate(15);
  // Set digital low-pass bandwidth
  mpu6050SetDLPFMode(MPU6050_DLPF_BW_256);
#else
  // To low DLPF bandwidth might cause instability and decrease agility
  // but it works well for handling vibrations and unbalanced propellers
  // Set output rate (1): 1000 / (1 + 1) = 500Hz
  printf("\r\nSet sample rate \r\n");
  mpu6050SetRate(1);
  // Set digital low-pass bandwidth
  printf("\r\nSet DLPF low-pass bandwidth \r\n");
  mpu6050SetDLPFMode(MPU6050_DLPF_BW_188);
#endif


#ifdef IMU_ENABLE_MAG_HMC5883
  hmc5883lInit();
  if (hmc5883lTestConnection() == TRUE)
  {
    isHmc5883lPresent = TRUE;
    DEBUG_PRINT("HMC5883 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("HMC5883L I2C connection [FAIL].\n");
  }
#endif

#ifdef IMU_ENABLE_PRESSURE_MS5611
  if (ms5611Init() == TRUE)
  {
    isMs5611Present = TRUE;
    DEBUG_PRINT("MS5611 I2C connection [OK].\n");
  }
  else
  {
    DEBUG_PRINT("MS5611 I2C connection [FAIL].\n");
  }
#endif

  imuBiasInit(&gyroBias);
  imuBiasInit(&accelBias);
  varianceSampleTime = -GYRO_MIN_BIAS_TIMEOUT_MS + 1;
  imuAccLpfAttFactor = IMU_ACC_IIR_LPF_ATT_FACTOR;

#if 0
  cosPitch = cos(configblockGetCalibPitch() * M_PI/180);
  sinPitch = sin(configblockGetCalibPitch() * M_PI/180);
  cosRoll = cos(configblockGetCalibRoll() * M_PI/180);
  sinRoll = sin(configblockGetCalibRoll() * M_PI/180);
#endif
  cosPitch = cos(0);
  sinPitch = sin(0);
  cosRoll = cos(0);
  sinRoll = sin(0);

  isInit = TRUE;
}

bool imu6Test(void)
{
  bool testStatus = TRUE;

  if (!isInit)
  {
    DEBUG_PRINT("Uninitialized");
    testStatus = FALSE;
  }
#if 0
  // Test for CF 10-DOF variant with none responding sensor
  if((isHmc5883lPresent && !isMs5611Present) ||
     (!isHmc5883lPresent && isMs5611Present))
  {
    DEBUG_PRINT("HMC5883L or MS5611 is not responding");
    testStatus = FALSE;
  }
#endif
  if (testStatus)
  {
    isMpu6050TestPassed = mpu6050SelfTest();
    testStatus = isMpu6050TestPassed ;
  }
  if (testStatus)// && isHmc5883lPresent)
  {
    isHmc5883lTestPassed = hmc5883lSelfTest();
    testStatus = isHmc5883lTestPassed;
  }
#if IMU_ENABLE_PRESSURE_MS5611
  if (testStatus && isMs5611Present)
  {
    isMs5611TestPassed = ms5611SelfTest();
    testStatus = isMs5611TestPassed;
  }
#endif
  return testStatus;
}


void imu6Read(Axis3f* gyroOut, Axis3f* accOut)
{
  mpu6050GetMotion6(&accelMpu.x, &accelMpu.y, &accelMpu.z, &gyroMpu.x, &gyroMpu.y, &gyroMpu.z);

  imuAddBiasValue(&gyroBias, &gyroMpu);
  if (!accelBias.isBiasValueFound)
  {
    imuAddBiasValue(&accelBias, &accelMpu);
  }
  if (!gyroBias.isBiasValueFound)
  {
    imuFindBiasValue(&gyroBias);
    if (gyroBias.isBiasValueFound)
    {
    	printf("gyroBias.isBiasValueFound");
      //ledseqRun(LED_RED, seq_calibrated);
//      uartPrintf("Gyro bias: %i, %i, %i\n",
//                  gyroBias.bias.x, gyroBias.bias.y, gyroBias.bias.z);
    }
  }

#ifdef IMU_TAKE_ACCEL_BIAS
  if (gyroBias.isBiasValueFound &&
      !accelBias.isBiasValueFound)
  {
    Axis3i32 mean;

    imuCalculateBiasMean(&accelBias, &mean);
    accelBias.bias.x = mean.x;
    accelBias.bias.y = mean.y;
    accelBias.bias.z = mean.z - IMU_1G_RAW;
    accelBias.isBiasValueFound = TRUE;
    //uartPrintf("Accel bias: %i, %i, %i\n",
    //            accelBias.bias.x, accelBias.bias.y, accelBias.bias.z);
  }
#endif


  imuAccIIRLPFilter(&accelMpu, &accelLPF, &accelStoredFilterValues,
                    (int32_t)imuAccLpfAttFactor);

  imuAccAlignToGravity(&accelLPF, &accelLPFAligned);

  // Re-map outputs
  gyroOut->x = (gyroMpu.x - gyroBias.bias.x) * IMU_DEG_PER_LSB_CFG;
  gyroOut->y = (gyroMpu.y - gyroBias.bias.y) * IMU_DEG_PER_LSB_CFG;
  gyroOut->z = (gyroMpu.z - gyroBias.bias.z) * IMU_DEG_PER_LSB_CFG;
  accOut->x = (accelLPFAligned.x - accelBias.bias.x) * IMU_G_PER_LSB_CFG;
  accOut->y = (accelLPFAligned.y - accelBias.bias.y) * IMU_G_PER_LSB_CFG;
  accOut->z = (accelLPFAligned.z - accelBias.bias.z) * IMU_G_PER_LSB_CFG;

  //accOut->x = (accelLPFAligned.x) * IMU_G_PER_LSB_CFG;
  //accOut->y = (accelLPFAligned.y) * IMU_G_PER_LSB_CFG;
  //accOut->z = (accelLPFAligned.z) * IMU_G_PER_LSB_CFG;
}

void imu9Read(Axis3f* gyroOut, Axis3f* accOut, Axis3f* magOut)
{
  imu6Read(gyroOut, accOut);

  if (isHmc5883lPresent)
  {
    hmc5883lGetHeading(&mag.x, &mag.y, &mag.z);
    magOut->x = (float)mag.x / MAG_GAUSS_PER_LSB;
    magOut->y = (float)mag.y / MAG_GAUSS_PER_LSB;
    magOut->z = (float)mag.z / MAG_GAUSS_PER_LSB;
  }
  else
  {
    magOut->x = 0.0;
    magOut->y = 0.0;
    magOut->z = 0.0;
  }
}

bool imu6IsCalibrated(void)
{
  bool status;

  status = gyroBias.isBiasValueFound;
#ifdef IMU_TAKE_ACCEL_BIAS
  status &= accelBias.isBiasValueFound;
#endif

  return status;
}

bool imuHasBarometer(void)
{
  return isMs5611Present;
}

bool imuHasMangnetometer(void)
{
  return isHmc5883lPresent;
}

static void imuBiasInit(BiasObj* bias)
{
  bias->isBufferFilled = FALSE;
  bias->bufHead = bias->buffer;
}

/**
 * Calculates the variance and mean for the bias buffer.
 */
static void imuCalculateVarianceAndMean(BiasObj* bias, Axis3i32* varOut, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};
  int64_t sumSq[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
    sumSq[0] += bias->buffer[i].x * bias->buffer[i].x;
    sumSq[1] += bias->buffer[i].y * bias->buffer[i].y;
    sumSq[2] += bias->buffer[i].z * bias->buffer[i].z;
  }

  varOut->x = (sumSq[0] - ((int64_t)sum[0] * sum[0]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->y = (sumSq[1] - ((int64_t)sum[1] * sum[1]) / IMU_NBR_OF_BIAS_SAMPLES);
  varOut->z = (sumSq[2] - ((int64_t)sum[2] * sum[2]) / IMU_NBR_OF_BIAS_SAMPLES);

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

  isInit = TRUE;
}

/**
 * Calculates the mean for the bias buffer.
 */
static void __attribute__((used)) imuCalculateBiasMean(BiasObj* bias, Axis3i32* meanOut)
{
  uint32_t i;
  int32_t sum[GYRO_NBR_OF_AXES] = {0};

  for (i = 0; i < IMU_NBR_OF_BIAS_SAMPLES; i++)
  {
    sum[0] += bias->buffer[i].x;
    sum[1] += bias->buffer[i].y;
    sum[2] += bias->buffer[i].z;
  }

  meanOut->x = sum[0] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->y = sum[1] / IMU_NBR_OF_BIAS_SAMPLES;
  meanOut->z = sum[2] / IMU_NBR_OF_BIAS_SAMPLES;

}

/**
 * Adds a new value to the variance buffer and if it is full
 * replaces the oldest one. Thus a circular buffer.
 */
static void imuAddBiasValue(BiasObj* bias, Axis3i16* dVal)
{
  bias->bufHead->x = dVal->x;
  bias->bufHead->y = dVal->y;
  bias->bufHead->z = dVal->z;
  bias->bufHead++;

  if (bias->bufHead >= &bias->buffer[IMU_NBR_OF_BIAS_SAMPLES])
  {
    bias->bufHead = bias->buffer;
    bias->isBufferFilled = TRUE;
  }
}

/**
 * Checks if the variances is below the predefined thresholds.
 * The bias value should have been added before calling this.
 * @param bias  The bias object
 */
static bool imuFindBiasValue(BiasObj* bias)
{
  bool foundBias = FALSE;

  if (bias->isBufferFilled)
  {
    Axis3i32 variance;
    Axis3i32 mean;

    imuCalculateVarianceAndMean(bias, &variance, &mean);

    //uartSendData(sizeof(variance), (uint8_t*)&variance);
    //uartSendData(sizeof(mean), (uint8_t*)&mean);
    //uartPrintf("%i, %i, %i", variance.x, variance.y, variance.z);
    //uartPrintf("    %i, %i, %i\n", mean.x, mean.y, mean.z);

    if (variance.x < GYRO_VARIANCE_THRESHOLD_X &&
        variance.y < GYRO_VARIANCE_THRESHOLD_Y &&
        variance.z < GYRO_VARIANCE_THRESHOLD_Z &&
        (varianceSampleTime + GYRO_MIN_BIAS_TIMEOUT_MS < gpioTick()))
    {
      varianceSampleTime = gpioTick();
      bias->bias.x = mean.x;
      bias->bias.y = mean.y;
      bias->bias.z = mean.z;
      foundBias = TRUE;
      bias->isBiasValueFound = TRUE;
    }
  }

  return foundBias;
}

static void imuAccIIRLPFilter(Axis3i16* in, Axis3i16* out, Axis3i32* storedValues, int32_t attenuation)
{
  out->x = iirLPFilterSingle(in->x, attenuation, &storedValues->x);
  out->y = iirLPFilterSingle(in->y, attenuation, &storedValues->y);
  out->z = iirLPFilterSingle(in->z, attenuation, &storedValues->z);
}


/**
 * Compensate for a miss-aligned accelerometer. It uses the trim
 * data gathered from the UI and written in the config-block to
 * rotate the accelerometer to be aligned with gravity.
 */
static void imuAccAlignToGravity(Axis3i16* in, Axis3i16* out)
{
  Axis3i16 rx;
  Axis3i16 ry;

  // Rotate around x-axis
  rx.x = in->x;
  rx.y = in->y * cosRoll - in->z * sinRoll;
  rx.z = in->y * sinRoll + in->z * cosRoll;

  // Rotate around y-axis
  ry.x = rx.x * cosPitch - rx.z * sinPitch;
  ry.y = rx.y;
  ry.z = -rx.x * sinPitch + rx.z * cosPitch;

  out->x = ry.x;
  out->y = ry.y;
  out->z = ry.z;
}

void imuUpdate()
{
	imu9Read(&imu.gyro, &imu.acc, &imu.mag);	//读取9轴数据

	// 250HZ
	sensfusion6UpdateQ(imu.gyro, imu.acc, FUSION_UPDATE_DT);	//数据融合 更新四元数
	sensfusion6GetEulerRPY(&imu.euler);			//计算欧拉角

	imu.accWZ = sensfusion6GetAccZWithoutGravity(imu.acc);	//计算Z轴加速度
	imu.accMAG = (imu.acc.x * imu.acc.x) + (imu.acc.y * imu.acc.y) + (imu.acc.z * imu.acc.z);
}

using namespace std;

void printImuData()
{
#if 0
	printf("gyro x:%10f y:%10f z:%12f\t", gyro.x, gyro.y, gyro.z);
	printf("acc x:%10f y:%10f z:%12f\t", acc.x, acc.y, acc.z);
	printf("mag x:%10f y:%10f z:%12f\r", mag.x, mag.y, mag.z);
#endif

	//printf("Roll:%6.3f Pitch:%6.3f Yaw:%6.3f\t", imu.euler.roll, imu.euler.pitch, imu.euler.yaw);

	cout << "Roll:"<<fixed<<setw(8)<<setprecision(3)<<imu.euler.roll<<"  ";
	cout << "Pitch:"<<fixed<<setw(8)<<setprecision(3)<<imu.euler.pitch<<"  ";
	cout << "Yaw:"<<fixed<<setw(8)<<setprecision(3)<<imu.euler.yaw<<endl;
}
