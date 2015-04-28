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
 * i2cdev.c - Functions to write to I2C devices
 */

//#include <stdint.h>
//#include <stdbool.h>

#include "header.h"

#if 0
static void i2cdevResetAndLowLevelInitBusI2c1(void);
static void i2cdevResetAndLowLevelInitBusI2c2(void);
static inline void i2cdevRuffLoopDelay(uint32_t us);
#endif

bool i2cdevCheckStatus(int result)
{
	bool status;

	if (result == PI_BAD_HANDLE || result == PI_BAD_PARAM ||
		result == PI_I2C_READ_FAILED || result == PI_I2C_WRITE_FAILED)
	{
		status = FALSE;
	}
	else
	{
		status = TRUE;
	}

	return status;
}

bool i2cdevReadByte(int handle, uint8_t memAddress, uint8_t *data)
{
   uint8_t byte;

   byte = i2cReadByteData(handle, memAddress);
   *data = byte;
   printf("i2c read address:[0x%x] value:[0x%x]\r\n", memAddress, byte);

   return i2cdevCheckStatus(byte);
}

bool i2cdevReadBit(int handle, uint8_t memAddress, uint8_t bitNum, uint8_t *data)
{
  uint8_t byte;
  
  byte = i2cReadByteData(handle, memAddress);
  *data = byte & (1 << bitNum);

  return i2cdevCheckStatus(byte);
}

bool i2cdevReadBits(int handle, uint8_t memAddress,
                    uint8_t bitStart, uint8_t length, uint8_t *data)
{
  bool status;
  uint8_t byte;

  if ((status = i2cdevReadByte(handle, memAddress, &byte)) == TRUE)
  {
      uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
      printf("i2c read byte register:[0x%x] byte:[0x%x] mask:0x%x ",memAddress, byte, mask);
      byte &= mask;
      printf("byte after mask:0x%x ",byte);
      byte >>= (bitStart - length + 1);
      *data = byte;
      printf("data:0x%x\r\n",byte);
  }
  return status;
}

#if 1
bool i2cdevRead(int handle, uint8_t memAddress,
               uint16_t len, uint8_t *data)
{
	uint8_t result = 0;
	bool status = TRUE;

	if (memAddress == I2CDEV_NO_MEM_ADDR)
	{
		result = i2cReadDevice(handle, (char*) data, len);
	}
	else
	{
		result = i2cReadI2CBlockData(handle, memAddress, (char*) data, len);
	}
#if 0
    printf("i2c read byte register:[0x%x] length:%d\r\n",memAddress, len);
    for(int i=0; i<len; i++)
    {
    	printf("0x%x ",data[i]);
    }

	if(i2cdevCheckStatus(result))
	{
		printf("[OK]\r\n");
	}
	else
	{
		printf("[FAIL]\r\n");
	}
#endif
#if 0
  if (memAddress != I2CDEV_NO_MEM_ADDR)
  {
    status = I2C_Master_BufferWrite(I2Cx, &memAddress,  1, INTERRUPT, devAddress << 1, I2C_TIMEOUT);
  }
  if (status)
  {
    //TODO: Fix DMA transfer if more then 3 bytes
    status = I2C_Master_BufferRead(I2Cx, (uint8_t*)data,  len, INTERRUPT, devAddress << 1, I2C_TIMEOUT);
  }
#endif
  return i2cdevCheckStatus(result);
}
#endif

bool i2cdevWriteByte(int handle, uint8_t memAddress, uint8_t data)
{
	  uint8_t byte = i2cWriteByteData(handle, memAddress, data);
#if 0
	  printf("i2c write byte register:[0x%x] value:[0x%x] ",memAddress,data);
		if(i2cdevCheckStatus(byte))
		{
			printf("[OK]\r\n");
		}
		else
		{
			printf("[FAIL]\r\n");
		}
#endif
		return i2cdevCheckStatus(byte);
}

bool i2cdevWriteBit(int handle, uint8_t memAddress, uint8_t bitNum, uint8_t data)
{
	uint8_t byte, result;
	printf("i2c write bit register:[0x%x] bit number:%d value:%d ",memAddress,bitNum,data);

	byte = i2cReadByteData(handle, memAddress);
	byte = (data != 0) ? (byte | (1 << bitNum)) : (byte & ~(1 << bitNum));

	printf("final value:[0x%x] ",byte);
	result = i2cWriteByteData(handle, memAddress, byte);
	if(i2cdevCheckStatus(result))
	{
		printf("[OK]\r\n");
	}
	else
	{
		printf("[FAIL]\r\n");
	}
	return i2cdevCheckStatus(result);
}

bool i2cdevWriteBits(int handle, uint8_t memAddress, uint8_t bitStart, uint8_t length, uint8_t data)
{
  bool status=0;
  uint8_t byte,result;

  printf("i2c write bits register:[0x%x] start bit:%d length:%d value:0x%x ",memAddress,bitStart, length, data);

  byte = i2cReadByteData(handle, memAddress);
  if(i2cdevCheckStatus(byte))
  {
      uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
      data <<= (bitStart - length + 1); // shift data into correct position
      data &= mask; // zero all non-important bits in data
      byte &= ~(mask); // zero all important bits in existing byte
      byte |= data; // combine data with existing byte
      printf("final value:[0x%x] ",byte);
      result = i2cWriteByteData(handle, memAddress, byte);
      status = i2cdevCheckStatus(result);
		if (status)
		{
			printf("[OK]\r\n");
		}
		else
		{
			printf("write [FAIL]\r\n");
		}

  }
  else
  {
	  printf("read [FAIL]\r\n");
  }

  return status;
}

bool i2cdevWrite(int handle, uint8_t memAddress,
                uint16_t len, uint8_t *data)
{
	uint8_t result = 0;

	  printf("i2c write bytes register:[0x%x] length:%d value:0x%x ",memAddress, len, data);

	if (memAddress == I2CDEV_NO_MEM_ADDR)
	{
		result = i2cWriteDevice(handle, (char*) data, len);
	}
	else
	{
		result = i2cWriteI2CBlockData(handle, memAddress, (char*) data, len);
	}

#if 0
	bool status;
	static uint8_t buffer[17];
	int i;

	if (memAddress != I2CDEV_NO_MEM_ADDR)
	{
		// Sorry ...
		if (len > 16)
			len = 16;

		if (len == 0)
			return 0;

		buffer[0] = memAddress;
		for (i = 0; i < len; i++)
			buffer[i + 1] = data[i];

		status = I2C_Master_BufferWrite(I2Cx, buffer, len + 1, INTERRUPT,
				devAddress << 1, I2C_TIMEOUT);
	}
	else
	{
		status = I2C_Master_BufferWrite(I2Cx, data, len, INTERRUPT,
				devAddress << 1, I2C_TIMEOUT);
	}
#endif

	if(i2cdevCheckStatus(result))
	{
		printf("[OK]\r\n");
	}
	else
	{
		printf("[FAIL]\r\n");
	}

	return i2cdevCheckStatus(result);
}

#if 0
static inline void i2cdevRuffLoopDelay(uint32_t us)
{
  volatile uint32_t delay;

  for(delay = I2CDEV_LOOPS_PER_US * us; delay > 0; delay--);
}

static void i2cdevResetAndLowLevelInitBusI2c1(void)
{
  /* Reset the I2C block */
  I2C_DeInit(I2C1);

  /* Make sure the bus is free by clocking it until any slaves release the bus. */
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* I2C1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);
  /* I2C1 SDA configuration */
  GPIO_InitStructure.GPIO_Pin = I2CDEV_I2C1_PIN_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* I2C1 SCL configuration */
  GPIO_InitStructure.GPIO_Pin = I2CDEV_I2C1_PIN_SCL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
  /* Check SDA line to determine if slave is asserting bus and clock out if so */
  while(GPIO_ReadInputDataBit(GPIOB, I2CDEV_I2C1_PIN_SDA) == Bit_RESET)
  {
    /* Set clock high */
    GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
    /* Wait for any clock stretching to finish. */
    GPIO_WAIT_LOW(GPIOB, I2CDEV_I2C1_PIN_SCL, 10 * I2CDEV_LOOPS_PER_MS);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);

    /* Generate a clock cycle */
    GPIO_ResetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
    GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  }

  /* Generate a start then stop condition */
  GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);

  /* Set data and clock high and wait for any clock stretching to finish. */
  GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SDA);
  GPIO_SetBits(GPIOB, I2CDEV_I2C1_PIN_SCL);
  GPIO_WAIT_LOW(GPIOB, I2CDEV_I2C1_PIN_SCL, 10 * I2CDEV_LOOPS_PER_MS);
  /* Wait for data to be high */
  GPIO_WAIT_HIGH(GPIOB, I2CDEV_I2C1_PIN_SDA, 10 * I2CDEV_LOOPS_PER_MS);

  /* Initialize the I2C block */
  I2C_LowLevel_Init(I2C1);

  /* Reset if I2C device is busy */
  if (I2C1->SR2 & 0x20)
  {
    /* Reset the I2C block */
    I2C_SoftwareResetCmd(I2C1, ENABLE);
    I2C_SoftwareResetCmd(I2C1, DISABLE);
  }
}

static void i2cdevResetAndLowLevelInitBusI2c2(void)
{
  /* Reset the I2C block */
  I2C_DeInit(I2C2);

  /* Make sure the bus is free by clocking it until any slaves release the bus. */
  GPIO_InitTypeDef  GPIO_InitStructure;

  /* I2C1 clock enable */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);
  /* I2C1 SDA configuration */
  GPIO_InitStructure.GPIO_Pin = I2CDEV_I2C2_PIN_SDA;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* I2C1 SCL configuration */
  GPIO_InitStructure.GPIO_Pin = I2CDEV_I2C2_PIN_SCL;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

  GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SDA);
  /* Check SDA line to determine if slave is asserting bus and clock out if so */
  while(GPIO_ReadInputDataBit(GPIOB, I2CDEV_I2C2_PIN_SDA) == Bit_RESET)
  {
    /* Set clock high */
    GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SCL);
    /* Wait for any clock stretching to finish. */
    GPIO_WAIT_LOW(GPIOB, I2CDEV_I2C2_PIN_SCL, 10 * I2CDEV_LOOPS_PER_MS);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);

    /* Generate a clock cycle */
    GPIO_ResetBits(GPIOB, I2CDEV_I2C2_PIN_SCL);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
    GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SCL);
    i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  }

  /* Generate a start then stop condition */
  GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SCL);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(GPIOB, I2CDEV_I2C2_PIN_SDA);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);
  GPIO_ResetBits(GPIOB, I2CDEV_I2C2_PIN_SDA);
  i2cdevRuffLoopDelay(I2CDEV_CLK_TS);

  /* Set data and clock high and wait for any clock stretching to finish. */
  GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SDA);
  GPIO_SetBits(GPIOB, I2CDEV_I2C2_PIN_SCL);
  GPIO_WAIT_LOW(GPIOB, I2CDEV_I2C2_PIN_SCL, 10 * I2CDEV_LOOPS_PER_MS);
  /* Wait for data to be high */
  GPIO_WAIT_HIGH(GPIOB, I2CDEV_I2C2_PIN_SDA, 10 * I2CDEV_LOOPS_PER_MS);

  /* Initialize the I2C block */
  I2C_LowLevel_Init(I2C2);

  /* Reset if I2C device is busy */
  if (I2C2->SR2 & 0x20)
  {
    /* Reset the I2C block */
    I2C_SoftwareResetCmd(I2C2, ENABLE);
    I2C_SoftwareResetCmd(I2C2, DISABLE);
  }
}

void i2cDmaInterruptHandlerI2c1(void)
{
  if(DMA_GetITStatus(DMA1_IT_TC6))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC6);

    xSemaphoreGive(i2cdevDmaEventI2c1);
  }
  if(DMA_GetITStatus(DMA1_IT_TC7))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC7);

    xSemaphoreGive(i2cdevDmaEventI2c1);
  }
}

void i2cDmaInterruptHandlerI2c2(void)
{
  if(DMA_GetITStatus(DMA1_IT_TC4))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC4);

    xSemaphoreGive(i2cdevDmaEventI2c2);
  }
  if(DMA_GetITStatus(DMA1_IT_TC5))
  {
    DMA_ClearITPendingBit(DMA1_IT_TC5);

    xSemaphoreGive(i2cdevDmaEventI2c2);
  }
}
#endif
