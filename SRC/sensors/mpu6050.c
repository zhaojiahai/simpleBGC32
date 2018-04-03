/*
  Sept 2013

  bgc32 Rev -

  Copyright (c) 2013 John Ihlein.  All rights reserved.

  Open Source STM32 Based Brushless Gimbal Controller Software

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick
  6)UAVX

  Designed to run on the EvvGC Brushless Gimbal Controller Board

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program. If not, see <http://www.gnu.org/licenses/>.
*/

///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////
// MPU6050 Defines and Variables
///////////////////////////////////////////////////////////////////////////////

// Registers

#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_FIFO_EN             0x23
#define MPU6050_INT_PIN_CFG         0x37
#define MPU6050_INT_ENABLE          0x38
#define MPU6050_INT_STATUS          0x3A
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48
#define MPU6050_USER_CTRL           0x6A
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_PWR_MGMT_2          0x6C
#define MPU6050_FIFO_COUNTH         0x72
#define MPU6050_FIFO_COUNTL         0x73
#define MPU6050_FIFO_R_W            0x74
#define MPU6050_WHOAMI              0x75

// Bits

#define BIT_SLEEP                   0x40
#define BIT_H_RESET                 0x80
#define BITS_CLKSEL                 0x07
#define MPU_CLK_SEL_PLLGYROX        0x01
#define MPU_CLK_SEL_PLLGYROZ        0x03
#define MPU_EXT_SYNC_GYROX          0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_2G                  0x00
#define BITS_FS_4G                  0x08
#define BITS_FS_8G                  0x10
#define BITS_FS_16G                 0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR        0x10
#define BIT_RAW_RDY_EN              0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA         0x01

///////////////////////////////////////

float accelOneG = 9.8065;

int16_t accelData500Hz[3];

float accelTCBias[3] = { 0.0f, 0.0f, 0.0f };

int16andUint8_t rawAccel[3];

///////////////////////////////////////

float gyroRTBias[3];

int16_t gyroData500Hz[3];

float gyroTCBias[3];

int16andUint8_t rawGyro[3];

///////////////////////////////////////

uint8_t mpu6050Calibrating = false;

float   mpu6050Temperature;

int16andUint8_t rawMPU6050Temperature;

///////////////////////////////////////

int16_t orientationMatrix[9];

///////////////////////////////////////////////////////////////////////////////
// MPU6050 Initialization
///////////////////////////////////////////////////////////////////////////////

void initMPU6050(void)
{
	i2cWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,   BIT_H_RESET);               // Device Reset

	delay(150);

	i2cWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_1,   MPU_CLK_SEL_PLLGYROZ);      // Clock Source
	i2cWrite(MPU6050_ADDRESS, MPU6050_PWR_MGMT_2,   0x00);                      // turn off all standby
	i2cWrite(MPU6050_ADDRESS, MPU6050_SMPLRT_DIV,   0x00);                      // Accel Sample Rate 1000 Hz, Gyro Sample Rate 8000 Hz
	i2cWrite(MPU6050_ADDRESS, MPU6050_CONFIG,       eepromConfig.dlpfSetting);  // Accel and Gyro DLPF Setting
	i2cWrite(MPU6050_ADDRESS, MPU6050_ACCEL_CONFIG, BITS_FS_4G);                // Accel +/- 4 G Full Scale
	i2cWrite(MPU6050_ADDRESS, MPU6050_GYRO_CONFIG,  BITS_FS_500DPS);            // Gyro +/- 500 DPS Full Scale
	//    i2cWrite(MPU6050_ADDRESS, MPU6050_INT_PIN_CFG,  0x00);                      // int pin config
	//    i2cWrite(MPU6050_ADDRESS, MPU6050_INT_ENABLE,   0x00);                      // int pin disable

	///////////////////////////////////

	delay(100);

	computeMPU6050RTData();
}

///////////////////////////////////////////////////////////////////////////////
// Read MPU6050
///////////////////////////////////////////////////////////////////////////////

void readMPU6050(void)
{
	uint8_t axis;

	uint8_t I2C2_Buffer_Rx[14];

	int16_t straightAccelData[3];
	int16_t rotatedAccelData[3];

	int16_t straightGyroData[3];
	int16_t rotatedGyroData[3];

	i2cRead(MPU6050_ADDRESS, MPU6050_ACCEL_XOUT_H, 14, I2C2_Buffer_Rx);//从MPU6050读取原始加速度和陀螺仪数据，还有温度数据

	rawAccel[YAXIS].bytes[1]       = I2C2_Buffer_Rx[ 0];//传输原始数据
	rawAccel[YAXIS].bytes[0]       = I2C2_Buffer_Rx[ 1];
	rawAccel[XAXIS].bytes[1]       = I2C2_Buffer_Rx[ 2];
	rawAccel[XAXIS].bytes[0]       = I2C2_Buffer_Rx[ 3];
	rawAccel[ZAXIS].bytes[1]       = I2C2_Buffer_Rx[ 4];
	rawAccel[ZAXIS].bytes[0]       = I2C2_Buffer_Rx[ 5];

	rawMPU6050Temperature.bytes[1] = I2C2_Buffer_Rx[ 6];
	rawMPU6050Temperature.bytes[0] = I2C2_Buffer_Rx[ 7];

	rawGyro[PITCH].bytes[1]        = I2C2_Buffer_Rx[ 8];
	rawGyro[PITCH].bytes[0]        = I2C2_Buffer_Rx[ 9];
	rawGyro[ROLL ].bytes[1]        = I2C2_Buffer_Rx[10];
	rawGyro[ROLL ].bytes[0]        = I2C2_Buffer_Rx[11];
	rawGyro[YAW  ].bytes[1]        = I2C2_Buffer_Rx[12];
	rawGyro[YAW  ].bytes[0]        = I2C2_Buffer_Rx[13];

	for (axis = 0; axis < 3; axis++)
	{
		straightAccelData[axis] = rawAccel[axis].value;//获取加速度矩阵B的值
		straightGyroData[axis]  = rawGyro[axis].value;//获取陀螺仪矩阵B的值
	}


	//矩阵orientationMatrix * 矩阵straightAccelData ，矩阵orientationMatrix维度是3*3 ，矩阵straightAccelData维度是3*1

	//结果放到矩阵rotatedAccelData ,矩阵维度是3*1
	matrixMultiply(3, 3, 1, rotatedAccelData, orientationMatrix, straightAccelData);



	//矩阵orientationMatrix * 矩阵straightGyroData ，矩阵orientationMatrix维度是3*3 ，矩阵straightGyroData维度是3*1

	//结果放到矩阵rotatedGyroData ,矩阵维度是3*1
	matrixMultiply(3, 3, 1, rotatedGyroData,  orientationMatrix, straightGyroData);

	for (axis = 0; axis < 3; axis++)
	{
		rawAccel[axis].value = rotatedAccelData[axis];//传递矩阵相乘后的结果
		rawGyro[axis].value  = rotatedGyroData[axis];
	}
}

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6050 Runtime Data
///////////////////////////////////////////////////////////////////////////////

void computeMPU6050RTData(void)
{
	uint8_t  axis;
	uint16_t samples;

	double accelSum[3]    = { 0.0f, 0.0f, 0.0f };
	double gyroSum[3]     = { 0.0f, 0.0f, 0.0f };

	mpu6050Calibrating = true;

	for (samples = 0; samples < 5000; samples++)
	{
		readMPU6050();

		computeMPU6050TCBias();
		//矩阵相乘后的结果-温度补偿偏差，每次都对结果进行积分，一共积分5000次
		accelSum[XAXIS] += (float)rawAccel[XAXIS].value - accelTCBias[XAXIS];
		accelSum[YAXIS] += (float)rawAccel[YAXIS].value - accelTCBias[YAXIS];
		accelSum[ZAXIS] += (float)rawAccel[ZAXIS].value - accelTCBias[ZAXIS];

		gyroSum[ROLL ]  += (float)rawGyro[ROLL ].value  - gyroTCBias[ROLL ];
		gyroSum[PITCH]  += (float)rawGyro[PITCH].value  - gyroTCBias[PITCH];
		gyroSum[YAW  ]  += (float)rawGyro[YAW  ].value  - gyroTCBias[YAW  ];

		delayMicroseconds(1000);//延时1000us
	}

	for (axis = 0; axis < 3; axis++)
	{

		//对之前积分的5000次加速度数据的结果求平均后然后乘以最小刻度值(1/8192) * 9.8065
		accelSum[axis]   = accelSum[axis] / 5000.0f * ACCEL_SCALE_FACTOR;



		//对之前积分的5000次加速度数据的结果直接求平均
		gyroRTBias[axis] = gyroSum[axis]  / 5000.0f;
	}


	//加速度标准化
	accelOneG = sqrt(SQR(accelSum[XAXIS]) + SQR(accelSum[YAXIS]) + SQR(accelSum[ZAXIS]));

	mpu6050Calibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
// Compute MPU6050 Temperature Compensation Bias
///////////////////////////////////////////////////////////////////////////////

void computeMPU6050TCBias(void)
{
	mpu6050Temperature = (float)(rawMPU6050Temperature.value) / 340.0f + 35.0f;//得到温度


	//	计算温度补偿偏差值
	accelTCBias[XAXIS] = eepromConfig.accelTCBiasSlope[XAXIS] * mpu6050Temperature + eepromConfig.accelTCBiasIntercept[XAXIS];
	accelTCBias[YAXIS] = eepromConfig.accelTCBiasSlope[YAXIS] * mpu6050Temperature + eepromConfig.accelTCBiasIntercept[YAXIS];
	accelTCBias[ZAXIS] = eepromConfig.accelTCBiasSlope[ZAXIS] * mpu6050Temperature + eepromConfig.accelTCBiasIntercept[ZAXIS];

	gyroTCBias[ROLL ]  = eepromConfig.gyroTCBiasSlope[ROLL ]  * mpu6050Temperature + eepromConfig.gyroTCBiasIntercept[ROLL ];
	gyroTCBias[PITCH]  = eepromConfig.gyroTCBiasSlope[PITCH]  * mpu6050Temperature + eepromConfig.gyroTCBiasIntercept[PITCH];
	gyroTCBias[YAW  ]  = eepromConfig.gyroTCBiasSlope[YAW  ]  * mpu6050Temperature + eepromConfig.gyroTCBiasIntercept[YAW  ];
}

///////////////////////////////////////////////////////////////////////////////
// Orient IMU
///////////////////////////////////////////////////////////////////////////////

void orientIMU(void) //根据IMU单元的方位确定矩阵A的值
{
	switch (eepromConfig.imuOrientation)
	{
		case 1: // Dot Front/Left/Top
			orientationMatrix[0] =  1;
			orientationMatrix[1] =  0;
			orientationMatrix[2] =  0;
			orientationMatrix[3] =  0;
			orientationMatrix[4] =  1;
			orientationMatrix[5] =  0;
			orientationMatrix[6] =  0;
			orientationMatrix[7] =  0;
			orientationMatrix[8] =  1;
			break;

		case 2: // Dot Front/Right/Top
			orientationMatrix[0] =  0;
			orientationMatrix[1] = -1;
			orientationMatrix[2] =  0;
			orientationMatrix[3] =  1;
			orientationMatrix[4] =  0;
			orientationMatrix[5] =  0;
			orientationMatrix[6] =  0;
			orientationMatrix[7] =  0;
			orientationMatrix[8] =  1;
			break;

		case 3: // Dot Back/Right/Top
			orientationMatrix[0] = -1;
			orientationMatrix[1] =  0;
			orientationMatrix[2] =  0;
			orientationMatrix[3] =  0;
			orientationMatrix[4] = -1;
			orientationMatrix[5] =  0;
			orientationMatrix[6] =  0;
			orientationMatrix[7] =  0;
			orientationMatrix[8] =  1;
			break;

		case 4: // Dot Back/Left/Top
			orientationMatrix[0] =  0;
			orientationMatrix[1] =  1;
			orientationMatrix[2] =  0;
			orientationMatrix[3] = -1;
			orientationMatrix[4] =  0;
			orientationMatrix[5] =  0;
			orientationMatrix[6] =  0;
			orientationMatrix[7] =  0;
			orientationMatrix[8] =  1;
			break;

		case 5: // Dot Front/Left/Bottom
			orientationMatrix[0] =  0;
			orientationMatrix[1] = -1;
			orientationMatrix[2] =  0;
			orientationMatrix[3] = -1;
			orientationMatrix[4] =  0;
			orientationMatrix[5] =  0;
			orientationMatrix[6] =  0;
			orientationMatrix[7] =  0;
			orientationMatrix[8] = -1;
			break;

		case 6: // Dot Front/Right/Bottom
			orientationMatrix[0] =  1;
			orientationMatrix[1] =  0;
			orientationMatrix[2] =  0;
			orientationMatrix[3] =  0;
			orientationMatrix[4] = -1;
			orientationMatrix[5] =  0;
			orientationMatrix[6] =  0;
			orientationMatrix[7] =  0;
			orientationMatrix[8] = -1;
			break;

		case 7: // Dot Back/Right/Bottom
			orientationMatrix[0] =  0;
			orientationMatrix[1] =  1;
			orientationMatrix[2] =  0;
			orientationMatrix[3] =  1;
			orientationMatrix[4] =  0;
			orientationMatrix[5] =  0;
			orientationMatrix[6] =  0;
			orientationMatrix[7] =  0;
			orientationMatrix[8] = -1;
			break;

		case 8: // Dot Back/Left/Bottom
			orientationMatrix[0] = -1;
			orientationMatrix[1] =  0;
			orientationMatrix[2] =  0;
			orientationMatrix[3] =  0;
			orientationMatrix[4] =  1;
			orientationMatrix[5] =  0;
			orientationMatrix[6] =  0;
			orientationMatrix[7] =  0;
			orientationMatrix[8] = -1;
			break;

		default: // Dot Front/Left/Top
			orientationMatrix[0] =  1;
			orientationMatrix[1] =  0;
			orientationMatrix[2] =  0;
			orientationMatrix[3] =  0;
			orientationMatrix[4] =  1;
			orientationMatrix[5] =  0;
			orientationMatrix[6] =  0;
			orientationMatrix[7] =  0;
			orientationMatrix[8] =  1;
			break;
	}
}

///////////////////////////////////////////////////////////////////////////////

