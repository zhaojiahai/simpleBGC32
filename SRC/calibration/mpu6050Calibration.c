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
// MPU6050 Calibration
///////////////////////////////////////////////////////////////////////////////

void mpu6050Calibration(void)
{
	uint16_t sampleRate      = 1000;
	uint16_t numberOfSamples = 2000;

	float accelBias1[3]       = { 0.0f, 0.0f, 0.0f };
	float gyroBias1[3]        = { 0.0f, 0.0f, 0.0f };
	float mpu6050Temperature1 = 0.0f;

	float accelBias2[3]       = { 0.0f, 0.0f, 0.0f };
	float gyroBias2[3]        = { 0.0f, 0.0f, 0.0f };
	float mpu6050Temperature2 = 0.0f;

	uint16_t index;

	mpu6050Calibrating = true;

	cliPrintF("\nMPU6050 Calibration:\n");

	///////////////////////////////////
	// Get samples at temperature1
	///////////////////////////////////

	cliPrintF("\nBegin 1st MPU6050 Measurements...\n");

	for (index = 0; index < numberOfSamples; index++)
	{
		readMPU6050();

		rawAccel[ZAXIS].value = rawAccel[ZAXIS].value - 8192;

		accelBias1[XAXIS]    += rawAccel[XAXIS].value;
		accelBias1[YAXIS]    += rawAccel[YAXIS].value;
		accelBias1[ZAXIS]    += rawAccel[ZAXIS].value;
		gyroBias1[ROLL ]     += rawGyro[ROLL ].value;
		gyroBias1[PITCH]     += rawGyro[PITCH].value;
		gyroBias1[YAW  ]     += rawGyro[YAW  ].value;
		mpu6050Temperature1  += (float)(rawMPU6050Temperature.value) / 340.0f + 35.0f;

		delayMicroseconds(sampleRate);
	}

	accelBias1[XAXIS]   /= (float) numberOfSamples;
	accelBias1[YAXIS]   /= (float) numberOfSamples;
	accelBias1[ZAXIS]   /= (float) numberOfSamples;
	gyroBias1[ROLL ]    /= (float) numberOfSamples;
	gyroBias1[PITCH]    /= (float) numberOfSamples;
	gyroBias1[YAW  ]    /= (float) numberOfSamples;
	mpu6050Temperature1 /= (float) numberOfSamples;

	cliPrintF("\nGyro Temperature Reading: %6.2f", mpu6050Temperature1);

	cliPrintF("\n\nEnd 1st MPU6050 Measurements\n");

	///////////////////////////////////
	// Time delay for temperature
	// Stabilization
	///////////////////////////////////

	cliPrintF("\nWaiting for 10 minutes for MPU6050 temp to rise...\n");
	delay(600000);    // Number of mSec in 10 minutes

	///////////////////////////////////
	// Get samples at temperature2
	///////////////////////////////////

	cliPrintF("\nBegin 2nd MPU6050 Measurements...\n");

	for (index = 0; index < numberOfSamples; index++)
	{
		readMPU6050();

		rawAccel[ZAXIS].value = rawAccel[ZAXIS].value - 8192;

		accelBias2[XAXIS]    += rawAccel[XAXIS].value;
		accelBias2[YAXIS]    += rawAccel[YAXIS].value;
		accelBias2[ZAXIS]    += rawAccel[ZAXIS].value;
		gyroBias2[ROLL ]     += rawGyro[ROLL ].value;
		gyroBias2[PITCH]     += rawGyro[PITCH].value;
		gyroBias2[YAW  ]     += rawGyro[YAW  ].value;
		mpu6050Temperature2  += (float)(rawMPU6050Temperature.value) / 340.0f + 35.0f;

		delayMicroseconds(sampleRate);
	}

	accelBias2[XAXIS]   /= (float) numberOfSamples;
	accelBias2[YAXIS]   /= (float) numberOfSamples;
	accelBias2[ZAXIS]   /= (float) numberOfSamples;
	gyroBias2[ROLL ]    /= (float) numberOfSamples;
	gyroBias2[PITCH]    /= (float) numberOfSamples;
	gyroBias2[YAW  ]    /= (float) numberOfSamples;
	mpu6050Temperature2 /= (float) numberOfSamples;

	cliPrintF("\nGyro Temperature Reading: %6.2f", mpu6050Temperature2);

	cliPrintF("\n\nEnd 2st MPU6050 Measurements\n");

	///////////////////////////////////

	eepromConfig.accelTCBiasSlope[XAXIS]     = (accelBias2[XAXIS] - accelBias1[XAXIS]) / (mpu6050Temperature2 - mpu6050Temperature1);
	eepromConfig.accelTCBiasSlope[YAXIS]     = (accelBias2[YAXIS] - accelBias1[YAXIS]) / (mpu6050Temperature2 - mpu6050Temperature1);
	eepromConfig.accelTCBiasSlope[ZAXIS]     = (accelBias2[ZAXIS] - accelBias1[ZAXIS]) / (mpu6050Temperature2 - mpu6050Temperature1);

	eepromConfig.accelTCBiasIntercept[XAXIS] = accelBias2[XAXIS] - eepromConfig.accelTCBiasSlope[XAXIS] * mpu6050Temperature2;
	eepromConfig.accelTCBiasIntercept[YAXIS] = accelBias2[YAXIS] - eepromConfig.accelTCBiasSlope[YAXIS] * mpu6050Temperature2;
	eepromConfig.accelTCBiasIntercept[ZAXIS] = accelBias2[ZAXIS] - eepromConfig.accelTCBiasSlope[ZAXIS] * mpu6050Temperature2;

	eepromConfig.gyroTCBiasSlope[ROLL ]      = (gyroBias2[ROLL ] - gyroBias1[ROLL ]) / (mpu6050Temperature2 - mpu6050Temperature1);
	eepromConfig.gyroTCBiasSlope[PITCH]      = (gyroBias2[PITCH] - gyroBias1[PITCH]) / (mpu6050Temperature2 - mpu6050Temperature1);
	eepromConfig.gyroTCBiasSlope[YAW  ]      = (gyroBias2[YAW  ] - gyroBias1[YAW  ]) / (mpu6050Temperature2 - mpu6050Temperature1);

	eepromConfig.gyroTCBiasIntercept[ROLL ]  = gyroBias2[ROLL ] - eepromConfig.gyroTCBiasSlope[ROLL ] * mpu6050Temperature2;
	eepromConfig.gyroTCBiasIntercept[PITCH]  = gyroBias2[PITCH] - eepromConfig.gyroTCBiasSlope[PITCH] * mpu6050Temperature2;
	eepromConfig.gyroTCBiasIntercept[YAW  ]  = gyroBias2[YAW  ] - eepromConfig.gyroTCBiasSlope[YAW  ] * mpu6050Temperature2;

	///////////////////////////////////

	cliPrintF("\nMPU6050 Calibration Complete.\n\n");

	mpu6050Calibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
