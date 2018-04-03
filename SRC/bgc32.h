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

//#pragma once

///////////////////////////////////////////////////////////////////////////////

#define __BGC32_VERSION "1.0"

///////////////////////////////////////////////////////////////////////////////

#define     PI 3.14159265f
#define TWO_PI 6.28318531f

#define D2R  (PI / 180.0f)

#define R2D  (180.0f / PI)

#define SQR(x)  ((x) * (x))

extern float   testPhase;
extern float   testPhaseDelta;

///////////////////////////////////////////////////////////////////////////////

#define ROLL     0
#define PITCH    1
#define YAW      2

#define XAXIS    0
#define YAXIS    1
#define ZAXIS    2

#define NUMAXIS  3

#define MINCOMMAND  2000
#define MIDCOMMAND  3000
#define MAXCOMMAND  4000

///////////////////////////////////////////////////////////////////////////////
// Misc Type Definitions
///////////////////////////////////////////////////////////////////////////////
#include "stm32f10x.h"
typedef union
{
	int16_t value;
	uint8_t bytes[2];
} int16andUint8_t;

typedef union
{
	int32_t value;
	uint8_t bytes[4];
} int32andUint8_t;

typedef union
{
	uint16_t value;
	uint8_t bytes[2];
} uint16andUint8_t;

///////////////////////////////////////

typedef volatile uint8_t semaphore_t;

///////////////////////////////////////////////////////////////////////////////
// Sensor Variables
///////////////////////////////////////////////////////////////////////////////

typedef struct sensors_t
{
	float accel500Hz[3];
	float evvgcCFAttitude500Hz[3];
	float margAttitude500Hz[3];
	float gyro500Hz[3];
	float mag10Hz[3];

} sensors_t;

extern sensors_t sensors;

///////////////////////////////////////////////////////////////////////////////
// PID Definitions
///////////////////////////////////////////////////////////////////////////////

#define NUMBER_OF_PIDS 3

#define ROLL_PID  0
#define PITCH_PID 1
#define YAW_PID   2

///////////////////////////////////////////////////////////////////////////////
// MPU6000 DLPF Configurations
///////////////////////////////////////////////////////////////////////////////

enum { DLPF_256HZ, DLPF_188HZ, DLPF_98HZ, DLPF_42HZ };

///////////////////////////////////////////////////////////////////////////////
// EEPROM
///////////////////////////////////////////////////////////////////////////////

typedef struct eepromConfig_t
{
	uint8_t version;

	float accelTCBiasSlope[3];
	float accelTCBiasIntercept[3];

	float gyroTCBiasSlope[3];
	float gyroTCBiasIntercept[3];

	float magBias[3];

	float accelCutoff;

	float KpAcc;

	float KiAcc;

	float KpMag;

	float KiMag;

	uint8_t dlpfSetting;

	float midCommand;

	PIDdata_t PID[NUMBER_OF_PIDS];

	float rollPower;
	float pitchPower;
	float yawPower;

	uint8_t rollEnabled;
	uint8_t pitchEnabled;
	uint8_t yawEnabled;

	uint8_t rollAutoPanEnabled;
	uint8_t pitchAutoPanEnabled;
	uint8_t yawAutoPanEnabled;

	uint8_t imuOrientation;

	float   rollMotorPoles;
	float   pitchMotorPoles;
	float   yawMotorPoles;

	float   rateLimit;

	uint8_t rollRateCmdInput;
	uint8_t pitchRateCmdInput;
	uint8_t yawRateCmdInput;

	float   gimbalRollRate;
	float   gimbalPitchRate;
	float   gimbalYawRate;

	float   gimbalRollLeftLimit;
	float   gimbalRollRightLimit;
	float   gimbalPitchDownLimit;
	float   gimbalPitchUpLimit;
	float   gimbalYawLeftLimit;
	float   gimbalYawRightLimit;

	float   accelX500HzLowPassTau;
	float   accelY500HzLowPassTau;
	float   accelZ500HzLowPassTau;

	float   rollRatePointingCmd50HzLowPassTau;
	float   pitchRatePointingCmd50HzLowPassTau;
	float   yawRatePointingCmd50HzLowPassTau;

	float   rollAttPointingCmd50HzLowPassTau;
	float   pitchAttPointingCmd50HzLowPassTau;
	float   yawAttPointingCmd50HzLowPassTau;

} eepromConfig_t;

extern eepromConfig_t eepromConfig;

///////////////////////////////////////////////////////////////////////////////
