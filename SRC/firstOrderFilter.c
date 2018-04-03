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

// TAU = Filter Time Constant
// T   = Filter Sample Time

// A   = 2 * TAU / T

// Low Pass:
// GX1 = 1 / (1 + A)
// GX2 = 1 / (1 + A)
// GX3 = (1 - A) / (1 + A)

// High Pass:
// GX1 =  A / (1 + A)
// GX2 = -A / (1 + A)
// GX3 = (1 - A) / (1 + A)

///////////////////////////////////////////////////////////////////////////////


firstOrderFilterData_t firstOrderFilters[NUMBER_OF_FIRST_ORDER_FILTERS];


void initFirstOrderFilter(void)
{
	float a;

	a = 2.0f * eepromConfig.accelX500HzLowPassTau * 500.0f;

	firstOrderFilters[ACCEL_X_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[ACCEL_X_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[ACCEL_X_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[ACCEL_X_500HZ_LOWPASS].previousInput  = 0.0f;
	firstOrderFilters[ACCEL_X_500HZ_LOWPASS].previousOutput = 0.0f;

	///////////////////////////////////

	a = 2.0f * eepromConfig.accelY500HzLowPassTau * 500.0f;

	firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].previousInput  = 0.0f;
	firstOrderFilters[ACCEL_Y_500HZ_LOWPASS].previousOutput = 0.0f;

	///////////////////////////////////

	a = 2.0f * eepromConfig.accelZ500HzLowPassTau * 500.0f;

	firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].previousInput  = -9.8065f;
	firstOrderFilters[ACCEL_Z_500HZ_LOWPASS].previousOutput = -9.8065f;

	///////////////////////////////////

	a = 2.0f * eepromConfig.rollRatePointingCmd50HzLowPassTau * 50.0f;

	firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
	firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

	///////////////////////////////////

	a = 2.0f * eepromConfig.pitchRatePointingCmd50HzLowPassTau * 50.0f;

	firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
	firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

	///////////////////////////////////

	a = 2.0f * eepromConfig.yawRatePointingCmd50HzLowPassTau * 50.0f;

	firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
	firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

	///////////////////////////////////

	a = 2.0f * eepromConfig.rollAttPointingCmd50HzLowPassTau * 50.0f;

	firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
	firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

	///////////////////////////////////

	a = 2.0f * eepromConfig.pitchAttPointingCmd50HzLowPassTau * 50.0f;

	firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
	firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

	///////////////////////////////////

	a = 2.0f * eepromConfig.yawAttPointingCmd50HzLowPassTau * 50.0f;

	firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].gx1 = 1.0f / (1.0f + a);
	firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].gx2 = 1.0f / (1.0f + a);
	firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].gx3 = (1.0f - a) / (1.0f + a);
	firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].previousInput  = 0.0f;
	firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS].previousOutput = 0.0f;

	///////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////

float firstOrderFilter(float input, struct firstOrderFilterData *filterParameters)
{
	float output;

	output = filterParameters->gx1 * input +
	         filterParameters->gx2 * filterParameters->previousInput -
	         filterParameters->gx3 * filterParameters->previousOutput;

	filterParameters->previousInput  = input;
	filterParameters->previousOutput = output;

	return output;
}

///////////////////////////////////////////////////////////////////////////////


