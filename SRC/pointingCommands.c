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
// Process Pilot Commands Defines and Variables
///////////////////////////////////////////////////////////////////////////////

float    pointingCmd[3] = { 0.0f, 0.0f, 0.0f };

float    rxCommand[3]   = { 0.0f, 0.0f, 0.0f };

uint8_t  commandInDetent[3]         = { true, true, true };
uint8_t  previousCommandInDetent[3] = { true, true, true };

///////////////////////////////////////////////////////////////////////////////
// Process Pointing Commands
///////////////////////////////////////////////////////////////////////////////

void processPointingCommands(void)
{
	uint8_t channel;

	if (rcActive == true)
	{
		// Read receiver commands
		for (channel = 0; channel < 3; channel++)
			rxCommand[channel] = (float)rxRead(channel);//读取三个轴的原始控制量


		//减去中位值
		rxCommand[ROLL]  -= eepromConfig.midCommand;                  // Roll Range    -1000:1000
		rxCommand[PITCH] -= eepromConfig.midCommand;                  // Pitch Range   -1000:1000
		rxCommand[YAW]   -= eepromConfig.midCommand;                  // Yaw Range     -1000:1000
	}

	// Set past command in detent values
	for (channel = 0; channel < 3; channel++)
		previousCommandInDetent[channel] = commandInDetent[channel];

	// Apply deadbands and set detent discretes
	for (channel = 0; channel < 3; channel++)
	{

		//如果控制量小于等于DEADBAND并且大于等于-DEADBAND，则认为没有控制量，DEADBAND在这里是24
		if ((rxCommand[channel] <= DEADBAND) && (rxCommand[channel] >= -DEADBAND))
		{
			rxCommand[channel] = 0;//控制量设为0
			commandInDetent[channel] = true;//制动标志置位
		}

		else
		{
			commandInDetent[channel] = false;//不制动

			if (rxCommand[channel] > 0)//控制信号的正负代表轴可以向两种方向转动
			{
				rxCommand[channel] = (rxCommand[channel] - DEADBAND) * DEADBAND_SLOPE;
			}

			else
			{
				rxCommand[channel] = (rxCommand[channel] + DEADBAND) * DEADBAND_SLOPE;
			}
		}
	}

	///////////////////////////////////
	//乘以一个比例因子，使控制量单位和下面运算处于同一个数量级
	rxCommand[ROLL]  *=  0.001f;  // Roll Range  -1:1
	rxCommand[PITCH] *= -0.001f;  // Pitch Range -1:1
	rxCommand[YAW]   *=  0.001f;  // Yaw Range   -1:1

	///////////////////////////////////

	if (eepromConfig.rollRateCmdInput == true)
	{
		if ((rxCommand[ROLL] >= 0.0f) && (pointingCmd[ROLL] <=  eepromConfig.gimbalRollRightLimit))//判断控制方向和约束可控角度范围
			pointingCmd[ROLL] += rxCommand[ROLL] * eepromConfig.gimbalRollRate * 0.02f;  ////角度补偿，补偿系数在这里取0.02足够好

		//Constant DT of 0.02 good enough here

		if ((rxCommand[ROLL] < 0.0f) && (pointingCmd[ROLL] >= -eepromConfig.gimbalRollLeftLimit))//判断控制方向和约束可控角度范围
			pointingCmd[ROLL] += rxCommand[ROLL] * eepromConfig.gimbalRollRate * 0.02f;  // Constant DT of 0.02 good enough here

		////角度补偿，补偿系数在这里取0.02足够好


		//低通滤波
		pointingCmd[ROLL] = firstOrderFilter(pointingCmd[ROLL], &firstOrderFilters[ROLL_RATE_POINTING_50HZ_LOWPASS]);
	}

	else
	{
		//不使用角度补偿
		if (rxCommand[ROLL] >= 0.0f)
			pointingCmd[ROLL] = rxCommand[ROLL] * eepromConfig.gimbalRollRightLimit;

		else
			pointingCmd[ROLL] = rxCommand[ROLL] * eepromConfig.gimbalRollLeftLimit;

		//低通滤波
		pointingCmd[ROLL] = firstOrderFilter(pointingCmd[ROLL], &firstOrderFilters[ROLL_ATT_POINTING_50HZ_LOWPASS]);
	}


	//限制角度范围
	pointingCmd[ROLL] = constrain(pointingCmd[ROLL], -eepromConfig.gimbalYawLeftLimit, eepromConfig.gimbalRollRightLimit);

	///////////////////////////////////
	//下面PITCH轴与YAW轴与 Roll轴同理，不再赘述。
	if (eepromConfig.pitchRateCmdInput == true)
	{
		if ((rxCommand[PITCH] >= 0.0f) && (pointingCmd[PITCH] <=  eepromConfig.gimbalPitchUpLimit))
			pointingCmd[PITCH] += rxCommand[PITCH] * eepromConfig.gimbalPitchRate * 0.02f;  // Constant DT of 0.02 good enough here

		if ((rxCommand[PITCH] < 0.0f) && (pointingCmd[PITCH] >= -eepromConfig.gimbalPitchDownLimit))
			pointingCmd[PITCH] += rxCommand[PITCH] * eepromConfig.gimbalPitchRate * 0.02f;  // Constant DT of 0.02 good enough here

		pointingCmd[PITCH] = firstOrderFilter(pointingCmd[PITCH], &firstOrderFilters[PITCH_RATE_POINTING_50HZ_LOWPASS]);
	}

	else
	{
		if (rxCommand[PITCH] >= 0.0f)
			pointingCmd[PITCH] = rxCommand[PITCH] * eepromConfig.gimbalPitchUpLimit;

		if (rxCommand[PITCH] < 0.0f)
			pointingCmd[PITCH] = rxCommand[PITCH] * eepromConfig.gimbalPitchDownLimit;

		pointingCmd[PITCH] = firstOrderFilter(pointingCmd[PITCH], &firstOrderFilters[PITCH_ATT_POINTING_50HZ_LOWPASS]);
	}

	pointingCmd[PITCH] = constrain(pointingCmd[PITCH], -eepromConfig.gimbalPitchDownLimit, eepromConfig.gimbalPitchUpLimit);

	///////////////////////////////////

	if (eepromConfig.yawRateCmdInput == true)
	{
		if ((rxCommand[YAW] >= 0.0f) && (pointingCmd[YAW] <=  eepromConfig.gimbalYawRightLimit))
			pointingCmd[YAW] += rxCommand[YAW] * eepromConfig.gimbalYawRate * 0.02f;  // Constant DT of 0.02 good enough here

		if ((rxCommand[YAW] < 0.0f) && (pointingCmd[YAW] >= -eepromConfig.gimbalYawLeftLimit))
			pointingCmd[YAW] += rxCommand[YAW] * eepromConfig.gimbalYawRate * 0.02f;  // Constant DT of 0.02 good enough here

		pointingCmd[YAW] = firstOrderFilter(pointingCmd[YAW], &firstOrderFilters[YAW_RATE_POINTING_50HZ_LOWPASS]);
	}

	else
	{
		if (rxCommand[YAW] >= 0.0f)
			pointingCmd[YAW] = rxCommand[YAW] * eepromConfig.gimbalYawRightLimit;

		if (rxCommand[YAW] < 0.0f)
			pointingCmd[YAW] = rxCommand[YAW] * eepromConfig.gimbalYawLeftLimit;

		pointingCmd[YAW] = firstOrderFilter(pointingCmd[YAW], &firstOrderFilters[YAW_ATT_POINTING_50HZ_LOWPASS]);
	}

	pointingCmd[YAW] = constrain(pointingCmd[YAW], -eepromConfig.gimbalYawLeftLimit, eepromConfig.gimbalYawRightLimit);

	///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////




