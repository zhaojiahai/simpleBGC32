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

float mechanical2electricalDegrees[3] = { 1.0f, 1.0f, 1.0f };
float electrical2mechanicalDegrees[3] = { 1.0f, 1.0f, 1.0f };

float outputRate[3];

float pidCmd[3];

float pidCmdPrev[3] = {0.0f, 0.0f, 0.0f};

float yawCmd;

#define YAP_DEADBAND     2.00f  // in radians with respect to one motor pole, actual angle is (DEADBAND / numberPoles) * R2D
#define MOTORPOS2SETPNT  0.35f  // scaling factor for how fast it should move
#define AUTOPANSMOOTH   40.00f

float centerPoint = 0.0f;
float stepSmooth  = 0.0f;
float step        = 0.0f;

///////////////////////////////////////////////////////////////////////////////
// Yaw AutoPan
///////////////////////////////////////////////////////////////////////////////

float autoPan(float motorPos, float setpoint)
{
	if (motorPos < centerPoint - YAP_DEADBAND)
	{
		centerPoint = YAP_DEADBAND;
		step = MOTORPOS2SETPNT * motorPos; //  dampening
	}

	else if (motorPos > centerPoint + YAP_DEADBAND)
	{
		centerPoint = -YAP_DEADBAND;
		step = MOTORPOS2SETPNT * motorPos; //  dampening
	}

	else
	{
		step = 0.0f;
		centerPoint = 0.0f;
	}

	stepSmooth = (stepSmooth * (AUTOPANSMOOTH - 1.0f) + step) / AUTOPANSMOOTH;

	return (setpoint -= stepSmooth);
}

///////////////////////////////////////////////////////////////////////////////
// Compute Motor Commands
///////////////////////////////////////////////////////////////////////////////

void computeMotorCommands(float dt)
{
	holdIntegrators = false;//启用积分作用

	///////////////////////////////////

	if (eepromConfig.rollEnabled == true)//如果使能了 滚转轴（ROLL）
	{

		//更新PID，结果放到pidCmd[ROLL]
		pidCmd[ROLL] = updatePID(pointingCmd[ROLL] * mechanical2electricalDegrees[ROLL],
		                         -sensors.margAttitude500Hz[ROLL] * mechanical2electricalDegrees[ROLL],
		                         dt, holdIntegrators, &eepromConfig.PID[ROLL_PID]);


		//当前PID输出减去上次PID输出得到变化量
		outputRate[ROLL] = pidCmd[ROLL] - pidCmdPrev[ROLL];

		if (outputRate[ROLL] > eepromConfig.rateLimit)//如果变化量大于旋转速度限制的最大值
			pidCmd[ROLL] = pidCmdPrev[ROLL] + eepromConfig.rateLimit;//那么使用上次PID输出结果+旋转速度限制的最大值 作为本次PID结果

		if (outputRate[ROLL] < -eepromConfig.rateLimit)//如果变化量小于旋转速度限制的最小值
			pidCmd[ROLL] = pidCmdPrev[ROLL] - eepromConfig.rateLimit;//那么使用上次PID输出结果-旋转速度限制的最小值（减去一个负数

		//	相当于加上这个数的正值），作为本次PID结果

		pidCmdPrev[ROLL] = pidCmd[ROLL];//保存本次PID输出结果到 pidCmdPrev[ROLL] 作为旧的值（相对于下次）。

		setRollMotor(pidCmd[ROLL], (int)eepromConfig.rollPower);
	}

	///////////////////////////////////

	if (eepromConfig.pitchEnabled == true)//如果使能了 俯仰轴（Pitch）
	{

		//更新PID，结果放到pidCmd[PITCH]
		pidCmd[PITCH] = updatePID(pointingCmd[PITCH] * mechanical2electricalDegrees[PITCH],
		                          sensors.margAttitude500Hz[PITCH] * mechanical2electricalDegrees[PITCH],
		                          dt, holdIntegrators, &eepromConfig.PID[PITCH_PID]);

		outputRate[PITCH] = pidCmd[PITCH] - pidCmdPrev[PITCH];//当前PID输出减去上次PID输出得到变化量

		if (outputRate[PITCH] > eepromConfig.rateLimit)//如果变化量大于旋转速度限制的最大值
			pidCmd[PITCH] = pidCmdPrev[PITCH] + eepromConfig.rateLimit;//那么使用上次PID输出结果+旋转速度限制的最大值 作为本次PID结果

		if (outputRate[PITCH] < -eepromConfig.rateLimit)//如果变化量小于旋转速度限制的最小值
			pidCmd[PITCH] = pidCmdPrev[PITCH] - eepromConfig.rateLimit;//那么使用上次PID输出结果-旋转速度限制的最小值（减去一个负数

		//	相当于加上这个数的正值），作为本次PID结果

		pidCmdPrev[PITCH] = pidCmd[PITCH];//保存本次PID输出结果到 pidCmdPrev[PITCH] 作为旧的值（相对于下次）。

		setPitchMotor(pidCmd[PITCH], (int)eepromConfig.pitchPower);
	}

	///////////////////////////////////

	if (eepromConfig.yawEnabled == true)//如果使能了 偏航轴（YAW）
	{
		if (eepromConfig.yawAutoPanEnabled == true)//如果使能了偏航轴自动全景
			yawCmd = autoPan(pidCmd[YAW], yawCmd * mechanical2electricalDegrees[YAW]) * electrical2mechanicalDegrees[YAW];

		else
			yawCmd = -pointingCmd[YAW];


		//更新PID，结果放到pidCmd[YAW]
		pidCmd[YAW] = updatePID(yawCmd * mechanical2electricalDegrees[YAW],
		                        sensors.margAttitude500Hz[YAW] * mechanical2electricalDegrees[YAW],
		                        dt, holdIntegrators, &eepromConfig.PID[YAW_PID]);

		outputRate[YAW] = pidCmd[YAW] - pidCmdPrev[YAW];//当前PID输出减去上次PID输出得到变化量

		if (outputRate[YAW] > eepromConfig.rateLimit)//如果变化量大于旋转速度限制的最大值
			pidCmd[YAW] = pidCmdPrev[YAW] + eepromConfig.rateLimit;//那么使用上次PID输出结果+旋转速度限制的最大值 作为本次PID结果

		if (outputRate[YAW] < -eepromConfig.rateLimit)//如果变化量小于旋转速度限制的最小值
			pidCmd[YAW] = pidCmdPrev[YAW] - eepromConfig.rateLimit;//那么使用上次PID输出结果-旋转速度限制的最小值（减去一个负数

		//	相当于加上这个数的正值），作为本次PID结果

		pidCmdPrev[YAW] = pidCmd[YAW];//保存本次PID输出结果到 pidCmdPrev[PITCH] 作为旧的值（相对于下次）。

		setYawMotor(pidCmd[YAW], (int)eepromConfig.yawPower);
	}

	///////////////////////////////////

}

///////////////////////////////////////////////////////////////////////////////
