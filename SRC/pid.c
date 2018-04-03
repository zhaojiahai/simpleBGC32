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

uint8_t holdIntegrators = true;

#define F_CUT 20.0f

static float rc;

///////////////////////////////////////////////////////////////////////////////

void initPID(void)
{
	uint8_t index;

	rc = 1.0f / ( TWO_PI * F_CUT );

	for (index = 0; index < NUMBER_OF_PIDS; index++)
	{
		eepromConfig.PID[index].iTerm          = 0.0f;
		eepromConfig.PID[index].lastDcalcValue = 0.0f;
		eepromConfig.PID[index].lastDterm      = 0.0f;
		eepromConfig.PID[index].lastLastDterm  = 0.0f;
	}
}

///////////////////////////////////////////////////////////////////////////////

float updatePID(float command/*遥控器控制的机械角度转换成电子角度*/,

                float state/*四元数计算的机械角度转换成电子角度*/,

                float deltaT/*时间增量dT*/,

                uint8_t iHold/*是否拥有积分I*/,

                struct PIDdata *PIDparameters/*PID参数*/)
{
	float error;
	float dTerm;
	float dTermFiltered;
	float dAverage;

	///////////////////////////////////

	//PID公式
	/* Kp*e + Ki*∫edt + Kd*（de/dt） */


	error = command - state;//遥控器控制的期望角度-四元数计算的机体角度 得到当前偏差 ，公式 e

	if (PIDparameters->type == ANGULAR)//如果类型为角度
		error = standardRadianFormat(error);//转换成标准弧度

	///////////////////////////////////

	if (iHold == false)//如果iHold == false才加入积分项，默认启用积分项
	{
		PIDparameters->iTerm += error * deltaT;//对误差进行积分，公式 ∫edt
		PIDparameters->iTerm = constrain(PIDparameters->iTerm, -PIDparameters->windupGuard, PIDparameters->windupGuard);//积分限幅
	}

	///////////////////////////////////

	if (PIDparameters->dErrorCalc == D_ERROR)  // Calculate D term from error
	{

		//通过error计算微分项，公式（de/dt），其中de就是(error - PIDparameters->lastDcalcValue)
		dTerm = (error - PIDparameters->lastDcalcValue) / deltaT;
		PIDparameters->lastDcalcValue = error;//保存当前偏差
	}

	else // Calculate D term from state
	{

		//否则，使用四元数计算的机械角度转换成电子角度的值进行微分运算
		dTerm = (PIDparameters->lastDcalcValue - state) / deltaT;

		if (PIDparameters->type == ANGULAR)//如果类型为角度
			dTerm = standardRadianFormat(dTerm);//转换成标准弧度

		PIDparameters->lastDcalcValue = state;//保存当前状态
	}

	///////////////////////////////////
	//对微分项进行一阶低通滤波 deltaT / (rc + deltaT) 结果就是滤波系数a ， rc=1.0f/(2.0f*PI*F)
	dTermFiltered = PIDparameters->lastDterm + deltaT / (rc + deltaT) * (dTerm - PIDparameters->lastDterm);


	//对历史三次微分项进行求平均
	dAverage = (dTermFiltered + PIDparameters->lastDterm + PIDparameters->lastLastDterm) * 0.333333f;

	PIDparameters->lastLastDterm = PIDparameters->lastDterm;//上次微分项保存到上上次微分项存储变量
	PIDparameters->lastDterm = dTermFiltered;//当前微分项保存到上次微分项存储变量

	///////////////////////////////////
	//返回PID运算结果
	if (PIDparameters->type == ANGULAR)//如果类型为角度
		return(PIDparameters->P * error     /*  Kp*e  */           +
		       PIDparameters->I * PIDparameters->iTerm + /*   Ki*∫edt   */
		       PIDparameters->D * dAverage);/*   Kd*（de/dt）  */
	else
		return(PIDparameters->P * PIDparameters->B * command /* Kp *(B * point) */  +
		       PIDparameters->I * PIDparameters->iTerm /*   Ki*∫edt   */      +
		       PIDparameters->D * dAverage   /*   Kd*（de/dt）  */ -
		       PIDparameters->P * state);//计算增量

	///////////////////////////////////
}

///////////////////////////////////////////////////////////////////////////////

void setPIDintegralError(uint8_t IDPid, float value)
{
	eepromConfig.PID[IDPid].iTerm = value;
}

///////////////////////////////////////////////////////////////////////////////

void zeroPIDintegralError(void)
{
	uint8_t index;

	for (index = 0; index < NUMBER_OF_PIDS; index++)
		setPIDintegralError(index, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////

void setPIDstates(uint8_t IDPid, float value)
{
	eepromConfig.PID[IDPid].lastDcalcValue = value;
	eepromConfig.PID[IDPid].lastDterm      = value;
	eepromConfig.PID[IDPid].lastLastDterm  = value;
}

///////////////////////////////////////////////////////////////////////////////

void zeroPIDstates(void)
{
	uint8_t index;

	for (index = 0; index < NUMBER_OF_PIDS; index++)
		setPIDstates(index, 0.0f);
}

///////////////////////////////////////////////////////////////////////////////


