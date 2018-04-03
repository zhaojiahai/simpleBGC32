
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

/*

instructions:

	SimpleBGC 开源三轴无刷云台算法完全解说。
	An interpreter：康朝阳
	Email：547336083@qq.com
	date:2016/09/19

*/
///////////////////////////////////////////////////////////////////////////////

#include "board.h"

///////////////////////////////////////////////////////////////////////////////

eepromConfig_t eepromConfig;

sensors_t      sensors;

float          testPhase      = -1.0f * D2R;
float          testPhaseDelta = 10.0f * D2R;

uint16_t       timerValue;

///////////////////////////////////////////////////////////////////////////////

int main(void)
{
	uint32_t currentTime;

	systemInit();

	initOrientation();

	systemReady = true;

	while (1)
	{
		///////////////////////////////

		//@HackOS: 50HZ循环
		if (frame_50Hz)
		{
			frame_50Hz = false;

			currentTime      = micros();
			deltaTime50Hz    = currentTime - previous50HzTime;
			previous50HzTime = currentTime;

			processPointingCommands();//接收遥控器控制指令。

			executionTime50Hz = micros() - currentTime;
		}

		///////////////////////////////

		if (frame_10Hz)
		{
			frame_10Hz = false;

			currentTime      = micros();
			deltaTime10Hz    = currentTime - previous10HzTime;
			previous10HzTime = currentTime;

			// HJI if (newMagData == true)
			// HJI {
			// HJI     sensors.mag10Hz[XAXIS] =   (float)rawMag[XAXIS].value * magScaleFactor[XAXIS] - eepromConfig.magBias[XAXIS];
			// HJI     sensors.mag10Hz[YAXIS] =   (float)rawMag[YAXIS].value * magScaleFactor[YAXIS] - eepromConfig.magBias[YAXIS];
			// HJI     sensors.mag10Hz[ZAXIS] = -((float)rawMag[ZAXIS].value * magScaleFactor[ZAXIS] - eepromConfig.magBias[ZAXIS]);

			// HJI     newMagData = false;
			// HJI     magDataUpdate = true;
			// HJI }

			cliCom();//接收来自串口的命令并处理

			executionTime10Hz = micros() - currentTime;
		}

		///////////////////////////////

		if (frame_500Hz)
		{
			//本if分支里面是云台算法核心
#ifdef _DTIMING
			LA2_ENABLE;
#endif
			frame_500Hz = false;

			currentTime       = micros();//获取当前时间
			deltaTime500Hz    = currentTime - previous500HzTime;//得到时间增量
			previous500HzTime = currentTime;//保存当前时间

			TIM_Cmd(TIM6, DISABLE);//关闭TIM6
			timerValue = TIM_GetCounter(TIM6);//得到TIM6计数值 ，单位 0.5 uSec Tick
			TIM_SetCounter(TIM6, 0);//清空 TIMx->CNT 寄存器
			TIM_Cmd(TIM6, ENABLE);//开启TIM6 ，准备获取下一次的时间增量

			//定时器计数值*0.5us,得到时间增量，单位是 us
			dt500Hz = (float)timerValue * 0.0000005f;

			//(1/8192) * 9.8065  (8192 LSB = 1 G)
			//（经过矩阵运算后的当前加速度数据-温度补偿偏差）* 最小分辨率 ，最小分辨率就是（(1/8192) * 9.8065）
			//1G量程的8192个数字量分之1，对应重力加速度9.8065m/1G的8192分之1
			//accelData500Hz数组是在Systick中断里面被更新的，更新的值是经过矩阵运算后的值，更新频率是500hz
			sensors.accel500Hz[XAXIS] =  ((float)accelData500Hz[XAXIS] - accelTCBias[XAXIS]) * ACCEL_SCALE_FACTOR;
			sensors.accel500Hz[YAXIS] =  ((float)accelData500Hz[YAXIS] - accelTCBias[YAXIS]) * ACCEL_SCALE_FACTOR;
			sensors.accel500Hz[ZAXIS] = -((float)accelData500Hz[ZAXIS] - accelTCBias[ZAXIS]) * ACCEL_SCALE_FACTOR;

			// (1/65.5) * pi/180   (65.5 LSB = 1 DPS)
			//（经过矩阵运算后的当前陀螺仪数据-陀螺仪静止状态下积分5000次的平均值-温度补偿偏差）* 最小分辨率
			//最小分辨率就是 (1/65.5) * pi/180
			//1DPS的陀螺仪数值是65.5, 65.5分之1 乘以 PI 然后除以180 就是最小分辨率
			//gyroData500Hz数组是在Systick中断里面被更新的，更新的值是经过矩阵运算后的值，更新频率是500hz
			sensors.gyro500Hz[ROLL ] =  ((float)gyroData500Hz[ROLL ] - gyroRTBias[ROLL ] - gyroTCBias[ROLL ]) * GYRO_SCALE_FACTOR;
			sensors.gyro500Hz[PITCH] =  ((float)gyroData500Hz[PITCH] - gyroRTBias[PITCH] - gyroTCBias[PITCH]) * GYRO_SCALE_FACTOR;
			sensors.gyro500Hz[YAW  ] = -((float)gyroData500Hz[YAW  ] - gyroRTBias[YAW  ] - gyroTCBias[YAW  ]) * GYRO_SCALE_FACTOR;


			//当前方位估计运算，其中 accAngleSmooth 是通过加速度数据经过atan2f函数计算得来的欧拉角，并且进行了一阶滞后滤波
			//（此滤波算法也属于低通滤波的一种），优点： 对周期性干扰具有良好的抑制作用 适用于波动频率较高的场合,
			// 缺点： 相位滞后，灵敏度低 滞后程度取决于a值大小 不能消除滤波频率高于采样频率的1/2的干扰信号,
			//getOrientation函数内部使用了accAngleSmooth欧拉角与陀螺仪数据进行了互补滤波融合算法得到稳定的欧拉角，并且
			//存放到sensors.evvgcCFAttitude500Hz里面，sensors.accel500Hz和sensors.gyro500Hz是经过上面算法处理后的加速度数据
			//和陀螺仪数据，dt500Hz是时间增量，也就是执行if (frame_500Hz){} 里面的代码间隔
			getOrientation(accAngleSmooth, sensors.evvgcCFAttitude500Hz, sensors.accel500Hz, sensors.gyro500Hz, dt500Hz);


			//对加速度数据进行 一阶低通滤波 ，其中sensors.accel500Hz是待进行滤波的值，firstOrderFilters是滤波器参数
			// Low Pass:
			// GX1 = 1 / (1 + A)
			// GX2 = 1 / (1 + A)
			// GX3 = (1 - A) / (1 + A)
			sensors.accel500Hz[ROLL ] = firstOrderFilter(sensors.accel500Hz[ROLL ], &firstOrderFilters[ACCEL_X_500HZ_LOWPASS ]);
			sensors.accel500Hz[PITCH] = firstOrderFilter(sensors.accel500Hz[PITCH], &firstOrderFilters[ACCEL_Y_500HZ_LOWPASS]);
			sensors.accel500Hz[YAW  ] = firstOrderFilter(sensors.accel500Hz[YAW  ], &firstOrderFilters[ACCEL_Z_500HZ_LOWPASS  ]);

			//航姿参考系统更新，入口参数是三轴陀螺仪数据，三轴加速度数据，三轴磁力计数据,以及指示是否更新磁力计数据的magDataUpdate参数，
			//magDataUpdate=false表示不更新磁力计数据，magDataUpdate=true表示更新磁力计数据，最后一个参数是时间增量Dt，也就是此函数本
			//次执行与上次执行的时间间隔。
			MargAHRSupdate(sensors.gyro500Hz[ROLL],   sensors.gyro500Hz[PITCH],  sensors.gyro500Hz[YAW],
			               sensors.accel500Hz[XAXIS], sensors.accel500Hz[YAXIS], sensors.accel500Hz[ZAXIS],
			               sensors.mag10Hz[XAXIS],    sensors.mag10Hz[YAXIS],    sensors.mag10Hz[ZAXIS],
			               magDataUpdate , dt500Hz);

			magDataUpdate = false;//默认不更新磁力计数据

			computeMotorCommands(dt500Hz);//计算电机控制量，入口参数时间增量Dt

			executionTime500Hz = micros() - currentTime;//本次运算执行时间长度保存到executionTime500Hz

#ifdef _DTIMING
			LA2_DISABLE;
#endif
		}

		///////////////////////////////

		if (frame_100Hz)
		{
			frame_100Hz = false;

			currentTime       = micros();
			deltaTime100Hz    = currentTime - previous100HzTime;
			previous100HzTime = currentTime;

			executionTime100Hz = micros() - currentTime;
		}

		///////////////////////////////

		if (frame_5Hz)
		{
			frame_5Hz = false;

			currentTime     = micros();
			deltaTime5Hz    = currentTime - previous5HzTime;
			previous5HzTime = currentTime;

			//  LED2_TOGGLE;

			executionTime5Hz = micros() - currentTime;
		}

		///////////////////////////////

		if (frame_1Hz)
		{
			frame_1Hz = false;

			currentTime     = micros();
			deltaTime1Hz    = currentTime - previous1HzTime;
			previous1HzTime = currentTime;

			//  LED1_TOGGLE;

			executionTime1Hz = micros() - currentTime;
		}

		////////////////////////////////
	}
}

///////////////////////////////////////////////////////////////////////////////

