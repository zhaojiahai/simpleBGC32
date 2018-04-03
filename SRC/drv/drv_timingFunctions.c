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
// Timing Functions Initialization
///////////////////////////////////////////////////////////////////////////////
#include "stm32f10x.h"
void timingFunctionsInit(void)
{
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	// Output timers

	TIM_TimeBaseStructure.TIM_Period            = 0xFFFF;              // Just roll over counter at max value
	TIM_TimeBaseStructure.TIM_Prescaler         = 36 - 1;              // 72 MHz / 36 = 2 MHz = 0.5 uSec Tick
	TIM_TimeBaseStructure.TIM_ClockDivision     = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;

	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure);

	TIM_SetCounter(TIM6, 4000);  // First pass value
}

///////////////////////////////////////////////////////////////////////////////
