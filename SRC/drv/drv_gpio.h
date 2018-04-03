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

#pragma once

///////////////////////////////////////////////////////////////////////////////
// LED Defines
///////////////////////////////////////////////////////////////////////////////

#define LED1_GPIO    GPIOB
#define LED1_PIN     GPIO_Pin_12

#define LED2_GPIO    GPIOA
#define LED2_PIN     GPIO_Pin_3

///////////////////////////////////////

#define LED1_ON      GPIO_SetBits(LED1_GPIO,    LED1_PIN);
#define LED1_OFF     GPIO_ResetBits(LED1_GPIO,  LED1_PIN);
#define LED1_TOGGLE  GPIO_ToggleBits(LED1_GPIO, LED1_PIN);

#define LED2_ON      GPIO_SetBits(LED2_GPIO,    LED2_PIN);
#define LED2_OFF     GPIO_ResetBits(LED2_GPIO,  LED2_PIN);
#define LED2_TOGGLE  GPIO_ToggleBits(LED2_GPIO, LED2_PIN);

///////////////////////////////////////////////////////////////////////////////
// Initialize GPIO
///////////////////////////////////////////////////////////////////////////////

void gpioInit(void);

///////////////////////////////////////////////////////////////////////////////
