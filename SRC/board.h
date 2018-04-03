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

#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include <ctype.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <errno.h>

#include "stm32f10x.h"
#include "stm32f10x_conf.h"

#include "hw_config.h"
#include "usb_lib.h"
#include "usb_desc.h"
#include "usb_pwr.h"

///////////////////////////////////////

#include "pid.h"

#include "bgc32.h"

#include "drv_cli.h"
#include "drv_gpio.h"
#include "drv_i2c.h"
#include "drv_irq.h"
#include "drv_pwmMotors.h"
#include "drv_rc.h"
#include "drv_system.h"
#include "drv_timingFunctions.h"
#include "drv_usart.h"
#include "ringbuffer.h"

#include "hmc5883.h"
#include "mpu6050.h"

#include "cli.h"
#include "computeMotorCommands.h"
#include "config.h"
#include "evvgcCF.h"
#include "fastTrig.h"
#include "firstOrderFilter.h"
#include "magCalibration.h"
#include "MargAHRS.h"
#include "mpu6050Calibration.h"
#include "pointingCommands.h"
#include "utilities.h"
#define M_TWOPI 6.28318531f

//#define M_TWOPI (2.0f*3.141592653f)
///////////////////////////////////////////////////////////////////////////////
