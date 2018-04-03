/*
  June 2012

  BaseFlightPlus Rev -

  An Open Source STM32 Based Multicopter

  Includes code and/or ideas from:

  1)AeroQuad
  2)BaseFlight
  3)CH Robotics
  4)MultiWii
  5)S.O.H. Madgwick

  Designed to run on Naze32 Flight Control Board

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

#pragma once

///////////////////////////////////////////////////////////////////////////////

void i2cInit(I2C_TypeDef *I2Cx);

///////////////////////////////////////////////////////////////////////////////

bool i2cWriteBuffer(uint8_t addr_, uint8_t reg_, uint8_t len_, uint8_t *data);

///////////////////////////////////////////////////////////////////////////////

bool i2cWrite(uint8_t addr_, uint8_t reg, uint8_t data);

///////////////////////////////////////////////////////////////////////////////

bool i2cRead(uint8_t addr_, uint8_t reg, uint8_t len, uint8_t *buf);

///////////////////////////////////////////////////////////////////////////////

uint16_t i2cGetErrorCounter(void);

///////////////////////////////////////////////////////////////////////////////
