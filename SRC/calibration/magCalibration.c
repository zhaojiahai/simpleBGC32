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

uint8_t magCalibrating = false;

///////////////////////////////////////////////////////////////////////////////
// Mag Calibration
///////////////////////////////////////////////////////////////////////////////

void magCalibration(void)
{
	uint16_t calibrationCounter = 0;
	uint16_t population[2][3];

	float    d[600][3];       // 600 Samples = 60 seconds of data at 10 Hz
	float    sphereOrigin[3];
	float    sphereRadius;

	magCalibrating = true;

	cliPrintF("\nMagnetometer Calibration:\n\n");

	cliPrintF("Rotate magnetometer around all axes multiple times\n");
	cliPrintF("Must complete within 60 seconds....\n\n");
	cliPrintF("  Send a character when ready to begin and another when complete\n\n");

	while (cliAvailable() == false);

	cliPrintF("  Start rotations.....\n");

	getChar();

	while ((cliAvailable() == false) && (calibrationCounter < 600))
	{
		if (readMag() == true)
		{
			d[calibrationCounter][XAXIS] = (float)rawMag[XAXIS].value * magScaleFactor[XAXIS];
			d[calibrationCounter][YAXIS] = (float)rawMag[YAXIS].value * magScaleFactor[YAXIS];
			d[calibrationCounter][ZAXIS] = (float)rawMag[ZAXIS].value * magScaleFactor[ZAXIS];

			calibrationCounter++;
		}

		delay(100);
	}

	cliPrintF("\n\nMagnetometer Bias Calculation, %3ld samples collected out of 600 max)\n", calibrationCounter);

	sphereFit(d, calibrationCounter, 100, 0.0f, population, sphereOrigin, &sphereRadius);

	eepromConfig.magBias[XAXIS] = sphereOrigin[XAXIS];
	eepromConfig.magBias[YAXIS] = sphereOrigin[YAXIS];
	eepromConfig.magBias[ZAXIS] = sphereOrigin[ZAXIS];

	magCalibrating = false;
}

///////////////////////////////////////////////////////////////////////////////
