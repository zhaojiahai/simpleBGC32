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
// Sensor CLI
///////////////////////////////////////////////////////////////////////////////

void sensorCLI()
{
	uint8_t  sensorQuery = 'x';
	uint8_t  tempInt;
	uint8_t  validQuery = false;

	cliBusy = true;

	cliPrintF("\nEntering Sensor CLI....\n\n");

	while (true)
	{
		cliPrintF("Sensor CLI -> ");

		while ((cliAvailable() == false) && (validQuery == false));

		if (validQuery == false)
			sensorQuery = getChar();

		//cliRead(sensorQuery, 1);

		cliPrintF("\n");

		switch (sensorQuery)
		{
			///////////////////////////

			case 'a': // Sensor Data
				cliPrintF("Accel Temp Comp Slope:     %9.4f, %9.4f, %9.4f\n",   eepromConfig.accelTCBiasSlope[XAXIS],
				          eepromConfig.accelTCBiasSlope[YAXIS],
				          eepromConfig.accelTCBiasSlope[ZAXIS]);
				cliPrintF("Accel Temp Comp Bias:      %9.4f, %9.4f, %9.4f\n",   eepromConfig.accelTCBiasIntercept[XAXIS],
				          eepromConfig.accelTCBiasIntercept[YAXIS],
				          eepromConfig.accelTCBiasIntercept[ZAXIS]);
				cliPrintF("Gyro Temp Comp Slope:      %9.4f, %9.4f, %9.4f\n",   eepromConfig.gyroTCBiasSlope[ROLL ],
				          eepromConfig.gyroTCBiasSlope[PITCH],
				          eepromConfig.gyroTCBiasSlope[YAW  ]);
				cliPrintF("Gyro Temp Comp Intercept:  %9.4f, %9.4f, %9.4f\n",   eepromConfig.gyroTCBiasIntercept[ROLL ],
				          eepromConfig.gyroTCBiasIntercept[PITCH],
				          eepromConfig.gyroTCBiasIntercept[YAW  ]);
				cliPrintF("Mag Bias:                  %9.4f, %9.4f, %9.4f\n",   eepromConfig.magBias[XAXIS],
				          eepromConfig.magBias[YAXIS],
				          eepromConfig.magBias[ZAXIS]);
				cliPrintF("Accel One G:               %9.4f\n",   accelOneG);
				cliPrintF("Accel Cutoff:              %9.4f\n",   eepromConfig.accelCutoff);
				cliPrintF("KpAcc (MARG):              %9.4f\n",   eepromConfig.KpAcc);
				cliPrintF("KiAcc (MARG):              %9.4f\n",   eepromConfig.KiAcc);
				cliPrintF("KpMag (MARG):              %9.4f\n",   eepromConfig.KpMag);
				cliPrintF("KiMag (MARG):              %9.4f\n",   eepromConfig.KiMag);

				cliPrintF("MPU6000 DLPF:                 ");

				switch (eepromConfig.dlpfSetting)
				{
					case DLPF_256HZ:
						cliPrintF("256 Hz\n");
						break;

					case DLPF_188HZ:
						cliPrintF("188 Hz\n");
						break;

					case DLPF_98HZ:
						cliPrintF("98 Hz\n");
						break;

					case DLPF_42HZ:
						cliPrintF("42 Hz\n");
						break;
				}

				validQuery = false;
				break;

			///////////////////////////

			case 'b': // MPU6050Calibration
				mpu6050Calibration();

				sensorQuery = 'a';
				validQuery = true;
				break;

			///////////////////////////

			// HJI case 'c': // Magnetometer Calibration
			// HJI     magCalibration();

			// HJI     sensorQuery = 'a';
			// HJI     validQuery = true;
			// HJI     break;

			///////////////////////////

			case 'x':
				cliPrintF("\nExiting Sensor CLI....\n\n");
				cliBusy = false;
				return;

			// break;

			///////////////////////////

			case 'A': // Set MPU6000 Digital Low Pass Filter
				tempInt = (uint8_t)readFloatCLI();

				switch (tempInt)
				{
					case DLPF_256HZ:
						eepromConfig.dlpfSetting = BITS_DLPF_CFG_256HZ;
						break;

					case DLPF_188HZ:
						eepromConfig.dlpfSetting = BITS_DLPF_CFG_188HZ;
						break;

					case DLPF_98HZ:
						eepromConfig.dlpfSetting = BITS_DLPF_CFG_98HZ;
						break;

					case DLPF_42HZ:
						eepromConfig.dlpfSetting = BITS_DLPF_CFG_42HZ;
						break;
				}

				i2cWrite(MPU6050_ADDRESS, MPU6050_CONFIG, eepromConfig.dlpfSetting);  // Accel and Gyro DLPF Setting

				sensorQuery = 'a';
				validQuery = true;
				break;

			///////////////////////////

			case 'B': // Accel Cutoff
				eepromConfig.accelCutoff = readFloatCLI();

				sensorQuery = 'a';
				validQuery = true;
				break;

			///////////////////////////

			case 'C': // kpAcc, kiAcc
				eepromConfig.KpAcc = readFloatCLI();
				eepromConfig.KiAcc = readFloatCLI();

				sensorQuery = 'a';
				validQuery = true;
				break;

			///////////////////////////

			case 'D': // kpMag, kiMag
				eepromConfig.KpMag = readFloatCLI();
				eepromConfig.KiMag = readFloatCLI();

				sensorQuery = 'a';
				validQuery = true;
				break;

			///////////////////////////

			case 'W': // Write EEPROM Parameters
				cliPrintF("\nWriting EEPROM Parameters....\n\n");
				writeEEPROM();
				break;

			///////////////////////////

			case '?':
				cliPrintF("\n");
				cliPrintF("'a' Display Sensor Data                    'A' Set MPU6000 DLPF                     A0 thru 3\n");
				cliPrintF("'b' MPU6050 Temp Calibration               'B' Set Accel Cutoff                     BAccelCutoff\n");
				cliPrintF("                                           'C' Set kpAcc/kiAcc                      CkpAcc;kiAcc\n");
				// HJI cliPrint("'c' Magnetometer Calibration               'C' Set kpAcc/kiAcc                      CkpAcc;kiAcc\n");
				cliPrintF("                                           'D' Set kpMag/kiMag                      DkpMag;kiMag\n");
				cliPrintF("                                           'W' Write EEPROM Parameters\n");
				cliPrintF("'x' Exit Sensor CLI                        '?' Command Summary\n\n");
				break;

				///////////////////////////
		}
	}

}

///////////////////////////////////////////////////////////////////////////////
