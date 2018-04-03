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

uint8_t cliBusy = false;

static volatile uint8_t cliQuery        = 'x';
static volatile uint8_t validCliCommand = false;

///////////////////////////////////////

uint8_t gimbalStateEnabled = true;

uint8_t savedRollState;
uint8_t savedPitchState;
uint8_t savedYawState;

///////////////////////////////////////////////////////////////////////////////
// Read Character String from CLI
///////////////////////////////////////////////////////////////////////////////

uint8_t *readStringCLI(uint8_t *data, uint8_t length)
{
	uint8_t index    = 0;
	uint8_t timeout  = 0;

	do
	{
		if (cliAvailable() == false)
		{
			delay(10);
			timeout++;
		}

		else
		{
			//data[index] = cliRead();
			cliRead(&data[index], 1);
			timeout = 0;
			index++;
		}
	}
	while ((index == 0 || data[index - 1] != ';') && (timeout < 5) && (index < length));

	data[index] = '\0';

	return data;
}

///////////////////////////////////////////////////////////////////////////////
// Read Float from CLI
///////////////////////////////////////////////////////////////////////////////

float readFloatCLI(void)
{
	uint8_t index    = 0;
	uint8_t timeout  = 0;
	uint8_t    data[13] = "";

	do
	{
		if (cliAvailable() == false)
		{
			delay(10);
			timeout++;
		}

		else
		{
			//data[index] = cliRead();
			cliRead(&data[index], 1);
			timeout = 0;
			index++;
		}
	}
	while ((index == 0 || data[index - 1] != ';') && (timeout < 5) && (index < sizeof(data) - 1));

	data[index] = '\0';

	return stringToFloat(data);
}

///////////////////////////////////////////////////////////////////////////////
// Read PID Values from CLI
///////////////////////////////////////////////////////////////////////////////

void readCliPID(unsigned char PIDid)
{
	struct PIDdata *pid = &eepromConfig.PID[PIDid];

	pid->B              = readFloatCLI();
	pid->P              = readFloatCLI();
	pid->I              = readFloatCLI();
	pid->D              = readFloatCLI();
	pid->windupGuard    = readFloatCLI();
	pid->iTerm          = 0.0f;
	pid->lastDcalcValue = 0.0f;
	pid->lastDterm      = 0.0f;
	pid->lastLastDterm  = 0.0f;
	pid->dErrorCalc     = (uint8_t)readFloatCLI();
}

///////////////////////////////////////////////////////////////////////////////
// Disable Gimbal
///////////////////////////////////////////////////////////////////////////////

void disableGimbal(void)
{
	savedRollState  = eepromConfig.rollEnabled;
	savedPitchState = eepromConfig.pitchEnabled;
	savedYawState   = eepromConfig.yawEnabled;

	eepromConfig.rollEnabled  = false;
	eepromConfig.pitchEnabled = false;
	eepromConfig.yawEnabled   = false;

	pwmMotorDriverInit();

	cliPrintF("\nGimbal Disabled....\n");
}

///////////////////////////////////////////////////////////////////////////////
// Enable Gimbal
///////////////////////////////////////////////////////////////////////////////

void enableGimbal(void)
{
	eepromConfig.rollEnabled  = savedRollState;
	eepromConfig.pitchEnabled = savedPitchState;
	eepromConfig.yawEnabled   = savedYawState;

	pwmMotorDriverInit();

	cliPrintF("\nGimbal Enabled....\n");
}

///////////////////////////////////////////////////////////////////////////////
// CLI Communication
///////////////////////////////////////////////////////////////////////////////

void cliCom(void)
{
	if ((cliAvailable() && !validCliCommand))
	{
		cliQuery = getChar();
	}

	validCliCommand = false;

	switch (cliQuery)
	{
		///////////////////////////////

		case 'a': // Rate PIDs
			cliPrintF("\nRoll Rate PID:  %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %s\n", eepromConfig.PID[ROLL_PID].B,
			          eepromConfig.PID[ROLL_PID].P,
			          eepromConfig.PID[ROLL_PID].I,
			          eepromConfig.PID[ROLL_PID].D,
			          eepromConfig.PID[ROLL_PID].windupGuard,
			          eepromConfig.PID[ROLL_PID].dErrorCalc ? "Error" : "State");

			cliPrintF("Pitch Rate PID: %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %s\n",   eepromConfig.PID[PITCH_PID].B,
			          eepromConfig.PID[PITCH_PID].P,
			          eepromConfig.PID[PITCH_PID].I,
			          eepromConfig.PID[PITCH_PID].D,
			          eepromConfig.PID[PITCH_PID].windupGuard,
			          eepromConfig.PID[PITCH_PID].dErrorCalc ? "Error" : "State");

			cliPrintF("Yaw Rate PID:   %9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %s\n",   eepromConfig.PID[YAW_PID].B,
			          eepromConfig.PID[YAW_PID].P,
			          eepromConfig.PID[YAW_PID].I,
			          eepromConfig.PID[YAW_PID].D,
			          eepromConfig.PID[YAW_PID].windupGuard,
			          eepromConfig.PID[YAW_PID].dErrorCalc ? "Error" : "State");
			cliQuery = 'x';
			break;

		///////////////////////////////

		case 'b': // Loop Delta Times
			cliPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n", deltaTime1000Hz,
			          deltaTime500Hz,
			          deltaTime100Hz,
			          deltaTime50Hz,
			          deltaTime10Hz,
			          deltaTime5Hz,
			          deltaTime1Hz);
			break;

		///////////////////////////////

		case 'c': // Loop Execution Times
			cliPrintF("%7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld, %7ld\n", executionTime1000Hz,
			          executionTime500Hz,
			          executionTime100Hz,
			          executionTime50Hz,
			          executionTime10Hz,
			          executionTime5Hz,
			          executionTime1Hz,
			          i2cGetErrorCounter());
			break;

		///////////////////////////////

		case 'd': // RC Parameters
			cliPrintF("\n       RC Response    Max Rate    Left/Down Limit    Right/Up Limit\n");

			cliPrintF("Roll:     %s         %4.1f", eepromConfig.rollRateCmdInput  ? "Rate" : "Att ", eepromConfig.gimbalRollRate * R2D);
			cliPrintF("          %5.1f              %5.1f\n", eepromConfig.gimbalRollLeftLimit * R2D, eepromConfig.gimbalRollRightLimit * R2D);

			cliPrintF("Pitch:    %s         %4.1f", eepromConfig.pitchRateCmdInput ? "Rate" : "Att ", eepromConfig.gimbalPitchRate * R2D);
			cliPrintF("          %5.1f              %5.1f\n", eepromConfig.gimbalPitchDownLimit * R2D, eepromConfig.gimbalPitchUpLimit * R2D);

			cliPrintF("Yaw:      %s         %4.1f", eepromConfig.yawRateCmdInput   ? "Rate" : "Att ", eepromConfig.gimbalYawRate * R2D);
			cliPrintF("          %5.1f              %5.1f\n", eepromConfig.gimbalYawLeftLimit * R2D, eepromConfig.gimbalYawRightLimit * R2D);

			cliQuery = 'x';
			break;

		///////////////////////////////

		case 'e': // 500 Hz Accels
			cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.accel500Hz[XAXIS],
			          sensors.accel500Hz[YAXIS],
			          sensors.accel500Hz[ZAXIS]);
			break;

		///////////////////////////////

		case 'f': // 500 Hz Gyros
			cliPrintF("%9.4f, %9.4f, %9.4f, %9.4f\n", sensors.gyro500Hz[ROLL ] * R2D,
			          sensors.gyro500Hz[PITCH] * R2D,
			          sensors.gyro500Hz[YAW  ] * R2D,
			          mpu6050Temperature);
			break;

		///////////////////////////////

		case 'g': // 10 Hz Mag Data
			cliPrintF("%9.4f, %9.4f, %9.4f\n", sensors.mag10Hz[XAXIS],
			          sensors.mag10Hz[YAXIS],
			          sensors.mag10Hz[ZAXIS]);
			break;

		///////////////////////////////

		case 'h': // Attitudes
			cliPrintF("%9.4f, %9.4f, %9.4f, %9.4f, %9.4f, %9.4f\n", sensors.evvgcCFAttitude500Hz[ROLL]  * R2D,
			          sensors.margAttitude500Hz[ROLL]     * R2D,
			          sensors.evvgcCFAttitude500Hz[PITCH] * R2D,
			          sensors.margAttitude500Hz[PITCH]    * R2D,
			          sensors.evvgcCFAttitude500Hz[YAW]   * R2D,
			          sensors.margAttitude500Hz[YAW]      * R2D);
			break;

		///////////////////////////////

		case 'i': // Gimbal Axis Enable Flags
			cliPrintF("Gimbal Roll Axis:  %s\n", eepromConfig.rollEnabled  ? "Enabled" : "Disabled");
			cliPrintF("Gimbal Pitch Axis: %s\n", eepromConfig.pitchEnabled ? "Enabled" : "Disabled");
			cliPrintF("Gimbal Yaw Axis:   %s\n", eepromConfig.yawEnabled   ? "Enabled" : "Disabled");

			cliQuery = 'x';
			break;

		///////////////////////////////

		case 'j': // Gimbal Axis Power Levels
			cliPrintF("Gimbal Roll Axis Power level:  %4.1f\n", eepromConfig.rollPower);
			cliPrintF("Gimbal Pitch Axis Power level: %4.1f\n", eepromConfig.pitchPower);
			cliPrintF("Gimbal Yaw Axis Power level:   %4.1f\n", eepromConfig.yawPower);

			cliQuery = 'x';
			break;

		///////////////////////////////

		case 'k': // Gimbal Rate Limit
			cliPrintF("Gimbal Rate Limit: %7.3f\n", eepromConfig.rateLimit * R2D);

			cliQuery = 'x';
			break;

		///////////////////////////////

		case 'l': // Gimbal IMU Orientation
			cliPrintF("Gimbal IMU Orientation: %1d\n", eepromConfig.imuOrientation);

			cliQuery = 'x';
			break;

		///////////////////////////////

		case 'm': // Test Phase Value
			cliPrintF("Test Phase Value: %6.2\n", testPhase * R2D);

			cliQuery = 'x';
			break;

		///////////////////////////////

		case 'n': // Test Phase Delta
			cliPrintF("Test Phase Delta: %6.2F\n", testPhaseDelta * R2D);

			cliQuery = 'x';
			break;

		///////////////////////////////

		case 'o': // Motor Poles
			cliPrintF("Roll Motor Poles:  %3.0f \n", eepromConfig.rollMotorPoles);
			cliPrintF("Pitch Motor Poles: %3.0f \n", eepromConfig.pitchMotorPoles);
			cliPrintF("Yaw Motor Poles:   %3.0f \n", eepromConfig.yawMotorPoles);

			cliQuery = 'x';
			break;

		///////////////////////////////

		case 'p': // Counters
			cliPrintF("Counter min %3d, %3d, %3d,  max %4d, %4d, %4d, count %3d, %3d, %3d\n",
			          minCnt[ROLL], minCnt[PITCH], minCnt[YAW],
			          maxCnt[ROLL], maxCnt[PITCH], maxCnt[YAW],
			          irqCnt[ROLL], irqCnt[PITCH], irqCnt[YAW]);
			break;

		///////////////////////////////

		case 'q': // Filter Time Constants
			cliPrintF("\n         Accel TC       Rate Cmd TC    Att Cmd TC\n");

			cliPrintF("Roll/Y:    %5.2f           %5.2f          %5.2f\n", eepromConfig.accelY500HzLowPassTau,
			          eepromConfig.rollRatePointingCmd50HzLowPassTau,
			          eepromConfig.rollAttPointingCmd50HzLowPassTau);

			cliPrintF("Pitch/X:   %5.2f           %5.2f          %5.2f\n", eepromConfig.accelX500HzLowPassTau,
			          eepromConfig.pitchRatePointingCmd50HzLowPassTau,
			          eepromConfig.pitchAttPointingCmd50HzLowPassTau);

			cliPrintF("Yaw/Z:     %5.2f           %5.2f          %5.2f\n", eepromConfig.accelZ500HzLowPassTau,
			          eepromConfig.yawRatePointingCmd50HzLowPassTau,
			          eepromConfig.yawAttPointingCmd50HzLowPassTau);
			cliQuery = 'x';
			break;

		///////////////////////////////

		case 's': // Raw Receiver Commands
			cliPrintF("%4i, ", rxRead(ROLL));
			cliPrintF("%4i, ", rxRead(PITCH));
			cliPrintF("%4i\n", rxRead(YAW));
			break;

		///////////////////////////////

		case 't': // Pointing Commands
			cliPrintF("%8.2f, ", pointingCmd[ROLL]  * R2D);
			cliPrintF("%8.2f, ", pointingCmd[PITCH] * R2D);
			cliPrintF("%8.2f\n", pointingCmd[YAW]   * R2D);
			break;

		///////////////////////////////

		case 'u': // PID Outputs
			cliPrintF("%12.4f, %12.4f, %12.4f\n", pidCmd[ROLL],
			          pidCmd[PITCH],
			          pidCmd[YAW]);
			break;

		///////////////////////////////

		case 'v': // Version

#if defined(__DATE__) && defined(__TIME__)
			cliPrintF("\nBGC32 Firmware V%s, Build Date " __DATE__ " "__TIME__" \n", __BGC32_VERSION);
#endif

			cliQuery = 'x';
			break;

		///////////////////////////////

		case 'x':
			break;

		///////////////////////////////

		case 'y':  // AutoPan Enable Status
			cliPrintF("Roll AutoPan:  (Not Implemented)  %s\n", eepromConfig.rollAutoPanEnabled  ? "Enabled" : "Disabled");
			cliPrintF("Pitch AutoPan: (Not Implemented)  %s\n", eepromConfig.pitchAutoPanEnabled ? "Enabled" : "Disabled");
			cliPrintF("Yaw AutoPan:                      %s\n", eepromConfig.yawAutoPanEnabled   ? "Enabled" : "Disabled");

			cliQuery = 'x';
			break;

		///////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////
		///////////////////////////////////////////////////////////////////////

		///////////////////////////////

		case 'A': // Read Roll PID Values
			readCliPID(ROLL_PID);
			cliPrintF("\nRoll Rate PID Received....\n");

			cliQuery = 'a';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'B': // Read Pitch PID Values
			readCliPID(PITCH_PID);
			cliPrintF("\nPitch Rate PID Received....\n");

			cliQuery = 'a';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'C': // Read Yaw PID Values
			readCliPID(YAW_PID);
			cliPrintF("\nYaw Rate PID Received....\n");

			cliQuery = 'a';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'D': // Read Roll RC Parameters
			eepromConfig.rollRateCmdInput     = (uint8_t)readFloatCLI();
			eepromConfig.gimbalRollRate       = readFloatCLI() * D2R;
			eepromConfig.gimbalRollLeftLimit  = readFloatCLI() * D2R;
			eepromConfig.gimbalRollRightLimit = readFloatCLI() * D2R;

			cliPrintF("\nRoll RC Parameters Received....\n");

			cliQuery = 'd';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'E': // Read Pitch RC Parameters
			eepromConfig.pitchRateCmdInput     = (uint8_t)readFloatCLI();
			eepromConfig.gimbalPitchRate       = readFloatCLI() * D2R;
			eepromConfig.gimbalPitchDownLimit  = readFloatCLI() * D2R;
			eepromConfig.gimbalPitchUpLimit    = readFloatCLI() * D2R;

			cliPrintF("\nPitch RC Parameters Received....\n");

			cliQuery = 'd';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'F': // Read Yaw RC Parameters
			eepromConfig.yawRateCmdInput     = (uint8_t)readFloatCLI();
			eepromConfig.gimbalYawRate       = readFloatCLI() * D2R;
			eepromConfig.gimbalYawLeftLimit  = readFloatCLI() * D2R;
			eepromConfig.gimbalYawRightLimit = readFloatCLI() * D2R;

			cliPrintF("\nYaw RC Parameters Received....\n");

			cliQuery = 'd';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'I': // Read Gimbal Axis Enable Flags
			eepromConfig.rollEnabled  = (uint8_t)readFloatCLI();
			eepromConfig.pitchEnabled = (uint8_t)readFloatCLI();
			eepromConfig.yawEnabled   = (uint8_t)readFloatCLI();

			cliPrintF("\nGimbal Axis Enable Flags Received....\n");

			pwmMotorDriverInit();

			cliQuery = 'i';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'J': // Read Gimbal Axis Power Levels
			eepromConfig.rollPower  = readFloatCLI();
			eepromConfig.pitchPower = readFloatCLI();
			eepromConfig.yawPower   = readFloatCLI();

			cliPrintF("\nGimbal Axis Power Levels Received....\n");

			cliQuery = 'j';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'K': // Read Gimbal Rate Limit
			eepromConfig.rateLimit = readFloatCLI() * D2R;

			cliPrintF("\nGimbal Rate Limit Received....\n");

			cliQuery = 'k';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'L': // Read Gimbal IMU Orientation
			eepromConfig.imuOrientation = (uint8_t)readFloatCLI();

			cliPrintF("\nGimbal IMU Orientation Received....\n");

			orientIMU();

			cliQuery = 'l';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'M': // Read Test Phase
			testPhase = readFloatCLI() * D2R;

			cliPrintF("\nTest Phase Received....\n");

			cliQuery = 'm';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'N': // Read Test Phase Delta
			testPhaseDelta = readFloatCLI() * D2R;

			cliPrintF("\nTest Phase Delta Received....\n");

			cliQuery = 'n';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'O': // Set Motor Poles
			eepromConfig.rollMotorPoles  = readFloatCLI();
			eepromConfig.pitchMotorPoles = readFloatCLI();
			eepromConfig.yawMotorPoles   = readFloatCLI();

			cliPrintF("\nMotor Pole Counts Received....\n");

			cliQuery = 'o';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'P': // Sensor CLI
			disableGimbal();

			sensorCLI();

			enableGimbal();

			cliQuery = 'x';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'Q': // Read Roll Filter Time Constants
			eepromConfig.accelY500HzLowPassTau             = readFloatCLI();
			eepromConfig.rollRatePointingCmd50HzLowPassTau = readFloatCLI();
			eepromConfig.rollAttPointingCmd50HzLowPassTau  = readFloatCLI();

			initFirstOrderFilter();
			firstOrderFilters[ACCEL_Y_500HZ_LOWPASS ].previousInput  = sensors.accel500Hz[YAXIS];
			firstOrderFilters[ACCEL_Y_500HZ_LOWPASS ].previousOutput = sensors.accel500Hz[YAXIS];

			cliPrintF("\nRoll Filter Time Constants Received....\n");

			cliQuery = 'q';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'R': // Reset to Bootloader
			cliPrintF("Entering Bootloader....\n\n");
			delay(1000);
			bootloader();
			break;

		///////////////////////////////

		case 'S': // Reset System
			cliPrintF("\nSystem Rebooting....\n\n");
			delay(1000);
			reboot();
			break;

		///////////////////////////////

		case 'T': // Read Pitch Filter Time Constants
			eepromConfig.accelX500HzLowPassTau              = readFloatCLI();
			eepromConfig.pitchRatePointingCmd50HzLowPassTau = readFloatCLI();
			eepromConfig.pitchAttPointingCmd50HzLowPassTau  = readFloatCLI();

			initFirstOrderFilter();
			firstOrderFilters[ACCEL_X_500HZ_LOWPASS].previousInput  = sensors.accel500Hz[XAXIS];
			firstOrderFilters[ACCEL_X_500HZ_LOWPASS].previousOutput = sensors.accel500Hz[XAXIS];

			cliPrintF("\nPitch Filter Time Constants Received....\n");

			cliQuery = 'q';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'U': // Read Yaw Filter Time Constants
			eepromConfig.accelZ500HzLowPassTau            = readFloatCLI();
			eepromConfig.yawRatePointingCmd50HzLowPassTau = readFloatCLI();
			eepromConfig.yawAttPointingCmd50HzLowPassTau  = readFloatCLI();

			initFirstOrderFilter();
			firstOrderFilters[ACCEL_Z_500HZ_LOWPASS  ].previousInput  = sensors.accel500Hz[ZAXIS];
			firstOrderFilters[ACCEL_Z_500HZ_LOWPASS  ].previousOutput = sensors.accel500Hz[ZAXIS];

			cliPrintF("\nYaw Filter Time Constants Received....\n");

			cliQuery = 'q';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'V': // Reset EEPROM Parameters
			cliPrintF("\nEEPROM Parameters Reset....\n");
			checkFirstTime(true);
			cliPrintF("\nSystem Rebooting....\n\n");
			delay(1000);
			reboot();
			break;

		///////////////////////////////

		case 'W': // Write EEPROM Parameters
			cliPrintF("\nWriting EEPROM Parameters....\n");
			writeEEPROM();

			cliQuery = 'x';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'X': // Not Used
			cliQuery = 'x';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'Y': // Read AutoPan Enable Flags
			eepromConfig.rollAutoPanEnabled  = (uint8_t)readFloatCLI();
			eepromConfig.pitchAutoPanEnabled = (uint8_t)readFloatCLI();
			eepromConfig.yawAutoPanEnabled   = (uint8_t)readFloatCLI();

			eepromConfig.rollAutoPanEnabled  = false;  // HJI Function not implemented yet
			eepromConfig.pitchAutoPanEnabled = false;  // HJI Function not implemented yet

			cliPrintF("\nAutoPan Enable Flags Received....\n");

			cliQuery = 'y';
			validCliCommand = true;
			break;

		///////////////////////////////

		case 'Z': // Toggle Gimbal Enable/Disable State
			if (gimbalStateEnabled == true)
			{
				disableGimbal();

				gimbalStateEnabled = false;
			}

			else
			{
				enableGimbal();

				gimbalStateEnabled = true;
			}

			cliQuery = 'x';
			break;

		///////////////////////////////

		case '+': // Increment Test Phase
			testPhase += testPhaseDelta;

			cliQuery = 'm';
			validCliCommand = true;
			break;

		///////////////////////////////

		case '-': // Decrement Test Phase
			testPhase -= testPhaseDelta;

			cliQuery = 'm';
			validCliCommand = true;
			break;

		///////////////////////////////

		case '?': // Command Summary
			cliBusy = true;

			cliPrintF("\n");
			cliPrintF("'a' Rate PIDs                      'A' Set Roll Rate PID Data       AB;P;I;D;windupGuard;dErrorCalc\n");
			cliPrintF("'b' Loop Delta Times               'B' Set Pitch Rate PID Data      BB;P;I;D;windupGuard;dErrorCalc\n");
			cliPrintF("'c' Loop Execution Times           'C' Set Yaw Rate PID Data        CB;P;I;D;windupGuard;dErrorCalc\n");
			cliPrintF("'d' RC Parameters                  'D' Set Roll RC Parameters       DResponse;Rate;Left Limit;Right Limit\n");
			cliPrintF("'e' 500 Hz Accels                  'E' Set Pitch RC Parameters      EResponse;Rate;Down Limit;Up Limit\n");;
			cliPrintF("'f' 500 Hz Gyros                   'F' Set Yaw RC Parameters        FResponse;Rate;Left Limit;Right Limit\n");
			cliPrintF("'g' 10 hz Mag Data                 'G' Not Used\n");
			cliPrintF("'h' Attitudes                      'H' Not Used\n");
			cliPrintF("'i' Gimbal Axis Enable Flags       'I' Set Gimbal Axis Enable Flags IR;P;Y\n");
			cliPrintF("'j' Gimbal Axis Power Settings     'J' Set Gimbal Axis Power Levels JR;P;Y\n");
			cliPrintF("'k' Gimbal Rate Limit              'K' Set Gimbal Rate Limit\n");
			cliPrintF("'l' Gimbal IMU Orientation         'L' Set Gimbal IMU Orientation   LX, X = 1 thru 4\n");
			cliPrintF("\n");

			cliPrintF("Press space bar for more, or enter a command....\n");

			while (cliAvailable() == false);

			cliQuery = getChar();

			if (cliQuery != ' ')
			{
				validCliCommand = true;
				cliBusy = false;
				return;
			}

			cliPrintF("\n");
			cliPrintF("'m' Test Phase                     'M' Set Test Phase\n");
			cliPrintF("'n' Test Phase Delta               'N' Set Test Phase Delta\n");
			cliPrintF("'o' Motor Pole Counts              'O' Set Motor Pole Counts        ORPC;PPC;YPC\n");
			cliPrintF("'p' Counters                       'P' Sensor CLI\n");
			cliPrintF("'q' Filter Time Constants          'Q' Set Roll Filters             QAtt;RateCmd;AttCmd\n");
			cliPrintF("'r' Not Used                       'R' Reset and Enter Bootloader\n");
			cliPrintF("'s' Raw Receiver Commands          'S' Reset\n");
			cliPrintF("'t' Pointing Commands              'T' Set Pitch Filters            TAtt;RateCmd;AttCmd\n");
			cliPrintF("'u' PID Outputs                    'U' Set Yaw Filters              UAtt;RateCmd;AttCmd\n");
			cliPrintF("'v' Version                        'V' Reset EEPROM Parameters\n");
			cliPrintF("'w' Not Used                       'W' Write EEPROM Parameters\n");
			cliPrintF("'x' Terminate CLI Communication    'X' Not Used\n");
			cliPrintF("\n");

			cliPrintF("Press space bar for more, or enter a command....\n");

			while (cliAvailable() == false);

			cliQuery = getChar();

			if (cliQuery != ' ')
			{
				validCliCommand = true;
				cliBusy = false;
				return;
			}

			cliPrintF("\n");
			cliPrintF("'y' AutoPan Enbale Flags           'Y' Set AutoPan Enable Flags     YR;P;Y\n");
			cliPrintF("'z' Not Used                       'Z' Toggle Gimbal Enable/Disable State\n");
			cliPrintF("'+' Increment Test Phase           '?' Command Summary\n");
			cliPrintF("'-' Decrement Test Phase\n");
			cliPrintF("\n");

			cliQuery = 'x';
			cliBusy = false;
			break;

		///////////////////////////////

		default:
			cliPrintF("\nIgnoring Unknown Command %c (0x%2X)\n", cliQuery, cliQuery);
			cliQuery = 'x';
	}
}

///////////////////////////////////////////////////////////////////////////////
