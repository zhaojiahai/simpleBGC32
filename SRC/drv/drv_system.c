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

// Cycle counter stuff - these should be defined by CMSIS, but they aren't
#define DWT_CTRL    (*(volatile uint32_t *)0xE0001000)
#define DWT_CYCCNT  ((volatile uint32_t *)0xE0001004)
#define CYCCNTENA   (1 << 0)

///////////////////////////////////////////////////////////////////////////////

// Cycles per microsecond
static volatile uint32_t usTicks = 0;

///////////////////////////////////////////////////////////////////////////////

// Current uptime for 1kHz systick timer. will rollover after 49 days.
// Hopefully we won't care.
static uint32_t sysTickUptime = 0;
static volatile uint32_t sysTickCycleCounter = 0;

///////////////////////////////////////////////////////////////////////////////
// Cycle Counter
///////////////////////////////////////////////////////////////////////////////

static void cycleCounterInit(void)
{
	RCC_ClocksTypeDef clocks;
	RCC_GetClocksFreq(&clocks);
	usTicks = clocks.SYSCLK_Frequency / 1000000;

	// enable DWT access
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	// enable the CPU cycle counter
	DWT_CTRL |= CYCCNTENA;
}

///////////////////////////////////////
// Frame Timing Variables
///////////////////////////////////////

uint16_t frameCounter = 0;

semaphore_t frame_500Hz = false;
semaphore_t frame_100Hz = false;
semaphore_t frame_50Hz  = false;
semaphore_t frame_10Hz  = false;
semaphore_t frame_5Hz   = false;
semaphore_t frame_1Hz   = false;

uint32_t deltaTime1000Hz, executionTime1000Hz, previous1000HzTime;
uint32_t deltaTime500Hz,  executionTime500Hz,  previous500HzTime;
uint32_t deltaTime100Hz,  executionTime100Hz,  previous100HzTime;
uint32_t deltaTime50Hz,   executionTime50Hz,   previous50HzTime;
uint32_t deltaTime10Hz,   executionTime10Hz,   previous10HzTime;
uint32_t deltaTime5Hz,    executionTime5Hz,    previous5HzTime;
uint32_t deltaTime1Hz,    executionTime1Hz,    previous1HzTime;

float dt500Hz;

uint8_t systemReady = false;

///////////////////////////////////////////////////////////////////////////////
// SysTick
///////////////////////////////////////////////////////////////////////////////

void SysTick_Handler(void)
{
	uint32_t currentTime;

#ifdef _DTIMING
	LA1_ENABLE;
#endif

	sysTickCycleCounter = *DWT_CYCCNT;
	sysTickUptime++;

	//@HackOS: 系统已经启动且MPU6050没有在校准
	if ((systemReady == true)  &&
	        (mpu6050Calibrating == false)) // HJI && (magCalibrating     == false))

	{
		frameCounter++;

		if (frameCounter > FRAME_COUNT)
			frameCounter = 1;

		///////////////////////////////

		currentTime = micros();
		deltaTime1000Hz = currentTime - previous1000HzTime;
		previous1000HzTime = currentTime;

		///////////////////////////////

		//@HackOS: 500HZ循环
		if ((frameCounter % COUNT_500HZ) == 0)
		{
			frame_500Hz = true;

			//@HackOS: 读取MPU6050
			readMPU6050();

			accelData500Hz[XAXIS] = rawAccel[XAXIS].value;
			accelData500Hz[YAXIS] = rawAccel[YAXIS].value;
			accelData500Hz[ZAXIS] = rawAccel[ZAXIS].value;

			gyroData500Hz[ROLL ] = rawGyro[ROLL ].value;
			gyroData500Hz[PITCH] = rawGyro[PITCH].value;
			gyroData500Hz[YAW  ] = rawGyro[YAW  ].value;
		}

		///////////////////////////////

		//@HackOS: 100HZ循环标志
		if ((frameCounter % COUNT_100HZ) == 0)
		{
			frame_100Hz = true;
		}

		//@HackOS: 50HZ循环标志
		///////////////////////////////

		if ((frameCounter % COUNT_50HZ) == 0)
		{
			frame_50Hz = true;
		}

		///////////////////////////////

		// HJI if (((frameCounter + 1) % COUNT_10HZ) == 0)
		// HJI     newMagData = readMag();

		if ((frameCounter % COUNT_10HZ) == 0)
			frame_10Hz = true;

		///////////////////////////////

		if ((frameCounter % COUNT_5HZ) == 0)
			frame_5Hz = true;

		///////////////////////////////

		if ((frameCounter % COUNT_1HZ) == 0)
			frame_1Hz = true;

		///////////////////////////////////

		executionTime1000Hz = micros() - currentTime;

		///////////////////////////////
	}

#ifdef _DTIMING
	LA1_DISABLE;
#endif

}

///////////////////////////////////////////////////////////////////////////////
// System Time in Microseconds
//
// Note: This can be called from within IRQ Handlers, so uses LDREX/STREX.
// If a higher priority IRQ or DMA or anything happens the STREX will fail
// and restart the loop. Otherwise the same number that was read is harmlessly
// written back.
///////////////////////////////////////////////////////////////////////////////

uint32_t micros(void)
{
	register uint32_t oldCycle, cycle, timeMs;

	do
	{
		timeMs = __LDREXW(&sysTickUptime);
		cycle = *DWT_CYCCNT;
		oldCycle = sysTickCycleCounter;
	}
	while (__STREXW(timeMs , &sysTickUptime));

	return (timeMs * 1000) + (cycle - oldCycle) / usTicks;
}

///////////////////////////////////////////////////////////////////////////////
// System Time in Milliseconds
///////////////////////////////////////////////////////////////////////////////

uint32_t millis(void)
{
	return sysTickUptime;
}

///////////////////////////////////////////////////////////////////////////////
// Set up loop timing pins
///////////////////////////////////////////////////////////////////////////////

#ifdef _DTIMING
void timingSetup(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC,   ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);

	// Init pins
	GPIO_InitStructure.GPIO_Pin   = GPIO_Pin_2 | GPIO_Pin_3;
	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

	GPIO_Init(GPIOC, &GPIO_InitStructure);

	LA2_DISABLE;
	LA1_DISABLE;
}
#endif

///////////////////////////////////////////////////////////////////////////////
// System Initialization
///////////////////////////////////////////////////////////////////////////////

void systemInit(void)
{
	// Init cycle counter
	cycleCounterInit();

	// SysTick
	SysTick_Config(SystemCoreClock / 1000);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
	                       RCC_APB2Periph_GPIOC | RCC_APB2Periph_AFIO  |
	                       RCC_APB2Periph_TIM1  | RCC_APB2Periph_TIM8, ENABLE);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3  | RCC_APB1Periph_TIM4  |
	                       RCC_APB1Periph_TIM5  | RCC_APB1Periph_TIM6  | RCC_APB1Periph_I2C2, ENABLE);

#ifdef _DTIMING
	timingSetup();
#endif

	///////////////////////////////////////////////////////////////////////////

	checkFirstTime(false);
	readEEPROM();

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);  // 2 bits for pre-emption priority, 2 bits for subpriority

	pwmMotorDriverInit();

	cliInit();
	gpioInit();

	LED2_ON;

	delay(10000);  // 10 seconds of 20 second delay for sensor stabilization

	if (GetVCPConnectMode() != eVCPConnectReset)
	{
		cliPrintF("\r\nUSB startup delay...\r\n");
		delay(3000);

		if (GetVCPConnectMode() == eVCPConnectData)
		{
			cliPrintF("\r\nBGC32 firmware starting up, USB connected...\r\n");
		}
	}

	else
	{
		cliPrintF("\r\nDelaying for usb/serial driver to settle\r\n");
		delay(3000);
		cliPrintF("\r\nBGC32 firmware starting up, serial active...\r\n");
	}

#if defined(__DATE__) && defined(__TIME__)
	cliPrintF("\nBGC32 Firmware V%s, Build Date " __DATE__ " "__TIME__" \n", __BGC32_VERSION);
#endif

	if ((RCC->CR & RCC_CR_HSERDY) != RESET)
	{
		cliPrintF("\nRunning on external HSE clock, clock rate is %dMHz\n", SystemCoreClock / 1000000);
	}

	else
	{
		cliPrintF("\nERROR: Running on internal HSI clock, clock rate is %dMHz\n", SystemCoreClock / 1000000);
	}

	delay(10000);  // Remaining 10 seconds of 20 second delay for sensor stabilization - probably not long enough..

	LED1_ON;

	i2cInit(I2C2);
	rcInit();
	timingFunctionsInit();

	BKPInit();

	initFirstOrderFilter();
	initPID();
	initSinArray();

	orientIMU();

	initMPU6050();
	// initMag();
}

///////////////////////////////////////////////////////////////////////////////
// Delay Microseconds
///////////////////////////////////////////////////////////////////////////////

void delayMicroseconds(uint32_t us)
{
	uint32_t elapsed = 0;
	uint32_t lastCount = *DWT_CYCCNT;

	for (;;)
	{
		register uint32_t current_count = *DWT_CYCCNT;
		uint32_t elapsed_us;

		// measure the time elapsed since the last time we checked
		elapsed += current_count - lastCount;
		lastCount = current_count;

		// convert to microseconds
		elapsed_us = elapsed / usTicks;

		if (elapsed_us >= us)
			break;

		// reduce the delay by the elapsed time
		us -= elapsed_us;

		// keep fractional microseconds for the next iteration
		elapsed %= usTicks;
	}
}

///////////////////////////////////////////////////////////////////////////////
// Delay Milliseconds
///////////////////////////////////////////////////////////////////////////////

void delay(uint32_t ms)
{
	while (ms--)
		delayMicroseconds(1000);
}

///////////////////////////////////////////////////////////////////////////////

#define AIRCR_RESET         0x05FA0000
#define AIRCR_RESET_REQ     (AIRCR_RESET | 0x04);


#define USER_CODE_RAM               (0x20000C00)
#define BOOT_LOADER_MAGIC_ADDR      ((unsigned long*)(USER_CODE_RAM-4))
#define START_BOOT_LOADER_MAGIC     (0x4AFC6BB2)
#define START_MAIN_MAGIC            (0x4AFC6BB3)
#define STAY_IN_BOOTLOADER_MAGIC    (0x4AFC6BB4)

///////////////////////////////////////////////////////////////////////////////

void BKPInit(void)
{
	/* Enable clock for Power interface */
	RCC->APB1ENR |= RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN;
}

///////////////////////////////////////////////////////////////////////////////

unsigned long BKPRead(void)
{
	unsigned long val ;
	/* Enable access to RTC/BKP registers */
	PWR->CR |= PWR_CR_DBP;

	val = BKP->DR41 | (BKP->DR42 << 16);

	/* Disable access to the RTC/BKP registers */
	PWR->CR &= ~PWR_CR_DBP;

	return val;
}

///////////////////////////////////////////////////////////////////////////////

void BKPWrite(unsigned long val)
{
	/* Enable access to RTC/BKP registers */
	PWR->CR |= PWR_CR_DBP;

	BKP->DR41 = val & 0xffff;
	BKP->DR42 = val >> 16;

	/* Disable access to the RTC/BKP registers */
	PWR->CR &= ~PWR_CR_DBP;
}

///////////////////////////////////////////////////////////////////////////////

void bootloader(void)
{
	BKPInit();
	BKPWrite(STAY_IN_BOOTLOADER_MAGIC);
	reboot();
}

///////////////////////////////////////////////////////////////////////////////

void reboot(void)
{
	/* Reset  */
	SCB->AIRCR = AIRCR_RESET_REQ;

	/*  should never get here */
	while (1)
	{
		__ASM volatile("nop");
	}
}

///////////////////////////////////////////////////////////////////////////////

void _init(void) __attribute__((weak));
void _init(void)
{
	;
}
