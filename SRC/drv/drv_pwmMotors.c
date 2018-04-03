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
#include "stm32f10x.h"
///////////////////////////////////////////////////////////////////////////////

// TIM8 Roll
// PC6, PC7, PC8 used for Roll,  TIM_OCPolarity_High
// PA7, PB0, PB1 used for RollN, TIM_OCPolarity_High

#define ROLL_A_GPIO   GPIOC
#define ROLL_A_PIN    GPIO_Pin_6
#define ROLL_B_GPIO   GPIOC
#define ROLL_B_PIN    GPIO_Pin_7
#define ROLL_C_GPIO   GPIOC
#define ROLL_C_PIN    GPIO_Pin_8

#define ROLL_AN_GPIO  GPIOA
#define ROLL_AN_PIN   GPIO_Pin_7
#define ROLL_BN_GPIO  GPIOB
#define ROLL_BN_PIN   GPIO_Pin_0
#define ROLL_CN_GPIO  GPIOB
#define ROLL_CN_PIN   GPIO_Pin_1

// TIM1 Pitch
// PA8,  PA9,  PA10 used for Pitch,  TIM_OCPolarity_High
// PB13, PB14, PB15 used for PitchN, TIM_OCPolarity_High

#define PITCH_A_GPIO    GPIOA
#define PITCH_A_PIN     GPIO_Pin_8
#define PITCH_B_GPIO    GPIOA
#define PITCH_B_PIN     GPIO_Pin_9
#define PITCH_C_GPIO    GPIOA
#define PITCH_C_PIN     GPIO_Pin_10

#define PITCH_AN_GPIO   GPIOB
#define PITCH_AN_PIN    GPIO_Pin_13
#define PITCH_BN_GPIO   GPIOB
#define PITCH_BN_PIN    GPIO_Pin_14
#define PITCH_CN_GPIO   GPIOB
#define PITCH_CN_PIN    GPIO_Pin_15

// TIM5 Yaw
// PA0, PA1, PA2 used for Yaw,  TIM_OCPolarity_High

#define YAW_A_GPIO     GPIOA
#define YAW_A_PIN      GPIO_Pin_0
#define YAW_B_GPIO     GPIOA
#define YAW_B_PIN      GPIO_Pin_1
#define YAW_C_GPIO     GPIOA
#define YAW_C_PIN      GPIO_Pin_2

// TIM4 YawN
// PB6, PB7, PB8 used for YawN, TIM_OCPolarity_Low

#define YAW_AN_GPIO    GPIOB
#define YAW_AN_PIN     GPIO_Pin_6
#define YAW_BN_GPIO    GPIOB
#define YAW_BN_PIN     GPIO_Pin_7
#define YAW_CN_GPIO    GPIOB
#define YAW_CN_PIN     GPIO_Pin_8

///////////////////////////////////////

#define PWM_PERIOD 1000

#define MAX_CNT (PWM_PERIOD * 8 / 10)

#define BB_PERIPH_ADDR(addr, bit) ((vu32*)(PERIPH_BB_BASE + ((void*)(addr)-(void*)PERIPH_BASE) * 32 + (bit) * 4))

///////////////////////////////////////

static int pwmMotorDriverInitDone = false;

int timer1timer8deadTimeRegister = 200; // this is not just a delay value, check CPU reference manual for TIMx_BDTR DTG bit 0-7
int timer4timer5deadTimeDelay    = 80;  // in 18MHz ticks

static int rollPhase[3], pitchPhase[3], yawPhase[3];

int maxCnt[NUMAXIS];
int minCnt[NUMAXIS];
int irqCnt[NUMAXIS];

uint8_t oddEvenFrame = 0;

///////////////////////////////////////////////////////////////////////////////
//  Counter Updates
///////////////////////////////////////////////////////////////////////////////

/*inline */void updateCounter(uint8_t channel, int value)
{
	irqCnt[channel]++;

	if (value > maxCnt[channel])
	{
		maxCnt[channel] = value;
	}

	if (value < minCnt[channel])
	{
		minCnt[channel] = value;
	}
}

///////////////////////////////////////////////////////////////////////////////
//  IRQ Setup
///////////////////////////////////////////////////////////////////////////////

static void setupPWMIrq(uint8_t irq)
{
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel                   = irq;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;       //Preemption Priority
	NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd                = ENABLE;

	NVIC_Init(&NVIC_InitStructure);
}

///////////////////////////////////////////////////////////////////////////////
//  TIM8 IRQ Handler (ROLL)
///////////////////////////////////////////////////////////////////////////////

void TIM8_UP_IRQHandler(void) // roll axis
{

	unsigned short cnt;
	TIM8->SR &= ~TIM_SR_UIF; // clear UIF flag

	__disable_irq_nested();
	cnt = TIM8->CNT;
	updateCounter(ROLL, cnt);

	if (cnt < MAX_CNT)
	{
		// make sure there is enough time to make all changes
		if (eepromConfig.rollEnabled)
		{
			TIM8->CCR1 = rollPhase[0];
			TIM8->CCR2 = rollPhase[1];
			TIM8->CCR3 = rollPhase[2];
		}

		else
		{
			TIM8->CCR1 = 0;
			TIM8->CCR2 = 0;
			TIM8->CCR3 = 0;
		}

		TIM8->DIER &= ~TIM_DIER_UIE; // disable update interrupt
	}

	__enable_irq_nested();
}

///////////////////////////////////////////////////////////////////////////////
//  TIM1 IRQ Handler (PITCH)
///////////////////////////////////////////////////////////////////////////////

void TIM1_UP_IRQHandler(void) // pitch axis
{

	unsigned short cnt ;
	TIM1->SR &= ~TIM_SR_UIF; // clear UIF flag

	__disable_irq_nested();
	cnt = TIM1->CNT;
	updateCounter(PITCH, cnt);

	if (cnt < MAX_CNT)
	{
		// make sure there is enough time to make all changes
		if (eepromConfig.pitchEnabled)
		{
			TIM1->CCR1 = pitchPhase[0];
			TIM1->CCR2 = pitchPhase[1];
			TIM1->CCR3 = pitchPhase[2];
		}

		else
		{
			TIM1->CCR1 = 0;
			TIM1->CCR2 = 0;
			TIM1->CCR3 = 0;
		}

		TIM1->DIER &= ~TIM_DIER_UIE; // disable update interrupt
	}

	__enable_irq_nested();
}

///////////////////////////////////////////////////////////////////////////////
//  TIM5 IRQ Handler (YAW)
///////////////////////////////////////////////////////////////////////////////

void TIM5_IRQHandler(void) // yaw axis
{

	unsigned short cnt ;

	if (TIM5->SR & TIM_SR_UIF) // if UIF flag is set
	{
		TIM5->SR &= ~TIM_SR_UIF; // clear UIF flag

		__disable_irq_nested();
		cnt = TIM5->CNT;
		updateCounter(YAW, cnt);

		if (cnt < MAX_CNT)
		{
			// make sure there is enough time to make all changes
			if (eepromConfig.yawEnabled)
			{
				int deadTime = 2 * timer4timer5deadTimeDelay;
				TIM4->CCR1 = yawPhase[0] + deadTime;
				TIM4->CCR2 = yawPhase[1] + deadTime;
				TIM4->CCR3 = yawPhase[2] + deadTime;

				TIM5->CCR1 = yawPhase[0];
				TIM5->CCR2 = yawPhase[1];
				TIM5->CCR3 = yawPhase[2];
			}

			else
			{
				TIM4->CCR1 = PWM_PERIOD + 1;
				TIM4->CCR2 = PWM_PERIOD + 1;
				TIM4->CCR3 = PWM_PERIOD + 1;

				TIM5->CCR1 = 0;
				TIM5->CCR2 = 0;
				TIM5->CCR3 = 0;
			}

			TIM5->DIER &= ~TIM_DIER_UIE;  // disable update interrupt
		}

		__enable_irq_nested();
	}
}

///////////////////////////////////////////////////////////////////////////////
//  Timer Channel Configuration
///////////////////////////////////////////////////////////////////////////////

static void timerChannelConfig(TIM_TypeDef *tim, TIM_OCInitTypeDef *OCInitStructure)
{
	TIM_OC1Init(tim, OCInitStructure);
	TIM_OC2Init(tim, OCInitStructure);
	TIM_OC3Init(tim, OCInitStructure);

	TIM_OC1PreloadConfig(tim, TIM_OCPreload_Enable);
	TIM_OC2PreloadConfig(tim, TIM_OCPreload_Enable);
	TIM_OC3PreloadConfig(tim, TIM_OCPreload_Enable);
}

///////////////////////////////////////////////////////////////////////////////
//  Advanced Timer Configuration
///////////////////////////////////////////////////////////////////////////////

static void timerPWMadvancedConfig(TIM_TypeDef *tim)
{
	TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef           TIM_OCInitStructure;
	TIM_BDTRInitTypeDef         TIM_BDTRInitStructure;

	//Time Base configuration
	TIM_TimeBaseInitStructure.TIM_Prescaler         = (4 - 1);                 // 72 Mhz / (3 + 1) = 18 MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode       = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period            = PWM_PERIOD - 1;          // 18 Mhz / 1000 = 18 kHz
	TIM_TimeBaseInitStructure.TIM_ClockDivision     = 0;
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;

	TIM_TimeBaseInit(tim, &TIM_TimeBaseInitStructure);

	//Automatic Output enable, Break, dead time and lock configuration
	TIM_BDTRInitStructure.TIM_OSSRState       = TIM_OSSRState_Enable;
	TIM_BDTRInitStructure.TIM_OSSIState       = TIM_OSSIState_Enable;
	TIM_BDTRInitStructure.TIM_LOCKLevel       = TIM_LOCKLevel_1;
	TIM_BDTRInitStructure.TIM_DeadTime        = timer1timer8deadTimeRegister;
	TIM_BDTRInitStructure.TIM_Break           = TIM_Break_Disable;
	TIM_BDTRInitStructure.TIM_BreakPolarity   = TIM_BreakPolarity_High;
	TIM_BDTRInitStructure.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;

	TIM_BDTRConfig(tim, &TIM_BDTRInitStructure);

	//Configuration in PWM mode
	TIM_OCInitStructure.TIM_OCMode       = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OutputState  = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Enable;
	TIM_OCInitStructure.TIM_Pulse        = 0;
	TIM_OCInitStructure.TIM_OCPolarity   = TIM_OCPolarity_High;
	TIM_OCInitStructure.TIM_OCNPolarity  = TIM_OCNPolarity_High;
	TIM_OCInitStructure.TIM_OCIdleState  = TIM_OCIdleState_Set;
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCIdleState_Reset;

	timerChannelConfig(tim, &TIM_OCInitStructure);
}

///////////////////////////////////////////////////////////////////////////////
//  General Timer Configuration
///////////////////////////////////////////////////////////////////////////////

static void timerPWMgeneralConfig(TIM_TypeDef *tim, int polarity)
{
	TIM_TimeBaseInitTypeDef     TIM_TimeBaseInitStructure;
	TIM_OCInitTypeDef           TIM_OCInitStructure;

	TIM_TimeBaseInitStructure.TIM_Prescaler     = (4 - 1);                 // 72 Mhz / (3 + 1) = 18 MHz
	TIM_TimeBaseInitStructure.TIM_CounterMode   = TIM_CounterMode_Up;
	TIM_TimeBaseInitStructure.TIM_Period        = PWM_PERIOD - 1;          // 18 Mhz / 1000 = 18 kHz
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV4;

	TIM_TimeBaseInit(tim, &TIM_TimeBaseInitStructure);

	TIM_OCInitStructure.TIM_OCMode      = TIM_OCMode_PWM1 ;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse       = 0;
	TIM_OCInitStructure.TIM_OCPolarity  = polarity;

	timerChannelConfig(tim, &TIM_OCInitStructure);
}

///////////////////////////////////////////////////////////////////////////////
//  Set PWM Via Table Lookup
//  @HackOS: 通过查找表设置PWM
///////////////////////////////////////////////////////////////////////////////

void setPWMFastTable(int *pwm, float angle, float power)
{

	int iPower;

	//@HackOS: 角度索引(对应2TT范围内的哪个查找表项)
	int angleInt;

	if (testPhase >= 0)
	{
		angle = testPhase;
	}

	//@HackOS: 得出索引项
	angleInt = (int)round(angle / M_TWOPI * SINARRAYSIZE);

	//@HackOS: 防止溢出
	angleInt = angleInt % SINARRAYSIZE;

	if (angleInt < 0)
	{
		angleInt = SINARRAYSIZE + angleInt;
	}

	//int iPower = 5 * (int)power;
	iPower = (int)((PWM_PERIOD / 2 - timer4timer5deadTimeDelay)  * power / 100);

	pwm[0] = (sinDataI16[ angleInt                               % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
	pwm[1] = (sinDataI16[(angleInt +  1 * SINARRAYSIZE / 3)      % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
	pwm[2] = (sinDataI16[(angleInt + (2 * SINARRAYSIZE + 1) / 3) % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
}

///////////////////////////////////////////////////////////////////////////////
//  Set PWM Via Table Lookup
///////////////////////////////////////////////////////////////////////////////

/*
void setPWMFastTable(int *pwm, float angle, float power, uint8_t reverse)
{
    if (testPhase >= 0)
    {
        angle = testPhase;
    }

    int angleInt = (int)round(angle / M_TWOPI * SINARRAYSIZE);

    angleInt = angleInt % SINARRAYSIZE;

    if (angleInt < 0)
    {
        angleInt = SINARRAYSIZE + angleInt;
    }

    int iPower = 5 * (int)power;

    if (reverse == false)
    {
        pwm[0] = (sinDataI16[ angleInt                               % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
        pwm[1] = (sinDataI16[(angleInt +  1 * SINARRAYSIZE / 3)      % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
        pwm[2] = (sinDataI16[(angleInt + (2 * SINARRAYSIZE + 1) / 3) % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
    }
    else
    {
        pwm[0] = -(sinDataI16[ angleInt                               % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
        pwm[1] = -(sinDataI16[(angleInt -  1 * SINARRAYSIZE / 3)      % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
        pwm[2] = -(sinDataI16[(angleInt - (2 * SINARRAYSIZE + 1) / 3) % SINARRAYSIZE] * iPower + SINARRAYSCALE / 2) / SINARRAYSCALE + PWM_PERIOD / 2;
    }
}
*/

///////////////////////////////////////////////////////////////////////////////
//  Set PWM
///////////////////////////////////////////////////////////////////////////////

void setPWM(int *pwm, float angle, float power)
{
	setPWMFastTable(pwm, angle, power);
}

///////////////////////////////////////////////////////////////////////////////
//  Set PWM Data
///////////////////////////////////////////////////////////////////////////////

void setPWMData(int *target, int *pwm)
{
	__disable_irq_nested();
	target[0] = pwm[0];
	target[1] = pwm[1];
	target[2] = pwm[2];
	__enable_irq_nested();
}

///////////////////////////////////////////////////////////////////////////////
//  Limit Yaw PWM
///////////////////////////////////////////////////////////////////////////////

void limitYawPWM(int *pwm)
{
	int i;
	int maxVal = PWM_PERIOD - 2 * timer4timer5deadTimeDelay;

	for (i = 0; i < 3; i++)
	{
		pwm[i] -= timer4timer5deadTimeDelay;

		if (pwm[i] >= maxVal)
		{
			pwm[i] = maxVal;
		}

		if (pwm[i] < timer4timer5deadTimeDelay)
		{
			pwm[i] = 0;
		}
	}
}

///////////////////////////////////////////////////////////////////////////////
//  Activate Interrupt
///////////////////////////////////////////////////////////////////////////////

void activateIRQ(TIM_TypeDef *tim)
{
	__disable_irq_nested();
	tim->SR &= ~TIM_SR_UIF;   // clear UIF flag
	tim->DIER = TIM_DIER_UIE; // Enable update interrupt
	__enable_irq_nested();
}

///////////////////////////////////////////////////////////////////////////////
//  Set Roll Axis PWM
///////////////////////////////////////////////////////////////////////////////

void setRollMotor(float phi, int power)
{
	int pwm[3];

	setPWM(pwm, phi, power);
	setPWMData(rollPhase, pwm);
	activateIRQ(TIM8);
}

///////////////////////////////////////////////////////////////////////////////
//  Set Pitch Axis PWM
///////////////////////////////////////////////////////////////////////////////

void setPitchMotor(float theta, int power)
{
	int pwm[3];

	setPWM(pwm, theta, power);
	setPWMData(pitchPhase, pwm);
	activateIRQ(TIM1);
}

///////////////////////////////////////////////////////////////////////////////
//  Set Yaw Axis PWM
///////////////////////////////////////////////////////////////////////////////

void setYawMotor(float psi, int power)
{
	int pwm[3];

	setPWM(pwm, psi, power);
	limitYawPWM(pwm);
	setPWMData(yawPhase, pwm);
	activateIRQ(TIM5);
}

///////////////////////////////////////////////////////////////////////////////
//  Force Motor Update
///////////////////////////////////////////////////////////////////////////////

void forceMotorUpdate(void)
{
	activateIRQ(TIM8);
	activateIRQ(TIM1);
	activateIRQ(TIM5);
}

///////////////////////////////////////////////////////////////////////////////
//  Initialize PWM Motor Drivers
///////////////////////////////////////////////////////////////////////////////

void pwmMotorDriverInit(void)
{

	GPIO_InitTypeDef         GPIO_InitStructure;


	if (pwmMotorDriverInitDone)
	{
		forceMotorUpdate();
		// make sure this init function is not called twice
		return;
	}



	///////////////////////////////////

	// Roll PWM Timer Initialization here

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin   = ROLL_A_PIN | ROLL_B_PIN | ROLL_C_PIN;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = ROLL_AN_PIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = ROLL_BN_PIN | ROLL_CN_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	irqCnt[ROLL] = 0;
	maxCnt[ROLL] = 0;
	minCnt[ROLL] = PWM_PERIOD + 1;

	timerPWMadvancedConfig(TIM8);

	TIM8->CNT = timer4timer5deadTimeDelay + 5 + PWM_PERIOD * 2 / 3;  // 751

	setupPWMIrq(TIM8_UP_IRQn);

	__disable_irq_nested();
	{
		//        vu32 *tim8Enable = BB_PERIPH_ADDR(&(TIM8->CR1), 0);
		//        *tim8Enable = 1;

		TIM_Cmd(TIM8, ENABLE);
		TIM_CtrlPWMOutputs(TIM8, ENABLE);
	}
	__enable_irq_nested();


	///////////////////////////////////
	// Pitch PWM Timer Initialization here

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin   = PITCH_A_PIN | PITCH_B_PIN | PITCH_C_PIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = PITCH_AN_PIN | PITCH_BN_PIN | PITCH_CN_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	irqCnt[PITCH] = 0;
	maxCnt[PITCH] = 0;
	minCnt[PITCH] = PWM_PERIOD + 1;

	timerPWMadvancedConfig(TIM1);

	TIM1->CNT = timer4timer5deadTimeDelay + 3 + PWM_PERIOD / 3;  // 416

	setupPWMIrq(TIM1_UP_IRQn);

	__disable_irq_nested();
	{
		//        vu32 *tim1Enable = BB_PERIPH_ADDR(&(TIM1->CR1), 0);
		//        *tim1Enable = 1;

		TIM_Cmd(TIM1, ENABLE);
		TIM_CtrlPWMOutputs(TIM1, ENABLE);
	}
	__enable_irq_nested();


	///////////////////////////////////
	// Yaw PWM Timers Initialization here

	GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;

	GPIO_InitStructure.GPIO_Pin   = YAW_A_PIN | YAW_B_PIN | YAW_C_PIN;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin   = YAW_AN_PIN | YAW_BN_PIN | YAW_CN_PIN;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	irqCnt[YAW] = 0;
	maxCnt[YAW] = 0;
	minCnt[YAW] = PWM_PERIOD + 1;

	timerPWMgeneralConfig(TIM5, TIM_OCPolarity_High);
	timerPWMgeneralConfig(TIM4, TIM_OCPolarity_Low);

	setupPWMIrq(TIM5_IRQn);

	__disable_irq_nested();
	{
		//        vu32 *tim5Enable = BB_PERIPH_ADDR(&(TIM5->CR1), 0);
		//        vu32 *tim4Enable = BB_PERIPH_ADDR(&(TIM4->CR1), 0);

		TIM4->CNT = timer4timer5deadTimeDelay;
		//        *tim5Enable = 1;
		//        *tim4Enable = 1;
		TIM_Cmd(TIM4, ENABLE);

		TIM_Cmd(TIM5, ENABLE);


		TIM_CtrlPWMOutputs(TIM5, ENABLE);
		TIM_CtrlPWMOutputs(TIM4, ENABLE);
	}
	__enable_irq_nested();

	///////////////////////////////////

	pwmMotorDriverInitDone = true;
}

///////////////////////////////////////////////////////////////////////////////
