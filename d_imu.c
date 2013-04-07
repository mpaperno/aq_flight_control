/*
    This file is part of AutoQuad.

    AutoQuad is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    AutoQuad is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
    You should have received a copy of the GNU General Public License
    along with AutoQuad.  If not, see <http://www.gnu.org/licenses/>.

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#include "imu.h"
#include "d_imu.h"
#include "util.h"
#include "aq_timer.h"

OS_STK *dIMUTaskStack;

dImuStruct_t dImuData __attribute__((section(".ccm")));

void dIMUCalcTempDiff(void) {
    float temp = 0.0f;
    int i = 0;

#ifdef DIMU_HAVE_MPU6000
    temp += mpu6000Data.temp;
    i++;
#endif
#ifdef DIMU_HAVE_MS5611
    temp += ms5611Data.temp;
    i++;
#endif
    dImuData.temp = temp / (float)i;

    dImuData.dTemp = dImuData.temp - IMU_ROOM_TEMP;
    dImuData.dTemp2 = dImuData.dTemp * dImuData.dTemp;
    dImuData.dTemp3 = dImuData.dTemp2 * dImuData.dTemp;
}

void dIMUTaskCode(void *unused) {
    uint32_t loops = 0;

    while (1) {
	// wait for work
	CoWaitForSingleFlag(dImuData.flag, 0);

	dIMUCalcTempDiff();

	// double rate gyo loop
#ifdef DIMU_HAVE_MPU6000
	mpu6000Decode();
#endif

	imuDImuDRateReady();

	// full sensor loop
	if ((loops & 0b1) == 0) {
#ifdef DIMU_HAVE_ADXL362
	    adxl362Decode();
#endif
#ifdef DIMU_HAVE_HMC5983
	    hmc5983Decode();
#endif
#ifdef DIMU_HAVE_MS5611
	    ms5611Decode();
#endif
	    dImuData.lastUpdate = timerMicros();
	    imuDImuSensorReady();
	}

	loops++;
    }
}

void dIMUInit(void) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

#ifdef DIMU_HAVE_MPU6000
    mpu6000PreInit();
#endif
#ifdef DIMU_HAVE_HMC5983
    hmc5983PreInit();
#endif
#ifdef DIMU_HAVE_MS5611
    ms5611PreInit();
#endif

#ifdef DIMU_HAVE_MPU6000
    mpu6000Init();
#endif
#ifdef DIMU_HAVE_HMC5983
    hmc5983Init();
#endif
#ifdef DIMU_HAVE_MS5611
    ms5611Init();
#endif
    dIMUTaskStack = aqStackInit(DIMU_STACK_SIZE, "DIMU");

    dImuData.flag = CoCreateFlag(1, 0);
    dImuData.task = CoCreateTask(dIMUTaskCode, (void *)0, DIMU_PRIORITY, &dIMUTaskStack[DIMU_STACK_SIZE-1], DIMU_STACK_SIZE);

    // setup digital IMU timer
    DIMU_EN;

    // Time base configuration for 1MHz (us)
    TIM_TimeBaseStructure.TIM_Period = 0xffff;
    TIM_TimeBaseStructure.TIM_Prescaler = (DIMU_CLOCK / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(DIMU_TIM, &TIM_TimeBaseStructure);

    // reset
    TIM_SetCounter(DIMU_TIM, 0);

    // Output Compare for alarms
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(DIMU_TIM, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(DIMU_TIM, TIM_OCPreload_Disable);

    TIM_OC2Init(DIMU_TIM, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(DIMU_TIM, TIM_OCPreload_Disable);

    TIM_OC3Init(DIMU_TIM, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(DIMU_TIM, TIM_OCPreload_Disable);

    TIM_OC4Init(DIMU_TIM, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(DIMU_TIM, TIM_OCPreload_Disable);

    // Enable the global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = DIMU_IRQ_CH;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // reset
    TIM_SetCounter(DIMU_TIM, 0);

    dIMUCancelAlarm1();
    dIMUCancelAlarm2();
    dIMUCancelAlarm3();

    // go...
    TIM_Cmd(DIMU_TIM, ENABLE);

#ifdef DIMU_HAVE_MPU6000
    mpu6000Enable();
#endif
#ifdef DIMU_HAVE_HMC5983
    hmc5983Enable();
#endif
#ifdef DIMU_HAVE_MS5611
    ms5611Enable();
#endif

    // setup IMU timestep alarm
    dImuData.nextPeriod = DIMU_TIM->CCR4 + DIMU_PERIOD/2;
    DIMU_TIM->CCR4 = dImuData.nextPeriod;
    DIMU_TIM->DIER |= TIM_IT_CC4;

#ifdef DIMU_HAVE_MPU6000
    mpu6600InitialBias();
#endif
#ifdef DIMU_HAVE_MS5611
    ms5611InitialBias();
#endif
}

void dIMUCancelAlarm1(void) {
    DIMU_TIM->DIER &= (uint16_t)~TIM_IT_CC1;
    TIM_ClearITPendingBit(DIMU_TIM, TIM_IT_CC1);
}

void dIMUCancelAlarm2(void) {
    DIMU_TIM->DIER &= (uint16_t)~TIM_IT_CC2;
    TIM_ClearITPendingBit(DIMU_TIM, TIM_IT_CC2);
}

void dIMUCancelAlarm3(void) {
    DIMU_TIM->DIER &= (uint16_t)~TIM_IT_CC3;
    TIM_ClearITPendingBit(DIMU_TIM, TIM_IT_CC3);
}

void dIMUSetAlarm1(int32_t us, dIMUCallback_t *callback, int parameter) {
    // schedule it
    dImuData.alarm1Callback = callback;
    dImuData.alarm1Parameter = parameter;

    DIMU_TIM->SR = (uint16_t)~TIM_IT_CC1;
    DIMU_TIM->CCR1 = DIMU_TIM->CNT + us;
    DIMU_TIM->DIER |= TIM_IT_CC1;
}

void dIMUSetAlarm2(int32_t us, dIMUCallback_t *callback, int parameter) {
    // schedule it
    dImuData.alarm2Callback = callback;
    dImuData.alarm2Parameter = parameter;

    DIMU_TIM->SR = (uint16_t)~TIM_IT_CC2;
    DIMU_TIM->CCR2 = DIMU_TIM->CNT + us;
    DIMU_TIM->DIER |= TIM_IT_CC2;
}

void dIMUSetAlarm3(int32_t us, dIMUCallback_t *callback, int parameter) {
    // schedule it
    dImuData.alarm3Callback = callback;
    dImuData.alarm3Parameter = parameter;

    DIMU_TIM->SR = (uint16_t)~TIM_IT_CC3;
    DIMU_TIM->CCR3 = DIMU_TIM->CNT + us;
    DIMU_TIM->DIER |= TIM_IT_CC3;
}

void DIMU_ISR(void) {
    // CC4 is used for IMU period timing
    if (TIM_GetITStatus(DIMU_TIM, TIM_IT_CC4) != RESET) {
	DIMU_TIM->SR = (uint16_t)~TIM_IT_CC4;

	// set next alarm
	dImuData.nextPeriod += DIMU_PERIOD/2;
	DIMU_TIM->CCR4 = dImuData.nextPeriod;

	CoEnterISR();
	isr_SetFlag(dImuData.flag);
	CoExitISR();
    }
    else if (TIM_GetITStatus(DIMU_TIM, TIM_IT_CC1) != RESET) {
	DIMU_TIM->SR = (uint16_t)~TIM_IT_CC1;

	// Disable the Interrupt
	DIMU_TIM->DIER &= (uint16_t)~TIM_IT_CC1;

	dImuData.alarm1Callback(dImuData.alarm1Parameter);
    }
    else if (TIM_GetITStatus(DIMU_TIM, TIM_IT_CC2) != RESET) {
	DIMU_TIM->SR = (uint16_t)~TIM_IT_CC2;

	// Disable the Interrupt
	DIMU_TIM->DIER &= (uint16_t)~TIM_IT_CC2;

	dImuData.alarm2Callback(dImuData.alarm2Parameter);
    }
    else if (TIM_GetITStatus(DIMU_TIM, TIM_IT_CC3) != RESET) {
	DIMU_TIM->SR = (uint16_t)~TIM_IT_CC3;

	// Disable the Interrupt
	DIMU_TIM->DIER &= (uint16_t)~TIM_IT_CC3;

	dImuData.alarm3Callback(dImuData.alarm3Parameter);
    }
}