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

#ifndef _digital_imu_h
#define _digital_imu_h

#include "rcc.h"
#include <CoOS.h>

#define DIMU_HAVE_MPU6000
//#define DIMU_HAVE_BMA250
#define DIMU_HAVE_HMC5983
#define DIMU_HAVE_MS5611

#define DIUM_IMUV1

#ifdef DIUM_IMUV1
#define DIMU_HAVE_EEPROM
#endif

#ifdef DIMU_HAVE_MPU6000
#include "mpu6000.h"
#endif
#ifdef DIMU_HAVE_EEPROM
#include "eeprom.h"
#endif
#ifdef DIMU_HAVE_HMC5983
#include "hmc5983.h"
#endif
#ifdef DIMU_HAVE_MS5611
#include "ms5611.h"
#endif

#define DIMU_STACK_SIZE	    250
#define DIMU_PRIORITY	    11

#define DIMU_PERIOD	    5000			// us
#define DIMU_DT		    ((float)DIMU_PERIOD / 1e6f)
#define DIMU_TEMP_TAU	    5.0f

#define DIMU_TIM	    TIM3
#define DIMU_CLOCK	    (rccClocks.PCLK1_Frequency * 2)
#define DIMU_EN		    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE)
#define DIMU_IRQ_CH	    TIM3_IRQn
#define DIMU_ISR	    TIM3_IRQHandler

#ifndef __rev16
    #define __rev16 __REV16
#endif

typedef void dIMUCallback_t(int);

typedef struct {
    OS_TID task;
    OS_FlagID flag;

    float temp;
    float dTemp, dTemp2, dTemp3;

    dIMUCallback_t *alarm1Callback, *alarm2Callback, *alarm3Callback;
    int alarm1Parameter, alarm2Parameter, alarm3Parameter;

    uint16_t nextPeriod;
    volatile uint32_t lastUpdate;
} dImuStruct_t;

extern dImuStruct_t dImuData;

extern void dIMUInit(void);
extern void dIMUTare(void);
extern void dIMUCancelAlarm1(void);
extern void dIMUCancelAlarm2(void);
extern void dIMUCancelAlarm3(void);
extern void dIMUSetAlarm1(int32_t us, dIMUCallback_t *callback, int parameter);
extern void dIMUSetAlarm2(int32_t us, dIMUCallback_t *callback, int parameter);
extern void dIMUSetAlarm3(int32_t us, dIMUCallback_t *callback, int parameter);

#endif
