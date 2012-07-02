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

    Copyright © 2011, 2012  Bill Nesbitt
*/

// NOTE: These parameters must be passed to GCC:
//
//  -fsingle-precision-constant
//  -fno-gcse
//

#ifndef _aq_h
#define _aq_h

#define USE_MAVLINK

//#define HAS_VN100
//#define USE_VN100

// Which pressure sensor to use: 1 == sensor #1, 2 == sensor #2, 3 == both
#define PRESSURE_SENSOR		1

#define USE_PRES_ALT		1	// uncomment to use pressure altitude instead of GPS

//#define USE_BACKSTEP_CONTROLLER	2	// 1 == Euler angles, 2 == quat

#include "stm32f4xx.h"

#include "board_6_1.h"

#define TASK_STACK_SIZE		256

#ifndef M_PI
#define M_PI			3.14159265f
#endif

#define RAD_TO_DEG		(180.0f / M_PI)
#define DEG_TO_RAD		(M_PI / 180.0f)

#define GRAVITY			9.80665f	// m/s^2

#ifdef USE_MAVLINK
    #define AQ_NOTICE		mavlinkNotice
#else
    #define AQ_NOTICE		noticeSend
#endif

#define AQ_US_PER_SEC		1001567		// originally calibrated by GPS timepulse

#define	AQ_NOP			{__asm volatile ("nop\n\t");}
#define	AQ_4_NOPS		{AQ_NOP; AQ_NOP; AQ_NOP; AQ_NOP;}
#define	AQ_16_NOPS		{AQ_4_NOPS; AQ_4_NOPS; AQ_4_NOPS; AQ_4_NOPS;}
#define	AQ_64_NOPS		{AQ_16_NOPS; AQ_16_NOPS; AQ_16_NOPS; AQ_16_NOPS;}
#define	AQ_256_NOPS		{AQ_64_NOPS; AQ_64_NOPS; AQ_64_NOPS; AQ_64_NOPS;}

extern volatile unsigned long counter;
extern volatile unsigned long minCycles;

#endif
