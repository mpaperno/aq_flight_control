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

    Copyright © 2011-2014  Bill Nesbitt
*/

#ifndef _util_h
#define _util_h

#include "aq.h"
#include "CoOS.h"
#include <stdlib.h>

#define	UTIL_STACK_CHECK	    CFG_MAX_USER_TASKS		// uncomment to allow system to self check for stack overflows

#define UTIL_CCM_HEAP_SIZE	    (0x2800)	// 40KB

#define UTIL_ISR_DISABLE	    __asm volatile ( "CPSID   F\n")
#define UTIL_ISR_ENABLE		    __asm volatile ( "CPSIE   F\n")

#define yield(n)		    CoTickDelay(n)

#define constrainInt(v, lo, hi)	    (((int)(v) < (int)(lo)) ? (int)(lo) : (((int)(v) > (int)(hi)) ? (int)(hi) : (int)(v)))
#define constrainFloat(v, lo, hi)   (((float)(v) < (float)(lo)) ? (float)(lo) : (((float)(v) > (float)(hi)) ? (float)(hi) : (float)(v)))

#define PERIPH2BB(addr, bit)        ((uint32_t *)(PERIPH_BB_BASE + ((addr) - PERIPH_BASE) * 32 + ((bit) * 4)))

// first order filter
typedef struct {
    float tc;
    float z1;
} utilFilter_t;

typedef struct {
    const float *window;
    float *data;
    uint8_t n;
    uint8_t i;
} utilFirFilter_t;

extern void delay(unsigned long t);
extern void delayMicros(unsigned long t);
extern void dumpFloat(unsigned char n, float *floats);
extern void dumpInt(unsigned char n, int *ints);
extern void info(void);
extern OS_STK *aqStackInit(uint16_t size, char *name);
extern void *aqCalloc(size_t count, size_t size);
extern void aqFree(void *ptr, size_t count, size_t size);
extern void *aqDataCalloc(uint16_t count, uint16_t size);
extern void utilFilterInit(utilFilter_t *f, float dt, float tau, float setpoint);
extern void utilFilterInit3(utilFilter_t *f, float dt, float tau, float setpoint);
extern float utilFilter(utilFilter_t *f, float signal);
extern float utilFilter3(utilFilter_t *f, float signal);
extern void utilFilterReset(utilFilter_t *f, float setpoint);
extern void utilFilterReset3(utilFilter_t *f, float setpoint);
extern int ftoa(char *buf, float f, unsigned int digits);
extern void utilSerialNoString(void);
extern void utilVersionString(void);
extern float utilFirFilter(utilFirFilter_t *f, float newValue);
extern void utilFirFilterInit(utilFirFilter_t *f, const float *window, float *buffer, uint8_t n);
#ifdef UTIL_STACK_CHECK
extern void utilStackCheck(void);
extern uint16_t stackFrees[UTIL_STACK_CHECK];
extern uint16_t utilGetStackFree(const char *stackName);
#endif

#endif
