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

#include "aq.h"
#include "fpu.h"
#include "aq_init.h"
#include "rcc.h"
#include "serial.h"
#include "motors.h"
#include "util.h"
#include <CoOS.h>

volatile unsigned long counter;
volatile unsigned long minCycles = 0xFFFFFFFF;

int main(void) {
    fpuInit();        // setup FPU context switching

    rccConfiguration();

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);

    CoInitOS();

    aqInitStack = aqStackInit(AQINIT_STACK_SIZE, "INIT");

    CoCreateTask(aqInit, (void *)0, AQINIT_PRIORITY, &aqInitStack[AQINIT_STACK_SIZE-1], AQINIT_STACK_SIZE);
    CoStartOS();

    return 0;
}

// self calibrating idle timer loop
void CoIdleTask(void* pdata) {
    uint32_t thisCycles, lastCycles = 0;
    volatile uint32_t cycles;
    volatile uint32_t *DWT_CYCCNT = (uint32_t *)0xE0001004;
    volatile uint32_t *DWT_CONTROL = (uint32_t *)0xE0001000;
    volatile uint32_t *SCB_DEMCR = (uint32_t *)0xE000EDFC;

    *SCB_DEMCR = *SCB_DEMCR | 0x01000000;
    *DWT_CONTROL = *DWT_CONTROL | 1; // enable the counter

    while (1) {
#ifdef UTIL_STACK_CHECK
        utilStackCheck();
#endif
        counter++;

        thisCycles = *DWT_CYCCNT;
        cycles = thisCycles - lastCycles;
        lastCycles = thisCycles;

        // record shortest number of instructions for loop
        if (cycles < minCycles)
            minCycles = cycles;
    }
}

void CoStkOverflowHook(OS_TID taskID) {
    // Process stack overflow here
    while (1)
        ;
}

void HardFault_Handler(void) {
    // to avoid the unpredictable flight in case of problems
    motorsOff();

    // Go to infinite loop when Hard Fault exception occurs
    while (1)
        ;
}

#ifdef USE_FULL_ASSERT
void assert_failed(uint8_t* file, uint32_t line) {
//    printf("Wrong parameters value: file %s on line %d\r\n", file, line);
    while (1)
        ;
}
#endif
