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

#include "aq.h"
#include "aq_timer.h"
#include "rcc.h"

timerStruct_t timerData;

// Use TIMER_TIM to create a us system clock
void timerInit(void) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    TIMER_EN;

    /* Time base configuration for 1MHz (us)*/
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = (TIMER_CLOCK / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMER_TIM, &TIM_TimeBaseStructure);

    // reset
    TIM_SetCounter(TIMER_TIM, 0);

    // go...
    TIM_Cmd(TIMER_TIM, ENABLE);
}

void timerStart(void) {
    timerData.timerStart = timerMicros();
}

uint32_t timerStop(void) {
    return timerMicros() - timerData.timerStart;
}

