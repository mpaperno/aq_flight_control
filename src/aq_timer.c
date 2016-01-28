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
#include "aq_timer.h"
#include "rcc.h"
#include "rtc.h"

timerStruct_t timerData;

// Use TIMER_TIM to create a us system clock
void timerInit(void) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    TIM_OCInitTypeDef  TIM_OCInitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable the TIMER_TIM global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = TIMER_IRQ_CH;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    TIMER_EN;

	// stop timer when core halted (debug)
	DBGMCU_APB1PeriphConfig(TIMER_CORE_HALT, ENABLE);

    /* Time base configuration for 1MHz (us)*/
    TIM_TimeBaseStructure.TIM_Period = 0xFFFFFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = (TIMER_CLOCK / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIMER_TIM, &TIM_TimeBaseStructure);

    // reset
    TIM_SetCounter(TIMER_TIM, 0);

    timerCancelAlarm1();
    timerCancelAlarm2();
    timerCancelAlarm3();
    timerCancelAlarm4();

    // Output Compare for alarms
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Inactive;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;

    TIM_OC1Init(TIMER_TIM, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIMER_TIM, TIM_OCPreload_Disable);

    TIM_OC2Init(TIMER_TIM, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIMER_TIM, TIM_OCPreload_Disable);

    TIM_OC3Init(TIMER_TIM, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIMER_TIM, TIM_OCPreload_Disable);

    TIM_OC4Init(TIMER_TIM, &TIM_OCInitStructure);
    TIM_OC4PreloadConfig(TIMER_TIM, TIM_OCPreload_Disable);

    // go...
    TIM_Cmd(TIMER_TIM, ENABLE);
}

void timerCancelAlarm1(void) {
    TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC1;
    TIM_ClearITPendingBit(TIMER_TIM, TIM_IT_CC1);
}

void timerCancelAlarm2(void) {
    TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC2;
    TIM_ClearITPendingBit(TIMER_TIM, TIM_IT_CC2);
}

void timerCancelAlarm3(void) {
    TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC3;
    TIM_ClearITPendingBit(TIMER_TIM, TIM_IT_CC3);
}

void timerCancelAlarm4(void) {
    TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC4;
    TIM_ClearITPendingBit(TIMER_TIM, TIM_IT_CC4);
}

void timerSetAlarm1(int32_t us, timerCallback_t *callback, int parameter) {
    // schedule it
    timerData.alarm1Callback = callback;
    timerData.alarm1Parameter = parameter;

    TIMER_TIM->SR = (uint16_t)~TIM_IT_CC1;
    TIMER_TIM->CCR1 = TIMER_TIM->CNT + us;
    TIMER_TIM->DIER |= TIM_IT_CC1;
}

void timerSetAlarm2(int32_t us, timerCallback_t *callback, int parameter) {
    // schedule it
    timerData.alarm2Callback = callback;
    timerData.alarm2Parameter = parameter;

    TIMER_TIM->SR = (uint16_t)~TIM_IT_CC2;
    TIMER_TIM->CCR2 = TIMER_TIM->CNT + us;
    TIMER_TIM->DIER |= TIM_IT_CC2;
}

void timerSetAlarm3(int32_t us, timerCallback_t *callback, int parameter) {
    // schedule it
    timerData.alarm3Callback = callback;
    timerData.alarm3Parameter = parameter;

    TIMER_TIM->SR = (uint16_t)~TIM_IT_CC3;
    TIMER_TIM->CCR3 = TIMER_TIM->CNT + us;
    TIMER_TIM->DIER |= TIM_IT_CC3;
}

void timerSetAlarm4(int32_t us, timerCallback_t *callback, int parameter) {
    // schedule it
    timerData.alarm4Callback = callback;
    timerData.alarm4Parameter = parameter;

    TIMER_TIM->SR = (uint16_t)~TIM_IT_CC4;
    TIMER_TIM->CCR4 = TIMER_TIM->CNT + us;
    TIMER_TIM->DIER |= TIM_IT_CC4;
}

void timerSetAbsoluteAlarm1(int32_t us, timerCallback_t *callback, int parameter) {
    // schedule it
    timerData.alarm1Callback = callback;
    timerData.alarm1Parameter = parameter;

    TIMER_TIM->SR = (uint16_t)~TIM_IT_CC1;
    TIMER_TIM->CCR1 = us;
    TIMER_TIM->DIER |= TIM_IT_CC1;
}

void timerSetAbsoluteAlarm2(int32_t us, timerCallback_t *callback, int parameter) {
    // schedule it
    timerData.alarm2Callback = callback;
    timerData.alarm2Parameter = parameter;

    TIMER_TIM->SR = (uint16_t)~TIM_IT_CC2;
    TIMER_TIM->CCR2 = us;
    TIMER_TIM->DIER |= TIM_IT_CC2;
}

void timerSetAbsoluteAlarm3(int32_t us, timerCallback_t *callback, int parameter) {
    // schedule it
    timerData.alarm3Callback = callback;
    timerData.alarm3Parameter = parameter;

    TIMER_TIM->SR = (uint16_t)~TIM_IT_CC3;
    TIMER_TIM->CCR3 = us;
    TIMER_TIM->DIER |= TIM_IT_CC3;
}

void timerSetAbsoluteAlarm4(int32_t us, timerCallback_t *callback, int parameter) {
    // schedule it
    timerData.alarm4Callback = callback;
    timerData.alarm4Parameter = parameter;

    TIMER_TIM->SR = (uint16_t)~TIM_IT_CC4;
    TIMER_TIM->CCR4 = us;
    TIMER_TIM->DIER |= TIM_IT_CC4;
}

void TIMER_ISR(void) {
	uint16_t itEnable = TIMER_TIM->DIER;
    uint16_t itStatus = TIMER_TIM->SR;

    if ((itEnable & TIM_IT_CC1) != RESET && (itStatus & TIM_IT_CC1) != RESET) {
		TIMER_TIM->SR = (uint16_t)~TIM_IT_CC1;

		// Disable the Interrupt
		TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC1;

		timerData.alarm1Callback(timerData.alarm1Parameter);
    }
    else if ((itEnable & TIM_IT_CC2) != RESET && (itStatus & TIM_IT_CC2) != RESET) {
		TIMER_TIM->SR = (uint16_t)~TIM_IT_CC2;

		// Disable the Interrupt
		TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC2;

		timerData.alarm2Callback(timerData.alarm2Parameter);
    }
    else if ((itEnable & TIM_IT_CC3) != RESET && (itStatus & TIM_IT_CC3) != RESET) {
		TIMER_TIM->SR = (uint16_t)~TIM_IT_CC3;

		// Disable the Interrupt
		TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC3;

		timerData.alarm3Callback(timerData.alarm3Parameter);
	}
    // CC4 is used for RTC calibration at startup
    else if ((itEnable & TIM_IT_CC4) != RESET && (itStatus & TIM_IT_CC4) != RESET) {
		TIMER_TIM->SR = (uint16_t)~TIM_IT_CC4;

        if (timerData.alarm4Callback) {
			// Disable the Interrupt
			TIMER_TIM->DIER &= (uint16_t)~TIM_IT_CC4;

			timerData.alarm4Callback(timerData.alarm4Parameter);
		}
        else {
            // Get the Input Capture value
            rtcData.captureLSI[rtcData.captureNumber++] = TIM_GetCapture4(TIMER_TIM);
		}
    }
}
