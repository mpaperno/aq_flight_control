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
#include "rcc.h"
#include "rtc.h"
#include "aq_timer.h"
#include "comm.h"
#include <stdio.h>
#include <string.h>

rtcStruct_t rtcData __attribute__((section(".ccm")));

// Sakamoto's algorithm (modified)
// return Day of Week, 1 == Monday, 7 == Sunday
int rtcDow(int y, int m, int d) {
    static int t[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};

    y -= m < 3;
    return ((y + y/4 - y/100 + y/400 + t[m-1] + d - 1) % 7) + 1;
}

uint8_t rtcByteToBcd2(uint8_t Value) {
    uint8_t bcdhigh = 0;

    while (Value >= 10) {
	bcdhigh++;
	Value -= 10;
    }

    return  ((uint8_t)(bcdhigh << 4) | Value);
}

int rtcSetDataTime(int year, int month, int day, int hour, int minute, int second) {
    // if RTC is not in Initialization mode
    if ((RTC->ISR & RTC_ISR_INITF) == (uint32_t)RESET) {
	RTC_WriteProtectionCmd(DISABLE);

	// set init mode (which takes a while)
	RTC->ISR = (uint32_t)RTC_INIT_MASK;

	// don't wait around, instead return failure this time
	return 0;
    }
    // otherwise set the clock
    else {
	RTC_TimeTypeDef time;
	RTC_DateTypeDef date;
	uint32_t tmpreg;

	date.RTC_Year = year - 2000;
	date.RTC_Month = month;
	date.RTC_Date = day;
	date.RTC_WeekDay = rtcDow(2000+date.RTC_Year, date.RTC_Month, date.RTC_Date);
//	RTC_SetDate(RTC_Format_BIN, &date);

	tmpreg = (((uint32_t)rtcByteToBcd2(date.RTC_Year) << 16) | \
	    ((uint32_t)rtcByteToBcd2(date.RTC_Month) << 8) | \
	    ((uint32_t)rtcByteToBcd2(date.RTC_Date)) | \
	    ((uint32_t)date.RTC_WeekDay << 13));

	RTC->DR = (uint32_t)(tmpreg & RTC_DR_RESERVED_MASK);

	time.RTC_Hours = hour;
	time.RTC_Minutes = minute;
	time.RTC_Seconds = second;
	time.RTC_H12 = 0x00;
//	RTC_SetTime(RTC_Format_BIN, &time);

	tmpreg = (uint32_t)(((uint32_t)rtcByteToBcd2(time.RTC_Hours) << 16) | \
	    ((uint32_t)rtcByteToBcd2(time.RTC_Minutes) << 8) | \
	    ((uint32_t)rtcByteToBcd2(time.RTC_Seconds)) | \
	    (((uint32_t)time.RTC_H12) << 16));

	RTC->TR = (uint32_t)(tmpreg & RTC_TR_RESERVED_MASK);

	// allow clock to run, don't wait for synchronization
	RTC_ExitInitMode();
	RTC_WriteProtectionCmd(ENABLE);

	AQ_PRINTF("RTC set: %d-%02d-%02d %02d:%02d:%02d UTC\n", date.RTC_Year+2000, date.RTC_Month, date.RTC_Date, time.RTC_Hours, time.RTC_Minutes, time.RTC_Seconds);

	// return success
	return 1;
    }
}

void rtcSetDateTimeLong(unsigned long dateTime) {
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    date.RTC_Year = (1980 + ((dateTime & RTC_MASK_YEAR)>>25)) - 2000;
    date.RTC_Month = (dateTime & RTC_MASK_MONTH)>>21;
    date.RTC_Date = (dateTime & RTC_MASK_DAY)>>16;
    date.RTC_WeekDay = rtcDow(2000+date.RTC_Year, date.RTC_Month, date.RTC_Date);
    RTC_SetDate(RTC_Format_BIN, &date);

    time.RTC_Hours = (dateTime & RTC_MASK_HOUR)>>11;
    time.RTC_Minutes = (dateTime & RTC_MASK_MINUTE)>>5;
    time.RTC_Seconds = (dateTime & RTC_MASK_SECOND)<<1;
    RTC_SetTime(RTC_Format_BIN, &time);
}

unsigned long rtcGetDateTime(void) {
    RTC_TimeTypeDef time;
    RTC_DateTypeDef date;

    RTC_GetDate(RTC_Format_BIN, &date);
    RTC_GetTime(RTC_Format_BIN, &time);

    return ((unsigned long)date.RTC_Year + 2000 - 1980)<<25 |
	((unsigned long)date.RTC_Month)<<21 |
	((unsigned long)date.RTC_Date)<<16 |
	((unsigned long)time.RTC_Hours)<<11 |
	((unsigned long)time.RTC_Minutes)<<5 |
	((unsigned long)time.RTC_Seconds)>>1;
}

void rtcInit(void) {
    RTC_InitTypeDef RTC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    TIM_ICInitTypeDef  TIM_ICInitStructure;
    unsigned long periodValue;

    memset((void *)&rtcData, 0, sizeof(rtcData));

    // Enable the PWR clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    // Allow access to RTC
    PWR_BackupAccessCmd(ENABLE);

    RCC_LSICmd(ENABLE);

    // Wait till LSI is ready
    while (RCC_GetFlagStatus(RCC_FLAG_LSIRDY) == RESET)
	;

    // Select the RTC Clock Source
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);

    RCC_RTCCLKCmd(ENABLE);

    // Wait for RTC APB registers synchronization
    RTC_WaitForSynchro();

    // EXTI configuration
    EXTI_ClearITPendingBit(EXTI_Line22);
    EXTI_InitStructure.EXTI_Line = EXTI_Line22;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    // Enable the RTC Wakeup Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = RTC_WKUP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    RTC_WakeUpCmd(DISABLE);

    // Configure the RTC WakeUp Clock source: CK_SPRE (1Hz)
    RTC_WakeUpClockConfig(RTC_WakeUpClock_CK_SPRE_16bits);
    RTC_SetWakeUpCounter(0x0);

    // Enable Wakeup Counter
    RTC_WakeUpCmd(ENABLE);

    // Enable the RTC Wakeup Interrupt
    RTC_ClearITPendingBit(RTC_IT_WUT);
    RTC_ITConfig(RTC_IT_WUT, ENABLE);

    // TIM5 configuration: Input Capture mode
    // The LSI oscillator is connected to TIM5 CH4
    // The Rising edge is used as active edge,
    // The TIM5 CCR4 is used to compute the frequency value

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);

    // Connect internally the TIM5_CH4 Input Capture to the LSI clock output
    TIM_RemapConfig(TIM5, TIM5_LSI);

    // Configure TIM5 presclaer
    TIM_PrescalerConfig(TIM5, 0, TIM_PSCReloadMode_Immediate);

    TIM_ICInitStructure.TIM_Channel = TIM_Channel_4;
    TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV8;
    TIM_ICInitStructure.TIM_ICFilter = 0;
    TIM_ICInit(TIM5, &TIM_ICInitStructure);

    // Enable TIM5 Interrupt channel
    NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // go...
    TIM_Cmd(TIM5, ENABLE);

    // Reset the flags
    TIM5->SR = 0;

    // Enable the CC4 Interrupt Request
    TIM_ITConfig(TIM5, TIM_IT_CC4, ENABLE);

    // wait
    while (rtcData.captureNumber < 2)
	;

    TIM_DeInit(TIM5);

    // Compute the period length
    periodValue = (uint16_t)(0xFFFF - rtcData.captureLSI[0] + rtcData.captureLSI[1] + 1);

    // Get PCLK1 prescaler
    rtcData.lsiFrequency = (((2 * rccClocks.PCLK1_Frequency / 1000000 * AQ_US_PER_SEC) / periodValue) * 8);

    rtcData.asyncPrediv = 4;	// low as possible for max resolution
    rtcData.syncPrediv = (rtcData.lsiFrequency / (rtcData.asyncPrediv + 1)) - 1;

    // Calendar Configuration with measured LSI frequency
    RTC_InitStructure.RTC_AsynchPrediv = rtcData.asyncPrediv;
    RTC_InitStructure.RTC_SynchPrediv = rtcData.syncPrediv;
    RTC_InitStructure.RTC_HourFormat = RTC_HourFormat_24;
    RTC_Init(&RTC_InitStructure);

    RTC_BypassShadowCmd(ENABLE);
    RTC_WriteProtectionCmd(DISABLE);
    // leave RTC in Initialization mode, ready to be set by the first GPS time update
    RTC_EnterInitMode();
}

void RTC_WKUP_IRQHandler(void) {
    if (RTC_GetITStatus(RTC_IT_WUT) != RESET) {
	// nothing at the moment

	RTC_ClearITPendingBit(RTC_IT_WUT);
	EXTI_ClearITPendingBit(EXTI_Line22);
    }
}

// only used for calibration at startup
//void TIM5_IRQHandler(void) {
//    if (TIM_GetITStatus(TIM5, TIM_IT_CC4) != RESET) {
//	// Get the Input Capture value
//	rtcData.captureLSI[rtcData.captureNumber++] = TIM_GetCapture4(TIM5);
//
//	// Clear CC4 Interrupt pending bit
//	TIM_ClearITPendingBit(TIM5, TIM_IT_CC4);
//    }
//}
