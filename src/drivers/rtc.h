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

#ifndef _rtc_h
#define _rtc_h

#ifndef RTC_INIT_MASK
    #define RTC_INIT_MASK	   ((uint32_t)0xFFFFFFFF)
#endif

#ifndef RTC_TR_RESERVED_MASK
    #define RTC_TR_RESERVED_MASK    ((uint32_t)0x007F7F7F)
#endif
#ifndef RTC_DR_RESERVED_MASK
    #define RTC_DR_RESERVED_MASK    ((uint32_t)0x00FFFF3F)
#endif

typedef struct {
    unsigned long lsiFrequency;
    uint32_t asyncPrediv, syncPrediv;
    volatile unsigned long captureLSI[2];
    volatile int captureNumber;
} rtcStruct_t;

extern rtcStruct_t rtcData;

extern void rtcInit(void);
extern unsigned long rtcGetDateTime(void);
extern int rtcSetDataTime(int year, int month, int day, int hour, int minute, int second);
extern void rtcSetDateTimeLong(unsigned long dateTime);

#endif
