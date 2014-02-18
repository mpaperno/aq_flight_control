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

#ifndef _rcc_h
#define _rcc_h

#define RTC_MASK_YEAR	(0b1111111<<25)
#define RTC_MASK_MONTH	(0b1111<<21)
#define RTC_MASK_DAY	(0b11111<<16)
#define RTC_MASK_HOUR	(0b11111<<11)
#define RTC_MASK_MINUTE	(0b111111<<5)
#define RTC_MASK_SECOND (0b11111)

extern RCC_ClocksTypeDef rccClocks;

extern void rccConfiguration(void);

#endif
