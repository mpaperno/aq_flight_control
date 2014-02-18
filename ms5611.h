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

#ifndef _ms5611_h
#define _ms5611_h

#include "spi.h"
#include "util.h"


#define MS5611_SPI_BAUD		    SPI_BaudRatePrescaler_4	// 10.5Mhz

#define MS5611_SLOTS		    8				// ~13 Hz
#define MS5611_RETRIES              5

typedef struct {
    utilFilter_t tempFilter;
    spiClient_t *spi;
    volatile uint32_t spiFlag;
    volatile uint32_t d1[MS5611_SLOTS];
    volatile uint32_t d2[MS5611_SLOTS];
    uint16_t p[8];
    volatile uint8_t slot;
    uint8_t step;
    uint8_t enabled;
    uint8_t startTempConv;
    uint8_t startPresConv;
    uint8_t adcRead;
    uint8_t initialized;
    float rawTemp;
    volatile float temp;
    volatile float pres;
    volatile uint32_t lastUpdate;
} ms5611Struct_t;

extern ms5611Struct_t ms5611Data;

extern void ms5611PreInit(void);
extern uint8_t ms5611Init(void);
extern void ms5611InitialBias(void);
extern void ms5611Result(int unused);
extern void ms5611Conversion(int unused);
extern void ms5611Decode(void);
extern void ms5611Enable(void);
extern void ms5611Disable(void);
extern void ms5611Enable(void);
extern void ms5611Disable(void);

#endif
