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

#ifndef _analog_h
#define _analog_h

#include "aq.h"
#ifdef HAS_AIMU
    #include "adc.h"
#endif

#define ANALOG_CHANNELS		    7
#define ANALOG_SAMPLES		    32
#define ANALOG_SAMPLE_TIME	    ADC_SampleTime_480Cycles
#define ANALOG_DIVISOR		    ((double)ANALOG_REF_VOLTAGE / (double)4096.0 / (double)ANALOG_SAMPLES)

#define ANALOG_VIN_SLOPE	    ((ANALOG_VIN_RTOP + ANALOG_VIN_RBOT) / ANALOG_VIN_RBOT)
#ifdef ANALOG_EXT_VOLT_RTOP
#define ANALOG_EXT_VOLT_SLOPE	    ((ANALOG_EXT_VOLT_RTOP + ANALOG_EXT_VOLT_RBOT) / ANALOG_EXT_VOLT_RBOT)
#endif
#ifdef ANALOG_EXT_AMP_RTOP
#define ANALOG_EXT_AMP_SLOPE	    ((ANALOG_EXT_AMP_RTOP + ANALOG_EXT_AMP_RBOT) / ANALOG_EXT_AMP_RBOT)
#endif

#define ANALOG_VOLTS_VIN	    0
#define ANALOG_VOLTS_EXT_VOLT	    1
#define ANALOG_VOLTS_EXT_AMP	    2

typedef struct {
    uint16_t rawSamples[ANALOG_SAMPLES*ANALOG_CHANNELS];
    uint32_t rawChannels[ANALOG_CHANNELS];
    float voltages[ANALOG_CHANNELS];

    float vIn;
    float extVolt;
    float extAmp;

    int8_t batCellCount;

    uint8_t vInSourceIndex;
    float vInSlope;

} analogStruct_t;

extern analogStruct_t analogData;

extern void analogInit(void);
extern void analogDecode(void);

#endif
