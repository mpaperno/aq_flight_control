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

#ifndef _radio_h
#define _radio_h

#include "serial.h"
#include "digital.h"
#include "util.h"

#define RADIO_STACK_SIZE        120
#define RADIO_PRIORITY          25

#define RADIO_NUM               3       // max number of RC radios
#define RADIO_MAX_CHANNELS      18
#define RADIO_UPDATE_TIMEOUT    50000	// maximum time in micros between valid radio updates before signal is considered unstable;
#define RADIO_LAST_UPDATE       (*radioData.lastUpdate)
#define RADIO_BINDING           (*radioData.binding)
#define RADIO_INITIALIZED       (radioData.lastUpdate)
#define RADIO_VALID             ((timerMicros() - RADIO_LAST_UPDATE) < RADIO_UPDATE_TIMEOUT)

#define RADIO_THROT		radioData.channels[(int)p[RADIO_THRO_CH]]
#define RADIO_ROLL		radioData.channels[(int)p[RADIO_ROLL_CH]]
#define RADIO_PITCH		radioData.channels[(int)p[RADIO_PITC_CH]]
#define RADIO_RUDD		radioData.channels[(int)p[RADIO_RUDD_CH]]
#define RADIO_GEAR		radioData.channels[(int)p[RADIO_GEAR_CH]]	// camera trigger
#define RADIO_FLAPS		radioData.channels[(int)p[RADIO_FLAP_CH]]	// flight mode
#define RADIO_AUX2		radioData.channels[(int)p[RADIO_AUX2_CH]]	// home functions
#define RADIO_AUX3		radioData.channels[(int)p[RADIO_AUX3_CH]]	// gimbal tilt
#define RADIO_AUX4		radioData.channels[(int)p[RADIO_AUX4_CH]]
#define RADIO_AUX5		radioData.channels[(int)p[RADIO_AUX5_CH]]
#define RADIO_AUX6		radioData.channels[(int)p[RADIO_AUX6_CH]]
#define RADIO_AUX7		radioData.channels[(int)p[RADIO_AUX7_CH]]

#define RADIO_MID_THROTTLE	700

#define RADIO_ERROR_COUNT       (*radioData.errorCount)
#define RADIO_QUALITY           (*radioData.quality)

enum radioTypes {
    RADIO_TYPE_NONE = 0,
    RADIO_TYPE_SPEKTRUM11,
    RADIO_TYPE_SPEKTRUM10,
    RADIO_TYPE_SBUS,
    RADIO_TYPE_PPM,
    RADIO_TYPE_SUMD,
    RADIO_TYPE_MLINK,
    RADIO_TYPE_DELTANG,
    RADIO_TYPE_CYRF6936
};

enum radioModes {
    RADIO_MODE_DIVERSITY = 0,
    RADIO_MODE_SPLIT
};

typedef struct {
    uint32_t lastUpdate;            // time of last valid data received from radio handler
    serialPort_t *serialPort;
    utilFilter_t qualityFilter;
    uint32_t errorCount;            // cumulative error/lost frame counter
    float quality;                  // running measure of radio data stability, in range of 0-100
    int16_t *channels;
    uint8_t radioType;
    uint8_t binding;
} radioInstance_t;

typedef struct {
    OS_TID radioTask;

    digitalPin *select[2];
    radioInstance_t radioInstances[RADIO_NUM];
    int16_t allChannels[RADIO_MAX_CHANNELS*RADIO_NUM];	// holds channel values received from radio handlers

    int16_t *channels;
    float *quality;
    uint32_t *errorCount;
    uint32_t *lastUpdate;
    uint8_t *binding;

    uint8_t mode;
} radioStruct_t;

extern radioStruct_t radioData;

extern void radioInit(void);

#endif

