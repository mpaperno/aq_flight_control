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

#ifndef radio_h
#define radio_h

#include "serial.h"
#include "spektrum.h"
#include "futaba.h"
#include "ppm.h"
#include "util.h"

#define RADIO_STACK_SIZE	64
#define RADIO_PRIORITY		25

#define RADIO_THROT             radioData.channels[(int)p[RADIO_THRO_CH]]
#define RADIO_ROLL		radioData.channels[(int)p[RADIO_ROLL_CH]]
#define RADIO_PITCH		radioData.channels[(int)p[RADIO_PITC_CH]]
#define RADIO_RUDD		radioData.channels[(int)p[RADIO_RUDD_CH]]
#define RADIO_GEAR		radioData.channels[(int)p[RADIO_GEAR_CH]]
#define RADIO_FLAPS		radioData.channels[(int)p[RADIO_FLAP_CH]]
#define RADIO_AUX2		radioData.channels[(int)p[RADIO_AUX2_CH]]
#define RADIO_AUX3		radioData.channels[(int)p[RADIO_AUX3_CH]]
#define RADIO_AUX4		radioData.channels[(int)p[RADIO_AUX4_CH]]
#define RADIO_AUX5		radioData.channels[(int)p[RADIO_AUX5_CH]]
#define RADIO_AUX6		radioData.channels[(int)p[RADIO_AUX6_CH]]
#define RADIO_AUX7		radioData.channels[(int)p[RADIO_AUX7_CH]]

#define RADIO_FRAME_COUNT       radioData.frameCount
#define RADIO_ERROR_COUNT       radioData.errorCount
#define RADIO_QUALITY           radioData.quality

typedef struct {
    OS_TID radioTask;

    serialPort_t *serialPort;

    int16_t channels[18];

    utilFilter_t qualityFilter;
    unsigned int errorCount;
    unsigned int frameCount;
    float quality;

    unsigned long lastUpdate;
} radioStruct_t;

extern radioStruct_t radioData;

extern void radioInit(void);

#endif
