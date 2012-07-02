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
#include "util.h"

#define RADIO_THROT		radioData.channels[0]
#define RADIO_ROLL		radioData.channels[1]
#define RADIO_PITCH		radioData.channels[2]
#define RADIO_RUDD		radioData.channels[3]
#define RADIO_GEAR		radioData.channels[4]
#define RADIO_FLAPS		radioData.channels[5]
#define RADIO_AUX2		radioData.channels[6]
#define RADIO_AUX3		radioData.channels[7]
#define RADIO_AUX4		radioData.channels[8]
#define RADIO_AUX5		radioData.channels[9]
#define RADIO_AUX6		radioData.channels[10]
#define RADIO_AUX7		radioData.channels[11]

#define RADIO_FRAME_COUNT   radioData.frameCount
#define RADIO_ERROR_COUNT   radioData.errorCount
#define RADIO_QUALITY	    radioData.quality

typedef struct {
    OS_TID radioTask;

    serialPort_t *serialPort;

    int channels[18];

    utilFilter_t qualityFilter;
    unsigned int errorCount;
    unsigned int frameCount;
    float quality;

    unsigned long lastUpdate;
} radioStruct_t;

extern radioStruct_t radioData;

extern void radioInit(void);

#endif
