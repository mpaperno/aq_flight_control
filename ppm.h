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

    Copyright © 2012  Bill Nesbitt
*/

/*
    PPM module written by TC
*/

#ifndef _ppm_h
#define _ppm_h

#include "pwm.h"

#define PPM_PWM_CHANNEL		    13	// which PWM channel to use for PPM capture
#define PPM_MAX_CHANNELS	    12	// can't be > 18
#define PPM_GUARD_PULSE_LENGTH	    2700
#define PPM_MIN_PULSE_WIDTH	    750
#define PPM_MAX_PULSE_WIDTH	    2250

#define PPM_THROT		    ppmData.channels[0]
#define PPM_ROLL		    ppmData.channels[1]
#define PPM_PITCH		    ppmData.channels[2]
#define PPM_RUDD		    ppmData.channels[3]
#define PPM_GEAR		    ppmData.channels[4]
#define PPM_FLAPS		    ppmData.channels[5]
#define PPM_AUX2		    ppmData.channels[6]
#define PPM_AUX3		    ppmData.channels[7]
#define PPM_AUX4		    ppmData.channels[8]
#define PPM_AUX5		    ppmData.channels[9]
#define PPM_AUX6		    ppmData.channels[10]
#define PPM_AUX7		    ppmData.channels[11]

typedef struct {
    pwmPortStruct_t *ppmPort;
    volatile int channelParsed;
    uint16_t currentCaptureValue;
    uint16_t lastCaptureValue;
    uint8_t lastChannel;

    int16_t channels[PPM_MAX_CHANNELS];
} ppmStruct_t;

extern void ppmInit(void);
extern int ppmDataAvailable(void);

#endif
