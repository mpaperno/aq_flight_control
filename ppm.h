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

/*
    PPM module written by TC
    Heavily modified by Menno de Gans & Maxim Paperno
*/

#ifndef _ppm_h
#define _ppm_h

#include "pwm.h"
#include "radio.h"

#define PPM_MAX_CHANNELS		12	// can't be > RADIO_MAX_CHANNELS
#define PPM_GUARD_PULSE_LENGTH          2700
#define PPM_MIN_PULSE_WIDTH             750
#define PPM_MAX_PULSE_WIDTH             2250
#define PPM_STAB_CHANNEL_FRAMES         20      // number of consecutive frames with the same number of channels after which we assume that number of channels is stable ;)
#define PPM_CAPTURE_EDGE                1       // "polarity" value for passing to pwmInitIn(); -1 for falling edge, 1 for rising edge

#define PPM_THROT_MIN			-338	// final value constraints
#define PPM_THROT_MAX			1709
#define PPM_CHAN_MIN			-1024
#define PPM_CHAN_MAX			1023

typedef struct {
    radioInstance_t *r;
    pwmPortStruct_t *ppmPort;
    volatile uint8_t frameParsed;

    uint32_t lastCaptureValue;
    uint8_t  lastChannel;		    // index into channels[]
    uint8_t  previousChannels;		    // number of channels seen in previous frame;
					    // used to autodetermine number of channels
    uint8_t  numberChannels;		    // autodetermined number of channels or 0
    uint8_t  stableChannelsCount;	    // number of frames with the same number of channels
    int8_t   signalQuality;		    // -1 critical error (lost frame)
					    // 0 non-critical error (invalid pulse)
					    // 1 normal operation (frame parsed OK)
    uint8_t  inputValid;		    // 1 valid
					    // 0 current frame is invalid

    int16_t  channels[PPM_MAX_CHANNELS];    // channel values are stored here after successful capture of the whole frame
    int16_t tmp_channels[PPM_MAX_CHANNELS]; // temporary channel values while capturing the frame

} ppmStruct_t;

extern void ppmInit(radioInstance_t *r);
extern int  ppmDataAvailable(radioInstance_t *r);
extern int8_t ppmGetSignalQuality(radioInstance_t *r);

#endif
