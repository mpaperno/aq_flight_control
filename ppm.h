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
    Heavily modified by Menno de Gans
*/

#ifndef _ppm_h
#define _ppm_h

#include "radio.h"
#include "pwm.h"
#include "config.h"

#define PPM_PWM_CHANNEL		    13    // which PWM channel to use for PPM capture
#define PPM_MAX_CHANNELS	    12    // can't be > 18
#define PPM_GUARD_PULSE_LENGTH	    3500
#define PPM_MIN_PULSE_WIDTH	    750
#define PPM_MAX_PULSE_WIDTH	    2250
#define PPM_STAB_CHANNEL_FRAMES     20    // number of consequitive frames with the same number of channels after which we assume that number of channels is stable ;)

#define PPM_THROT           ppmData.channels[(int)p[RADIO_THRO_CH]]
#define PPM_ROLL            ppmData.channels[(int)p[RADIO_ROLL_CH]]
#define PPM_PITCH           ppmData.channels[(int)p[RADIO_PITC_CH]]
#define PPM_RUDD            ppmData.channels[(int)p[RADIO_RUDD_CH]]
#define PPM_GEAR            ppmData.channels[(int)p[RADIO_GEAR_CH]]
#define PPM_FLAPS           ppmData.channels[(int)p[RADIO_FLAP_CH]]
#define PPM_AUX2            ppmData.channels[(int)p[RADIO_AUX2_CH]]
#define PPM_AUX3            ppmData.channels[(int)p[RADIO_AUX3_CH]]
#define PPM_AUX4            ppmData.channels[(int)p[RADIO_AUX4_CH]]
#define PPM_AUX5            ppmData.channels[(int)p[RADIO_AUX5_CH]]
#define PPM_AUX6            ppmData.channels[(int)p[RADIO_AUX6_CH]]
#define PPM_AUX7            ppmData.channels[(int)p[RADIO_AUX7_CH]]

#define ppmLimitRange( v ) ( ( v < -1024 ) ? -1024 : ( ( v > 1023 ) ? 1023 : v ) )
#define ppmLimitRangeThrottle( v ) ( ( v < -338 ) ? -338 : ( ( v > 1709 ) ? 1709 : v ) )

typedef struct {
    pwmPortStruct_t *ppmPort;
    volatile int frameParsed;
    uint32_t lastCaptureValue;
    uint8_t lastChannel;		    // index into channels[]
    uint8_t previousChannels;		    // number of channels seen in previous frame;
					    // used to autodetermine number of channels
    uint8_t numberChannels;		    // autodetermined number of channels or 0
    uint8_t stableChannelsCount;	    // number of frames with the same number of channels
    uint8_t signalQuality;		    // -1 serious error
					    //  0 signal is there but not stable
					    //  1 signal Ok
    uint8_t inputValid;			    // 1 valid
					    // 0 current frame is invalid

    int16_t channels[PPM_MAX_CHANNELS];	    // channel values are stored here after successful
					    // capture of the whole frame
    int16_t tmp_channels[PPM_MAX_CHANNELS]; // temporary channel values while capturing the frame
} ppmStruct_t;

extern void ppmInit( void );
extern int  ppmDataAvailable( void );
extern int  ppmGetSignalQuality( void );

#endif
