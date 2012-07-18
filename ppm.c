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

#include "aq.h"
#include "radio.h"
#include "ppm.h"
#include "aq_timer.h"
#include <string.h>

ppmStruct_t ppmData __attribute__((section(".ccm")));

void ppmCallback(uint32_t capture, uint8_t bitstatus) {
    uint16_t diff;

    ppmData.lastCaptureValue = ppmData.currentCaptureValue;
    ppmData.currentCaptureValue = (uint16_t)capture;

    diff = ppmData.currentCaptureValue - ppmData.lastCaptureValue;

    if (diff > PPM_GUARD_PULSE_LENGTH) { // 2.7ms guard interval between frames
        ppmData.lastChannel = 0;
    }
    else {
        // PPM_MIN_PULSE_WIDTH to PPM_MAX_PULSE_WIDTH ms is our 'valid' channel range
        if (diff > PPM_MIN_PULSE_WIDTH && diff < PPM_MAX_PULSE_WIDTH && ppmData.lastChannel < PPM_MAX_CHANNELS) {
            ppmData.channels[ppmData.lastChannel] = diff;
            ppmData.channelParsed = 1;
        }
        ppmData.lastChannel++;
    }
}

void ppmInit(void) {
    memset((void *)&ppmData, 0, sizeof(ppmData));
    ppmData.ppmPort = pwmInitIn(PPM_PWM_CHANNEL, 1 /* rising polarity */, 0xFFFF, ppmCallback);
}

int ppmDataAvailable(void) {
    if (ppmData.channelParsed) {
	ppmData.channelParsed = 0;

	// Copy ppmData.channels[] to radioData.channels[]. seriosuly, this needs to be dynamic :(
	RADIO_THROT	= PPM_THROT;
	RADIO_ROLL 	= PPM_ROLL;
	RADIO_PITCH	= PPM_PITCH;
	RADIO_RUDD 	= PPM_RUDD;
	RADIO_GEAR 	= PPM_GEAR;
	RADIO_FLAPS	= PPM_FLAPS;
	RADIO_AUX2 	= PPM_AUX2;
	RADIO_AUX3 	= PPM_AUX3;
	RADIO_AUX4 	= PPM_AUX4;
	RADIO_AUX5 	= PPM_AUX5;
	RADIO_AUX6 	= PPM_AUX6;
	RADIO_AUX7 	= PPM_AUX7;

	radioData.lastUpdate = timerMicros();

	return 1;
    }
    else
	return 0;
}
