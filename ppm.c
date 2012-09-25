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
    // Calculate time difference, that is - period in us
    if( capture < ppmData.lastCaptureValue ) {
        // This could be only if timer counter reached it maximum
        // value and started counting from 0. Handle that.
        // (Please note that several consecutive overflows will be
        // treated as a single one - which will of course affect
        // measured time. On the other hand if there was no pulse within
        // 65000 us, something is wrong already. To handle this situation
        // more carefully, we might also ask the timer to generate
        // interrupts on each overflow so that we know how many
        // consecutive overflows there were).
        diff = (uint16_t)(0x010000 + capture - ppmData.lastCaptureValue);
    } else {
        diff = (uint16_t)(capture - ppmData.lastCaptureValue);
    }

    // Store current time value for future reference
    ppmData.lastCaptureValue = capture;
    // Now parse CPPM. The code below is a bit oversimplified.
    if (diff > PPM_GUARD_PULSE_LENGTH) { // New frame started
        // Try to autodetermine number of channels
        // If we have seen enough frames with the same number of channels,
        // store this number of channels in number_of_channels
        if( ppmData.numberChannels == 0 ) {
            // We are still determining number of channels
            ppmData.signalQuality = 0; // "signal not stable enough" ;)
            if( ppmData.lastChannel <= PPM_MAX_CHANNELS
                && ppmData.lastChannel == ppmData.previousChannels ) {

                if( ppmData.stableChannelsCount >= PPM_STAB_CHANNEL_FRAMES ) {
                    ppmData.numberChannels = ppmData.lastChannel;
                    ppmData.signalQuality  = 1;
                } else {
                    (ppmData.stableChannelsCount)++;
                }
            } else {
                ppmData.stableChannelsCount = 0;
            }
        } else {
            // Number of channels is already known. So verify that there were
            // no errors during last frame capture and store channel values
            if( ppmData.inputValid ) {
                if( ppmData.lastChannel != ppmData.numberChannels ) { // Wrong number of channels
                    ppmData.inputValid    = 0;
                    ppmData.frameParsed   = 0;
                    ppmData.signalQuality = 0;
                } else {
                    ppmData.signalQuality = 1;
                    ppmData.frameParsed = 1;
                    memcpy( ppmData.channels, ppmData.tmp_channels, sizeof(ppmData.channels) );
	            radioData.lastUpdate = timerMicros();
                }
            }
        }
        ppmData.previousChannels = ppmData.lastChannel;
        ppmData.lastChannel = 0;
        ppmData.inputValid  = 1;
    } else if( ppmData.inputValid ) { // We are inside the frame and no errors found this far in the frame
        if( ppmData.lastChannel >= PPM_MAX_CHANNELS ) {    // Too many channels
            ppmData.inputValid    = 0;
            ppmData.signalQuality = 0;
        } else if (diff > PPM_MIN_PULSE_WIDTH && diff < PPM_MAX_PULSE_WIDTH ) {
            ppmData.tmp_channels[(ppmData.lastChannel)++] = diff;
        } else {
            ppmData.inputValid    = 0;
            ppmData.signalQuality = 0;
            ppmData.channels[(ppmData.lastChannel)++] = 0;
            }
        }
}

void ppmInit(void) {
    memset((void *)&ppmData, 0, sizeof(ppmData));
    ppmData.ppmPort = pwmInitIn(PPM_PWM_CHANNEL, 1 /* rising polarity */, 0x10000, ppmCallback);
}


#define ppmLimitRange( v ) ( ( v < -1024 ) ? -1024 : ( ( v > 1023 ) ? 1023 : v ) )
#define ppmLimitRangeThrottle( v ) ( ( v < -338 ) ? -338 : ( ( v > 1709 ) ? 1709 : v ) )

int ppmDataAvailable(void) {

    if (ppmData.frameParsed) {
	ppmData.frameParsed = 0;

	RADIO_THROT	= ppmLimitRangeThrottle( (PPM_THROT-p[PPM_THROT_LOW])*5/p[PPM_SCALER] );
	RADIO_ROLL 	= ppmLimitRange( (PPM_ROLL-p[PPM_CHAN_MID])*5/p[PPM_SCALER] );
	RADIO_PITCH	= ppmLimitRange( (PPM_PITCH-p[PPM_CHAN_MID])*5/p[PPM_SCALER] );
	RADIO_RUDD 	= ppmLimitRange( (PPM_RUDD-p[PPM_CHAN_MID])*5/p[PPM_SCALER] );
	RADIO_GEAR 	= ppmLimitRange( (PPM_GEAR-p[PPM_CHAN_MID])*5/p[PPM_SCALER] );
	RADIO_FLAPS	= ppmLimitRange( (PPM_FLAPS-p[PPM_CHAN_MID])*5/p[PPM_SCALER] );
	RADIO_AUX2 	= ppmLimitRange( (PPM_AUX2-p[PPM_CHAN_MID])*5/p[PPM_SCALER] );
	RADIO_AUX3 	= ppmLimitRange( (PPM_AUX3-p[PPM_CHAN_MID])*5/p[PPM_SCALER] );
	RADIO_AUX4 	= ppmLimitRange( (PPM_AUX4-p[PPM_CHAN_MID])*5/p[PPM_SCALER] );
	RADIO_AUX5 	= ppmLimitRange( (PPM_AUX5-p[PPM_CHAN_MID])*5/p[PPM_SCALER] );
	RADIO_AUX6 	= ppmLimitRange( (PPM_AUX6-p[PPM_CHAN_MID])*5/p[PPM_SCALER] );
	RADIO_AUX7 	= ppmLimitRange( (PPM_AUX7-p[PPM_CHAN_MID])*5/p[PPM_SCALER] );

        return 1;
    } else {
        return 0;
    }
}

int ppmGetSignalQuality(void) {
    return ppmData.signalQuality;
}
