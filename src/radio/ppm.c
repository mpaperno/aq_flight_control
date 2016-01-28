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
    Heavily modified by Menno de Gans
*/

#include "aq.h"
#include "radio.h"
#include "ppm.h"
#include "aq_timer.h"
#include "config.h"
#include "util.h"
#include <string.h>

ppmStruct_t ppmData __attribute__((section(".ccm")));

void ppmCallback(uint32_t capture, uint8_t bitstatus) {
    uint16_t diff;

    // Calculate time difference, that is - period in us
    if (capture < ppmData.lastCaptureValue)
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
    else
        diff = (uint16_t)(capture - ppmData.lastCaptureValue);

    // Store current time value for future reference
    ppmData.lastCaptureValue = capture;

    // Now parse CPPM. The code below is a bit oversimplified.
    if (diff > PPM_GUARD_PULSE_LENGTH) { // Frame separator detected
        // Try to autodetermine number of channels
        // If we have seen enough frames with the same number of channels,
        // store this number of channels in number_of_channels
        if (ppmData.numberChannels == 0) {
            // We are still determining number of channels, "signal not stable"
            ppmData.signalQuality = -1;

            if (ppmData.lastChannel <= PPM_MAX_CHANNELS
                && ppmData.lastChannel == ppmData.previousChannels) {

                if (ppmData.stableChannelsCount >= PPM_STAB_CHANNEL_FRAMES) {
                    ppmData.numberChannels = ppmData.lastChannel;
                }
                else {
                    ppmData.stableChannelsCount++;
                }
            }
            else {
                ppmData.stableChannelsCount = 0;
            }
        }
        // Number of channels is already known. So verify that there were
        // no errors during last frame capture and store channel values
        else {
            if (ppmData.lastChannel != ppmData.numberChannels) // Wrong number of channels
                ppmData.inputValid = 0;

            // valid frame, pass values to ppmData.channels
            if (ppmData.inputValid) {
		memcpy(ppmData.channels, ppmData.tmp_channels, sizeof(ppmData.channels));
		ppmData.frameParsed = 1;
	        ppmData.signalQuality = 1;  // normal operation
            }
            // handle invalid frame
            else {
                ppmData.signalQuality = -1; // critical error
                ppmData.r->errorCount++;     // increment shared cumulative error counter
            }
        }

        ppmData.previousChannels = ppmData.lastChannel;
        ppmData.lastChannel = 0;
        ppmData.inputValid  = 1;

    } // end handling of frame separator

    else if (ppmData.inputValid) { // We are inside the frame and no errors found this far in the frame

	// Too many channels, invalidate the whole frame
        if (ppmData.lastChannel >= PPM_MAX_CHANNELS)
            ppmData.inputValid = 0;

        // Pulse length is valid, count it as the next channel value
        else if (diff >= PPM_MIN_PULSE_WIDTH && diff <= PPM_MAX_PULSE_WIDTH)
            ppmData.tmp_channels[(ppmData.lastChannel)++] = diff;

        // Invalid pulse invalidates the whole frame.
        // (also tried ignoring them because "If pulse was actually a channel, this will be caught later when the frame is validated."
        //  but in practice this seems to cause random channel value drops. )
        else
            ppmData.inputValid = 0;
            //ppmData.signalQuality = 0; // non-critical error

    }
}

void ppmInit(radioInstance_t *r) {
    memset((void *)&ppmData, 0, sizeof(ppmData));

    ppmData.r = r;
    ppmData.ppmPort = pwmInitIn(PPM_PWM_CHANNEL, PPM_CAPTURE_EDGE, 0x10000, ppmCallback);
}

int ppmDataAvailable(radioInstance_t *r) {
    int i;

    if (ppmData.frameParsed) {
        ppmData.frameParsed = 0;

        for (i = 0; i < ppmData.numberChannels; ++i) {
            if (&RADIO_THROT == &r->channels[i])
                r->channels[i] = constrainInt((int)((ppmData.channels[i] - p[PPM_THROT_LOW])*5 / p[PPM_SCALER]), PPM_THROT_MIN, PPM_THROT_MAX);
            else
        	r->channels[i] = constrainInt((int)((ppmData.channels[i] - p[PPM_CHAN_MID])*5 / p[PPM_SCALER]), PPM_CHAN_MIN, PPM_CHAN_MAX);
        }

        return 1;
    }
    else {
        return 0;
    }
}

int8_t ppmGetSignalQuality(radioInstance_t *r) {
    return ppmData.signalQuality;
}

