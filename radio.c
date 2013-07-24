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

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#include "aq.h"
#include "config.h"
#include "radio.h"
#include "util.h"
#include "aq_timer.h"
#include "comm.h"
#include <string.h>

radioStruct_t radioData __attribute__((section(".ccm")));

OS_STK *radioTaskStack;

// calculate radio reception quality
#pragma GCC optimize ("-fno-inline")
static void radioReceptionQuality(int8_t q) {
    radioData.quality = utilFilter(&radioData.qualityFilter, (float)(q + 1)) * 0.5f * 100.0f;
}

void radioTaskCode(void *unused) {
    serialPort_t *s = radioData.serialPort;
    int8_t q;

    AQ_NOTICE("Radio task started\n");

    while (1) {
	// wait for data
	yield(2); // 2ms

	switch (radioData.radioType) {
	case RADIO_TYPE_SPEKTRUM11:
	case RADIO_TYPE_SPEKTRUM10:
	    while (serialAvailable(s))
		if (spektrumCharIn(serialRead(s))) {
		    radioData.lastUpdate = timerMicros();
		    radioReceptionQuality(1);
		}
	    break;

	case RADIO_TYPE_SBUS:
	    while (serialAvailable(s))
		if ((q = futabaCharIn(serialRead(s)))) {
		    radioData.lastUpdate = timerMicros();
		    radioReceptionQuality(q);
		}
	    break;

	case RADIO_TYPE_PPM:
	    if (ppmDataAvailable())
                radioData.lastUpdate = timerMicros();
            break;

	case RADIO_TYPE_SUMD:
	    while (serialAvailable(s))
		if ((q = grhottCharIn(serialRead(s)))) {
		    radioData.lastUpdate = timerMicros();
		    radioReceptionQuality(q);
		}
	    break;
	}

	// no radio?
	if (timerMicros() - radioData.lastUpdate > RADIO_UPDATE_TIMEOUT)
	    radioReceptionQuality(-1);  // minimum signal quality (0%) if no updates within timeout value
	else if (radioData.radioType == RADIO_TYPE_PPM)
            radioReceptionQuality(ppmGetSignalQuality());  // signal quality based on PPM status
    }
}

void radioRCSelect(uint8_t level) {
#ifdef RADIO_RC1_SELECT_PORT
    radioData.select[0] = digitalInit(RADIO_RC1_SELECT_PORT, RADIO_RC1_SELECT_PIN);
    if (level) {
	digitalHi(radioData.select[0]);
    }
    else {
	digitalLo(radioData.select[0]);
    }
#endif

#ifdef RADIO_RC2_SELECT_PORT
    radioData.select[1] = digitalInit(RADIO_RC2_SELECT_PORT, RADIO_RC2_SELECT_PIN);
    if (level) {
	digitalHi(radioData.select[1]);
    }
    else {
	digitalLo(radioData.select[1]);
    }
#endif
}

void radioInit(void) {
    AQ_NOTICE("Radio init\n");

    memset((void *)&radioData, 0, sizeof(radioData));

    utilFilterInit(&radioData.qualityFilter, (1.0f / 50.0f), 0.75f, 0.0f);

    radioData.radioType = (int8_t)p[RADIO_TYPE];

    for (int i=0; i < RADIO_MAX_CHANNELS; ++i)
	radioData.channels[i] = (i == (int)p[RADIO_FLAP_CH]) ? -700 : 0;

    switch (radioData.radioType) {
    case RADIO_TYPE_SPEKTRUM11:
    case RADIO_TYPE_SPEKTRUM10:
	spektrumInit();
	radioRCSelect(0);
	break;

    case RADIO_TYPE_SBUS:
	futabaInit();
	radioRCSelect(1);
	break;

    case RADIO_TYPE_PPM:
	ppmInit();
	break;

    case RADIO_TYPE_SUMD:
        grhottInit();
	radioRCSelect(0);
        break;

    default:
	AQ_NOTICE("WARNING: Invalid radio type!");
	return;
    }

    radioTaskStack = aqStackInit(RADIO_STACK_SIZE, "RADIO");

    radioData.radioTask = CoCreateTask(radioTaskCode, (void *)0, RADIO_PRIORITY, &radioTaskStack[RADIO_STACK_SIZE-1], RADIO_STACK_SIZE);
}
