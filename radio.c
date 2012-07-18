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

#include "aq.h"
#include "config.h"
#include "radio.h"
#include "util.h"
#include "aq_timer.h"
#include "notice.h"
#include <string.h>

radioStruct_t radioData __attribute__((section(".ccm")));

OS_STK *radioTaskStack;

// calculate radio reception quality
void radioReceptionQuality(int q) {
    radioData.quality = utilFilter(&radioData.qualityFilter, (float)(q + 1)) * 0.5f * 100.0f;
}

void radioTaskCode(void *unused) {
    serialPort_t *s = radioData.serialPort;
    int q;

    AQ_NOTICE("Radio task started\n");

    while (1) {
	// wait for data
	yield(2);

	switch ((int)p[RADIO_TYPE]) {
	case 0:
	case 1:
	    while (serialAvailable(s))
		if ((q = spektrumCharIn(serialRead(s)))) {
		    radioData.lastUpdate = timerMicros();
		    radioReceptionQuality(q);
		}
	    break;
	case 2:
	    while (serialAvailable(s))
		if ((q = futabaCharIn(serialRead(s)))) {
		    radioData.lastUpdate = timerMicros();
		    radioReceptionQuality(q);
		}
	    break;
	case 3:
	    if (ppmDataAvailable())
		radioReceptionQuality(1);
	    break;
	}

	// no radio?
	if (timerMicros() - radioData.lastUpdate > 50000)
	    radioReceptionQuality(-1);
    }
}

void radioInit(void) {
    AQ_NOTICE("Radio init\n");

    memset((void *)&radioData, 0, sizeof(radioData));

    utilFilterInit(&radioData.qualityFilter, (1.0f / 50.0f), 0.75f, 0.0f);

    switch ((int)p[RADIO_TYPE]) {
    case 0:
    case 1:
	spektrumInit();
	break;
    case 2:
	futabaInit();
	break;
    case 3:
        ppmInit();
        break;
    case 4:
        // TODO
        break;
    }

    radioTaskStack = aqStackInit(RADIO_STACK_SIZE);

    radioData.radioTask = CoCreateTask(radioTaskCode, (void *)0, RADIO_PRIORITY, &radioTaskStack[RADIO_STACK_SIZE-1], RADIO_STACK_SIZE);
}
