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
#include "supervisor.h"
#include "aq_mavlink.h"
#include "notice.h"
#include "digital.h"
#include "radio.h"
#include "nav.h"
#include "adc.h"
#include "aq_timer.h"
#include "util.h"

supervisorStruct_t supervisorData;

OS_STK supervisorTaskStack[TASK_STACK_SIZE];

void supervisorTaskCode(void *unused) {
    uint32_t count = 0;

    AQ_NOTICE("Supervisor task started...\n");

    while (1) {
	yield(1000/SUPERVISOR_RATE);

	if (supervisorData.state & STATE_DISARMED) {
	    // 0.5 Hz blink debug LED if config file could be found on uSD card
	    if (!(count % 10) && supervisorData.configRead)
		digitalTogg(supervisorData.debugLed);

	    // 1 Hz blink if disarmed, 5 Hz if writing to uSD card
	    if (!(count % ((supervisorData.diskWait) ? 1 : 5)))
		digitalTogg(supervisorData.readyLed);

	    // Arm if all the switches are in default (startup positions) - flaps down, aux2 centered
	    if (RADIO_THROT < p[CTRL_MIN_THROT] && RADIO_RUDD > +500 && RADIO_FLAPS < -250 && RADIO_AUX2 > -250 && RADIO_AUX2 < 250) {
		if (!supervisorData.armTime) {
		    supervisorData.armTime = timerMicros();
		}
		else if ((timerMicros() - supervisorData.armTime) > SUPERVISOR_DISARM_TIME) {
		    supervisorData.state = STATE_ARMED;
		    supervisorData.armTime = 0;
		    digitalHi(supervisorData.readyLed);
		    AQ_NOTICE("Supervisor: armed\n");
		}
	    }
	    else {
    		supervisorData.armTime = 0;
	    }
	}
	else if (supervisorData.state & STATE_ARMED) {
	    // Disarm only if in manual mode
	    if (RADIO_THROT < p[CTRL_MIN_THROT] && RADIO_RUDD < -500 && navData.mode == NAV_STATUS_MANUAL) {
		if (!supervisorData.armTime) {
		    supervisorData.armTime = timerMicros();
		}
		else if ((timerMicros() - supervisorData.armTime) > SUPERVISOR_DISARM_TIME) {
		    supervisorData.state = STATE_DISARMED;
		    supervisorData.armTime = 0;
		    AQ_NOTICE("Supervisor: disarmed\n");
		}
	    }
	    else {
    		supervisorData.armTime = 0;
	    }
	}

	// radio loss
	if ((supervisorData.state & STATE_FLYING) && navData.mode < NAV_STATUS_MISSION) {
	    if (RADIO_QUALITY > 1.0f) {
		supervisorData.lastGoodRadioMicros = timerMicros();

		if (supervisorData.state & STATE_RADIO_LOSS1)
		    AQ_NOTICE("Supervisor: radio signal regained\n");

		supervisorData.state &= ~(STATE_RADIO_LOSS1 | STATE_RADIO_LOSS2);
	    }
	    else if (!(supervisorData.state & STATE_RADIO_LOSS1) && (timerMicros() - supervisorData.lastGoodRadioMicros) > SUPERVISOR_RADIO_LOSS1) {
		supervisorData.state |= STATE_RADIO_LOSS1;
		AQ_NOTICE("Supervisor: radio loss stage 1 detected\n");

		// switch to manual mode (need to reload again alt hold position and fix the copter guarantee in the space)
		navData.mode = NAV_STATUS_MANUAL;

		// hold position
		RADIO_FLAPS = 0;    // position hold
		RADIO_PITCH = 0;    // center sticks
		RADIO_ROLL = 0;
		RADIO_RUDD = 0;
//		RADIO_THROT = 700;  // center throttle
	    }
	    else if (!(supervisorData.state & STATE_RADIO_LOSS2) && (timerMicros() - supervisorData.lastGoodRadioMicros) > SUPERVISOR_RADIO_LOSS2) {
		supervisorData.state |= STATE_RADIO_LOSS2;
		AQ_NOTICE("Supervisor: radio loss stage 2 detected\n");

		// TODO: something
                // for example:
		// RADIO_THROT -= 150; // ~0.5 m/s down speed
	    }
	}

	// low battery
	// smooth vIn readings
	supervisorData.vInLPF += (adcData.vIn - supervisorData.vInLPF) * 0.01;

	if (!(supervisorData.state & STATE_LOW_BATTERY1) && supervisorData.vInLPF < (p[SPVR_LOW_BAT1]*adcData.batCellCount)) {
	    supervisorData.state |= STATE_LOW_BATTERY1;
	    AQ_NOTICE("Supervisor: low battery stage 1 detected\n");

	    // TODO: something
	}
	else if (!(supervisorData.state & STATE_LOW_BATTERY2) && supervisorData.vInLPF < (p[SPVR_LOW_BAT2]*adcData.batCellCount)) {
	    supervisorData.state |= STATE_LOW_BATTERY2;
	    AQ_NOTICE("Supervisor: low battery stage 2 detected\n");

	    // TODO: something
	}

	count++;
    }
}

void supervisorInitComplete(void) {
    supervisorData.state = STATE_DISARMED;
}

void supervisorDiskWait(uint8_t waiting) {
    supervisorData.diskWait = waiting;
}

void supervisorThrottleUp(uint8_t throttle) {
    if (throttle) {
	supervisorData.state |= STATE_FLYING;
    }
    else {
	supervisorData.state &= ~STATE_FLYING;
    }
}

void supervisorSendDataStart(void) {
    digitalTogg(supervisorData.debugLed);
}

void supervisorSendDataStop(void) {
    digitalTogg(supervisorData.debugLed);
}

void supervisorConfigRead(void) {
    supervisorData.configRead = 1;
    digitalHi(supervisorData.debugLed);
}

void supervisorInit(void) {
    supervisorData.readyLed = digitalInit(SUPERVISOR_READY_PORT, SUPERVISOR_READY_PIN);
    digitalLo(supervisorData.readyLed);

    supervisorData.debugLed = digitalInit(SUPERVISOR_DEBUG_PORT, SUPERVISOR_DEBUG_PIN);
    digitalLo(supervisorData.debugLed);

    supervisorData.state = STATE_INITIALIZING;
    supervisorData.vInLPF = 21.0f;

    supervisorData.supervisorTask = CoCreateTask(supervisorTaskCode, (void *)0, 34, &supervisorTaskStack[TASK_STACK_SIZE-1], TASK_STACK_SIZE);
}
