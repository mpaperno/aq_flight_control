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
#include "supervisor.h"
#include "aq_mavlink.h"
#include "comm.h"
#include "digital.h"
#include "radio.h"
#include "nav.h"
#include "analog.h"
#include "aq_timer.h"
#include "util.h"
#ifdef USE_SIGNALING
   #include "signaling.h"
#endif
#include <stdlib.h>
#include <string.h>


supervisorStruct_t supervisorData __attribute__((section(".ccm")));

OS_STK *supervisorTaskStack;

float supervisorSOCTableLookup(float vBat) {
    float soc;
    int i;

    i = 0;
    while (i < SUPERVISOR_SOC_TABLE_SIZE+1 && supervisorData.socTable[i] < vBat)
	i++;

    soc = (float)i * (100.0f / SUPERVISOR_SOC_TABLE_SIZE);

    if (i > 0 && supervisorData.socTable[i] > vBat) {
	float a, b;

	a = supervisorData.socTable[i];
	b = supervisorData.socTable[i-1];

	soc -= (a - vBat) / (a - b) * (100.0f / SUPERVISOR_SOC_TABLE_SIZE);
    }

    return soc;
}

void supervisorCreateSOCTable(void) {
    int i;

    for (i = 0; i < SUPERVISOR_SOC_TABLE_SIZE+1; i++) {
	float x = (float)i * (100.0f / SUPERVISOR_SOC_TABLE_SIZE) * 0.01f;

	supervisorData.socTable[i] = p[SPVR_BAT_CRV1] + p[SPVR_BAT_CRV2]*x + p[SPVR_BAT_CRV3]*x*x + p[SPVR_BAT_CRV4]*x*x*x + p[SPVR_BAT_CRV5]*x*x*x*x + p[SPVR_BAT_CRV6]*x*x*x*x*x;
    }
}

void supervisorArm(void) {
    supervisorData.state = STATE_ARMED;
    AQ_NOTICE("Armed\n");
#ifdef USE_SIGNALING
    signalingOnetimeEvent(SIG_EVENT_OT_ARMING);
#endif
}

void supervisorDisarm(void) {
    supervisorData.state = STATE_DISARMED;
    AQ_NOTICE("Disarmed\n");
#ifdef USE_SIGNALING
    signalingOnetimeEvent(SIG_EVENT_OT_DISARMING);
#endif
}

void supervisorTaskCode(void *unused) {
    uint32_t count = 0;

    AQ_NOTICE("Supervisor task started\n");

    // wait for ADC vIn data
    while (analogData.batCellCount == 0)
	yield(100);

    supervisorCreateSOCTable();

    supervisorData.vInLPF = analogData.vIn;
    supervisorData.soc = 100.0f;

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
		    supervisorArm();
		    supervisorData.armTime = 0;
		    digitalHi(supervisorData.readyLed);
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
		    supervisorDisarm();
		    supervisorData.armTime = 0;
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
		    AQ_NOTICE("Warning: radio signal regained\n");

		supervisorData.state &= ~(STATE_RADIO_LOSS1 | STATE_RADIO_LOSS2);
	    }
	    else if (!(supervisorData.state & STATE_RADIO_LOSS1) && (timerMicros() - supervisorData.lastGoodRadioMicros) > SUPERVISOR_RADIO_LOSS1) {
		supervisorData.state |= STATE_RADIO_LOSS1;
		AQ_NOTICE("Warning: Radio loss stage 1 detected\n");

		// hold position
		RADIO_FLAPS = 0;    // position hold
		RADIO_AUX2 = 0;     // normal home mode
		RADIO_PITCH = 0;    // center sticks
		RADIO_ROLL = 0;
		RADIO_RUDD = 0;
		RADIO_THROT = 700;  // center throttle
	    }
	    else if (!(supervisorData.state & STATE_RADIO_LOSS2) && (timerMicros() - supervisorData.lastGoodRadioMicros) > SUPERVISOR_RADIO_LOSS2) {
		supervisorData.state |= STATE_RADIO_LOSS2;
		AQ_NOTICE("Warning: Radio loss stage 2 detected\n");

		RADIO_THROT -= 200; // slow decent

		if ((int)p[SPVR_FS_RAD_ST2]==1)
                  RADIO_AUX2 = -700;  // return to home

	    }
	}

	// smooth vIn readings
	supervisorData.vInLPF += (analogData.vIn - supervisorData.vInLPF) * 0.05f;

	// determine battery state of charge
	supervisorData.soc = supervisorSOCTableLookup(supervisorData.vInLPF);

	if (supervisorData.state & STATE_FLYING) {
	    // count flight time in seconds
	    supervisorData.flightTime += (1.0f / SUPERVISOR_RATE);

	    // calculate remaining flight time
	    if (supervisorData.soc < 99.0f) {
		supervisorData.flightSecondsAvg += (supervisorData.flightTime / (100.0f - supervisorData.soc) - supervisorData.flightSecondsAvg) * 0.001f;
		supervisorData.flightTimeRemaining = supervisorData.flightSecondsAvg * supervisorData.soc;
	    }
	    else {
		supervisorData.flightSecondsAvg = supervisorData.flightTime;
		supervisorData.flightTimeRemaining = 999.9f * 60.0f;		// unknown
	    }
	}

	// low battery
	if (!(supervisorData.state & STATE_LOW_BATTERY1) && supervisorData.vInLPF < (p[SPVR_LOW_BAT1]*analogData.batCellCount)) {
	    supervisorData.state |= STATE_LOW_BATTERY1;
	    AQ_NOTICE("Warning: Low battery stage 1\n");

	    // TODO: something
	}
	else if (!(supervisorData.state & STATE_LOW_BATTERY2) && supervisorData.vInLPF < (p[SPVR_LOW_BAT2]*analogData.batCellCount)) {
	    supervisorData.state |= STATE_LOW_BATTERY2;
	    AQ_NOTICE("Warning: Low battery stage 2\n");

	    // TODO: something
	}

	count++;

        #ifdef USE_SIGNALING
	    signalingEvent();
        #endif
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
    memset((void *)&supervisorData, 0, sizeof(supervisorData));

    supervisorData.readyLed = digitalInit(SUPERVISOR_READY_PORT, SUPERVISOR_READY_PIN);
    digitalLo(supervisorData.readyLed);

    supervisorData.debugLed = digitalInit(SUPERVISOR_DEBUG_PORT, SUPERVISOR_DEBUG_PIN);
    digitalLo(supervisorData.debugLed);

    supervisorData.state = STATE_INITIALIZING;
    supervisorTaskStack = aqStackInit(SUPERVISOR_STACK_SIZE, "SUPERVISOR");

    supervisorData.supervisorTask = CoCreateTask(supervisorTaskCode, (void *)0, SUPERVISOR_PRIORITY, &supervisorTaskStack[SUPERVISOR_STACK_SIZE-1], SUPERVISOR_STACK_SIZE);
}
