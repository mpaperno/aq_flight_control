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

#include "signaling.h"
#include "analog.h"
#include "config.h"
#include "nav.h"
#include "supervisor.h"
#include "util.h"
#include "comm.h"
#include <string.h>

/* Led patterns, 0=Off, 1=On, every 0.1s (10Hz) the position in the array is chosen and use to control the led's.
 * Beeper patterns specify how long to beep per position, maximum of 100ms per position. Any positive number after a value
 * of 100 will produce a continuous beep (eg. 100,50 will produce one beep 150ms long).
 *
 * Set the patterns to your liking
 */
const uint8_t sig_pattern[12][30] = {
/*         led 1                 led 2               Beeper                          				 */
/*  0 1 2 3 4 5 6 7 8 9   0 1 2 3 4 5 6 7 8 9     0   1   2   3   4   5   6   7   8   9 	counter position */

  { 0,0,0,0,0,0,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0  ,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }, 	/* 0 (All-Off)                 Led 1: Off, Led 2: off */
  { 1,1,0,0,1,1,0,0,1,1  ,0,0,0,0,0,0,0,0,0,0  ,  0,  0,100,  0,100,  0,  0,  0,  0,  0 }, 	/* 1 (Disarmed - no GPS fix)   Led 1: fast flashing, Led 2: off */
  { 1,1,1,1,1,0,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0  ,  0,  0,100,  0,100,  0,  0,  0,  0,  0 }, 	/* 2 (Disarmed - GPS fix)      Led 1: slow flashing (1Hz), Led 2: off */
  { 1,1,1,1,1,1,1,1,1,1  ,0,0,0,0,0,0,0,0,0,0  ,  0,  0,100,  0,  0,  0,  0,  0,  0,  0 }, 	/* 3 (Armed)                   Led 1: On, Led 2: Off */
  { 1,1,1,1,1,1,1,1,0,0  ,0,0,0,0,0,0,0,0,0,0  ,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }, 	/* 4 (Flying)                  Led 1: long on, Led 2: off */
  { 1,1,1,1,1,1,1,1,1,1  ,1,0,0,0,0,1,0,0,0,0  ,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }, 	/* 5 (Alt hold)                Led1: On, Led 2: slow flashing (2Hz) */
  { 1,1,1,1,1,1,1,1,1,1  ,1,1,1,1,1,1,1,1,1,1  ,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }, 	/* 6 (Pos hold)                Led1: On, Led 2: On */
  { 1,1,1,1,1,1,1,1,1,1  ,0,0,0,0,0,1,0,0,0,0  ,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }, 	/* 7 (Mission)                 Led1: On, Led 2: slow flashing (1Hz) */
  { 1,1,1,1,1,1,1,1,1,1  ,0,1,0,1,0,1,0,1,0,1  ,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0 }, 	/* 8 (DVH)	              Led1: On, Led 2: fast flashing (5Hz) */
  { 1,1,0,0,0,0,0,0,0,0  ,0,0,0,0,0,0,0,0,1,1  , 50,  0,  0, 50,  0,  0,  0,  0,  0,  0 }, 	/* 9 (lowBatt 1)               Led 1 & Led 2 alternating slow flashing (1Hz) */
  { 0,1,0,0,1,1,0,0,1,0  ,1,0,0,1,0,0,1,0,0,0  ,100,100,  0,  0,  0,  0,100,100,  0,  0 },	/* 10 (Radio loss/ lowBatt 2)  Led 1 & Led 2 alternating fast flashing (5Hz) */
  { 0,1,0,0,1,1,0,0,1,0  ,1,0,0,1,0,0,1,0,0,0  ,100,100,  0, 50,  0,100,100,  0, 50,  0 }	/* 11 (Radio loss 2)           Led 1 & Led 2 alternating fast flashing (5Hz) */
};

sigStruct_t sigData __attribute__((section(".ccm")));

void signalingBeep(unsigned long hz, unsigned long ms, uint8_t stayOn) {
    if (sigData.beep) {
	if (p[SIG_BEEP_PRT] < 0) { /* negative sign before port number indicates using a piezo speaker */
	    unsigned long us = 500 / (hz / 1000); /* reference: 1000hz = 500 usec/cycle, 2000hz = 250 usec/cycle */
	    unsigned long rep = ((ms * 1000) / us) / 2; /* input = ms in miliseconds, us is in microseconds, delay loop runs twice for dutycycle */
	    for (long i = 0; i < rep; i++) { /* create a square wave for driving the piezo speaker.. */
		pwmDigitalHi(sigData.beep);
		if (i+1 < rep || !stayOn) {
		    delayMicros(us);
		    pwmDigitalLo(sigData.beep);
		    delayMicros(us);
		}
	    }
	}
	else { /* positive port number is used for a piezo buzzer with its own oscillator or LED */
	    pwmDigitalHi(sigData.beep);
	    if (!stayOn) {
		delay(ms);
		pwmDigitalLo(sigData.beep);
	    }
	}
    }
}

void signalingWriteLeds(int statusId) {
    if (sigData.Led_1) {
	if (sig_pattern[statusId][sigData.countPos])
	    pwmDigitalHi(sigData.Led_1)
	else
	    pwmDigitalLo(sigData.Led_1)
    }
    if (sigData.Led_2) {
	if (sig_pattern[statusId][sigData.countPos+sigData.patNo])
	    pwmDigitalHi(sigData.Led_2)
	else
	    pwmDigitalLo(sigData.Led_2)
    }
}

void signalingWriteBeep(int statusId) {
    if (sigData.beep && sig_pattern[statusId][sigData.countPos + sigData.patNo * 2]) {
	uint8_t stayOn = (sigData.countPos + 1 < sigData.patNo && sig_pattern[statusId][sigData.countPos+1 + sigData.patNo * 2] == 100);
	signalingBeep(2000, sig_pattern[statusId][sigData.countPos + sigData.patNo * 2], stayOn);
    }
}

void signalingInit(void) {

    memset((void *)&sigData, 0, sizeof(sigData));

    sigData.countPos = 0;
    sigData.countMax = 10; /* 10 = 1Hz. Changing it will! affect the led pattern event */
    sigData.armStat = 0;
    sigData.patNo = 10;    /* number of positions in pattern per output device (3 x number is total) */

    if (p[SIG_LED_1_PRT]) {
	sigData.Led_1 = pwmInitDigitalOut(p[SIG_LED_1_PRT]-1);

	if (sigData.Led_1)
	    sigData.enabled = 1;
	else
	    AQ_NOTICE("Warning: Led 1 port already in use!\n");
    }
    if (p[SIG_LED_2_PRT]) {
	sigData.Led_2 = pwmInitDigitalOut(p[SIG_LED_2_PRT]-1);

	if (sigData.Led_2)
	    sigData.enabled = 1;
	else
	    AQ_NOTICE("Warning: Led 2 port already in use!\n");
    }
    if (p[SIG_BEEP_PRT]) {
	sigData.beep = pwmInitDigitalOut(abs(p[SIG_BEEP_PRT])-1);

	if (sigData.beep)
	    sigData.enabled = 1;
	else
	    AQ_NOTICE("Warning: Beeper port already in use!\n");
    }

    signalingWriteLeds(SIG_EVENT_NONE);

    /* beep the cell count */
    if (sigData.beep) {
	int i;
	for (i = 0; i < analogData.batCellCount; i++) {
	    signalingBeep(2000, 50, 0);
	    delay(150);
	}
	delay(300);
	signalingBeep(2000, 250, 0) /* ready beep */;
    }
}

void signalingEvent() {

    if (!sigData.enabled)
	return;

    switch (supervisorData.state) {
    case STATE_DISARMED:
	if (navData.fixType == 3)
	    signalingWriteLeds(SIG_EVENT_DISARMED_GPS);
	else
	    signalingWriteLeds(SIG_EVENT_DISARMED_NOGPS);

	if (sigData.armStat == 2 || sigData.armStat == 3) {
	    sigData.armStat = 1; // disarming
	    sigData.countPos = 0; // reset the counter
	}
	if (sigData.armStat == 1) { /* this section may only run once during 1 full pattern cycle */
	    signalingWriteBeep(SIG_EVENT_DISARMED_NOGPS);
	    if (sigData.countPos + 1 == sigData.patNo)
		sigData.armStat = 0; // disarmed
	}
	break;

    case STATE_ARMED:
	signalingWriteLeds(SIG_EVENT_ARMED);

	if (sigData.armStat == 0 || sigData.armStat == 1) {
	    sigData.armStat = 3; // arming
	    sigData.countPos = 0; // reset the counter
	}
	if (sigData.armStat == 3) { /* this section may only run once during 1 full pattern cycle */
	    signalingWriteBeep(SIG_EVENT_ARMED);
	    if (sigData.countPos + 1 == sigData.patNo)
		sigData.armStat = 2; //armed
	}
	break;

    case STATE_FLYING:
    case STATE_ARMED | STATE_FLYING:
	switch (navData.mode) {
	case NAV_STATUS_ALTHOLD:
	    signalingWriteLeds(SIG_EVENT_ALTHOLD);
	    break;
	case NAV_STATUS_POSHOLD:
	    signalingWriteLeds(SIG_EVENT_POSHOLD);
	    break;
	case NAV_STATUS_MISSION:
	    signalingWriteLeds(SIG_EVENT_MISSION);
	    break;
	case NAV_STATUS_DVH:
	    signalingWriteLeds(SIG_EVENT_DVH);
	    break;
	default:
	    signalingWriteLeds(SIG_EVENT_FLYING);
	    break;
	}
	break;

    case STATE_ARMED | STATE_FLYING | STATE_RADIO_LOSS1:
	signalingWriteLeds(SIG_EVENT_RADIOLOSS);
	signalingWriteBeep(SIG_EVENT_RADIOLOSS);
	break;

    case STATE_ARMED | STATE_FLYING | STATE_RADIO_LOSS1 | STATE_RADIO_LOSS2:
	signalingWriteLeds(SIG_EVENT_RADIOLOSS2);
	signalingWriteBeep(SIG_EVENT_RADIOLOSS2);
	break;

    case STATE_ARMED | STATE_LOW_BATTERY1:
    case STATE_ARMED | STATE_FLYING | STATE_LOW_BATTERY1:
	signalingWriteLeds(SIG_EVENT_LOWBATT);
	signalingWriteBeep(SIG_EVENT_LOWBATT);
	break;

    case STATE_DISARMED | STATE_LOW_BATTERY1:
	signalingWriteLeds(SIG_EVENT_LOWBATT);
	signalingWriteBeep(SIG_EVENT_LOWBATT);
	break;

    case STATE_ARMED | STATE_LOW_BATTERY1 | STATE_LOW_BATTERY2:
    case STATE_ARMED | STATE_FLYING | STATE_LOW_BATTERY1 | STATE_LOW_BATTERY2:
	signalingWriteLeds(SIG_EVENT_RADIOLOSS);
	signalingWriteBeep(SIG_EVENT_RADIOLOSS);
	break;

    case STATE_DISARMED | STATE_LOW_BATTERY1 | STATE_LOW_BATTERY2:
	signalingWriteLeds(SIG_EVENT_RADIOLOSS);
	signalingWriteBeep(SIG_EVENT_RADIOLOSS);
	break;

    default:
	signalingWriteLeds(SIG_EVENT_NONE);
	break;
    }

    sigData.countPos++;

    if (sigData.countPos == sigData.countMax)
	sigData.countPos = 0;

} /* end signaling event */
