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

#include "signaling.h"
#include "analog.h"
#include "config.h"
#include "nav.h"
#include "supervisor.h"
#include "util.h"
#include "comm.h"
#include <string.h>


// Led/Beeper patterns: 0=Off, 1=On, every 0.1s (10Hz) the position in the array is chosen and used to control the LED/beeper.
// Last column is PWM pulse width to output for given event.

const uint16_t sig_pattern[SIG_EVENT_ENUM_END][31] = {
//         led 1                 led 2               Beeper           pwm
//  0 1 2 3 4 5 6 7 8 9   0 1 2 3 4 5 6 7 8 9   0 1 2 3 4 5 6 7 8 9  		   counter position
// ongoing events
  { 0,0,0,0,0,0,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0 ,1000 }, 	// 0 (All-Off)			L1: Off, L2: Off
  { 1,1,0,0,1,1,0,0,1,1  ,0,0,0,0,0,0,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0 ,1025 }, 	// 1 (Disarmed - no GPS fix)	L1: 3 200ms flashes, L2: Off;
  { 1,1,1,1,1,0,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0 ,1075 }, 	// 2 (Disarmed - GPS fix)	L1: slow flashing (1Hz), L2: off;
  { 1,1,1,1,1,1,1,1,1,1  ,0,0,0,0,0,0,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0 ,1125 }, 	// 3 (Armed)			L1: On, L2: Off;
  { 1,1,1,1,1,1,1,1,1,1  ,0,0,0,0,0,0,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0 ,1175 }, 	// 4 (Flying)			L1: On, L2: Off
  { 1,1,1,1,1,1,1,1,0,0  ,0,0,0,0,0,0,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0 ,1575 }, 	// 5 (Flying HeadingFree)	L1: .8s on/.2s off, L2: off;
  { 1,1,1,1,1,1,1,1,1,1  ,1,0,0,0,0,1,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0 ,1225 }, 	// 6 (Alt hold)			L1: On, L2: slow flashing (2Hz)
  { 1,1,1,1,1,1,1,1,1,1  ,1,1,1,1,1,1,1,1,1,1  ,0,0,0,0,0,0,0,0,0,0 ,1275 }, 	// 7 (Pos hold)			L1: On, L2: On
  { 1,1,1,1,1,1,1,1,1,1  ,0,0,0,0,0,1,0,0,0,0  ,0,0,0,0,0,0,0,0,0,0 ,1325 }, 	// 8 (Mission)			L1: On, L2: slow flashing (1Hz)
  { 1,1,1,1,1,1,1,1,1,1  ,0,1,0,1,0,1,0,1,0,1  ,0,0,0,0,0,0,0,0,0,0 ,1375 }, 	// 9 (DVH)			L1: On, L2: fast flashing (5Hz)
  { 1,1,0,0,0,0,0,0,0,0  ,0,0,0,0,0,0,0,0,1,1  ,1,0,0,1,0,0,0,0,0,0 ,1425 }, 	// 10 (Low batt 1)		L1 & L2 alternating slow flashing (1Hz); 2x100ms beeps per sec
  { 0,1,0,0,1,1,0,0,1,0  ,1,0,0,1,0,0,1,0,0,0  ,1,1,0,0,0,0,1,1,0,0 ,1475 },	// 11 (Radio loss 1/Low batt 2)	L1 & L2 alternating fast flashing (5Hz); 2x200ms beeps per sec
  { 0,1,0,0,1,1,0,0,1,0  ,1,0,0,1,0,0,1,0,0,0  ,1,1,0,1,0,1,1,0,1,0 ,1525 },	// 12 (Radio loss 2)		L1 & L2 alternating fast flashing (5Hz); 2x200ms 1x100ms beep per sec
// one-time events
  { 1,1,1,1,1,1,1,1,1,1  ,1,1,1,1,1,0,0,0,0,0  ,0,0,1,0,0,0,0,0,0,0 ,1025 }, 	// 13 (Arming)			L1: On, L2: 500ms; 1x100ms beep
  { 1,1,1,1,1,1,1,1,1,1  ,0,0,0,0,0,1,1,1,1,1  ,0,0,1,0,1,0,0,0,0,0 ,1125 }, 	// 14 (Disarming)		L1: On, L2: 500ms; 2x100ms beeps
  { 1,1,0,0,1,1,0,0,1,1  ,0,0,1,1,0,0,1,1,0,0  ,1,1,1,1,1,1,1,1,1,1 ,1525 }, 	// 15 (HeadingFree ref. set)	L1 & L2 alternating 200ms flashes; 1s beep
};

sigStruct_t sigData __attribute__((section(".ccm")));

// warning: this is a blocking event, only call it when not flying or from a non-critical process
void signalingBeep(unsigned long hz, unsigned long ms) {
    if (sigData.beeperPort && ms) {
	// using a piezo speaker
	if (sigData.beeperType) {
	    *sigData.beeperPort->ccr = SIG_SPEAKER_PULSE_LEN;
	    delay(ms);
	    *sigData.beeperPort->ccr = 0;
		}
	// piezo buzzer with its own oscillator, or LED
	else {
	    pwmDigitalHi(sigData.beeperPort);
		delay(ms);
	    pwmDigitalLo(sigData.beeperPort);
	    }
	}
}

void signalingWriteOutput(int eventType) {
    if (sigData.ledPort1) {
	if (sig_pattern[eventType][sigData.patPos]) {
	    pwmDigitalHi(sigData.ledPort1);
	} else {
	    pwmDigitalLo(sigData.ledPort1);
	}
    }
    if (sigData.ledPort2) {
	if (sig_pattern[eventType][sigData.patPos + sigData.patLen]) {
	    pwmDigitalHi(sigData.ledPort2);
	} else {
	    pwmDigitalLo(sigData.ledPort2);
	}
    }
    if (sigData.beeperPort) {
	if (sigData.beeperType) {  // speaker
	    *sigData.beeperPort->ccr = sig_pattern[eventType][sigData.patPos + sigData.patLen * 2] * SIG_SPEAKER_PULSE_LEN;
	}
	else {  // buzzer
	    if (sig_pattern[eventType][sigData.patPos + sigData.patLen * 2]) {
		pwmDigitalHi(sigData.beeperPort); }
	    else {
		pwmDigitalLo(sigData.beeperPort); }
	}
    }
    if (sigData.pwmPort) {
	*sigData.pwmPort->ccr = sig_pattern[eventType][sigData.patLen * 3];
    }
}

void signalingInit(void) {

    memset((void *)&sigData, 0, sizeof(sigData));

    sigData.patPos = 0;
    sigData.patLen = 10;  // 10 = 1Hz. Changing it will! affect the led pattern event

    if (p[SIG_LED_1_PRT]) {
	sigData.ledPort1 = pwmInitDigitalOut(p[SIG_LED_1_PRT]-1);

	if (sigData.ledPort1)
	    sigData.enabled = 1;
	else
	    AQ_NOTICE("Warning: Could not open LED 1 signaling port!\n");
    }
    if (p[SIG_LED_2_PRT]) {
	sigData.ledPort2 = pwmInitDigitalOut(p[SIG_LED_2_PRT]-1);

	if (sigData.ledPort2)
	    sigData.enabled = 1;
	else
	    AQ_NOTICE("Warning: Could not open LED 2 signaling port!\n");
    }
    if (p[SIG_BEEP_PRT]) {
	if (p[SIG_BEEP_PRT] > 0) {  // piezo buzzer
	    sigData.beeperPort = pwmInitDigitalOut(abs(p[SIG_BEEP_PRT])-1);
	    sigData.beeperType = 0;
	}
	else {	// piezo speaker
	    sigData.beeperPort = pwmInitOut(abs(p[SIG_BEEP_PRT])-1, 1000000, SIG_SPEAKER_FREQ, 0, 0);
	    sigData.beeperType = 1;
	}

	if (sigData.beeperPort)
	    sigData.enabled = 1;
	else
	    AQ_NOTICE("Warning: Could not open Beeper signaling port!\n");
    }
    if (p[SIG_PWM_PRT]) {
	sigData.pwmPort = pwmInitOut(p[SIG_PWM_PRT]-1, 1000000, 2500, 950, 0);

	if (sigData.pwmPort)
	    sigData.enabled = 1;
	else
	    AQ_NOTICE("Warning: Could not open PWM signaling port!\n");
    }

    signalingWriteOutput(SIG_EVENT_NONE);

    // beep the cell count
    if (sigData.beeperPort) {
	int i;
	for (i = 0; i < analogData.batCellCount; i++) {
	    signalingBeep(2000, 50);
	    delay(150);
	}
	delay(300);
	signalingBeep(2000, 250);  // ready beep
    }
}

// initiate a one-time event, eg. arming/disarming
// eventTyp should be one of signalingEventTypes
void signalingOnetimeEvent(int eventType) {
    if (eventType < 0 || eventType >= SIG_EVENT_ENUM_END)
	return;

    // shut down any current event signals
    // TODO: set up a signaling event queue
    if (sigData.oneTimeEvtTyp) {
	signalingWriteOutput(SIG_EVENT_NONE);
    }
    sigData.oneTimeEvtStat = 0;
    sigData.oneTimeEvtTyp = eventType;
}

void signalingEvent() {

    if (!sigData.enabled)
	return;

    // watch for one-time event being triggered
    if (sigData.oneTimeEvtTyp) {
	switch (sigData.oneTimeEvtStat) {
	case 0:	// start the event
	    sigData.oneTimeEvtStat = 2;
	    sigData.patPos = 0;
	    break;
	case 1: // event has finished
	    sigData.oneTimeEvtStat = 0;
	    sigData.oneTimeEvtTyp = SIG_EVENT_NONE;
	    sigData.patPos = 0;
	    // make sure everything is off
	    signalingWriteOutput(SIG_EVENT_NONE);
	    break;
	case 2: // event is in progress
	    signalingWriteOutput(sigData.oneTimeEvtTyp);
	    // see if we're done  TODO: allow events longer than one pattern length
	    if (++sigData.patPos == sigData.patLen)
		sigData.oneTimeEvtStat = 1;
	    break;
	}
	return;
    }

    switch (supervisorData.state) {
    case STATE_DISARMED:
	if (navData.fixType == 3)
	    signalingWriteOutput(SIG_EVENT_DISARMED_GPS);
	else
	    signalingWriteOutput(SIG_EVENT_DISARMED_NOGPS);
	break;

    case STATE_ARMED:
	signalingWriteOutput(SIG_EVENT_ARMED);
	break;

    case STATE_FLYING:
    case STATE_ARMED | STATE_FLYING:
	switch (navData.mode) {
	case NAV_STATUS_ALTHOLD:
	    signalingWriteOutput(SIG_EVENT_ALTHOLD);
	    break;
	case NAV_STATUS_POSHOLD:
	    signalingWriteOutput(SIG_EVENT_POSHOLD);
	    break;
	case NAV_STATUS_MISSION:
	    signalingWriteOutput(SIG_EVENT_MISSION);
	    break;
	case NAV_STATUS_DVH:
	    signalingWriteOutput(SIG_EVENT_DVH);
	    break;
	default:
	    if (navData.headFreeMode > NAV_HEADFREE_OFF)
		signalingWriteOutput(SIG_EVENT_FLYING_HF);
	    else
	    signalingWriteOutput(SIG_EVENT_FLYING);
	    break;
	}
	break;

    case STATE_ARMED | STATE_FLYING | STATE_RADIO_LOSS1:
	signalingWriteOutput(SIG_EVENT_RADIOLOSS);
	break;

    case STATE_ARMED | STATE_FLYING | STATE_RADIO_LOSS1 | STATE_RADIO_LOSS2:
	signalingWriteOutput(SIG_EVENT_RADIOLOSS2);
	break;

    case STATE_ARMED | STATE_LOW_BATTERY1:
    case STATE_ARMED | STATE_FLYING | STATE_LOW_BATTERY1:
	signalingWriteOutput(SIG_EVENT_LOWBATT);
	break;

    case STATE_DISARMED | STATE_LOW_BATTERY1:
	signalingWriteOutput(SIG_EVENT_LOWBATT);
	break;

    case STATE_ARMED | STATE_LOW_BATTERY1 | STATE_LOW_BATTERY2:
    case STATE_ARMED | STATE_FLYING | STATE_LOW_BATTERY1 | STATE_LOW_BATTERY2:
	signalingWriteOutput(SIG_EVENT_RADIOLOSS);
	break;

    case STATE_DISARMED | STATE_LOW_BATTERY1 | STATE_LOW_BATTERY2:
	signalingWriteOutput(SIG_EVENT_RADIOLOSS);
	break;

    default:
	signalingWriteOutput(SIG_EVENT_NONE);
	break;
    }

    if (++sigData.patPos == sigData.patLen)
	sigData.patPos = 0;

}
