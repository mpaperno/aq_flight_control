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

#ifndef _signaling_h
#define _signaling_h

#include "pwm.h"

#define SIG_SPEAKER_FREQ	2000	// frequency for piezo speaker in Hz
#define SIG_SPEAKER_PULSE_LEN	(500 / (SIG_SPEAKER_FREQ / 1000))  // pwm pulse length to achieve ~50% duty cycle. reference: 1000hz = 500 usec/cycle, 2000hz = 250 usec/cycle

enum signalingEventTypes {
    SIG_EVENT_NONE = 0,
    // ongoing events
    SIG_EVENT_DISARMED_NOGPS,
    SIG_EVENT_DISARMED_GPS,
    SIG_EVENT_ARMED,
    SIG_EVENT_FLYING,
    SIG_EVENT_FLYING_HF,
    SIG_EVENT_ALTHOLD,
    SIG_EVENT_POSHOLD,
    SIG_EVENT_MISSION,
    SIG_EVENT_DVH,
    SIG_EVENT_LOWBATT,
    SIG_EVENT_RADIOLOSS,
    SIG_EVENT_RADIOLOSS2,
    // one-time events
    SIG_EVENT_OT_ARMING,
    SIG_EVENT_OT_DISARMING,
    SIG_EVENT_OT_HF_SET,
    SIG_EVENT_ENUM_END
};

typedef struct {
  pwmPortStruct_t *beeperPort;
  pwmPortStruct_t *ledPort1;
  pwmPortStruct_t *ledPort2;
  pwmPortStruct_t *pwmPort;
  uint8_t enabled;	// flag indicating if any signaling is used (any ports are enabled)
  uint8_t patPos;	    // loop counter used in Led patterns
  uint8_t patLen;	    // number of positions in pattern per output device; 10 = 1Hz. Changing it will! affect the led pattern event
  uint8_t oneTimeEvtTyp;    // if set, a one-time event is signaled, overriding any other current events
  uint8_t oneTimeEvtStat;   // current one-time event stage: 0=not active; 1=event is done; 2=event is in progress
  uint8_t beeperType;	    // 0 = buzzer, 1 = speaker
} sigStruct_t;

extern sigStruct_t sigData;

extern void signalingInit(void);
extern void signalingEvent(void);
extern void signalingOnetimeEvent(int eventTyp);

#endif
