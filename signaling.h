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

#ifndef _signaling_h
#define _signaling_h

#include "pwm.h"

enum signalingEventTypes {
    SIG_EVENT_NONE = 0,
    SIG_EVENT_DISARMED_NOGPS,
    SIG_EVENT_DISARMED_GPS,
    SIG_EVENT_ARMED,
    SIG_EVENT_FLYING,
    SIG_EVENT_ALTHOLD,
    SIG_EVENT_POSHOLD,
    SIG_EVENT_MISSION,
    SIG_EVENT_DVH,
    SIG_EVENT_LOWBATT,
    SIG_EVENT_RADIOLOSS,
    SIG_EVENT_RADIOLOSS2,
};

typedef struct {
  pwmPortStruct_t *beep;
  pwmPortStruct_t *Led_1;
  pwmPortStruct_t *Led_2;
  uint8_t enabled;	// flag indicating if any signaling is used (any ports are enabled)
  uint8_t countPos;	// loop counter used in Led patterns
  uint8_t countMax;	// Maximum number loop counter, default on 10 = 1Hz. Changing it will! affect the led pattern event
  uint8_t armStat;	// detect status change disarmed-armed and armed-disarmed. 0=disarmed, 1=disarming, 2=armed, 3=arming
  uint8_t patNo;    	// number of positions in pattern per output device (3 x number is total)
} sigStruct_t;

extern sigStruct_t sigData;
extern void signalingBeep (unsigned long hz, unsigned long ms, uint8_t stayOn);
extern void signalingWriteLeds(int statusId);
extern void signalingInit(void);
extern void signalingEvent(void);

#endif
