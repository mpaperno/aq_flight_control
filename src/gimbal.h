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

#ifndef _gimbal_h
#define _gimbal_h

#include "pwm.h"

#define GMBL_DEGREES_TO_PW	(2000 - 1000)		// degrees multiplier to obtain PWM pulse width before scaling

// PWM values for triggerPort and passthroughPort outputs
#define GMBL_TRIG_NTRL_PWM	1500			// middle position pulse width (this gets added to AQ radio value)
#define GMBL_TRIG_MIN_PWM	750			// minimum pulse width
#define GMBL_TRIG_MAX_PWM	2250			// maximum pulse width

typedef struct {
    float tilt;			// last gimbal pitch output position
    uint8_t trigger;		// boolean indicating automatic trigger is activated (can be set externally, eg. to trigger photo at wpt)
    uint16_t triggerCount;	// total times trigger has been activated
    uint16_t triggerLogVal;	// value to be logged == 0 when not active, == triggerCount when active
    uint32_t triggerTimer;	// keep track of how long trigger has been active. == 0 when trigger is not active
    uint32_t triggerLastTime;	// keep track of last trigger activation (for trigger by time interval)
    double triggerLastLat;	// latitude of last trigger activation (for trigger by distance interval)
    double triggerLastLon;	// longitude of last trigger activation

    pwmPortStruct_t *pitchPort;
    pwmPortStruct_t *rollPort;
    pwmPortStruct_t *tiltPort;
    pwmPortStruct_t *triggerPort;
    pwmPortStruct_t *passthroughPort;
} gimbalStruct_t;

extern gimbalStruct_t gimbalData;

extern void gimbalInit(void);
extern void gimbalUpdate(void);

#endif
