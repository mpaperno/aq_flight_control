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

#ifndef _control_h
#define _control_h

#include "pid.h"
#include "util.h"
#include <CoOS.h>

#define CONTROL_STACK_SIZE	    200
#define CONTROL_PRIORITY	    5

#define CONTROL_MIN_YAW_OVERRIDE    300	    // in cycles (0.75 s)

typedef struct {
    OS_TID controlTask;

    unsigned long loops;

    float userPitchTarget;  // smoothed user pitch
    float userRollTarget;   // smoothed user roll

    float navPitchTarget;   // smoothed nav pitch
    float navRollTarget;    // smoothed nav roll

    float pitch;	    // final requested pitch
    float roll;		    // final requested roll
    float yaw;

    utilFilter_t userPitchFilter[3];
    utilFilter_t userRollFilter[3];

    utilFilter_t navPitchFilter[3];
    utilFilter_t navRollFilter[3];

    pidStruct_t *rollRatePID;
    pidStruct_t *pitchRatePID;
    pidStruct_t *yawRatePID;

    pidStruct_t *rollAnglePID;
    pidStruct_t *pitchAnglePID;
    pidStruct_t *yawAnglePID;

    unsigned long lastUpdate;		// time of raw data that this structure is based on
} controlStruct_t;

extern controlStruct_t controlData;

extern void controlInit(void);

#endif
