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

#define CONTROL_STACK_SIZE	    200     // must be evenly divisible by 8
#define CONTROL_PRIORITY	    5

// time before recovery of angle control after manual rotation rate override of pitch/roll/yaw, in cycles @ 400Hz
#define CONTROL_MIN_PR_OVERRIDE     50	    // (0.125 s)
#define CONTROL_MIN_YAW_OVERRIDE    300	    // (0.75 s)

#define CONTROL_RADIO_THROT_SCALE   MOTORS_SCALE / RADIO_MID_THROTTLE        // thottle stick scaling
#define CONTROL_RADIO_VALUE_SCALE   DEG_TO_RAD * (1.0f / RADIO_STICK_SCALE)  // convert RC-desired angle deg to rad

#ifdef HAS_QUATOS
    #define USE_QUATOS		    (controlData.controllerType == CONTROLLER_TYPE_QUATOS)
#else
    #define USE_QUATOS		    0
#endif

enum controlAxes {
    RPY_R = 0,
    RPY_P,
    RPY_Y
};

enum controllerTypes {
    CONTROLLER_TYPE_PID_P = 0,
    CONTROLLER_TYPE_PID_C,
    CONTROLLER_TYPE_QUATOS
};

enum controlModes {
    CTRL_MODE_NAV = 0,    // nav controller is active
    CTRL_MODE_ANGLE,      // angle-control mode
    CTRL_MODE_RATE,       // pure rate-of-rotation control
    CTRL_MODE_LTD_RATE    // rate control with limited tilt angles
};

typedef struct {
    // RPY PIDs
    pidStruct_t *anglePID[3];    // for angle-control mode
    pidStruct_t *ratePID[3];     // for angle-control mode
    pidStruct_t *rateModePID[3]; // for rates-only control mode

    // input filters
    utilFilter_t angleFilter[2][3];  // no yaw angle filter
    utilFilter_t rateFilter[3][3];

    float anglesDesired[3];      // final requested RPY rotation angles, degrees
    float ratesDesired[3];       // final requested RPY rotation rates, radians

    uint32_t loops;
    uint32_t lastUpdate;         // time of raw data that this structure is based on
    uint8_t controllerType;      // PID type 0, type 1, or Quatos
    uint8_t controlMode;         // control input type, one of controlModes enum

    OS_TID controlTask;

} controlStruct_t __attribute__((aligned));

extern controlStruct_t controlData;

extern void controlInit(void);

#endif
