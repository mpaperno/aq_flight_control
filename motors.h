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

#ifndef _motors_h
#define _motors_h

#include "aq.h"
#include "pwm.h"

#define MOTORS_NUM		    14
#define MOTORS_CELL_VOLTS	    3.7f
#define MOTORS_THROTTLE_LIMITER	    0.3f

typedef struct {
    float throttle;
    float pitch;
    float roll;
    float yaw;
} motorsPowerStruct_t;

typedef struct {
    motorsPowerStruct_t *distribution;
    pwmStruct_t *pwm[MOTORS_NUM];
    int16_t value[MOTORS_NUM];	    // in us
    float thrust[MOTORS_NUM];
    float pitch, roll, yaw;
    float throttle;
    float throttleLimiter;
    uint8_t active[MOTORS_NUM];
} motorsStruct_t;

extern motorsStruct_t motorsData;

extern void motorsInit(void);
extern void motorsCommands(float throtCommand, float pitchCommand, float rollCommand, float ruddCommand);
extern void motorsSendValues(void);
extern void motorsOff(void);

#endif
