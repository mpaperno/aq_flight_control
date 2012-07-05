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

#ifndef _gimbal_h
#define _gimbal_h

#include "pwm.h"

typedef struct {
    float tilt;
    pwmPortStruct_t *pitch;
    pwmPortStruct_t *roll;
} gimbalStruct_t;

extern gimbalStruct_t gimbalData;

extern void gimbalInit(void);
extern void gimbalUpdate(void);

#endif
