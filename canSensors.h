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

#ifndef _cansensors_h
#define _cansensors_h

#include "can.h"

#define CAN_SENSORS_RATE    10    // Hz

// sensor types
enum {
    CAN_SENSORS_PDB_BATV = 0,
    CAN_SENSORS_PDB_BATA,
    CAN_SENSORS_PDB_TEMP,
    CAN_SENSORS_GIMBAL_ACCX,
    CAN_SENSORS_GIMBAL_ACCY,
    CAN_SENSORS_GIMBAL_ACCZ,
    CAN_SENSORS_GIMBAL_GYOX,
    CAN_SENSORS_GIMBAL_GYOY,
    CAN_SENSORS_GIMBAL_GYOZ,
    CAN_SENSORS_NUM
};

typedef struct {
    canNodes_t *nodes[CAN_SENSORS_NUM];
    float values[CAN_SENSORS_NUM];
    uint32_t rcvTimes[CAN_SENSORS_NUM];
} canSensorsStruct_t;

extern canSensorsStruct_t canSensorsData;

extern void canSensorsInit(void);

#endif
