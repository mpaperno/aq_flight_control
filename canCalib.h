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

#ifndef _canCalib_h
#define _canCalib_h

#define CAN_ID			    0x0

#define CAN_AXIS_STATUS		    0x100
#define CAN_AXIS_TARGET_POS	    0x110
#define CAN_AXIS_TARGET_VEL	    0x120
#define CAN_ARM			    0x130
#define CAN_DISARM		    0x140
#define CAN_SET_PARAM		    0x150
#define CAN_GET_PARAM		    0x160
#define CAN_PARAM_DATA		    0x170
#define CAN_CONFIG_READ		    0x180
#define CAN_CONFIG_WRITE	    0x190
#define CAN_CONFIG_DEFAULT	    0x1A0
#define CAN_IMU_VALUES_1	    0x1B0   // accX, accY
#define CAN_IMU_VALUES_2	    0x1C0   // accZ, magX
#define CAN_IMU_VALUES_3	    0x1D0   // magY, magZ
#define CAN_IMU_VALUES_4	    0x1E0   // gyoX, gyoY
#define CAN_IMU_VALUES_5	    0x1F0   // gyoZ, temp

typedef struct {
    uint32_t txErrors;
    CanRxMsg rxMessage;
    CanTxMsg txMessage;
    uint8_t initialized;
    int8_t lastTxMailbox;
} canCalibStruct_t;

extern canCalibStruct_t canCalibData;

extern void canCalibInit(void);
extern void canTxIMUData(uint32_t loop);

#endif
