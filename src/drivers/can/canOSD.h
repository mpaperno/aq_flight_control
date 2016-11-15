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

#ifndef _can_osd_h
#define _can_osd_h

#include "can.h"

enum osdTelemetryTypes {
    OSD_TELEM_STATUS = 1,
    OSD_TELEM_LAT_LON,
    OSD_TELEM_VELNE,
    OSD_TELEM_VELD,
    OSD_TELEM_ALT,
    OSD_TELEM_HOME,
    OSD_TELEM_RC_QUALITY,
    OSD_TELEM_GPS_HACC,
    OSD_TELEM_Q1_Q2,
    OSD_TELEM_Q3_Q4,
    OSD_TELEM_NUM
};

typedef struct {
    canNodes_t *node;
    uint8_t slots[OSD_TELEM_NUM];
    uint16_t rate;
} canOSDStruct_t;

extern void canOSDInit(void);
extern void canOSDRequestValue(canNodes_t *node, uint8_t seqId, uint8_t *data);
extern void canOSDRequstRate(canNodes_t *node, uint8_t seqId, uint8_t *data);
extern void canOSDTelemetry(uint32_t loop);

#endif