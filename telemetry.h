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

#ifndef _telemetry_h
#define _telemetry_h

#include "serial.h"
#include <CoOS.h>

#define TELEMETRY_COMMAND_BUFSIZE     256

typedef struct {
    OS_TID telemetryTask;

    unsigned long loops;
    unsigned char telemetryEnable;
    unsigned long lastAqCounter;
    float idlePercent;

    uint8_t ckA, ckB;
} telemetryStruct_t;

extern telemetryStruct_t telemetryData;

extern void telemetryInit(void);
extern void telemetryDo(void);
extern void telemetrySendNotice(const char *s);
extern void telemetryEnable(void);
extern void telemetryDisable(void);

#endif
