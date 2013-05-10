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

#ifndef _downlink_h
#define _downlink_h

#include "aq.h"
#include "serial.h"
#include "digital.h"
#include <CoOS.h>

typedef struct {
    serialPort_t *serialPort;
    OS_MutexID serialPortMutex;
} downlinkStruct_t;

extern downlinkStruct_t downlinkData;

extern void downlinkInit(void);
extern void downlinkSendChar(unsigned char c);
extern void downlinkSendInt(unsigned int i);
extern void downlinkSendFloat(float f);
extern void downlinkSendString(const char *s);
extern void downlinkResetChecksum(void);
extern void downlinkSendChecksum(void);
extern unsigned char downlinkReadChar(void);
extern void downlinkGetLong(unsigned long *v);

#endif
