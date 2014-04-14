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

#ifndef _gps_h
#define _gps_h

#include "aq.h"
#include "serial.h"
#include "ublox.h"
#include "digital.h"
#include <CoOS.h>

#define GPS_STACK_SIZE          200
#define GPS_PRIORITY            35

#define GPS_BAUD_RATE           230400

#define GPS_LATENCY             75000       // us (comment out to use uBlox timepulse)

//#define GPS_LOG_BUF             2048        // comment out to disable logging
//#define GPS_FNAME               "GPS"
//#define GPS_DO_RTK                          // comment out to disable GPS Raw data reports
//#define GPS_DEBUG                           // uncomment to enable extra GPS messages

typedef struct {
    OS_TID gpsTask;
    OS_FlagID gpsVelFlag;
    OS_FlagID gpsPosFlag;

    serialPort_t *gpsPort;
    unsigned int baudCycle[7];
    int8_t baudSlot;
    uint8_t logHandle;

    digitalPin *gpsEnable;

    unsigned long iTOW;
    double lat;
    double lon;
    float height;   // above mean sea level (m)
    float hAcc;     // horizontal accuracy est (m)
    float vAcc;     // vertical accuracy est (m)
    float velN;     // north velocity (m/s)
    float velE;     // east velocity (m/s)
    float velD;     // down velocity (m/s)
    float speed;    // ground speed (m/s)
    float heading;  // deg
    float sAcc;     // speed accuracy est (m/s)
    float cAcc;     // course accuracy est (deg)
    float pDOP;     // position Dilution of Precision
    float hDOP;
    float vDOP;
    float tDOP;
    float nDOP;
    float eDOP;
    float gDOP;

    unsigned long TPtowMS;    // timepulse time of week (ms)
    unsigned long lastReceivedTPtowMS;

    unsigned long lastTimepulse;
    unsigned long lastPosUpdate;
    unsigned long lastVelUpdate;
    unsigned long lastMessage;
    signed long microsPerSecond;
} gpsStruct_t;

extern gpsStruct_t gpsData;

extern void gpsInit(void);
extern void gpsSendPacket(unsigned char len, char *buf);

#endif
