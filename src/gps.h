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

#define GPS_STACK_SIZE          200   // must be evenly divisible by 8
#define GPS_PRIORITY            35

#define GPS_BAUD_RATE           230400

#define GPS_LATENCY             75000       // us (comment out to use uBlox timepulse)

//#define GPS_LOG_BUF             2048        // comment out to disable logging
//#define GPS_FNAME               "GPS"
//#define GPS_DO_RTK                          // comment out to disable GPS Raw data reports
//#define GPS_DEBUG                           // uncomment to enable extra GPS messages

typedef struct {
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
    float heading;  // course over ground (deg)
    float sAcc;     // speed accuracy est (m/s)
    float cAcc;     // course accuracy est (deg)
    float pDOP;     // Position Dilution of Precision
    float hDOP;     // Horizontal DOP
    float vDOP;     // Vertical DOP
    float tDOP;     // Time DOP
    float nDOP;     // Northing DOP
    float eDOP;     // Easting DOP
    float gDOP;     // Geometric DOP

    unsigned long TPtowMS;    // timepulse time of week (ms)
    unsigned long lastReceivedTPtowMS;
    unsigned long lastPosUpdate;
    unsigned long lastVelUpdate;
    unsigned long lastMessage;
} gpsStruct_t __attribute__((aligned));

typedef struct {
    serialPort_t *gpsPort;
    //digitalPin *gpsEnable;
    int32_t microsPerSecond;
    uint32_t lastTimepulse;
    uint32_t baudCycle[7];
    int8_t baudSlot;
    uint8_t logHandle;
    uint8_t ubloxEnabled;

    OS_TID gpsTask;
    OS_FlagID gpsVelFlag;
    OS_FlagID gpsPosFlag;
} gpsTask_t __attribute__((aligned));

extern gpsStruct_t gpsData;
extern gpsTask_t gpsTaskData;

extern void gpsInit(void);
extern void gpsSendPacket(unsigned char len, char *buf);
extern void gpsSetEnabled(bool enable);

#endif
