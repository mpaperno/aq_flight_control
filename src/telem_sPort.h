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

 Copyright 2015-2016 Maxim Paperno. All rights reserved.

*/

#ifndef _telem_sPort_h
#define _telem_sPort_h

#include "comm.h"
#include <stdint.h>

#define SPORT_TEXT_NUM_BUFFERS         10      // hold this many text messages in memory
#define SPORT_TEXT_MAX_MSG_LEN         52      // message size
#define SPORT_SENSOR_DETECT_WAIT       3e6     // usec to wait after coming online to look for other SPort sensors on the wire and before sending any data. If we see a sensor we simulate, disable ours.

// SPort framing flags
#define SPORT_DATA_REQUEST       0x7E
#define SPORT_DATA_START         0x10
#define SPORT_DATA_PADDING       0x7D
#define SPORT_DATA_ESCAPE        0x20


// SPort DATA ID's                            OpenTx name      (used by) SensID    Typ OTx_t   OTx notes
#define SPORT_DATAID_ADC1         0xF102   // ADC1_ID                    ?         U8          All ADCx values get: applyChannelRatio(idx, value)/100.0 (ratio depends on user settings).
#define SPORT_DATAID_ADC2         0xF103   // ADC2_ID                    ?         U8          They seem to get some kind of smoothing/filter applied on receiving side, changes are relatively slow.
#define SPORT_DATAID_ADC3         0x0900   // A3_FIRST_ID                ?         U32         ". Apply scaling before sending: (value * 330 - 165) / 255.
#define SPORT_DATAID_ADC4         0x0910   // A4_FIRST_ID                ?         U32         ". Apply scaling before sending: (value * 330 - 165) / 255.
#define SPORT_DATAID_ACCX         0x0700   // ACCX_FIRST_ID              ?         S32 (s16)   /100 when displayed in Lua
#define SPORT_DATAID_ACCY         0x0710   // ACCY_FIRST_ID              ?         S32 (s16)   /100 when displayed in Lua
#define SPORT_DATAID_ACCZ         0x0720   // ACCZ_FIRST_ID              ?         S32 (s16)   /100 when displayed in Lua
#define SPORT_DATAID_AIR_SPEED    0x0A00   // AIR_SPEED_FIRST_ID         ?         U32 (u16)   Some unexplained Voodoo magic happens with this value in OpenTx.  TODO
#define SPORT_DATAID_ALTITUDE     0x0100   // ALT_FIRST_ID               VARIO     S32 (s32)   Baro Alt. /100 when displayed in Lua. First rcvd alt is recorded as offset, subsequent values are then adjusted for offset
#define SPORT_DATAID_CELLS        0x0300   // CELLS_FIRST_ID             FLVSS     U32         0xF=1st cell number; 0xF0=ttl cells; 0x000FFF00=1st cell; 0xFFF00000=2nd cell. Cell values should be *500
#define SPORT_DATAID_CURRENT      0x0200   // CURR_FIRST_ID              FAS       U32 (u16)   + "FAS Offset" setting (dflt zero), then /10 when displayed with GetValue()
#define SPORT_DATAID_FUEL         0x0600   // FUEL_FIRST_ID              ?         U32 (u16)
#define SPORT_DATAID_GPS_ALT      0x0820   // GPS_ALT_FIRST_ID           GPS       S32 (s32)   GPS Alt. First rcvd alt is stored as offset
#define SPORT_DATAID_GPS_COURSE   0x0840   // GPS_COURS_FIRST_ID         GPS       U32 (u32)   /100 when displayed in Lua
#define SPORT_DATAID_GPS_SPEED    0x0830   // GPS_SPEED_FIRST_ID         GPS       U32 (u16)   in knots (m/s * 1944)
#define SPORT_DATAID_GPS_TIME     0x0850   // GPS_TIME_DATE_FIRST_ID     GPS       U32         UTC time. 0x000000ff=true:date/false:time; 0xff000000=yr/hr; 0x00ff0000=mo/min; 0x0000ff00=day/sec
#define SPORT_DATAID_LATLONG      0x0800   // GPS_LONG_LATI_FIRST_ID     GPS       U32 (u32)   2 MSB store direction (N/S/E/W), rest is Int(abs(dec.deg) * 6e5);
#define SPORT_DATAID_RPM          0x0500   // RPM_FIRST_ID               RPM       U32 (u16)   gets divided by configured number of blades (default 2)
#define SPORT_DATAID_T1           0x0400   // T1_FIRST_ID                RPM       S32 (s16)
#define SPORT_DATAID_T2           0x0410   // T2_FIRST_ID                RPM       S32 (s16)
#define SPORT_DATAID_VSPEED       0x0110   // VARIO_FIRST_ID             VARIO     S32         Vertical speed. /100 when displayed in Lua.
#define SPORT_DATAID_VFAS         0x0210   // VFAS_FIRST_ID              FAS       U32 (u16)   Pack volts. /10 when displayed in Lua

/*
   Our mappings of data to S.Port fields -- warning: WIP!

Lua Name         Used For                       Notes
a1               not used                       Probably best not to mess with this since some Rxs send it as Rx voltage.
a2-4             not used                       A3-A4 need proper scaling set up on user side.  Values are smoothed (or LPF?) at Taranis.
accx/y/z         xacc/yacc/zacc                 Accelerometer data.
air-speed        not used
altitude         UKF_POSD                       in whole cm
cells            batt. cell V                   calculated voltage per cell based on current total voltage and detected cell count
current          amps                           floor(mA * 10)
fuel             sytem mode & status            2 MSB are packet type, last 14 bits are data. Packet types: 0=custom_mode bits 0-13, 1=custom_mode b14-27, 2=custom_mode b28-31
gps-altitude     gps_altitude                   in whole cm
heading          gps CoG                        floor(hdg * 100) degrees.  This is COURSE not really heading.  Magnetic heading is sent via temp2 data field (see below)
gps-speed        gps_speed                      sent in knots * 1000 (m/s * 1.9438 * 1000)
latitude         gps_latitude                   Lat/Lon sent as per frsky spec, above
longitude        gps_longitude
rpm              used for text message          two chars are sent at a time in one 16bit package. * User must set # of blades to 1 in OpenTx
temp1            gps status packets             2 MSB are packet type, next 2 are Fix Type, last 12 bits are packet value data.
                                                  Packet types: 0=HDOP*100, 1=VDOP*100, 2=Sats Visible (not used)
temp2            various values                 3 MSB are packet type, rest is data. data may contain 2 bit precision indicator.
                                                  Packet types: 0=magnetic heading*10, 1=temperature*10, 2=battery remaining %, 3=next mission wp, 4=wp dist (0,1,or 2 digits precision)
vertical-speed   climb rate                     in whole cm/s
vfas             voltage                        floor(mV * 10) This is limited to 1/10V display resolution on Taranis side

*/

#define sPortCheckConfigFlag(Flg_id)	((uint32_t)p[TELEMETRY_RX_CFG] & Flg_id)

// These are some actual FrSky sensor IDs with checksum as seen on the wire (0x1F bits are ID, 0xE0 is checksum)
enum SPortSensorId {
    SPORT_SENSID_VARIO = 0x00,
    SPORT_SENSID_FLVSS = 0xA1,
    SPORT_SENSID_FAS   = 0x22,
    SPORT_SENSID_GPS   = 0x83,
    SPORT_SENSID_RPM   = 0xE4,
    SPORT_SENSID_SP2UH = 0x45,
    SPORT_SENSID_SP2UR = 0xC6,
    SPORT_SENSID_7     = 0x67,
    SPORT_SENSID_8     = 0x48,
    SPORT_SENSID_9     = 0xE9,
    SPORT_SENSID_10    = 0x6A,
    SPORT_SENSID_11    = 0xCB,
    SPORT_SENSID_12    = 0xAC,
    SPORT_SENSID_13    = 0x0D,
    SPORT_SENSID_14    = 0x8E,
    SPORT_SENSID_15    = 0x2F,
    SPORT_SENSID_16    = 0xD0,
    SPORT_SENSID_17    = 0x71,
    SPORT_SENSID_18    = 0xF2,
    SPORT_SENSID_19    = 0x53,
    SPORT_SENSID_20    = 0x34,
    SPORT_SENSID_21    = 0x95,
    SPORT_SENSID_22    = 0x16,
    SPORT_SENSID_23    = 0xB7,
    SPORT_SENSID_24    = 0x98,
    SPORT_SENSID_25    = 0x39,
    SPORT_SENSID_26    = 0xBA,
    SPORT_SENSID_27    = 0x1B
};

// Sets of data we send. When possible these correspond to the data sets sent by official FrSky sensors. The mapping to frSky sensor IDs is done in sPortGetSensorEnum().
enum SPortDataSets {
    SPORT_DS_UNUSED = 0,
    SPORT_DS_VARIO,        // altitude and vertical speed
    SPORT_DS_FLVSS,        // individual cell voltages (totalV / cellCount)
    SPORT_DS_FAS,          // total voltage and current (if known)
    SPORT_DS_GPS,          // GPS data (lat/lon, speed, course, alt.)
    SPORT_DS_RPM,          // RPM/temperature sensor -- only used when not sending custom data
    SPORT_DS_FUEL,         // sends estimated battery charge state -- only used when not sending custom data
    SPORT_DS_SYS_STAT,     // for system status codes (see notes for "fuel" above)
    SPORT_DS_GPS_STAT,     // for GPS status (see notes for "temp1" above)
    SPORT_DS_MISC,         // for various data (see notes for "temp2" above)
    SPORT_DS_ACC,          // for accelerometer X/Y/Z values
    SPORT_DS_MSG,          // we use the "RPM" data type to send text messages, and may respond to multiple sensor IDs to increase throughput (these are idle when no msgs are queued)
    SPORT_DS_ENUM_END
};

enum SPortProcessStates {
    SPORT_PROC_WAIT_RCV = 0,
    SPORT_PROC_WAIT_SEND,
    SPORT_PROC_WAIT_LISTEN
};

enum SPortConfigFlags {
    SPORT_CFG_SEND_CUSTOM  = (1<<4),  // must start at bit 4, 24b max.
    SPORT_CFG_SEND_ACC	   = (1<<5),
    SPORT_CFG_WAIT_GPS	   = (1<<6),
    SPORT_CFG_WAIT_ALT	   = (1<<7),
    SPORT_CFG_SEND_TXT_MSG = (1<<8),
};

typedef struct {
    uint32_t rxFirstContact;
    uint16_t sendErrCount;
    uint16_t messageCount;
    uint16_t currMessageNum;
    uint16_t sentMessageNum;

    uint8_t sensorsFound[SPORT_DS_ENUM_END];
    uint8_t sensorSendState[SPORT_DS_ENUM_END];
    char messageDataBuffer[SPORT_TEXT_NUM_BUFFERS][SPORT_TEXT_MAX_MSG_LEN];

    uint8_t processState;
    uint8_t datasetRequested;
    uint8_t msgPacketSeq;
    uint8_t msgCharIdx;
    uint8_t gpsFirstLock;
    uint8_t sensorSearchTimeout;
} sPortData_t __attribute__((aligned));

extern sPortData_t sPortData;

extern void sPortSendTextMessage(const char *msg);
extern void sPortProcessInput(commRcvrStruct_t *r);
extern void sPortInit(void);

#endif
