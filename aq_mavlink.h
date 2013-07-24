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

#ifndef _aq_mavlink_h
#define _aq_mavlink_h

#include "aq.h"
#ifdef USE_MAVLINK
#include "serial.h"
#include "digital.h"
#include "config.h"
// lots of warnings coming from mavlink
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include "../mavlink_types.h"
#pragma GCC diagnostic pop

#define MAVLINK_HEARTBEAT_INTERVAL	    1e6f		    // 1Hz
#define MAVLINK_PARAM_INTERVAL		    (1e6f / 150.0f)	    // 150Hz
#define MAVLINK_WP_TIMEOUT		    1e6f		    // 1 second
#define MAVLINK_NOTICE_DEPTH		    25
#define MAVLINK_PARAMID_LEN		    16
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEND_UART_BYTES		    mavlinkSendPacket

typedef struct {
    mavlink_status_t mavlinkStatus;
    unsigned long nextHeartbeat;
    unsigned long nextParam;
    unsigned int currentParam;

    // TODO: pull actual count from header files
    // the array lengths should equal MAV_DATA_STREAM_ENUM_END from mavlink.h
    unsigned long streamInterval[13];
    unsigned long streamNext[13];

    // this is a temporary implementation until we adopt mavlink completely
    int numParams;

    uint16_t packetDrops;
    uint16_t idlePercent;
    uint8_t mode;
    uint32_t nav_mode;
    uint8_t status;
    uint8_t wpTargetSysId;
    uint8_t wpTargetCompId;
    uint8_t wpCount;
    uint8_t wpCurrent;
    uint32_t wpNext;

    unsigned long lastCounter;

    uint8_t sendTelemetry;  // enable/disable telemetry messages
    uint8_t indexTelemetry; // current index in telemetry sequence
    unsigned long telemetryFrequency; // how often to send data, in us
    unsigned long nextTelemetry; // micros counter for next data send

} mavlinkStruct_t;

extern mavlinkStruct_t mavlinkData;
extern mavlink_system_t mavlink_system;

extern void mavlinkInit(void);
extern void mavlinkSendNotice(const char *s);
extern void mavlinkWpReached(uint16_t seqId);
extern void mavlinkWpAnnounceCurrent(uint16_t seqId);
extern void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
extern void mavlinkDo(void);
extern void mavlinkSendPacket(mavlink_channel_t chan, const uint8_t *buf, uint16_t len);

#endif
#endif	// USE_MAVLINK
