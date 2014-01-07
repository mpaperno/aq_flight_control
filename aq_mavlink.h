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

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEND_UART_BYTES		    mavlinkSendPacket

#define AQMAVLINK_HEARTBEAT_INTERVAL	    1e6f		    // 1Hz
#define AQMAVLINK_PARAM_INTERVAL		    (1e6f / 150.0f)	    // 150Hz
#define AQMAVLINK_WP_TIMEOUT		    1e6f		    // 1 second - retry frequency for waypoint requests to planner
#define AQMAVLINK_WP_MAX_ATTEMPTS		    20			    // maximum number of retries for wpnt. requests

//#define MAVLINK_NOTICE_DEPTH		    25
#define AQMAVLINK_PARAMID_LEN		    16

// this should equal MAV_DATA_STREAM_ENUM_END from mavlink.h
#define AQMAVLINK_TOTAL_STREAMS		    13

// default stream rates in microseconds
#define AQMAVLINK_STREAM_RATE_ALL		0
#define AQMAVLINK_STREAM_RATE_RAW_SENSORS	0
#define AQMAVLINK_STREAM_RATE_EXTENDED_STATUS	0
#define AQMAVLINK_STREAM_RATE_RC_CHANNELS	1e6	    // channels and outputs at 1Hz
#define AQMAVLINK_STREAM_RATE_RAW_CONTROLLER	(1e6/10)    // attitude at 10Hz
#define AQMAVLINK_STREAM_RATE_POSITION		1e6	    // position at 1Hz
#define AQMAVLINK_STREAM_RATE_EXTRA1		0
#define AQMAVLINK_STREAM_RATE_EXTRA2		0
#define AQMAVLINK_STREAM_RATE_EXTRA3		0

typedef struct {
    unsigned long interval;
    unsigned long dfltInterval;
    unsigned long next;
    uint8_t enable;
} mavlinkStreams_t;

typedef struct {
    mavlink_status_t mavlinkStatus;
    mavlinkStreams_t streams[AQMAVLINK_TOTAL_STREAMS];
    unsigned long nextHeartbeat;
    unsigned long nextParam;
    unsigned int currentParam;

    // this is a temporary implementation until we adopt mavlink completely
    int numParams;

    uint16_t packetDrops;
    uint16_t idlePercent;
    uint8_t wpTargetSysId;
    uint8_t wpTargetCompId;
    uint8_t wpCount;
    uint8_t wpCurrent;
    uint32_t wpNext;
    uint8_t wpAttempt;

    unsigned long lastCounter;

    uint8_t indexPort;		// current port # in channels/servo outputs sequence
    uint8_t indexTelemetry;	// current index in telemetry sequence

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
extern void mavlinkSendParameter(uint8_t sysId, uint8_t compId, const char *paramName, float value);

#endif
#endif	// USE_MAVLINK
