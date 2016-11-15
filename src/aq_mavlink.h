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
    Copyright 2013-2016 Maxim Paperno
*/

#ifndef _aq_mavlink_h
#define _aq_mavlink_h

#include "aq.h"
#ifdef USE_MAVLINK
#include "serial.h"
#include "digital.h"
#include "config.h"
#include "../mavlink_types.h"

#include <stdint.h>

#define MAVLINK_USE_CONVENIENCE_FUNCTIONS
#define MAVLINK_SEND_UART_BYTES			mavlinkSendPacket

extern mavlink_system_t mavlink_system;
extern void mavlinkSendPacket(mavlink_channel_t chan, const uint8_t *buf, uint16_t len);

#include "mavlink.h"

#define AQMAVLINK_HEARTBEAT_INTERVAL		1e6f		    // 1Hz
#define AQMAVLINK_PARAM_INTERVAL		(1e6f / 150.0f)	    // 150Hz
#define AQMAVLINK_WP_TIMEOUT			1e6f		    // 1 second - retry frequency for waypoint requests to planner
#define AQMAVLINK_WP_MAX_ATTEMPTS		20		    // maximum number of retries for wpnt. requests
#define AQMAVLINK_DEFAULT_COMP_ID		MAV_COMP_ID_MISSIONPLANNER

// this should equal MAV_DATA_STREAM_ENUM_END from mavlink.h
#define AQMAVLINK_TOTAL_STREAMS			MAV_DATA_STREAM_ENUM_END
// default stream rates in microseconds
#define AQMAVLINK_STREAM_RATE_ALL		0
#define AQMAVLINK_STREAM_RATE_RAW_SENSORS	0	    // IMU and baro
#define AQMAVLINK_STREAM_RATE_EXTENDED_STATUS	1e6	    // system status at 1Hz
#define AQMAVLINK_STREAM_RATE_RC_CHANNELS	0	    // radio channels
#define AQMAVLINK_STREAM_RATE_RAW_CONTROLLER	(1e6/10)    // attitude at 10Hz
#define AQMAVLINK_STREAM_RATE_POSITION		1e6	    // GPS and UKF position/velocity at 1Hz
#define AQMAVLINK_STREAM_RATE_EXTRA1		0	    // unused
#define AQMAVLINK_STREAM_RATE_EXTRA2		0	    // unused
#define AQMAVLINK_STREAM_RATE_EXTRA3		0	    // AQ custom telemetry options
#define AQMAVLINK_STREAM_RATE_PROPULSION	0	    // ESC/Motor telemetry

enum mavlinkCustomDataSets {
//  AQMAV_DATASET_LEGACY1 = 0,  // legacy sets can eventually be phased out
//  AQMAV_DATASET_LEGACY2,
//  AQMAV_DATASET_LEGACY3,
    AQMAV_DATASET_ALL = 3,        // use this to toggle all datasets at once
    AQMAV_DATASET_GPS,
    AQMAV_DATASET_UKF,
    AQMAV_DATASET_SUPERVISOR,
    AQMAV_DATASET_STACKSFREE,
    AQMAV_DATASET_GIMBAL,
    AQMAV_DATASET_MOTORS,
    AQMAV_DATASET_MOTORS_PWM,
    AQMAV_DATASET_NAV,
    AQMAV_DATASET_IMU,
    AQMAV_DATASET_DEBUG,
    AQMAV_DATASET_RC,
    AQMAV_DATASET_CONFIG,
    AQMAV_DATASET_ENUM_END
};

typedef struct {
    uint32_t interval;		// how often to send stream in us (zero to disable)
    uint32_t dfltInterval;	// default stream interval at startup
    uint32_t next;		// when to send next stream data
    uint8_t enable;		// enable/disable stream
} mavlinkStreams_t;

typedef struct {
    mavlink_status_t mavlinkStatus;
    mavlinkStreams_t streams[AQMAVLINK_TOTAL_STREAMS];

    uint32_t nextHeartbeat;	// time when to send next heartbeat
    uint32_t nextParam;		// time when to send next param value
    uint32_t nextWP;		// when to send the next wpt request to planner

    uint16_t currentParam;	// keep track of parameter send sequence
    uint16_t packetDrops;	// global packet drop counter

    uint8_t sys_type;		// System type (MAV_TYPE enum)
    uint8_t sys_state;		// System state (MAV_STATE enum)
    uint8_t sys_mode;		// System operating mode (MAV_MODE_FLAG enum bitmask)

    uint8_t indexPort;		// current port # in channels outputs sequence
    uint8_t paramCompId;	// component ID to use for params list

    // waypoint programming from mission planner
    uint8_t wpTargetSysId;
    uint8_t wpTargetCompId;
    uint8_t wpCount;		// total waypoints to expect from planner after mission_count msg
    uint8_t wpCurrent;		// current wpt sequence # requested/expected from planner
    uint8_t wpAttempt;		// count of consecutive wpt requests for same sequence #

    uint8_t customDatasets[AQMAV_DATASET_ENUM_END];

} mavlinkStruct_t;

extern mavlinkStruct_t mavlinkData;

extern void mavlinkInit(void);
extern void mavlinkSendNotice(const char *s);
extern void mavlinkWpReached(uint16_t seqId);
extern void mavlinkWpAnnounceCurrent(uint16_t seqId);
extern void mavlinkWpSendCount(void);
extern void mavlinkAnnounceHome(void);
extern void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
extern void mavlinkDo(void);
extern void mavlinkSendParameter(uint8_t sysId, uint8_t compId, const char *paramName, float value);

#endif	// USE_MAVLINK
#endif  // _aq_mavlink_h
