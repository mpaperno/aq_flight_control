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

#include "aq.h"
#ifdef USE_MAVLINK
#include "aq_mavlink.h"

#include "alt_ukf.h"
#include "analog.h"
#include "aq_timer.h"
#include "comm.h"
#include "config.h"
#include "control.h"
#include "d_imu.h"
#include "flash.h"
#include "gimbal.h"
#include "gps.h"
#include "imu.h"
#include "motors.h"
#include "nav.h"
#include "nav_ukf.h"
#include "radio.h"
#include "rc.h"
#include "rcc.h"
#include "run.h"
#include "supervisor.h"
#include "util.h"

#include <CoOS.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

mavlinkStruct_t mavlinkData;
mavlink_system_t mavlink_system;

void mavlinkSendPacket(mavlink_channel_t chan, const uint8_t *buf, uint16_t len) {
    commTxBuf_t *txBuf;
    uint8_t *ptr;
    int i;

    txBuf = commGetTxBuf(COMM_STREAM_TYPE_MAVLINK, len);
    // cannot block, must fail
    if (txBuf != 0) {
	ptr = &txBuf->buf;

	for (i = 0; i < len; i++)
	    *ptr++ = *buf++;

	commSendTxBuf(txBuf, len);
    }
}

void mavlinkWpReached(uint16_t seqId) {
    mavlink_msg_mission_item_reached_send(MAVLINK_COMM_0, seqId);
}

void mavlinkWpAnnounceCurrent(uint16_t seqId) {
    mavlink_msg_mission_current_send(MAVLINK_COMM_0, seqId);
}

void mavlinkWpSendCount(void) {
    mavlink_msg_mission_count_send(MAVLINK_COMM_0, mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, navGetWaypointCount());
}

// send new home position coordinates
void mavlinkAnnounceHome(void) {
    mavlink_msg_gps_global_origin_send(MAVLINK_COMM_0, navData.homeLeg.targetLat*(double)1e7f, navData.homeLeg.targetLon*(double)1e7, (navData.homeLeg.targetAlt + navData.presAltOffset)*1e3);
    // announce ceiling altitude if any
    float ca = navData.ceilingAlt;
    if (ca)
	// convert to GPS altitude
	ca += navData.presAltOffset;
    mavlink_msg_safety_allowed_area_send(MAVLINK_COMM_0, MAV_FRAME_GLOBAL, 0, 0, ca, 0, 0, 0);
}

void mavlinkSendNotice(const char *s) {
    mavlink_msg_statustext_send(MAVLINK_COMM_0, MAV_SEVERITY_INFO, (const char *)s);
}

static uint8_t mavlinkConvertDataTypeAQ2MAV(uint8_t typ) {
    switch (typ) {
	case AQ_TYPE_DBL:
	    return MAV_PARAM_TYPE_REAL64;
	case AQ_TYPE_U32:
	    return MAV_PARAM_TYPE_UINT32;
	case AQ_TYPE_S32:
	    return MAV_PARAM_TYPE_INT32;
	case AQ_TYPE_U16:
	    return MAV_PARAM_TYPE_UINT16;
	case AQ_TYPE_S16:
	    return MAV_PARAM_TYPE_INT16;
	case AQ_TYPE_U8:
	    return MAV_PARAM_TYPE_UINT8;
	case AQ_TYPE_S8:
	    return MAV_PARAM_TYPE_INT8;
	case AQ_TYPE_FLT:
	default:
	    return MAV_PARAM_TYPE_REAL32;
    }
}

static void mavlinkSendParamValue(uint16_t id, uint16_t ttl, bool raw) {
    mavlink_system.compid = mavlinkData.paramCompId;
    mavlink_msg_param_value_send(MAVLINK_COMM_0, configGetParamName(id), raw ? configGetParamValueRaw(id) : configGetParamValueForSave(id),
	    mavlinkConvertDataTypeAQ2MAV(configGetParamDataType(id)), ttl, id);
    mavlink_system.compid = AQMAVLINK_DEFAULT_COMP_ID;
}

static void mavlinkToggleStreams(uint8_t enable) {
    for (uint8_t i = 0; i < AQMAVLINK_TOTAL_STREAMS; i++)
	mavlinkData.streams[i].enable = enable && mavlinkData.streams[i].interval;
}

static void mavlinkSetSystemData(void) {

    if ((supervisorData.state & STATE_RADIO_LOSS1) || (supervisorData.state & STATE_LOW_BATTERY2))
	mavlinkData.sys_state =  MAV_STATE_CRITICAL;
    else if (supervisorData.state & STATE_FLYING)
	mavlinkData.sys_state = MAV_STATE_ACTIVE;
    else if (supervisorData.state & STATE_CALIBRATION)
	mavlinkData.sys_state =  MAV_STATE_CALIBRATING;
    else
	mavlinkData.sys_state = MAV_STATE_STANDBY;

    if (navData.mode > NAV_STATUS_MANUAL) {
	mavlinkData.sys_mode = MAV_MODE_FLAG_STABILIZE_ENABLED;

	if (navData.mode == NAV_STATUS_MISSION)
	    mavlinkData.sys_mode |= MAV_MODE_FLAG_AUTO_ENABLED;
	else {
	    if (navData.mode > NAV_STATUS_ALTHOLD)
		mavlinkData.sys_mode |= MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;

	    if (navData.mode == NAV_STATUS_DVH)
		mavlinkData.sys_mode |= MAV_MODE_FLAG_GUIDED_ENABLED;
	}
    }
    else
	mavlinkData.sys_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;

    if (supervisorData.state & STATE_ARMED)
	mavlinkData.sys_mode |= MAV_MODE_FLAG_SAFETY_ARMED;

    if (supervisorData.state & STATE_SIM_ENABLED)
	mavlinkData.sys_mode |= MAV_MODE_FLAG_HIL_ENABLED;

}

void mavlinkDo(void) {
    static unsigned long lastMicros = 0;
    unsigned long micros;
    int8_t streamAll;
    int8_t currDraw = -1;

    micros = timerMicros();

    // handle rollover
    if (micros < lastMicros) {
	mavlinkData.nextHeartbeat = 0;
	mavlinkData.nextParam = 0;
	mavlinkData.nextWP = 0;
	for (uint8_t i=0; i < AQMAVLINK_TOTAL_STREAMS; ++i)
	    mavlinkData.streams[i].next = 0;
    }

    supervisorSendDataStart();

    // heartbeat
    if (mavlinkData.nextHeartbeat < micros) {
	mavlinkSetSystemData();
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0, mavlinkData.sys_type, MAV_AUTOPILOT_AUTOQUAD, mavlinkData.sys_mode, supervisorData.systemStatus, mavlinkData.sys_state);
	mavlinkData.nextHeartbeat = micros + AQMAVLINK_HEARTBEAT_INTERVAL;
    }

    // send streams

    // first check if "ALL" stream is requested
    if ((streamAll = mavlinkData.streams[MAV_DATA_STREAM_ALL].enable && mavlinkData.streams[MAV_DATA_STREAM_ALL].next < micros))
	mavlinkData.streams[MAV_DATA_STREAM_ALL].next = micros + mavlinkData.streams[MAV_DATA_STREAM_ALL].interval;

    // status
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].enable && mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].next < micros)) {
	currDraw = (supervisorData.aOutLPF == SUPERVISOR_INVALID_AMPSOUT_VALUE) ? -1 : supervisorData.aOutLPF * 100;

	mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, (uint16_t)(1000L - LROUNDF(supervisorData.idlePercent * 10.0f)), supervisorData.vInLPF * 1000, currDraw,
		supervisorData.battRemainingPrct, 0, mavlinkData.packetDrops, 0, 0, 0, 0);
	mavlink_msg_radio_status_send(MAVLINK_COMM_0, RADIO_QUALITY, 0, 0, 0, 0, RADIO_ERROR_COUNT, 0);

	mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].next = micros + mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].interval;
    }
    // raw sensors
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_RAW_SENSORS].enable && mavlinkData.streams[MAV_DATA_STREAM_RAW_SENSORS].next < micros)) {
	mavlink_msg_scaled_imu_send(MAVLINK_COMM_0, micros, IMU_ACCX*1000.0f, IMU_ACCY*1000.0f, IMU_ACCZ*1000.0f, IMU_RATEX*1000.0f, IMU_RATEY*1000.0f, IMU_RATEZ*1000.0f,
		IMU_MAGX*1000.0f, IMU_MAGY*1000.0f, IMU_MAGZ*1000.0f);
	mavlink_msg_scaled_pressure_send(MAVLINK_COMM_0, micros, AQ_PRESSURE*0.01f, 0.0f, IMU_TEMP*100);
	mavlinkData.streams[MAV_DATA_STREAM_RAW_SENSORS].next = micros + mavlinkData.streams[MAV_DATA_STREAM_RAW_SENSORS].interval;
    }
    // position -- gps and ukf
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_POSITION].enable && mavlinkData.streams[MAV_DATA_STREAM_POSITION].next < micros)) {
	mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, micros, navData.fixType, gpsData.lat*(double)1e7, gpsData.lon*(double)1e7, gpsData.height*1e3,
		gpsData.hAcc*100, gpsData.vAcc*100, gpsData.speed*100, gpsData.heading, 255);
	mavlink_msg_local_position_ned_send(MAVLINK_COMM_0, micros, UKF_POSN, UKF_POSE, ALTITUDE, UKF_VELN, UKF_VELE, -VELOCITYD);
	mavlinkData.streams[MAV_DATA_STREAM_POSITION].next = micros + mavlinkData.streams[MAV_DATA_STREAM_POSITION].interval;
    }
    // radio channels
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].enable && mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].next < micros)) {
	int16_t *rc;
	mavlinkData.indexPort++;
	if (radioData.mode == RADIO_MODE_SPLIT) {
	    rc = radioData.allChannels;
	    if (mavlinkData.indexPort > 4)
		mavlinkData.indexPort = 0;
	} else {
	    rc = radioData.channels;
	    if (mavlinkData.indexPort > 1)
		mavlinkData.indexPort = 0;
	}
	uint8_t ci = mavlinkData.indexPort * 8;
	mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0, micros, mavlinkData.indexPort, rc[ci]+1024, rc[ci+1]+1024, rc[ci+2]+1024, rc[ci+3]+1024, rc[ci+4]+1024, rc[ci+5]+1024, rc[ci+6]+1024, rc[ci+7]+1024, RADIO_QUALITY);
	mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].next = micros + mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].interval;
    }
    // raw controller -- attitude and nav data
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].enable && mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].next < micros)) {
	mavlink_msg_attitude_send(MAVLINK_COMM_0, micros, AQ_ROLL*DEG_TO_RAD, AQ_PITCH*DEG_TO_RAD, AQ_YAW*DEG_TO_RAD, -(IMU_RATEX - UKF_GYO_BIAS_X)*DEG_TO_RAD,
		(IMU_RATEY - UKF_GYO_BIAS_Y)*DEG_TO_RAD, (IMU_RATEZ - UKF_GYO_BIAS_Z)*DEG_TO_RAD);
	mavlink_msg_nav_controller_output_send(MAVLINK_COMM_0, navData.holdTiltE, navData.holdTiltN, navData.holdHeading*100, navData.holdCourse*100, navData.holdDistance*100, navData.holdAlt, 0, 0);

	mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].next = micros + mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].interval;
    }
#ifdef MAVLINK_MSG_ID_AQ_ESC_TELEMETRY
    // ESC/Motor telemetry
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_PROPULSION].enable && mavlinkData.streams[MAV_DATA_STREAM_PROPULSION].next < micros)) {
	uint8_t id, i, m, s;
	uint8_t mId[4], dataVer[4];
	uint16_t statAge[4];
	uint32_t data[2][4];
	uint32_t ms = micros / 1000;
	memset(data, 0, sizeof(data));
	i = m = s = 0;
	for (; i < motorsData.numActive; ++i) {
	    id = motorsData.activeList[i];
	    mId[m] = id + 1;
	    dataVer[m] = (uint8_t)(motorsData.esc32Version[id] / 10);
	    if (!motorsData.canTelemReqTime[id] || micros - motorsData.canStatusTime[id] > 3e6)
		statAge[m] = 0xffff;
	    else {
		statAge[m] = ms - (motorsData.canStatusTime[id] / 1000);
		memcpy(&data[0][m], &motorsData.canStatus[id], sizeof(uint32_t));
		memcpy(&data[1][m], (uint32_t *)&motorsData.canStatus[id] + 1, sizeof(uint32_t));
	    }
	    if (++m == 4 || i == motorsData.numActive - 1) {
		mavlink_msg_aq_esc_telemetry_send(MAVLINK_COMM_0, ms, ++s, motorsData.numActive, m, mId, statAge, dataVer, data[0], data[1]);
		memset(data, 0, sizeof(data));
		m = 0;
	    }
	}
	mavlinkData.streams[MAV_DATA_STREAM_PROPULSION].next = micros + mavlinkData.streams[MAV_DATA_STREAM_PROPULSION].interval;
    }
#endif
    // EXTRA3 stream -- AQ custom telemetry
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].enable && mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].next < micros)) {
	for (uint8_t i=0; i < AQMAV_DATASET_ENUM_END; ++i) {
	    if (!mavlinkData.customDatasets[i])
		continue;
	    switch(i) {
	    case AQMAV_DATASET_GPS :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, gpsData.pDOP, gpsData.tDOP, gpsData.sAcc, gpsData.cAcc, gpsData.velN, gpsData.velE, gpsData.velD,
			micros - gpsData.lastPosUpdate, micros - gpsData.lastMessage, gpsData.vAcc, gpsData.lat, gpsData.lon, gpsData.hAcc, gpsData.heading, gpsData.height, gpsData.iTOW, 0,0,0,0);
		break;
	    case AQMAV_DATASET_UKF :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, UKF_GYO_BIAS_X, UKF_GYO_BIAS_Y, UKF_GYO_BIAS_Z, UKF_ACC_BIAS_X, UKF_ACC_BIAS_Y, UKF_ACC_BIAS_Z, UKF_Q1, UKF_Q2, UKF_Q3, UKF_Q4,
			UKF_ALTITUDE, UKF_POSN, UKF_POSE, UKF_POSD, UKF_VELN, UKF_VELE, UKF_VELD, UKF_PRES_ALT, ALT_POS, ALT_VEL);
		break;
	    case AQMAV_DATASET_SUPERVISOR :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, supervisorData.state, supervisorData.flightTime, 0, 0,
			supervisorData.vInLPF, 0, supervisorData.lastGoodRadioMicros, supervisorData.idlePercent, 0,0,0,0,0,0,0, micros - RADIO_LAST_UPDATE, commData.txBufStarved, analogData.vIn, RADIO_QUALITY, RADIO_ERROR_COUNT);
		break;
	    case AQMAV_DATASET_STACKSFREE :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, utilGetStackFree("INIT"), utilGetStackFree("FILER"), utilGetStackFree("SUPERVISOR"), utilGetStackFree("ADC"),
			utilGetStackFree("RADIO"), utilGetStackFree("CONTROL"), utilGetStackFree("GPS"), utilGetStackFree("RUN"), utilGetStackFree("COMM"), utilGetStackFree("DIMU"),
			utilGetStackFree("CYRF"), 0,0,0,0,0,0, heapUsed, heapHighWater, dataSramUsed * 4);
		break;
	    case AQMAV_DATASET_GIMBAL :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, *gimbalData.pitchPort->ccr, *gimbalData.tiltPort->ccr, *gimbalData.rollPort->ccr, *gimbalData.triggerPort->ccr,
			*gimbalData.passthroughPort->ccr, gimbalData.tilt, gimbalData.trigger, gimbalData.triggerLastTime, gimbalData.triggerLastLat, gimbalData.triggerLastLon,
			gimbalData.triggerCount, 0,0,0,0,0,0,0,0,0);
		break;
	    case AQMAV_DATASET_MOTORS :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i,
			motorsData.value[0], motorsData.value[1], motorsData.value[2], motorsData.value[3], motorsData.value[4], motorsData.value[5], motorsData.value[6], motorsData.value[7],
			motorsData.value[8], motorsData.value[9], motorsData.value[10], motorsData.value[11], motorsData.value[12], motorsData.value[13], motorsData.value[14], motorsData.value[15],
			motorsData.throttle, motorsData.pitch, motorsData.roll, motorsData.yaw);
		break;
	    case AQMAV_DATASET_MOTORS_PWM :
	    {
		uint32_t pwm[14];
		for (int i = 0; i < 14; ++i)
		    pwm[i] = PWM_NUM_PORTS > i && motorsData.pwm[i] ? *motorsData.pwm[i]->ccr : 0;
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, pwm[0], pwm[1], pwm[2], pwm[3], pwm[4], pwm[5], pwm[6], pwm[7], pwm[8], pwm[9], pwm[10], pwm[11], pwm[12], pwm[13],
			0, 0, 0, 0, 0, 0);
		break;
	    }
	    case AQMAV_DATASET_NAV :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, navData.holdHeading, navData.holdCourse, navData.holdDistance, navData.holdAlt, navData.holdTiltN, navData.holdTiltE,
			navData.holdSpeedN, navData.holdSpeedE, navData.holdSpeedAlt, navData.targetHoldSpeedAlt, navData.presAltOffset, 0, 0, 0, 0, 0, 0, 0, micros - navData.lastUpdate, navData.fixType);
		break;
	    case AQMAV_DATASET_IMU :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, AQ_ROLL, AQ_PITCH, AQ_YAW, IMU_RATEX, IMU_RATEY, IMU_RATEZ, IMU_ACCX, IMU_ACCY, IMU_ACCZ, IMU_MAGX, IMU_MAGY, IMU_MAGZ,
			IMU_TEMP, micros - IMU_LASTUPD, 0, 0, 0, 0, 0, AQ_PRESSURE);
		break;
	    case AQMAV_DATASET_RC :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, RADIO_THROT, RADIO_RUDD, RADIO_PITCH, RADIO_ROLL, rcIsSwitchActive(NAV_CTRL_AH), rcIsSwitchActive(NAV_CTRL_PH),
			rcIsSwitchActive(NAV_CTRL_MISN), rcIsSwitchActive(NAV_CTRL_HOM_SET), rcIsSwitchActive(NAV_CTRL_HOM_GO), rcIsSwitchActive(NAV_CTRL_HF_SET), rcIsSwitchActive(NAV_CTRL_HF_LOCK),
			rcIsSwitchActive(NAV_CTRL_WP_REC), 0, 0, 0, 0, 0,
			rcIsSwitchActive(GMBL_CTRL_TRG_ON), rcGetControlValue(GMBL_CTRL_TILT), rcGetControlValue(GMBL_PSTHR_CHAN));
		break;
	    case AQMAV_DATASET_CONFIG :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i,
			configGetAdjParamId(CONFIG_ADJUST_P1), (configGetAdjParamId(CONFIG_ADJUST_P1) ? configGetParamValue(configGetAdjParamId(CONFIG_ADJUST_P1)) : 0),
			configGetAdjParamId(CONFIG_ADJUST_P2), (configGetAdjParamId(CONFIG_ADJUST_P2) ? configGetParamValue(configGetAdjParamId(CONFIG_ADJUST_P2)) : 0),
			configGetAdjParamId(CONFIG_ADJUST_P3), (configGetAdjParamId(CONFIG_ADJUST_P3) ? configGetParamValue(configGetAdjParamId(CONFIG_ADJUST_P3)) : 0),
			configGetAdjParamId(CONFIG_ADJUST_P4), (configGetAdjParamId(CONFIG_ADJUST_P4) ? configGetParamValue(configGetAdjParamId(CONFIG_ADJUST_P4)) : 0),
			configGetAdjParamId(CONFIG_ADJUST_P5), (configGetAdjParamId(CONFIG_ADJUST_P5) ? configGetParamValue(configGetAdjParamId(CONFIG_ADJUST_P5)) : 0),
			configGetAdjParamId(CONFIG_ADJUST_P6), (configGetAdjParamId(CONFIG_ADJUST_P6) ? configGetParamValue(configGetAdjParamId(CONFIG_ADJUST_P6)) : 0),
			0, 0, 0, 0, 0, 0, 0, 0);
		break;
	    case AQMAV_DATASET_DEBUG :
		// First 10 values are displayed in QGC as integers, the other 10 as floats.
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i,
			/* ints */ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
			/*floats*/ 0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
		break;
	    }
	}

	mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].next = micros + mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].interval;
    }
    // end streams

    // list all requested/remaining parameters
    if (mavlinkData.currentParam < CONFIG_NUM_PARAMS && mavlinkData.nextParam < micros) {
	uint16_t totalParams = CONFIG_NUM_PARAMS;

	// check if only adjustable params are being requested
	if (mavlinkData.paramCompId == MAV_COMP_ID_SYSTEM_CONTROL) {
	    totalParams = configGetNumPossibleAdjParams();
	    int16_t t = configGetNextAdjustableParam(mavlinkData.currentParam);
	    if (t > -1)
		mavlinkData.currentParam = t;
	    else {
		mavlinkData.currentParam = CONFIG_NUM_PARAMS;
		return;
	    }
	}
	mavlinkSendParamValue(mavlinkData.currentParam, totalParams, false);

	++mavlinkData.currentParam;

	mavlinkData.nextParam = micros + AQMAVLINK_PARAM_INTERVAL;
    }

    // request announced waypoints from mission planner
    if (mavlinkData.wpCurrent < mavlinkData.wpCount && mavlinkData.wpAttempt <= AQMAVLINK_WP_MAX_ATTEMPTS && mavlinkData.nextWP < micros) {
	mavlinkData.wpAttempt++;
	mavlink_msg_mission_request_send(MAVLINK_COMM_0, mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, mavlinkData.wpCurrent);
	mavlinkData.nextWP = micros + AQMAVLINK_WP_TIMEOUT;
    }
    // or ack that last waypoint received
    else if (mavlinkData.wpCurrent == mavlinkData.wpCount) {
	mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, 0);
	mavlinkData.wpCurrent++;
	AQ_PRINTF("%u waypoints loaded.", mavlinkData.wpCount);
    } else if (mavlinkData.wpAttempt > AQMAVLINK_WP_MAX_ATTEMPTS) {
	mavlinkData.wpCurrent = mavlinkData.wpCount + 1;
	AQ_NOTICE("Error: Waypoint request timeout!");
    }

    supervisorSendDataStop();

    lastMicros = micros;
}

void mavlinkDoCommand(mavlink_message_t *msg) {
    uint16_t command;
    float param, param2, param3;
    uint8_t enable, i, compId,
	ack = MAV_CMD_ACK_ERR_NOT_SUPPORTED;

    command = mavlink_msg_command_long_get_command(msg);
    compId = mavlink_msg_command_long_get_target_component(msg);

    switch (command) {

	case MAV_CMD_PREFLIGHT_CALIBRATION:
	    param = mavlink_msg_command_long_get_param2(msg);  // MAG
	    param2 = mavlink_msg_command_long_get_param5(msg); // ACC
	    if (param && supervisorRequestConfigAction(SPVR_ACT_REQ_CALIB_MAG))
		ack = MAV_CMD_ACK_OK;
	    else if (param2 && supervisorRequestConfigAction(SPVR_ACT_REQ_CALIB_ACC))
		ack = MAV_CMD_ACK_OK;
	    else
		ack = MAV_CMD_ACK_ERR_FAIL;
	    break;  // case MAV_CMD_PREFLIGHT_CALIBRATION

	case MAV_CMD_PREFLIGHT_STORAGE:
	    param = mavlink_msg_command_long_get_param1(msg);
	    switch (compId) {
		// IMU calibration parameters
		case MAV_COMP_ID_IMU:
		    if (param == 0.0f && supervisorRequestConfigAction(SPVR_ACT_REQ_DIMU_CFG_READ))
			ack = MAV_CMD_ACK_OK;
		    else if (param == 1.0f && supervisorRequestConfigAction(SPVR_ACT_REQ_DIMU_CFG_WRITE))
			ack = MAV_CMD_ACK_OK;
		    else
			ack = MAV_CMD_ACK_ERR_FAIL;

		    break;

		// main parameters
		default:
		    if (param == 0.0f && supervisorRequestConfigAction(SPVR_ACT_REQ_CFG_READ_FLASH))
			ack = MAV_CMD_ACK_OK;
		    else if (param == 1.0f && supervisorRequestConfigAction(SPVR_ACT_REQ_CFG_WRITE_FLASH))
			ack = MAV_CMD_ACK_OK;
		    else if (param == 2.0f && supervisorRequestConfigAction(SPVR_ACT_REQ_CFG_READ_FILE))
			ack = MAV_CMD_ACK_OK;
		    else if (param == 3.0f && supervisorRequestConfigAction(SPVR_ACT_REQ_CFG_WRITE_FILE))
			ack = MAV_CMD_ACK_OK;
		    else if (param == 4.0f && supervisorRequestConfigAction(SPVR_ACT_REQ_CFG_DEFAULTS))
			ack = MAV_CMD_ACK_OK;
		    else
			ack = MAV_CMD_ACK_ERR_FAIL;

		    break;
	    } // switch(component ID)

	    break; // case MAV_CMD_PREFLIGHT_STORAGE

	// remote reboot
	case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
	    param = mavlink_msg_command_long_get_param1(msg);	// flight controller actions: 0=noOp; 1=reboot; 2=shutdown
	    if (param == 1.0f) {
		if (supervisorRequestConfigAction(SPVR_ACT_REQ_SYSTEM_RESET))
		    ack = MAV_CMD_ACK_OK;
		else
		    ack = MAV_CMD_ACK_ERR_FAIL;
	    } else {
		ack = MAV_CMD_ACK_ERR_FAIL;
		AQ_NOTICE("Error: Shutdown and computer actions not supported.");
	    }

	    break;  // case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN

	// enable/disable AQ custom dataset messages and set frequency
	case MAV_CMD_AQ_TELEMETRY:
	    param = mavlink_msg_command_long_get_param1(msg);	// start/stop
	    param2 = mavlink_msg_command_long_get_param2(msg);	// interval in us
	    param3 = mavlink_msg_command_long_get_param3(msg);	// dataset id

	    if (param3 >= AQMAV_DATASET_ALL && param3 < AQMAV_DATASET_ENUM_END) {
		enable = (uint8_t)param;

		// enable/disable this dataset
		mavlinkData.customDatasets[(uint8_t)param3] = enable;

		// check if any datasets are active and enable/disable EXTRA3 stream accordingly
		// AQMAV_DATASET_ALL is special and toggles all datasets
		if (!enable && param3 != AQMAV_DATASET_ALL) {
		    for (i=0; i < AQMAV_DATASET_ENUM_END; ++i)
			if ((enable = mavlinkData.customDatasets[i]) > 0)
			    break;
		}

		// enable/disable EXTRA3 stream
		mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].enable = enable;
		// set stream interval (note that specified interval currently affects all custom datasets in the EXTRA3 stream)
		if (param2)
		    mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].interval = (unsigned long)param2;

		ack = MAV_CMD_ACK_OK;
		mavlink_msg_data_stream_send(MAVLINK_COMM_0, MAV_DATA_STREAM_EXTRA3, 1e6 / (unsigned long)param2, enable);

	    } else {
		ack = MAV_CMD_ACK_ERR_FAIL;
		AQ_NOTICE("Error: Unknown telemetry dataset requested.");
	    }

	    break; // case MAV_CMD_AQ_TELEMETRY

	// send firmware version number;
	case MAV_CMD_AQ_REQUEST_VERSION:
#ifdef MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES
	case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES:
#endif
	    utilVersionString();
	    utilSerialNoString();
	    if (USE_QUATOS)
		AQ_NOTICE("Quatos enabled.");
	    ack = MAV_CMD_ACK_OK;
	    break;

	default:
	    AQ_PRINTF("Error: Unrecognized command: %u", command);
	    break;

    } // switch(command)

    mavlink_msg_command_ack_send(MAVLINK_COMM_0, command, ack);
}

void mavlinkRecvTaskCode(commRcvrStruct_t *r) {
    mavlink_message_t msg;
    uint8_t c, ack;

    // process incoming data
    while (commAvailable(r)) {
	c = commReadChar(r);
	// Try to get a new message
	if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &mavlinkData.mavlinkStatus)) {
	    // Handle message
	    switch(msg.msgid) {
		// TODO: finish this block
		case MAVLINK_MSG_ID_COMMAND_LONG:
		    if (mavlink_msg_command_long_get_target_system(&msg) == mavlink_system.sysid) {
			mavlinkDoCommand(&msg);
		    }
		    break;

		case MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL:
		    if (mavlink_msg_change_operator_control_get_target_system(&msg) == mavlink_system.sysid) {
			mavlink_msg_change_operator_control_ack_send(MAVLINK_COMM_0, msg.sysid, mavlink_msg_change_operator_control_get_control_request(&msg), 0);
		    }
		    break;

//		case MAVLINK_MSG_ID_SET_MODE:
//		    if (mavlink_msg_set_mode_get_target_system(&msg) == mavlink_system.sysid) {
//			mavlinkData.sys_mode = mavlink_msg_set_mode_get_base_mode(&msg);
//			mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, 1000-mavlinkData.idlePercent, analogData.vIn * 1000, -1, (analogData.vIn - 9.8f) / 12.6f * 1000, 0, mavlinkData.packetDrops, 0, 0, 0, 0);
//		    }
//		    break;

		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
		    if (mavlink_msg_mission_request_list_get_target_system(&msg) == mavlink_system.sysid)
			mavlinkWpSendCount();
		    break;

		case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
		    if (mavlink_msg_mission_clear_all_get_target_system(&msg) == mavlink_system.sysid) {
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, (navClearWaypoints() ? MAV_MISSION_ACCEPTED : MAV_MISSION_ERROR));
		    }
		    break;

		case MAVLINK_MSG_ID_MISSION_REQUEST:
		    if (mavlink_msg_mission_request_get_target_system(&msg) == mavlink_system.sysid) {
			uint16_t seqId;
			uint16_t mavFrame;
			navMission_t *wp;

			seqId = mavlink_msg_mission_request_get_seq(&msg);
			wp = navGetWaypoint(seqId);
			if (wp->relativeAlt == 1)
			    mavFrame = MAV_FRAME_GLOBAL_RELATIVE_ALT;
			else
			    mavFrame = MAV_FRAME_GLOBAL;

			if (wp->type == NAV_LEG_HOME) {
			    wp = navGetHomeWaypoint();

			    mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg.sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, MAV_FRAME_GLOBAL, MAV_CMD_NAV_RETURN_TO_LAUNCH, (navData.missionLeg == seqId) ? 1 : 0, 1,
				wp->targetRadius, wp->loiterTime/1000, 0.0f, wp->poiHeading, wp->targetLat, wp->targetLon, wp->targetAlt);
			}
			else if (wp->type == NAV_LEG_GOTO) {
			    mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg.sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, mavFrame, MAV_CMD_NAV_WAYPOINT, (navData.missionLeg == seqId) ? 1 : 0, 1,
				wp->targetRadius, wp->loiterTime/1000, wp->maxHorizSpeed, wp->poiHeading, wp->targetLat, wp->targetLon, wp->targetAlt);
			}
			else if (wp->type == NAV_LEG_TAKEOFF) {
			    mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg.sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, mavFrame, MAV_CMD_NAV_TAKEOFF, (navData.missionLeg == seqId) ? 1 : 0, 1,
				wp->targetRadius, wp->loiterTime/1000, wp->poiHeading, wp->maxVertSpeed, wp->targetLat, wp->targetLon, wp->targetAlt);
			}
			else if (wp->type == NAV_LEG_ORBIT) {
			    mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg.sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, mavFrame, 1, (navData.missionLeg == seqId) ? 1 : 0, 1,
				wp->targetRadius, wp->loiterTime/1000, wp->maxHorizSpeed, wp->poiHeading, wp->targetLat, wp->targetLon, wp->targetAlt);
			}
			else if (wp->type == NAV_LEG_LAND) {
			    mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg.sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, mavFrame, MAV_CMD_NAV_LAND, (navData.missionLeg == seqId) ? 1 : 0, 1,
				0.0f, wp->maxVertSpeed, wp->maxHorizSpeed, wp->poiAltitude, wp->targetLat, wp->targetLon, wp->targetAlt);
			}
			else {
			    mavlink_msg_mission_item_send(MAVLINK_COMM_0, msg.sysid, MAV_COMP_ID_MISSIONPLANNER,
				seqId, mavFrame, MAV_CMD_NAV_WAYPOINT, (navData.missionLeg == seqId) ? 1 : 0, 1,
				wp->targetRadius, wp->loiterTime/1000, 0.0f, wp->poiHeading, wp->targetLat, wp->targetLon, wp->targetAlt);
			}
		    }
		    break; // MAVLINK_MSG_ID_MISSION_REQUEST

		case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
		    if (mavlink_msg_mission_count_get_target_system(&msg) == mavlink_system.sysid) {
			uint16_t seqId;
			ack = MAV_MISSION_ACCEPTED;

			seqId = mavlink_msg_mission_set_current_get_seq(&msg);
			if (seqId < NAV_MAX_MISSION_LEGS && navData.missionLegs[seqId].type)
			    navLoadLeg(seqId);
			else
			    ack = MAV_MISSION_INVALID_SEQUENCE;

			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, mavlink_system.compid, ack);
		    }
		    break;

		case MAVLINK_MSG_ID_MISSION_COUNT:
		    if (mavlink_msg_mission_count_get_target_system(&msg) == mavlink_system.sysid) {
			uint8_t count;

			count = mavlink_msg_mission_count_get_count(&msg);
			if (count > NAV_MAX_MISSION_LEGS || navClearWaypoints() != 1) {
			    // NACK
			    ack = MAV_MISSION_NO_SPACE;
			    AQ_PRINTF("Error: %u waypoints exceeds system maximum of %u.", count, NAV_MAX_MISSION_LEGS);
			}
			else {
			    mavlinkData.wpTargetSysId = msg.sysid;
			    mavlinkData.wpTargetCompId = msg.compid;
			    mavlinkData.wpCount = count;
			    mavlinkData.wpCurrent = mavlinkData.wpAttempt = 0;
			    mavlinkData.nextWP = timerMicros();
			    ack = MAV_MISSION_ACCEPTED;
			}

			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, mavlink_system.compid, ack);
		    }
		    break;

		case MAVLINK_MSG_ID_MISSION_ITEM:
		    if (mavlink_msg_mission_item_get_target_system(&msg) == mavlink_system.sysid) {
			uint16_t seqId;
			ack = MAV_MISSION_ACCEPTED;
			uint8_t frame;

			seqId = mavlink_msg_mission_item_get_seq(&msg);
			frame = mavlink_msg_mission_item_get_frame(&msg);

			if (seqId >= NAV_MAX_MISSION_LEGS) {
			    ack = MAV_MISSION_INVALID_SEQUENCE;
			}
			else if (frame != MAV_FRAME_GLOBAL && frame != MAV_FRAME_GLOBAL_RELATIVE_ALT) {
			    ack = MAV_MISSION_INVALID;
			}
			else {
			    navMission_t *wp;
			    uint8_t command;

			    command = mavlink_msg_mission_item_get_command(&msg);
			    if (command == MAV_CMD_NAV_RETURN_TO_LAUNCH) {
				wp = navGetWaypoint(seqId);

				wp->type = NAV_LEG_HOME;

				wp = navGetHomeWaypoint();

				wp->type = NAV_LEG_GOTO;
				wp->targetLat = mavlink_msg_mission_item_get_x(&msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(&msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(&msg);
				wp->targetRadius = mavlink_msg_mission_item_get_param1(&msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param2(&msg) * 1000;
				wp->poiHeading = mavlink_msg_mission_item_get_param4(&msg);
				wp->maxHorizSpeed = NAV_DFLT_HOR_SPEED;
			    }
			    else if (command == MAV_CMD_NAV_WAYPOINT) {
				wp = navGetWaypoint(seqId);
				if (frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
				    wp->relativeAlt = 1;
				else
				    wp->relativeAlt = 0;

				wp->type = NAV_LEG_GOTO;
				wp->targetLat = mavlink_msg_mission_item_get_x(&msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(&msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(&msg);
				wp->targetRadius = mavlink_msg_mission_item_get_param1(&msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param2(&msg) * 1000;
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param3(&msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(&msg);
			    }
			    else if (command == MAV_CMD_DO_SET_HOME) {
				// use current location
				if (mavlink_msg_mission_item_get_current(&msg)) {
				    navSetHomeCurrent();
				}
				// use given location
				else {
				    wp = navGetHomeWaypoint();

				    wp->type = NAV_LEG_GOTO;
				    wp->targetLat = mavlink_msg_mission_item_get_x(&msg);
				    wp->targetLon = mavlink_msg_mission_item_get_y(&msg);
				    wp->targetAlt = mavlink_msg_mission_item_get_z(&msg);
				    wp->targetRadius = mavlink_msg_mission_item_get_param1(&msg);
				    wp->loiterTime = mavlink_msg_mission_item_get_param2(&msg) * 1000;
				    wp->poiHeading = mavlink_msg_mission_item_get_param4(&msg);
				    wp->maxHorizSpeed = NAV_DFLT_HOR_SPEED;
				}
			    }
			    else if (command == MAV_CMD_NAV_TAKEOFF) {
				wp = navGetWaypoint(seqId);
				if (frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
				    wp->relativeAlt = 1;
				else
				    wp->relativeAlt = 0;

				wp->type = NAV_LEG_TAKEOFF;
				wp->targetLat = mavlink_msg_mission_item_get_x(&msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(&msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(&msg);
				wp->targetRadius = mavlink_msg_mission_item_get_param1(&msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param2(&msg) * 1000;
				wp->poiHeading = mavlink_msg_mission_item_get_param3(&msg);
				wp->maxVertSpeed = mavlink_msg_mission_item_get_param4(&msg);
			    }
			    else if (command == MAV_CMD_AQ_NAV_LEG_ORBIT) {
				wp = navGetWaypoint(seqId);
				if (frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
				    wp->relativeAlt = 1;
				else
				    wp->relativeAlt = 0;

				wp->type = NAV_LEG_ORBIT;
				wp->targetLat = mavlink_msg_mission_item_get_x(&msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(&msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(&msg);
				wp->targetRadius = mavlink_msg_mission_item_get_param1(&msg);
				wp->loiterTime = mavlink_msg_mission_item_get_param2(&msg) * 1000;
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param3(&msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(&msg);
			    }
			    else if (command == MAV_CMD_NAV_LAND) {
				wp = navGetWaypoint(seqId);
				if (frame == MAV_FRAME_GLOBAL_RELATIVE_ALT)
				    wp->relativeAlt = 1;
				else
				    wp->relativeAlt = 0;

				wp->type = NAV_LEG_LAND;
				wp->targetLat = mavlink_msg_mission_item_get_x(&msg);
				wp->targetLon = mavlink_msg_mission_item_get_y(&msg);
				wp->targetAlt = mavlink_msg_mission_item_get_z(&msg);
				wp->maxVertSpeed = mavlink_msg_mission_item_get_param2(&msg);
				wp->maxHorizSpeed = mavlink_msg_mission_item_get_param3(&msg);
				wp->poiHeading = mavlink_msg_mission_item_get_param4(&msg);
			    }
			    else {
				// NACK
				ack = MAV_MISSION_UNSUPPORTED;
			    }
			}

			mavlinkData.wpCurrent = seqId + 1;
			mavlinkData.wpAttempt = 0;
			mavlinkData.nextWP = timerMicros();
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, mavlink_system.compid, ack);
		    }
		    break; //MAVLINK_MSG_ID_MISSION_ITEM

		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
		    if (mavlink_msg_param_request_read_get_target_system(&msg) == mavlink_system.sysid) {
			uint16_t paramIndex;

			mavlinkData.paramCompId = mavlink_msg_param_request_read_get_target_component(&msg);
			paramIndex = mavlink_msg_param_request_read_get_param_index(&msg);
			if (paramIndex < CONFIG_NUM_PARAMS)
			    mavlinkSendParamValue(paramIndex, CONFIG_NUM_PARAMS, false);
		    }
		    break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		    if (mavlink_msg_param_request_list_get_target_system(&msg) == mavlink_system.sysid) {
			mavlinkData.paramCompId = mavlink_msg_param_request_list_get_target_component(&msg);
			mavlinkData.currentParam = 0;
			mavlinkData.nextParam = 0;
		    }
		    break;

		case MAVLINK_MSG_ID_PARAM_SET:
		    if (mavlink_msg_param_set_get_target_system(&msg) == mavlink_system.sysid) {
			int paramIndex;
			char paramName[17];

			mavlink_msg_param_set_get_param_id(&msg, paramName);
			paramIndex = configGetParamIdByName((char *)paramName);
			if (paramIndex > -1) {
			    if (!(supervisorData.state & STATE_FLYING))
				configSetParamByID(paramIndex, mavlink_msg_param_set_get_param_value(&msg));
			    // send back what we have no matter what
			    mavlinkData.paramCompId = mavlink_msg_param_set_get_target_component(&msg);
			    mavlinkSendParamValue(paramIndex, CONFIG_NUM_PARAMS, true);
			}
		    }
		    break;

		case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
		    if (mavlink_msg_request_data_stream_get_target_system(&msg) == mavlink_system.sysid) {
			uint16_t rate;
			uint8_t stream_id, enable;

			stream_id = mavlink_msg_request_data_stream_get_req_stream_id(&msg);
			rate = mavlink_msg_request_data_stream_get_req_message_rate(&msg);
			rate = constrainInt(rate, 0, 200);
			enable = mavlink_msg_request_data_stream_get_start_stop(&msg);

			// STREAM_ALL is special:
			// to disable all streams entirely (except heartbeat), set STREAM_ALL start = 0
			// to enable literally all streams at a certain rate, set STREAM_ALL rate > 0 and start = 1;
			// set rate = 0 and start = 1 to disable sending all data and return back to your regularly scheduled streams :)
			if (stream_id == MAV_DATA_STREAM_ALL)
			    mavlinkToggleStreams(enable);

			if (stream_id < AQMAVLINK_TOTAL_STREAMS) {
			    mavlinkData.streams[stream_id].enable = rate && enable;
			    mavlinkData.streams[stream_id].interval = rate ? 1e6 / rate : 0;
			    mavlink_msg_data_stream_send(MAVLINK_COMM_0, stream_id, rate, mavlinkData.streams[stream_id].enable);
			}
		    }
		    break;

		case MAVLINK_MSG_ID_OPTICAL_FLOW:
		    navUkfOpticalFlow(mavlink_msg_optical_flow_get_flow_x(&msg),
					mavlink_msg_optical_flow_get_flow_y(&msg),
					mavlink_msg_optical_flow_get_quality(&msg),
					mavlink_msg_optical_flow_get_ground_distance(&msg));
		    break;

		default:
		    // Do nothing
		    break;
	    }
	}

	// Update global packet drops counter
	mavlinkData.packetDrops += mavlinkData.mavlinkStatus.packet_rx_drop_count;
    }
}

static void mavlinkSetSystemType(void) {
    uint8_t motCount = 0;

    // get motor count
    motorsPowerStruct_t *d = (motorsPowerStruct_t *)&p[MOT_PWRD_01_T];
    for (uint8_t i = 0; i < MOTORS_NUM; ++i)
	if (d[i].throttle != 0.0f)
	    motCount++;

    switch (motCount) {
    case 3:
	mavlinkData.sys_type = MAV_TYPE_TRICOPTER;
	break;
    case 4:
	mavlinkData.sys_type = MAV_TYPE_QUADROTOR;
	break;
    case 6:
	mavlinkData.sys_type = MAV_TYPE_HEXAROTOR;
	break;
    case 8:
	mavlinkData.sys_type = MAV_TYPE_OCTOROTOR;
	break;
    default:
	mavlinkData.sys_type = MAV_TYPE_GENERIC;
	break;
    }
}

void mavlinkSendParameter(uint8_t sysId, uint8_t compId, const char *paramName, float value) {
    mavlink_msg_param_set_send(MAVLINK_COMM_0, sysId, compId, paramName, value, MAV_PARAM_TYPE_REAL32);
}

void mavlinkInit(void) {
    unsigned long micros, hz;
    int i;

    memset((void *)&mavlinkData, 0, sizeof(mavlinkData));

    // register notice function with comm module
    commRegisterNoticeFunc(mavlinkSendNotice);
    commRegisterTelemFunc(mavlinkDo);
    commRegisterRcvrFunc(COMM_STREAM_TYPE_MAVLINK, mavlinkRecvTaskCode);

    AQ_NOTICE("Mavlink init\n");

    mavlinkData.currentParam = CONFIG_NUM_PARAMS;
    mavlinkData.wpCount = navGetWaypointCount();
    mavlinkData.wpCurrent = mavlinkData.wpCount + 1;
    mavlinkData.wpTargetCompId = MAV_COMP_ID_MISSIONPLANNER;
    mavlinkData.paramCompId = MAV_COMP_ID_ALL;
    mavlinkData.sys_mode = MAV_MODE_PREFLIGHT;
    mavlinkData.sys_state = MAV_STATE_BOOT;
    mavlink_system.sysid = flashSerno(0) % 250;
    mavlink_system.compid = AQMAVLINK_DEFAULT_COMP_ID;
    mavlinkSetSystemType();

    mavlinkData.streams[MAV_DATA_STREAM_ALL].dfltInterval = AQMAVLINK_STREAM_RATE_ALL;
    mavlinkData.streams[MAV_DATA_STREAM_RAW_SENSORS].dfltInterval = AQMAVLINK_STREAM_RATE_RAW_SENSORS;
    mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].dfltInterval = AQMAVLINK_STREAM_RATE_EXTENDED_STATUS;
    mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].dfltInterval = AQMAVLINK_STREAM_RATE_RC_CHANNELS;
    mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].dfltInterval = AQMAVLINK_STREAM_RATE_RAW_CONTROLLER;
    mavlinkData.streams[MAV_DATA_STREAM_POSITION].dfltInterval = AQMAVLINK_STREAM_RATE_POSITION;
    mavlinkData.streams[MAV_DATA_STREAM_EXTRA1].dfltInterval = AQMAVLINK_STREAM_RATE_EXTRA1;
    mavlinkData.streams[MAV_DATA_STREAM_EXTRA2].dfltInterval = AQMAVLINK_STREAM_RATE_EXTRA2;
    mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].dfltInterval = AQMAVLINK_STREAM_RATE_EXTRA3;
    mavlinkData.streams[MAV_DATA_STREAM_PROPULSION].dfltInterval = AQMAVLINK_STREAM_RATE_PROPULSION;

    // turn on streams & spread them out
    micros = timerMicros();
    for (i = 0; i < AQMAVLINK_TOTAL_STREAMS; i++) {
	mavlinkData.streams[i].interval = mavlinkData.streams[i].dfltInterval;
	mavlinkData.streams[i].next = micros + 5e6f + i * 5e3f;
	mavlinkData.streams[i].enable = mavlinkData.streams[i].interval ? 1 : 0;
	hz = mavlinkData.streams[i].interval ? 1e6 / mavlinkData.streams[i].interval : 0;
	mavlink_msg_data_stream_send(MAVLINK_COMM_0, i, hz, mavlinkData.streams[i].enable);
    }
}

#endif	// USE_MAVLINK
