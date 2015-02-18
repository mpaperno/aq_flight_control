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

#include "aq.h"
#ifdef USE_MAVLINK
#include "aq_mavlink.h"
#include "mavlink.h"
#include "config.h"
#include "aq_timer.h"
#include "imu.h"
#include "radio.h"
#include "gps.h"
#include "flash.h"
#include "motors.h"
#include "control.h"
#include "nav.h"
#include "math.h"
#include "util.h"
#include "rcc.h"
#include "supervisor.h"
#include "nav_ukf.h"
#include "analog.h"
#include "comm.h"
#include "gimbal.h"
#include "d_imu.h"
#include "run.h"
#include <CoOS.h>
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
    mavlink_msg_statustext_send(MAVLINK_COMM_0, 0, (const char *)s);
}

void mavlinkToggleStreams(uint8_t enable) {
    for (uint8_t i = 0; i < AQMAVLINK_TOTAL_STREAMS; i++)
	mavlinkData.streams[i].enable = enable && mavlinkData.streams[i].interval;
}

void mavlinkSetSystemData(void) {
    mavlinkData.sys_state = MAV_STATE_STANDBY;
    mavlinkData.sys_mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    mavlinkData.sys_nav_mode = AQ_NAV_STATUS_STANDBY;

    if ((supervisorData.state & STATE_RADIO_LOSS1) || (supervisorData.state & STATE_LOW_BATTERY2)) {
	mavlinkData.sys_state =  MAV_STATE_CRITICAL;
    }
    else if (supervisorData.state & STATE_FLYING) {
	mavlinkData.sys_state = MAV_STATE_ACTIVE;
	mavlinkData.sys_nav_mode = AQ_NAV_STATUS_MANUAL;
    }

    switch(navData.mode) {
    case NAV_STATUS_ALTHOLD:
	mavlinkData.sys_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED;
	mavlinkData.sys_nav_mode = AQ_NAV_STATUS_ALTHOLD;
	break;

    case NAV_STATUS_POSHOLD:
	mavlinkData.sys_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
	mavlinkData.sys_nav_mode = AQ_NAV_STATUS_ALTHOLD | AQ_NAV_STATUS_POSHOLD;
	break;

    case NAV_STATUS_DVH:
	mavlinkData.sys_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_GUIDED_ENABLED;
	mavlinkData.sys_nav_mode = AQ_NAV_STATUS_ALTHOLD | AQ_NAV_STATUS_POSHOLD | AQ_NAV_STATUS_DVH;
	break;

    case NAV_STATUS_MISSION:
	mavlinkData.sys_mode |= MAV_MODE_FLAG_STABILIZE_ENABLED | MAV_MODE_FLAG_AUTO_ENABLED;
	mavlinkData.sys_nav_mode = AQ_NAV_STATUS_MISSION;
	break;
    }

    if ((supervisorData.state & STATE_RADIO_LOSS2))
	mavlinkData.sys_nav_mode |= AQ_NAV_STATUS_FAILSAFE;

    if (navData.headFreeMode == NAV_HEADFREE_DYNAMIC)
	mavlinkData.sys_nav_mode |= AQ_NAV_STATUS_HF_DYNAMIC;
    else if (navData.headFreeMode == NAV_HEADFREE_LOCKED)
	mavlinkData.sys_nav_mode |= AQ_NAV_STATUS_HF_LOCKED;

    if (navData.ceilingAlt) {
	mavlinkData.sys_nav_mode |= AQ_NAV_STATUS_CEILING;
	if (navData.setCeilingReached)
	    mavlinkData.sys_nav_mode |= AQ_NAV_STATUS_CEILING_REACHED;
    }

    if (supervisorData.state & STATE_ARMED)
	mavlinkData.sys_mode |= MAV_MODE_FLAG_SAFETY_ARMED;
}

void mavlinkDo(void) {
    static unsigned long mavCounter;
    static unsigned long lastMicros = 0;
    unsigned long micros, statusInterval;
    int8_t battRemainPct, streamAll;

    micros = timerMicros();

    // handle rollover
    if (micros < lastMicros) {
	mavlinkData.nextHeartbeat = 0;
	mavlinkData.nextParam = 0;
	for (uint8_t i=0; i < AQMAVLINK_TOTAL_STREAMS; ++i)
	    mavlinkData.streams[i].next = 0;
    }

    supervisorSendDataStart();

    // heartbeat
    if (mavlinkData.nextHeartbeat < micros) {
	mavlinkSetSystemData();
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0, mavlinkData.sys_type, MAV_AUTOPILOT_AUTOQUAD, mavlinkData.sys_mode, mavlinkData.sys_nav_mode, mavlinkData.sys_state);
	mavlinkData.nextHeartbeat = micros + AQMAVLINK_HEARTBEAT_INTERVAL;
    }

    // send streams

    // first check if "ALL" stream is requested
    if ((streamAll = mavlinkData.streams[MAV_DATA_STREAM_ALL].enable && mavlinkData.streams[MAV_DATA_STREAM_ALL].next < micros))
	mavlinkData.streams[MAV_DATA_STREAM_ALL].next = micros + mavlinkData.streams[MAV_DATA_STREAM_ALL].interval;

    // status
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].enable && mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].next < micros)) {
	// calculate idle time
	statusInterval = streamAll ? mavlinkData.streams[MAV_DATA_STREAM_ALL].interval : mavlinkData.streams[MAV_DATA_STREAM_EXTENDED_STATUS].interval;
	mavCounter = counter;
	mavlinkData.idlePercent = (mavCounter - mavlinkData.lastCounter) * minCycles * 1000.0f / ((float)statusInterval * rccClocks.SYSCLK_Frequency / 1e6f);
	mavlinkData.lastCounter = mavCounter;
	//calculate remaining battery % based on supervisor low batt stg 2 level
	battRemainPct = (analogData.vIn - p[SPVR_LOW_BAT2] * analogData.batCellCount) / ((4.2 - p[SPVR_LOW_BAT2]) * analogData.batCellCount) * 100;

	mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, 1000-mavlinkData.idlePercent, analogData.vIn * 1000, -1, battRemainPct, 0, mavlinkData.packetDrops, 0, 0, 0, 0);
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
	mavlink_msg_local_position_ned_send(MAVLINK_COMM_0, micros, UKF_POSN, UKF_POSE, -UKF_POSD, UKF_VELN, UKF_VELE, -VELOCITYD);
	mavlinkData.streams[MAV_DATA_STREAM_POSITION].next = micros + mavlinkData.streams[MAV_DATA_STREAM_POSITION].interval;
    }
    // rc channels and pwm outputs (would be nice to separate these)
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].enable && mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].next < micros)) {
	if (!mavlinkData.indexPort++) {
	    mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0, micros, 0, RADIO_THROT+1024, RADIO_ROLL+1024, RADIO_PITCH+1024, RADIO_RUDD+1024,
		    RADIO_GEAR+1024, RADIO_FLAPS+1024, RADIO_AUX2+1024, RADIO_AUX3+1024, RADIO_QUALITY);
	    mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, micros, 0, motorsData.value[0], motorsData.value[1], motorsData.value[2], motorsData.value[3],
		    motorsData.value[4], motorsData.value[5], motorsData.value[6], motorsData.value[7]);
	} else {
	    mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0, micros, 1, RADIO_AUX4+1024, RADIO_AUX5+1024, RADIO_AUX6+1024, RADIO_AUX7+1024,
		    radioData.channels[12]+1024, radioData.channels[13]+1024, radioData.channels[14]+1024, radioData.channels[15]+1024, RADIO_QUALITY);
	    mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, micros, 1, motorsData.value[8], motorsData.value[9], motorsData.value[10], motorsData.value[11],
		    motorsData.value[12], motorsData.value[13], motorsData.value[14], motorsData.value[15]);
	    mavlinkData.indexPort = 0;
	}
	mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].next = micros + mavlinkData.streams[MAV_DATA_STREAM_RC_CHANNELS].interval / 2;
    }
    // raw controller -- attitude and nav data
    if (streamAll || (mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].enable && mavlinkData.streams[MAV_DATA_STREAM_RAW_CONTROLLER].next < micros)) {
	mavlink_msg_attitude_send(MAVLINK_COMM_0, micros, AQ_ROLL*DEG_TO_RAD, AQ_PITCH*DEG_TO_RAD, AQ_YAW*DEG_TO_RAD, -(IMU_RATEX - UKF_GYO_BIAS_X)*DEG_TO_RAD,
		(IMU_RATEY - UKF_GYO_BIAS_Y)*DEG_TO_RAD, (IMU_RATEZ - UKF_GYO_BIAS_Z)*DEG_TO_RAD);
	mavlink_msg_nav_controller_output_send(MAVLINK_COMM_0, navData.holdTiltE, navData.holdTiltN, navData.holdHeading, navData.holdCourse, navData.holdDistance, navData.holdAlt, 0, 0);
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
	    if (!motorsData.canTelemReqTime[id] || micros - motorsData.canStatusTime[id] > 3e6f)
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
	    case AQMAV_DATASET_LEGACY1 :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, AQ_ROLL, AQ_PITCH, AQ_YAW, IMU_RATEX, IMU_RATEY, IMU_RATEZ, IMU_ACCX, IMU_ACCY, IMU_ACCZ, IMU_MAGX, IMU_MAGY, IMU_MAGZ,
			navData.holdHeading, AQ_PRESSURE, IMU_TEMP, ALTITUDE, analogData.vIn, UKF_POSN, UKF_POSE, UKF_POSD);
		break;
	    case AQMAV_DATASET_LEGACY2 :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, gpsData.lat, gpsData.lon, gpsData.hAcc, gpsData.heading, gpsData.height, gpsData.pDOP,
			navData.holdCourse, navData.holdDistance, navData.holdAlt, navData.holdTiltN, navData.holdTiltE, UKF_VELN, UKF_VELE, VELOCITYD,
			RADIO_QUALITY, UKF_ACC_BIAS_X, UKF_ACC_BIAS_Y, UKF_ACC_BIAS_Z, supervisorData.flightTimeRemaining, RADIO_ERROR_COUNT);
		break;
	    case AQMAV_DATASET_LEGACY3 :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, RADIO_THROT, RADIO_RUDD, RADIO_PITCH, RADIO_ROLL, RADIO_FLAPS, RADIO_AUX2,
			motorsData.value[0], motorsData.value[1], motorsData.value[2], motorsData.value[3], motorsData.value[4], motorsData.value[5], motorsData.value[6],
			motorsData.value[7], motorsData.value[8], motorsData.value[9], motorsData.value[10], motorsData.value[11], motorsData.value[12], motorsData.value[13]);
		break;
	    case AQMAV_DATASET_GPS_XTRA :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, gpsData.pDOP, gpsData.tDOP, gpsData.sAcc, gpsData.cAcc, gpsData.velN, gpsData.velE, gpsData.velD,
			gpsData.lastPosUpdate, gpsData.lastMessage, 0,0,0,0,0,0,0,0,0,0,0);
		break;
	    case AQMAV_DATASET_UKF_XTRA :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, UKF_GYO_BIAS_X, UKF_GYO_BIAS_Y, UKF_GYO_BIAS_Z, UKF_ACC_BIAS_X, UKF_ACC_BIAS_Y, UKF_ACC_BIAS_Z, UKF_Q1, UKF_Q2, UKF_Q3, UKF_Q4,
			0,0,0,0,0,0,0,0,0,0);
		break;
	    case AQMAV_DATASET_SUPERVISOR :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, supervisorData.state, supervisorData.flightTime, supervisorData.flightTimeRemaining, supervisorData.flightSecondsAvg,
			supervisorData.vInLPF, supervisorData.soc, supervisorData.lastGoodRadioMicros, 0,0,0,0,0,0,0,0,0,0,0,0,0);
		break;
	    case AQMAV_DATASET_STACKSFREE :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, utilGetStackFree("INIT"), utilGetStackFree("FILER"), utilGetStackFree("SUPERVISOR"), utilGetStackFree("ADC"),
			utilGetStackFree("RADIO"), utilGetStackFree("CONTROL"), utilGetStackFree("GPS"), utilGetStackFree("RUN"), utilGetStackFree("COMM"), utilGetStackFree("DIMU"),
			0,0,0,0,0,0,0,0,0,0);
		break;
	    case AQMAV_DATASET_GIMBAL :
		mavlink_msg_aq_telemetry_f_send(MAVLINK_COMM_0, i, *gimbalData.pitchPort->ccr, *gimbalData.tiltPort->ccr, *gimbalData.rollPort->ccr, *gimbalData.triggerPort->ccr,
			*gimbalData.passthroughPort->ccr, gimbalData.tilt, gimbalData.trigger, gimbalData.triggerLastTime, gimbalData.triggerLastLat, gimbalData.triggerLastLon,
			gimbalData.triggerCount, 0,0,0,0,0,0,0,0,0);
		break;
	    }
	}

	mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].next = micros + mavlinkData.streams[MAV_DATA_STREAM_EXTRA3].interval;
    }
    // end streams

    // list all requested/remaining parameters
    if (mavlinkData.currentParam < CONFIG_NUM_PARAMS && mavlinkData.nextParam < micros) {
	mavlink_msg_param_value_send(MAVLINK_COMM_0, configParameterStrings[mavlinkData.currentParam], p[mavlinkData.currentParam], MAVLINK_TYPE_FLOAT, CONFIG_NUM_PARAMS, mavlinkData.currentParam);
	mavlinkData.currentParam++;
	mavlinkData.nextParam = micros + AQMAVLINK_PARAM_INTERVAL;
    }

    // request announced waypoints from mission planner
    if (mavlinkData.wpCurrent < mavlinkData.wpCount && mavlinkData.wpAttempt <= AQMAVLINK_WP_MAX_ATTEMPTS && mavlinkData.wpNext < micros) {
	mavlinkData.wpAttempt++;
	mavlink_msg_mission_request_send(MAVLINK_COMM_0, mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, mavlinkData.wpCurrent);
	mavlinkData.wpNext = micros + AQMAVLINK_WP_TIMEOUT;
    }
    // or ack that last waypoint received
    else if (mavlinkData.wpCurrent == mavlinkData.wpCount) {
	mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, 0);
	mavlinkData.wpCurrent++;
	AQ_PRINTF("%u waypoints loaded.", mavlinkData.wpCount);
    } else if (mavlinkData.wpAttempt > AQMAVLINK_WP_MAX_ATTEMPTS) {
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
	    if (!(supervisorData.state & STATE_ARMED)) {
		param = mavlink_msg_command_long_get_param2(msg);  // MAG
		param2 = mavlink_msg_command_long_get_param5(msg); // ACC
		if (param) {
		    supervisorCalibrate();
		    ack = MAV_CMD_ACK_OK;
		}
		if (param2) {
#ifdef HAS_DIGITAL_IMU
		    supervisorTare();
		    ack = MAV_CMD_ACK_OK;
#else
		    ack = MAV_CMD_ACK_ERR_FAIL;
		    AQ_NOTICE("Error: Can't perform Tare, no Digital IMU.");
#endif
		}
	    }
	    else {
		ack = MAV_CMD_ACK_ERR_FAIL;
		AQ_NOTICE("Error: Can't calibrate while armed!");
	    }

	    break;  // case MAV_CMD_PREFLIGHT_CALIBRATION

	case MAV_CMD_PREFLIGHT_STORAGE:
	    if (!(supervisorData.state & STATE_FLYING)) {
		param = mavlink_msg_command_long_get_param1(msg);
		switch (compId) {

		    // IMU calibration parameters
		    case MAV_COMP_ID_IMU:
#if defined(HAS_DIGITAL_IMU) && defined(DIMU_HAVE_EEPROM)
			if (param == 0.0f) {
			    dIMURequestCalibRead();
			    ack = MAV_CMD_ACK_OK;
			}
			else if (param == 1.0f) {
			    dIMURequestCalibWrite();
			    ack = MAV_CMD_ACK_OK;
			}
#else
			ack = MAV_CMD_ACK_ERR_FAIL;
			AQ_NOTICE("Error: No Digital IMU with EEPROM is available.");
#endif

			break; // case MAV_COMP_ID_IMU

		    // main parameters
		    default:
			if (param == 0.0f) {  					// read flash
			    configFlashRead();
			    ack = MAV_CMD_ACK_OK;
			}
			else if (param == 1.0f && configFlashWrite()) 		// write flash
			    ack = MAV_CMD_ACK_OK;
			else if (param == 2.0f && configReadFile(0) >= 0)	// read file
			    ack = MAV_CMD_ACK_OK;
			else if (param == 3.0f && configWriteFile(0) >= 0)	// write file
			    ack = MAV_CMD_ACK_OK;
			else if (param == 4.0f) {				// load defaults
			    configLoadDefault();
			    ack = MAV_CMD_ACK_OK;
			}
			else
			    ack = MAV_CMD_ACK_ERR_FAIL;

			break; // case default (main params)

		} // switch(component ID)
	    }
	    else {
		ack = MAV_CMD_ACK_ERR_FAIL;
		AQ_NOTICE("Error: Can't save or load parameters while flying!");
	    }

	    break; // case MAV_CMD_PREFLIGHT_STORAGE

	// remote reboot
	case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
	    if (!(supervisorData.state & STATE_ARMED)) {
		param = mavlink_msg_command_long_get_param1(msg);	// flight controller actions: 0=noOp; 1=reboot; 2=shutdown
		//param2 = mavlink_msg_command_long_get_param2(msg);	// onboard computer actions: 0=noOp; 1=reboot; 2=shutdown
		if (param == 1.0f) {
		    AQ_NOTICE("Flight controller restarting...");
		    ack = MAV_CMD_ACK_OK;
		    yield(100);
		    NVIC_SystemReset();
		} else {
		    ack = MAV_CMD_ACK_ERR_FAIL;
		    AQ_NOTICE("Error: Shutdown and computer actions not supported.");
		}
	    }
	    else {
		ack = MAV_CMD_ACK_ERR_FAIL;
		AQ_NOTICE("Error: Can't reset while armed!");
	    }

	    break;  // case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN

	// enable/disable AQ custom dataset messages and set frequency
	case MAV_CMD_AQ_TELEMETRY:
	    param = mavlink_msg_command_long_get_param1(msg);	// start/stop
	    param2 = mavlink_msg_command_long_get_param2(msg);	// interval in us
	    param3 = mavlink_msg_command_long_get_param3(msg);	// dataset id

	    if (param3 < AQMAV_DATASET_ENUM_END) {
		enable = (uint8_t)param;

		// enable/disable this dataset
		mavlinkData.customDatasets[(uint8_t)param3] = enable;

		// "legacy" diagnostic telemetry mode enables 2 more datasets
		if ((uint8_t)param3 == AQMAV_DATASET_LEGACY1) {
		    mavlinkData.customDatasets[AQMAV_DATASET_LEGACY2] = enable;
		    mavlinkData.customDatasets[AQMAV_DATASET_LEGACY3] = enable;
		    // toggle all other streams because legacy mode sends a lot of data
		    mavlinkToggleStreams(!enable);
		}

		// check if any datasets are active and enable/disable EXTRA3 stream accordingly
		// AQMAV_DATASET_ALL is special and toggles all datasets
		if (!enable && (uint8_t)param3 != AQMAV_DATASET_ALL) {
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

	    } else {
		ack = MAV_CMD_ACK_ERR_FAIL;
		AQ_NOTICE("Error: Unknown telemetry dataset requested.");
	    }

	    break; // case MAV_CMD_AQ_TELEMETRY

	// send firmware version number;
	case MAV_CMD_AQ_REQUEST_VERSION:
	    utilVersionString();
	    utilSerialNoString();
#ifdef USE_QUATOS
	    AQ_NOTICE("Quatos enabled.");
#endif
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
    char paramId[17];
    uint8_t c;

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

		case MAVLINK_MSG_ID_SET_MODE:
		    if (mavlink_msg_set_mode_get_target_system(&msg) == mavlink_system.sysid) {
			mavlinkData.sys_mode = mavlink_msg_set_mode_get_base_mode(&msg);
			mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, 1000-mavlinkData.idlePercent, analogData.vIn * 1000, -1, (analogData.vIn - 9.8f) / 12.6f * 1000, 0, mavlinkData.packetDrops, 0, 0, 0, 0);
		    }
		    break;

		case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
		    if (mavlink_msg_mission_request_list_get_target_system(&msg) == mavlink_system.sysid) {
			mavlink_msg_mission_count_send(MAVLINK_COMM_0, msg.sysid, msg.compid, navGetWaypointCount());
		    }
		    break;

		case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
		    if (mavlink_msg_mission_clear_all_get_target_system(&msg) == mavlink_system.sysid) {
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, MAV_COMP_ID_MISSIONPLANNER, navClearWaypoints());
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
		    break;

		case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
		    if (mavlink_msg_mission_count_get_target_system(&msg) == mavlink_system.sysid) {
			uint16_t seqId;

			seqId = mavlink_msg_mission_set_current_get_seq(&msg);
			if (seqId < NAV_MAX_MISSION_LEGS)
			    navLoadLeg(seqId);
		    }
		    break;

		case MAVLINK_MSG_ID_MISSION_COUNT:
		    if (mavlink_msg_mission_count_get_target_system(&msg) == mavlink_system.sysid) {
			uint8_t count, ack;

			count = mavlink_msg_mission_count_get_count(&msg);
			if (count > NAV_MAX_MISSION_LEGS || navClearWaypoints() != 1) {
			    // NACK
			    ack = 1;
			    AQ_PRINTF("Error: %u waypoints exceeds system maximum of %u.", count, NAV_MAX_MISSION_LEGS);
			}
			else {
			    mavlinkData.wpTargetSysId = msg.sysid;
			    mavlinkData.wpTargetCompId = msg.compid;
			    mavlinkData.wpCount = count;
			    mavlinkData.wpCurrent = mavlinkData.wpAttempt = 0;
			    mavlinkData.wpNext = timerMicros();
			    ack = 0;
			}

			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, mavlink_system.compid, ack);
		    }
		    break;

		case MAVLINK_MSG_ID_MISSION_ITEM:
		    if (mavlink_msg_mission_item_get_target_system(&msg) == mavlink_system.sysid) {
			uint16_t seqId;
			uint8_t ack = 0;
			uint8_t frame;

			seqId = mavlink_msg_mission_item_get_seq(&msg);
			frame = mavlink_msg_mission_item_get_frame(&msg);

			if (seqId >= NAV_MAX_MISSION_LEGS || (frame != MAV_FRAME_GLOBAL && frame != MAV_FRAME_GLOBAL_RELATIVE_ALT)) {
			    // NACK
			    ack = 1;
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
				wp->maxHorizSpeed = p[NAV_MAX_SPEED];
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
				    wp->maxHorizSpeed = p[NAV_MAX_SPEED];
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
			    else if (command == 1) { // TODO: stop using hard-coded values, use enums like intended
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
				wp->poiAltitude = mavlink_msg_mission_item_get_param4(&msg);
			    }
			    else {
				// NACK
				ack = 1;
			    }
			}

			mavlinkData.wpCurrent = seqId + 1;
			mavlinkData.wpAttempt = 0;
			mavlinkData.wpNext = timerMicros();
			mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, mavlink_system.compid, ack);
		    }
		    break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
		    if (mavlink_msg_param_request_read_get_target_system(&msg) == mavlink_system.sysid) {
			uint16_t paramIndex;

			paramIndex = mavlink_msg_param_request_read_get_param_index(&msg);
			if (paramIndex < CONFIG_NUM_PARAMS)
			    mavlink_msg_param_value_send(MAVLINK_COMM_0, configParameterStrings[paramIndex], p[paramIndex], MAVLINK_TYPE_FLOAT, CONFIG_NUM_PARAMS, paramIndex);
		    }
		    break;

		case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
		    if (mavlink_msg_param_request_list_get_target_system(&msg) == mavlink_system.sysid) {
			mavlinkData.currentParam = 0;
			mavlinkData.nextParam = 0;
		    }
		    break;

		case MAVLINK_MSG_ID_PARAM_SET:
		    if (mavlink_msg_param_set_get_target_system(&msg) == mavlink_system.sysid) {
			int paramIndex = -1;
			float paramValue;

			mavlink_msg_param_set_get_param_id(&msg, paramId);
			paramIndex = configGetParamIdByName((char *)paramId);
			if (paramIndex >= 0 && paramIndex < CONFIG_NUM_PARAMS) {
			    paramValue = mavlink_msg_param_set_get_param_value(&msg);
			    if (!isnan(paramValue) && !isinf(paramValue) && !(supervisorData.state & STATE_FLYING))
				p[paramIndex] = paramValue;
			    // send back what we have no matter what
			    mavlink_msg_param_value_send(MAVLINK_COMM_0, paramId, p[paramIndex], MAVLINK_TYPE_FLOAT, CONFIG_NUM_PARAMS, paramIndex);
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

		case MAVLINK_MSG_ID_HEARTBEAT:
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

void mavlinkSetSystemType(void) {
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
    mavlinkData.sys_mode = MAV_MODE_PREFLIGHT;
    mavlinkData.sys_state = MAV_STATE_BOOT;
    mavlinkData.sys_nav_mode = AQ_NAV_STATUS_INIT;
    mavlink_system.sysid = flashSerno(0) % 250;
    mavlink_system.compid = MAV_COMP_ID_MISSIONPLANNER;
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
