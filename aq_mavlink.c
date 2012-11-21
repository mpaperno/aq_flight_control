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

    Copyright © 2011, 2012  Bill Nesbitt
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
#include <CoOS.h>
#include <string.h>

mavlinkStruct_t mavlinkData;
mavlink_system_t mavlink_system;

OS_STK *mavlinkRecvTaskStack;

void comm_send_ch(mavlink_channel_t chan, uint8_t ch) {
    serialWrite(mavlinkData.serialPort, ch);
}

void mavlinkNotice(const char *s) {
    // queue message, notify and leave
    CoPostQueueMail(mavlinkData.notices, (void *)s);
}

void mavlinkWpReached(uint16_t seqId) {
    mavlink_msg_mission_item_reached_send(MAVLINK_COMM_0, seqId);
}

void mavlinkWpAnnounceCurrent(uint16_t seqId) {
    mavlink_msg_mission_current_send(MAVLINK_COMM_0, seqId);
}

void mavlinkDo(void) {
    StatusType result;
    void *msg;
    static unsigned long mavCounter;
    static unsigned long lastMicros = 0;
    unsigned long micros;

    micros = timerMicros();

    // handle rollover
    if (micros < lastMicros) {
	mavlinkData.nextHeartbeat = 0;
	mavlinkData.nextParam = 0;
	memset(mavlinkData.streamNext, 0, sizeof(mavlinkData.streamInterval));
    }

    supervisorSendDataStart();
    CoEnterMutexSection(mavlinkData.serialPortMutex);

    switch (supervisorData.state) {
    case STATE_DISARMED:
	mavlinkData.status = MAV_STATE_STANDBY;
	break;

    case STATE_ARMED:
	mavlinkData.status = MAV_STATE_ACTIVE;
	break;

    case STATE_RADIO_LOSS1:
	mavlinkData.status =  MAV_STATE_CRITICAL;
	break;

    case STATE_RADIO_LOSS2:
	mavlinkData.status =  MAV_STATE_CRITICAL;
	break;
    }

    switch(navData.mode) {
    case NAV_STATUS_ALTHOLD:
	mavlinkData.mode= 128 + 16;
	break;

    case NAV_STATUS_POSHOLD:
	mavlinkData.mode= 128 + 16 + 1;
	break;

    case NAV_STATUS_MISSION:
	mavlinkData.mode= 128 + 4;
	break;

    case NAV_STATUS_DVH:
	mavlinkData.mode= 128 + 64 + 16;
	break;

    case NAV_STATUS_MANUAL:
	mavlinkData.mode= 128 + 64;
	break;

    default:
	if (supervisorData.state == STATE_DISARMED)
	    mavlinkData.mode = 64;
	if (supervisorData.state == STATE_ARMED)
	    mavlinkData.mode = 128 + 64;
    }


    // heartbeat & status
    if (mavlinkData.nextHeartbeat < micros) {
	mavlink_msg_heartbeat_send(MAVLINK_COMM_0, mavlink_system.type, MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY, mavlinkData.mode, mavlinkData.nav_mode, mavlinkData.status);
	// calculate idle time
	mavCounter = counter;
	mavlinkData.idlePercent = (mavCounter - mavlinkData.lastCounter) * minCycles * 1000.0f / (MAVLINK_HEARTBEAT_INTERVAL * rccClocks.SYSCLK_Frequency / 1e6f);
	mavlinkData.lastCounter = mavCounter;

	mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, 1000-mavlinkData.idlePercent, adcData.vIn * 1000, -1, (adcData.vIn - 9.8f) / 12.6f * 1000, 0, mavlinkData.packetDrops, 0, 0, 0, 0);

	mavlinkData.nextHeartbeat = micros + MAVLINK_HEARTBEAT_INTERVAL;
    }
    // raw sensors
    else if ((mavlinkData.streamInterval[MAV_DATA_STREAM_ALL] || mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_SENSORS]) && mavlinkData.streamNext[MAV_DATA_STREAM_RAW_SENSORS] < micros) {
	mavlink_msg_scaled_imu_send(MAVLINK_COMM_0, micros, IMU_ACCX*1000.0f, IMU_ACCY*1000.0f, IMU_ACCZ*1000.0f, IMU_RATEX*1000.0f, IMU_RATEY*1000.0f, IMU_RATEZ*1000.0f, IMU_MAGX*1000.0f, IMU_MAGY*1000.0f, IMU_MAGZ*1000.0f);
	mavlink_msg_scaled_pressure_send(MAVLINK_COMM_0, micros, AQ_PRESSURE*0.01f, 0.0f, IMU_TEMP*100);
	mavlinkData.streamNext[MAV_DATA_STREAM_RAW_SENSORS] = micros + mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_SENSORS];
    }
    // position
    else if ((mavlinkData.streamInterval[MAV_DATA_STREAM_ALL] || mavlinkData.streamInterval[MAV_DATA_STREAM_POSITION]) && mavlinkData.streamNext[MAV_DATA_STREAM_POSITION] < micros) {
	mavlink_msg_gps_raw_int_send(MAVLINK_COMM_0, micros, navData.fixType, gpsData.lat*1e7, gpsData.lon*1e7, gpsData.height*1e3, gpsData.hDOP, gpsData.vDOP, gpsData.speed, gpsData.heading, 255);
//	    mavlink_msg_gps_raw_send(MAVLINK_COMM_0, micros, 3, gpsData.lat, gpsData.lon, gpsData.height, gpsData.hDOP, gpsData.vDOP, gpsData.speed, gpsData.heading);
	mavlinkData.streamNext[MAV_DATA_STREAM_POSITION] = micros + mavlinkData.streamInterval[MAV_DATA_STREAM_POSITION];
    }
    // extended status
//	else if ((mavlinkData.streamInterval[MAV_DATA_STREAM_ALL] || mavlinkData.streamInterval[MAV_DATA_STREAM_EXTENDED_STATUS]) && mavlinkData.streamNext[MAV_DATA_STREAM_EXTENDED_STATUS] < micros) {
//	    mavlinkData.streamNext[MAV_DATA_STREAM_EXTENDED_STATUS] = micros + mavlinkData.streamInterval[MAV_DATA_STREAM_EXTENDED_STATUS];
//	}
    // rc channels
    else if ((mavlinkData.streamInterval[MAV_DATA_STREAM_ALL] || mavlinkData.streamInterval[MAV_DATA_STREAM_RC_CHANNELS]) && mavlinkData.streamNext[MAV_DATA_STREAM_RC_CHANNELS] < micros) {
	mavlink_msg_rc_channels_raw_send(MAVLINK_COMM_0, micros, 0, RADIO_THROT+1024, RADIO_ROLL+1024, RADIO_PITCH+1024, RADIO_RUDD+1024, RADIO_GEAR+1024, RADIO_FLAPS+1024, RADIO_AUX2+1024, RADIO_AUX3+1024, RADIO_QUALITY);
	mavlink_msg_rc_channels_scaled_send(MAVLINK_COMM_0, micros, 0, (RADIO_THROT-750)*13, RADIO_ROLL*13, RADIO_PITCH*13, RADIO_RUDD*13, RADIO_GEAR*13, RADIO_FLAPS*13, RADIO_AUX2*13, RADIO_AUX3*13, RADIO_QUALITY);
	mavlinkData.streamNext[MAV_DATA_STREAM_RC_CHANNELS] = micros + mavlinkData.streamInterval[MAV_DATA_STREAM_RC_CHANNELS];
    }
    // raw controller
    else if ((mavlinkData.streamInterval[MAV_DATA_STREAM_ALL] || mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_CONTROLLER]) && mavlinkData.streamNext[MAV_DATA_STREAM_RAW_CONTROLLER] < micros) {
	mavlink_msg_attitude_send(MAVLINK_COMM_0, micros, AQ_ROLL*DEG_TO_RAD, AQ_PITCH*DEG_TO_RAD, AQ_YAW*DEG_TO_RAD, -(IMU_RATEX - UKF_GYO_BIAS_X)*DEG_TO_RAD, (IMU_RATEY - UKF_GYO_BIAS_Y)*DEG_TO_RAD, (IMU_RATEZ - UKF_GYO_BIAS_Z)*DEG_TO_RAD);
	mavlink_msg_servo_output_raw_send(MAVLINK_COMM_0, micros, 0, motorsData.value[0], motorsData.value[1], motorsData.value[2], motorsData.value[3], motorsData.value[4], motorsData.value[5], motorsData.value[6], motorsData.value[7]);
	mavlinkData.streamNext[MAV_DATA_STREAM_RAW_CONTROLLER] = micros + mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_CONTROLLER];
    }

    // send pending notices
    msg = CoAcceptQueueMail(mavlinkData.notices, &result);
    if (result == E_OK)
	    mavlink_msg_statustext_send(MAVLINK_COMM_0, 0, (const char *)msg);

    // list all parameters
    if (mavlinkData.currentParam < mavlinkData.numParams && mavlinkData.nextParam < micros) {
	mavlink_msg_param_value_send(MAVLINK_COMM_0, configParameterStrings[mavlinkData.currentParam], p[mavlinkData.currentParam], MAVLINK_TYPE_FLOAT, mavlinkData.numParams, mavlinkData.currentParam);
	mavlinkData.currentParam++;
	mavlinkData.nextParam = micros + MAVLINK_PARAM_INTERVAL;
    }
    else if (mavlinkData.wpCurrent < mavlinkData.wpCount && mavlinkData.wpNext < micros) {
	mavlink_msg_mission_request_send(MAVLINK_COMM_0, mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, mavlinkData.wpCurrent);
	mavlinkData.wpNext = micros + MAVLINK_WP_TIMEOUT;
    }
    else if (mavlinkData.wpCurrent == mavlinkData.wpCount) {
	mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlinkData.wpTargetSysId, mavlinkData.wpTargetCompId, 0);
	mavlinkData.wpCurrent++;
    }

    supervisorSendDataStop();
    CoLeaveMutexSection(mavlinkData.serialPortMutex);

    lastMicros = micros;
}

void mavlinkDoCommand(mavlink_message_t *msg) {
    uint16_t command;
    float param;
    uint8_t ack = MAV_CMD_ACK_ERR_NOT_SUPPORTED;

    command = mavlink_msg_command_long_get_command(msg);

    switch (command) {
	case MAV_CMD_PREFLIGHT_STORAGE:
	    if (!(supervisorData.state & STATE_FLYING)) {
		param = mavlink_msg_command_long_get_param1(msg);
		if (param == 0.0f) {
		    configFlashRead();
		    ack = MAV_CMD_ACK_OK;
		}
		else if (param == 1.0f) {
		    configFlashWrite();
		    ack = MAV_CMD_ACK_OK;
		}
		else if (param == 2.0f) {
		    if (configReadFile(0) >= 0)
			ack = MAV_CMD_ACK_OK;
		}
		else if (param == 3.0f) {
		    if (configWriteFile(0) >= 0)
			ack = MAV_CMD_ACK_OK;
		}
	    }
	    else {
		ack = MAV_CMD_ACK_ERR_FAIL;
	    }

	    break;

	default:
	    break;
    }

    mavlink_msg_command_ack_send(MAVLINK_COMM_0, command, ack);
}

void mavlinkRecvTaskCode(void *unused) {
    serialPort_t *s = mavlinkData.serialPort;
    mavlink_message_t msg;
    mavlink_status_t status;
    char paramId[16];
    uint8_t c;

    AQ_NOTICE("Mavlink receive task started\n");

    while (1) {
	yield(5);

	// process incoming data
	while (serialAvailable(s)) {
	    c = serialRead(s);
	    // Try to get a new message
	    if (mavlink_parse_char(MAVLINK_COMM_0, c, &msg, &status)) {
		CoEnterMutexSection(mavlinkData.serialPortMutex);

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
			    mavlinkData.mode = mavlink_msg_set_mode_get_base_mode(&msg);
			    mavlink_msg_sys_status_send(MAVLINK_COMM_0, 0, 0, 0, 1000-mavlinkData.idlePercent, adcData.vIn * 1000, -1, (adcData.vIn - 9.8f) / 12.6f * 1000, 0, mavlinkData.packetDrops, 0, 0, 0, 0);
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
			    }
			    else {
				mavlinkData.wpTargetSysId = msg.sysid;
				mavlinkData.wpTargetCompId = msg.compid;
				mavlinkData.wpCount = count;
				mavlinkData.wpCurrent = 0;
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
				else if (command == 1) {
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
			    mavlinkData.wpNext = timerMicros();
			    mavlink_msg_mission_ack_send(MAVLINK_COMM_0, mavlink_system.sysid, mavlink_system.compid, ack);
			}
			break;

		    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
			if (mavlink_msg_param_request_read_get_target_system(&msg) == mavlink_system.sysid) {
			    uint16_t paramIndex;

			    paramIndex = mavlink_msg_param_request_read_get_param_index(&msg);
			    if (paramIndex < mavlinkData.numParams)
				mavlink_msg_param_value_send(MAVLINK_COMM_0, configParameterStrings[paramIndex], p[paramIndex], MAVLINK_TYPE_FLOAT, mavlinkData.numParams, paramIndex);
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
			    int i, j;

			    mavlink_msg_param_set_get_param_id(&msg, paramId);
			    for (i = 0; i < mavlinkData.numParams; i++) {
				for (j = 0; j < MAVLINK_PARAMID_LEN; j++)
				    if (paramId[j] != configParameterStrings[i][j] || paramId[j] == 0)
					break;
				if (paramId[j] == 0) {
				    paramIndex = i;
				    break;
				}
			    }
			    if (paramIndex >= 0 && paramIndex < mavlinkData.numParams) {
				paramValue = mavlink_msg_param_set_get_param_value(&msg);
				if (!isnan(paramValue) && !isinf(paramValue) && !(supervisorData.state & STATE_FLYING))
				    p[i] = paramValue;
				// send back what we have no matter what
				mavlink_msg_param_value_send(MAVLINK_COMM_0, paramId, p[i], MAVLINK_TYPE_FLOAT, mavlinkData.numParams, i);
			    }
			}
			break;

		    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
			if (mavlink_msg_request_data_stream_get_target_system(&msg) == mavlink_system.sysid) {
			    uint16_t rate;
			    uint8_t stream_id;

			    stream_id = mavlink_msg_request_data_stream_get_req_stream_id(&msg);
			    rate = mavlink_msg_request_data_stream_get_req_message_rate(&msg);

			    if (rate > 0 && rate < 200 && stream_id < MAV_DATA_STREAM_ENUM_END && mavlink_msg_request_data_stream_get_start_stop(&msg)) {
				mavlinkData.streamInterval[stream_id] = 1e6 / rate;
			    }
			    else {
				mavlinkData.streamInterval[stream_id] = 0;
			    }
			}
			break;

		    default:
			// Do nothing
			break;
		}

		CoLeaveMutexSection(mavlinkData.serialPortMutex);
	    }

	    // Update global packet drops counter
	    mavlinkData.packetDrops += status.packet_rx_drop_count;
	}
    }
}

void mavlinkInit(void) {
    unsigned long micros;
    int i;

    memset((void *)&mavlinkData, 0, sizeof(mavlinkData));

    mavlinkData.serialPort = serialOpen(MAVLINK_USART, p[DOWNLINK_BAUD], USART_HardwareFlowControl_RTS_CTS, 0, 0);

    // setup mutex for serial port access
    mavlinkData.serialPortMutex = CoCreateMutex();

    mavlinkData.notices = CoCreateQueue(mavlinkData.noticeQueue, MAVLINK_NOTICE_DEPTH, EVENT_SORT_TYPE_FIFO);

    AQ_NOTICE("Mavlink init\n");

    mavlinkData.numParams = CONFIG_NUM_PARAMS;
    mavlinkData.currentParam = mavlinkData.numParams;
    mavlinkData.wpCount = navGetWaypointCount();
    mavlinkData.wpCurrent = mavlinkData.wpCount + 1;

    mavlinkRecvTaskStack = aqStackInit(MAVLINK_STACK_SIZE);

    mavlinkData.recvTask = CoCreateTask(mavlinkRecvTaskCode, (void *)0, MAVLINK_PRIORITY, &mavlinkRecvTaskStack[MAVLINK_STACK_SIZE-1], MAVLINK_STACK_SIZE);

    mavlink_system.sysid = flashSerno(0) % 250;
    mavlink_system.compid = MAV_COMP_ID_MISSIONPLANNER;
    mavlink_system.type = MAV_TYPE_QUADROTOR;

    mavlinkData.mode = MAV_MODE_FLAG_MANUAL_INPUT_ENABLED;
    mavlinkData.nav_mode = MAV_STATE_STANDBY;
    mavlinkData.status = MAV_STATE_ACTIVE;

    // turn on all streams at 1Hz & spread them out
    micros = timerMicros();
    for (i = 1; i < 13; i++) {	// TODO: remove hardcoded value
	mavlinkData.streamInterval[i] = 1e6f;
	mavlinkData.streamNext[i] = micros + 5e6f + i * 5e3f;
    }
    // send IMU data at an initial rate of 10Hz
    mavlinkData.streamInterval[MAV_DATA_STREAM_RAW_CONTROLLER] = 1e6f/10.0f;
}
#endif	// USE_MAVLINK
