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

#include "telem_sPort.h"
#include "aq.h"
#include "aq_timer.h"
#include "aq_math.h"
#include "config.h"
#include "aq_mavlink.h"
#include "supervisor.h"
#include "run.h"
#include "gps.h"
#include "imu.h"
#include "nav.h"
#include "nav_ukf.h"
#include "analog.h"
#include "canSensors.h"

#include <stdlib.h>
#include <string.h>


sPortData_t sPortData __attribute__((section(".ccm")));


static inline uint8_t *sPortAddByte(uint8_t *ptr, const uint8_t byte, uint16_t *crc) {

    if (byte == SPORT_DATA_REQUEST || byte == SPORT_DATA_PADDING) {
	// framing flags escape sequence
	*ptr++ = SPORT_DATA_PADDING;
	*ptr++ = byte ^ SPORT_DATA_ESCAPE;
    } else
	*ptr++ = byte;

    if (crc) {
	*crc += byte;
	*crc += *crc >> 8;
	*crc &= 0x00FF;
    }

    return ptr;
}

static void sPortSendPackage(uint16_t id, uint32_t value) {
    commTxBuf_t *txBuf;
    uint8_t *ptr;
    uint8_t *bytes;
    uint16_t crc = 0;

    txBuf = commGetTxBuf(COMM_STREAM_TYPE_RC_RX_TELEM, 16);
    // cannot block, must fail
    if (txBuf) {
	supervisorSendDataStart();
	ptr = &txBuf->buf;

	ptr = sPortAddByte(ptr, SPORT_DATA_START, &crc);
	bytes = (uint8_t *)&id;
	ptr = sPortAddByte(ptr, bytes[0], &crc);
	ptr = sPortAddByte(ptr, bytes[1], &crc);
	bytes = (uint8_t *)&value;
	ptr = sPortAddByte(ptr, bytes[0], &crc);
	ptr = sPortAddByte(ptr, bytes[1], &crc);
	ptr = sPortAddByte(ptr, bytes[2], &crc);
	ptr = sPortAddByte(ptr, bytes[3], &crc);
	// send crc
	ptr = sPortAddByte(ptr, 0xFF - crc, 0);

	commSendTxBuf(txBuf, ptr - &txBuf->buf);
	supervisorSendDataStop();
    }
    else
	sPortData.sendErrCount++;
}


void sPortSendTextMessage(const char *msg) {
    int idx = (sPortData.messageCount + 1) % SPORT_TEXT_NUM_BUFFERS;
    sPortData.messageDataBuffer[idx][0] = '6';  // add severity indicator; 6 = MAV_SEVERITY_INFO
    strncpy(&sPortData.messageDataBuffer[idx][1], msg, SPORT_TEXT_MAX_MSG_LEN - 1);
    sPortData.messageDataBuffer[idx][SPORT_TEXT_MAX_MSG_LEN-1] = 0; // ensure NUL terminator
    sPortData.messageCount++;  // don't increment this until the msg is ready for consumption
}

static inline char sPortGetValidChar(char c) {
    if (!c || c == 10 || c == 13)
	return 0;
    else if (c > 31 && c < 127)
	return c;
    else
	return 32;
}

static inline char sPortGetNextMsgByte(void) {
    uint8_t atEOM;
    int idx;
    char ret = 0;

    if (sPortData.sentMessageNum < sPortData.currMessageNum) {
	atEOM = (sPortData.msgCharIdx >= SPORT_TEXT_MAX_MSG_LEN);
	idx = sPortData.currMessageNum % SPORT_TEXT_NUM_BUFFERS;
	if (!atEOM) {
	    ret = sPortGetValidChar(sPortData.messageDataBuffer[idx][sPortData.msgCharIdx++] & 0x7F);
	    atEOM = !ret;
	}
	if (atEOM) {
	    //ret = 3; // ETX
	    sPortData.msgCharIdx = 0;
	    sPortData.messageDataBuffer[idx][0] = 0;
	    sPortData.sentMessageNum = sPortData.currMessageNum;
	}
    } else
	sPortData.msgCharIdx = 0;

    if ((!ret /*|| atEOM*/) && sPortData.messageCount > sPortData.sentMessageNum)
	sPortData.currMessageNum++;  // advance buffer

    return ret;
}

static inline uint16_t sPortGetNextMsgWord(void) {
    uint16_t ret = 0;
    char ch = sPortGetNextMsgByte();

//    if (!ch)
//	return 4; // EOT

    ret = (sPortData.msgPacketSeq++ & 0x01) << 15;   // MSB will change on each packet so rx knows to update message
//    if (ch != 3) {
	ret |= ch << 8;

	if ((ch = sPortGetNextMsgByte())/* != 3 */)
	    ret |= ch /* << 1 */;	// LSB is lost when # of rpm blades set to 2 in user display settings
//    }

    return ret;
}

//static inline uint32_t sPortEncodeAdcValue(float value) {
//    return (uint32_t) ((value * 330 - 165) / 0xFF);
//}

static void sPortProcessSensorRequest(uint8_t sId) {
    uint8_t states;
    uint32_t tmp_u32;
    uint16_t tmp_u16;
    int32_t tmp_i32;

    if (!sPortData.gpsFirstLock && navData.fixType > (sPortCheckConfigFlag(SPORT_CFG_WAIT_GPS) ? 2 : 1))
	sPortData.gpsFirstLock = 1;

    switch (sId) {

	case SPORT_DS_VARIO:
	    // first reported altitude is considered "ground level" by Taranis, so don't send it until we're sure of our estimate
	    states = (!sPortCheckConfigFlag(SPORT_CFG_WAIT_ALT) || sPortData.gpsFirstLock) ? 2 : 1;
	    switch (sPortData.sensorSendState[SPORT_DS_VARIO]++ % states) {
		case 0:
		    sPortSendPackage(SPORT_DATAID_VSPEED, (int32_t)(-VELOCITYD * 100.0f));
		    break;
		case 1:
		    sPortSendPackage(SPORT_DATAID_ALTITUDE, (int32_t)(ALTITUDE * 100.0f));
		    break;
	    }
	    break;

	case SPORT_DS_FLVSS:
	    // calculated voltage per cell * 500 for SPort format, constrain to 12 bits
	    tmp_u16 = (uint16_t)(supervisorData.vInLPF / analogData.batCellCount * 500.0f) & 0xFFF;
	    // first cell # in this packet (0=cell 1)
	    tmp_u32 = sPortData.sensorSendState[SPORT_DS_FLVSS];
	    // total cells
	    tmp_u32 |= (analogData.batCellCount & 0xF) << 4;
	    // first voltage
	    tmp_u32 |= tmp_u16 << 8;
	    if (analogData.batCellCount > sPortData.sensorSendState[SPORT_DS_FLVSS] + 1)
		// second voltage
		tmp_u32 |= tmp_u16 << 20;

	    sPortData.sensorSendState[SPORT_DS_FLVSS] += 2;
	    if (sPortData.sensorSendState[SPORT_DS_FLVSS] >= analogData.batCellCount)
		sPortData.sensorSendState[SPORT_DS_FLVSS] = 0;

	    sPortSendPackage(SPORT_DATAID_CELLS, tmp_u32);
	    break;

	case SPORT_DS_FAS:
	    states = (supervisorData.aOutLPF == SUPERVISOR_INVALID_AMPSOUT_VALUE) ? 1 : 2;
	    switch (sPortData.sensorSendState[SPORT_DS_FAS]++ % states) {
		case 0:
		    sPortSendPackage(SPORT_DATAID_VFAS, (int32_t)(supervisorData.vInLPF * 100));
		    break;
		case 1:
		    sPortSendPackage(SPORT_DATAID_CURRENT, (int32_t)(supervisorData.aOutLPF * 100));
		    break;
	    }
	    break;

	case SPORT_DS_GPS:
	    if (!sPortData.gpsFirstLock || navData.fixType < 2)
		break;

	    switch (sPortData.sensorSendState[SPORT_DS_GPS]++ % 5) {
		case 0: // Sends the longitude value, setting bit 31 high
		    tmp_u32 = (uint32_t)(fabs(gpsData.lon) * (double)6e5f) & 0x3FFFFFFF;
		    if (gpsData.lon < (double)0.0f)
			tmp_u32 |= 0xC0000000;
		    else
			tmp_u32 |= 0x80000000;
		    sPortSendPackage(SPORT_DATAID_LATLONG, tmp_u32);
		    break;
		case 1:
		    tmp_u32 = (uint32_t)(fabs(gpsData.lat) * (double)6e5f) & 0x3FFFFFFF;
		    if (gpsData.lat < (double)0.0f)
			tmp_u32 |= 0x40000000;
		    sPortSendPackage(SPORT_DATAID_LATLONG, tmp_u32);
		    break;
		case 2:
		    sPortSendPackage(SPORT_DATAID_GPS_ALT, (int32_t)(gpsData.height * 100.0f));
		    break;
		case 3:
		    sPortSendPackage(SPORT_DATAID_GPS_SPEED, (uint32_t)(gpsData.speed * 1.9438e3)); // sent in knots to the thousandth's place (1.9438*1000)
		    break;
		case 4:
		    sPortSendPackage(SPORT_DATAID_GPS_COURSE, (int32_t)(gpsData.heading * 100.0f));
		    break;
	    }
	    break;

	// IMU temperature (only used if sending custom data is disabled, otherwise it is sent in SPORT_DS_MISC)
	case SPORT_DS_RPM :
	    sPortSendPackage(SPORT_DATAID_T2, (int32_t)IMU_TEMP);
	    break;

	// battery percent (only used if sending custom data is disabled, otherwise it is sent in SPORT_DS_MISC)
	case SPORT_DS_FUEL :
	    sPortSendPackage(SPORT_DATAID_FUEL, supervisorData.battRemainingPrct);
	    break;

	// Text message bytes in SPORT_DATAID_RPM
	case SPORT_DS_MSG :
	    //if ((tmp_u16 = sPortGetNextMsgWord()) != 4)
		sPortSendPackage(SPORT_DATAID_RPM, sPortGetNextMsgWord());
	    break;

	// Flight mode and flags in SPORT_DATAID_FUEL
	case SPORT_DS_SYS_STAT:
	    tmp_u16 = sPortData.sensorSendState[SPORT_DS_SYS_STAT] << 14; // 2 MSB indicate packet contents
	    switch (sPortData.sensorSendState[SPORT_DS_SYS_STAT]) {
		case 0:
		    // bits 0-13
		    tmp_u16 |= supervisorData.systemStatus & 0x3FFF;
		    break;
		case 1:
		    // bits 14-27
		    tmp_u16 |= (supervisorData.systemStatus >> 14) & 0x3FFF;
		    break;
		case 2:
		    // bits 28-31
		    tmp_u16 |= (supervisorData.systemStatus >> 28) & 0xF;
		    break;
	    }
	    sPortData.sensorSendState[SPORT_DS_SYS_STAT] = (sPortData.sensorSendState[SPORT_DS_SYS_STAT] + 1) % 3;

	    sPortSendPackage(SPORT_DATAID_FUEL, tmp_u16);
	    break;

	// GPS Status in SPORT_DATAID_T1
	case SPORT_DS_GPS_STAT:
	    states = 2;
	    tmp_u32 = sPortData.sensorSendState[SPORT_DS_GPS_STAT] << 14; // 2 MSB indicate packet contents
	    tmp_u32 |= navData.fixType << 12; // next 2 bits for fix type (0-3)
	    switch (sPortData.sensorSendState[SPORT_DS_GPS_STAT]) {
		case 0:
		    tmp_u32 |= MIN((uint32_t)(gpsData.hAcc * 100), (uint32_t)0xFFF);
		    break;
		case 1:
		    tmp_u32 |= MIN((uint32_t)(gpsData.vAcc * 100), (uint32_t)0xFFF);
		    break;
	    }
	    sPortData.sensorSendState[SPORT_DS_GPS_STAT] = (sPortData.sensorSendState[SPORT_DS_GPS_STAT] + 1) % states;

	    sPortSendPackage(SPORT_DATAID_T1, tmp_u32);
	    break;

	// various values in SPORT_DATAID_T2
	case SPORT_DS_MISC:
	    states = navData.missionLeg ? 5 : 3;
	    tmp_u32 = sPortData.sensorSendState[SPORT_DS_MISC] << 13; // 3 MSB indicate packet contents
	    switch (sPortData.sensorSendState[SPORT_DS_MISC]) {
		// magnetic heading (yaw degrees)
		case 0:
		    tmp_u32 |= (uint32_t)(AQ_YAW * 10) & 0x1FFF;
		    break;
		// IMU temperature
		case 1:
		    tmp_u32 |= (uint32_t)(IMU_TEMP * 10) & 0x1FFF; // 10th precision only
		    break;
		// battery percent remaining
		case 2:
		    tmp_u32 |= supervisorData.battRemainingPrct;
		    break;
		// next mission waypoint
		case 3:
		    tmp_u32 |= navData.missionLeg & 0x1FFF;
		    break;
		// distance to waypoint
		case 4:
		    tmp_u16 = 0; // precision
		    tmp_i32 = 1; // multiplier
		    if (navData.holdDistance <= 20.0f) {
			tmp_u16 = 2;
			tmp_i32 = 10;
		    } else if (navData.holdDistance <= 200.0f) {
			tmp_u16 = 1;
			tmp_i32 = 100;
		    }
		    tmp_u32 |= tmp_u16 << 11; // 2 bits indicate precision
		    tmp_u32 |= (uint32_t) (navData.holdDistance * tmp_i32) & 0x1FFF;
		    break;
	    }
	    sPortData.sensorSendState[SPORT_DS_MISC] = (sPortData.sensorSendState[SPORT_DS_MISC] + 1) % states;

	    sPortSendPackage(SPORT_DATAID_T2, tmp_u32);
	    break;

	// ACC
	case SPORT_DS_ACC:
	    switch (sPortData.sensorSendState[SPORT_DS_ACC]++ % 3) {
		case 0:
		    sPortSendPackage(SPORT_DATAID_ACCX, (int32_t)(IMU_ACCX * 100.0f));
		    break;
		case 1:
		    sPortSendPackage(SPORT_DATAID_ACCY, (int32_t)(IMU_ACCY * 100.0f));
		    break;
		case 2:
		    sPortSendPackage(SPORT_DATAID_ACCZ, (int32_t)(IMU_ACCZ * 100.0f));
		    break;
	    }
	    break;

	default:
	    break;
    }
}

static inline uint8_t sPortIsSensorDisabled(uint8_t sId) {
    return sPortData.sensorsFound[sId];
}

static inline void sPortSetSensorDisabled(uint8_t sId) {
    sPortData.sensorsFound[sId] = 1;
}

static uint8_t sPortGetDatasetId(uint8_t sportId) {
    switch (sportId) {
	case SPORT_SENSID_VARIO:
	    return SPORT_DS_VARIO;

	case SPORT_SENSID_FAS:
	    return SPORT_DS_FAS;

	case SPORT_SENSID_FLVSS:
	    return SPORT_DS_FLVSS;

	case SPORT_SENSID_GPS:
	    return SPORT_DS_GPS;

	case SPORT_SENSID_RPM:
	    if (sPortCheckConfigFlag(SPORT_CFG_SEND_CUSTOM) && sPortCheckConfigFlag(SPORT_CFG_SEND_TXT_MSG))
		return SPORT_DS_MSG;
	    else
		return SPORT_DS_RPM;

	case SPORT_SENSID_8:
	    if (sPortCheckConfigFlag(SPORT_CFG_SEND_CUSTOM))
		return SPORT_DS_SYS_STAT;
	    else
		return SPORT_DS_FUEL;

	case SPORT_SENSID_9:
	    if (sPortCheckConfigFlag(SPORT_CFG_SEND_CUSTOM))
		return SPORT_DS_GPS_STAT;
	    else
		return SPORT_DS_UNUSED;

	case SPORT_SENSID_10:
	    if (sPortCheckConfigFlag(SPORT_CFG_SEND_CUSTOM))
		return SPORT_DS_MISC;
	    else
		return SPORT_DS_UNUSED;

	case SPORT_SENSID_11:
	    if (sPortCheckConfigFlag(SPORT_CFG_SEND_ACC))
		return SPORT_DS_ACC;
	    else
		return SPORT_DS_UNUSED;

	case SPORT_SENSID_12:
	case SPORT_SENSID_13:
	    if (sPortCheckConfigFlag(SPORT_CFG_SEND_CUSTOM))
		return SPORT_DS_MSG;
	    else
		return SPORT_DS_UNUSED;

	default:
	    return SPORT_DS_UNUSED;
    }
}

void sPortProcessInput(commRcvrStruct_t *r) {
    uint8_t data = 0;

    while (commAvailable(r)) {
	data = commReadChar(r);
	switch (sPortData.processState) {
	    case SPORT_PROC_WAIT_RCV:
		if (data == SPORT_DATA_REQUEST) {
		    sPortData.processState = SPORT_PROC_WAIT_SEND;
		    if (!sPortData.rxFirstContact)
			sPortData.rxFirstContact = timerMicros();
		}
		break;

	    case SPORT_PROC_WAIT_SEND:
		sPortData.datasetRequested = sPortGetDatasetId(data);
		// ignore any sensors we don't know about or are already disabled
		if (sPortData.datasetRequested == SPORT_DS_UNUSED || sPortIsSensorDisabled(sPortData.datasetRequested))
		    sPortData.processState = SPORT_PROC_WAIT_RCV;
		// listen for other sensors for a while before sending any data on the line
		else if (!sPortData.sensorSearchTimeout)
		    sPortData.processState = SPORT_PROC_WAIT_LISTEN;
		// otherwise process the sensor request ourselves
		else {
		    sPortData.processState = SPORT_PROC_WAIT_RCV;
		    sPortProcessSensorRequest(sPortData.datasetRequested);
		}
		break;

	    case SPORT_PROC_WAIT_LISTEN:
		// if another sensor is broadcasting on a known ID, disable it on our end
		if (data == SPORT_DATA_START)
		    sPortSetSensorDisabled(sPortData.datasetRequested);
		if (timerMicros() - sPortData.rxFirstContact > SPORT_SENSOR_DETECT_WAIT)
		    sPortData.sensorSearchTimeout = 1;
		sPortData.processState = SPORT_PROC_WAIT_RCV;
		break;

	    default:
		sPortData.processState = SPORT_PROC_WAIT_RCV;
		break;
	}
    }
}

void sPortInit(void) {
    memset((void *) &sPortData, 0, sizeof(sPortData));

    if (sPortCheckConfigFlag(SPORT_CFG_SEND_CUSTOM) && sPortCheckConfigFlag(SPORT_CFG_SEND_TXT_MSG))
	commRegisterNoticeFunc(sPortSendTextMessage);
    commRegisterRcvrFunc(COMM_STREAM_TYPE_RC_RX_TELEM, sPortProcessInput);

    AQ_NOTICE("FrSky SPort init\n");
}

