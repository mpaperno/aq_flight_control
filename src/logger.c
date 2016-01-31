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
#include "logger.h"
#include "sdio.h"
#include "filer.h"
#include "imu.h"
#include "nav_ukf.h"
#include "gps.h"
#include "nav.h"
#include "ff.h"
#include "motors.h"
#include "command.h"
#include "control.h"
#include "aq_mavlink.h"
#include "supervisor.h"
#include "radio.h"
#include "util.h"
#include "analog.h"
#include "gimbal.h"
#include "canSensors.h"
#include "alt_ukf.h"
#include <CoOS.h>
#include <stdio.h>
#include <string.h>

loggerStruct_t loggerData __attribute__((section(".ccm")));

loggerFields_t loggerFields[] = {
    {LOG_LASTUPDATE, AQ_TYPE_U32},
    {LOG_VOLTAGE0, AQ_TYPE_FLT},
    {LOG_VOLTAGE1, AQ_TYPE_FLT},
    {LOG_VOLTAGE2, AQ_TYPE_FLT},
    {LOG_VOLTAGE3, AQ_TYPE_FLT},
    {LOG_VOLTAGE4, AQ_TYPE_FLT},
    {LOG_VOLTAGE5, AQ_TYPE_FLT},
    {LOG_VOLTAGE6, AQ_TYPE_FLT},
    {LOG_VOLTAGE7, AQ_TYPE_FLT},
    {LOG_VOLTAGE8, AQ_TYPE_FLT},
    {LOG_VOLTAGE9, AQ_TYPE_FLT},
    {LOG_VOLTAGE10, AQ_TYPE_FLT},
#ifdef HAS_AIMU
    {LOG_VOLTAGE11, AQ_TYPE_FLT},
    {LOG_VOLTAGE12, AQ_TYPE_FLT},
    {LOG_VOLTAGE13, AQ_TYPE_FLT},
    {LOG_VOLTAGE14, AQ_TYPE_FLT},
#endif
    {LOG_IMU_RATEX, AQ_TYPE_FLT},
    {LOG_IMU_RATEY, AQ_TYPE_FLT},
    {LOG_IMU_RATEZ, AQ_TYPE_FLT},
    {LOG_IMU_ACCX, AQ_TYPE_FLT},
    {LOG_IMU_ACCY, AQ_TYPE_FLT},
    {LOG_IMU_ACCZ, AQ_TYPE_FLT},
    {LOG_IMU_MAGX, AQ_TYPE_FLT},
    {LOG_IMU_MAGY, AQ_TYPE_FLT},
    {LOG_IMU_MAGZ, AQ_TYPE_FLT},
    {LOG_GPS_PDOP, AQ_TYPE_FLT},
    {LOG_GPS_HDOP, AQ_TYPE_FLT},
    {LOG_GPS_VDOP, AQ_TYPE_FLT},
    {LOG_GPS_TDOP, AQ_TYPE_FLT},
    {LOG_GPS_NDOP, AQ_TYPE_FLT},
    {LOG_GPS_EDOP, AQ_TYPE_FLT},
    {LOG_GPS_ITOW, AQ_TYPE_U32},
    {LOG_GPS_POS_UPDATE, AQ_TYPE_U32},
    {LOG_GPS_LAT, AQ_TYPE_DBL},
    {LOG_GPS_LON, AQ_TYPE_DBL},
    {LOG_GPS_HEIGHT, AQ_TYPE_FLT},
    {LOG_GPS_HACC, AQ_TYPE_FLT},
    {LOG_GPS_VACC, AQ_TYPE_FLT},
    {LOG_GPS_VEL_UPDATE, AQ_TYPE_U32},
    {LOG_GPS_VELN, AQ_TYPE_FLT},
    {LOG_GPS_VELE, AQ_TYPE_FLT},
    {LOG_GPS_VELD, AQ_TYPE_FLT},
    {LOG_GPS_SACC, AQ_TYPE_FLT},
    {LOG_ADC_PRESSURE1, AQ_TYPE_FLT},
#ifdef HAS_AIMU
    {LOG_ADC_PRESSURE2, AQ_TYPE_FLT},
#endif
    {LOG_ADC_TEMP0, AQ_TYPE_FLT},
    {LOG_ADC_VIN, AQ_TYPE_FLT},
#ifdef HAS_AIMU
    {LOG_ADC_MAG_SIGN, AQ_TYPE_S8},
#endif
    {LOG_UKF_Q1, AQ_TYPE_FLT},
    {LOG_UKF_Q2, AQ_TYPE_FLT},
    {LOG_UKF_Q3, AQ_TYPE_FLT},
    {LOG_UKF_Q4, AQ_TYPE_FLT},
    {LOG_UKF_POSN, AQ_TYPE_FLT},
    {LOG_UKF_POSE, AQ_TYPE_FLT},
    {LOG_UKF_POSD, AQ_TYPE_FLT},
    {LOG_UKF_PRES_ALT, AQ_TYPE_FLT},
    {LOG_UKF_ALT, AQ_TYPE_FLT},
    {LOG_UKF_ALT_VEL, AQ_TYPE_FLT},
    {LOG_UKF_VELN, AQ_TYPE_FLT},
    {LOG_UKF_VELE, AQ_TYPE_FLT},
    {LOG_UKF_VELD, AQ_TYPE_FLT},
    {LOG_MOT_MOTOR0, AQ_TYPE_U16},
    {LOG_MOT_MOTOR1, AQ_TYPE_U16},
    {LOG_MOT_MOTOR2, AQ_TYPE_U16},
    {LOG_MOT_MOTOR3, AQ_TYPE_U16},
    {LOG_MOT_MOTOR4, AQ_TYPE_U16},
    {LOG_MOT_MOTOR5, AQ_TYPE_U16},
    {LOG_MOT_MOTOR6, AQ_TYPE_U16},
    {LOG_MOT_MOTOR7, AQ_TYPE_U16},
    {LOG_MOT_MOTOR8, AQ_TYPE_U16},
    {LOG_MOT_MOTOR9, AQ_TYPE_U16},
    {LOG_MOT_MOTOR10, AQ_TYPE_U16},
    {LOG_MOT_MOTOR11, AQ_TYPE_U16},
    {LOG_MOT_MOTOR12, AQ_TYPE_U16},
    {LOG_MOT_MOTOR13, AQ_TYPE_U16},
    {LOG_MOT_THROTTLE, AQ_TYPE_FLT},
    {LOG_MOT_PITCH, AQ_TYPE_FLT},
    {LOG_MOT_ROLL, AQ_TYPE_FLT},
    {LOG_MOT_YAW, AQ_TYPE_FLT},
    {LOG_RADIO_QUALITY, AQ_TYPE_FLT},
    {LOG_RADIO_CHANNEL0, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL1, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL2, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL3, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL4, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL5, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL6, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL7, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL8, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL9, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL10, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL11, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL12, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL13, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL14, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL15, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL16, AQ_TYPE_S16},
    {LOG_RADIO_CHANNEL17, AQ_TYPE_S16},
    {LOG_RADIO_ERRORS, AQ_TYPE_U16},
    {LOG_GMBL_TRIGGER, AQ_TYPE_U16},
    {LOG_ACC_BIAS_X, AQ_TYPE_FLT},
    {LOG_ACC_BIAS_Y, AQ_TYPE_FLT},
    {LOG_ACC_BIAS_Z, AQ_TYPE_FLT},
    {LOG_CURRENT_PDB, AQ_TYPE_FLT},
#ifdef ANALOG_CHANNEL_EXT_AMP
    {LOG_CURRENT_EXT, AQ_TYPE_FLT},
#endif
    {LOG_VIN_PDB, AQ_TYPE_FLT},
};

int loggerCopy8(void *to, void *from) {
    *(uint32_t *)(to + 0) = *(uint32_t *)(from + 0);
    *(uint32_t *)(to + 4) = *(uint32_t *)(from + 4);

    return 8;
}

int loggerCopy4(void *to, void *from) {
    *(uint32_t *)to = *(uint32_t *)from;

    return 4;
}

int loggerCopy2(void *to, void *from) {
    *(uint16_t *)to = *(uint16_t *)from;

    return 2;
}

int loggerCopy1(void *to, void *from) {
    *(uint8_t *)to = *(uint8_t *)from;

    return 1;
}

void loggerDoHeader(void) {
    int32_t head;
    char *buf;
    char ckA, ckB;
    int i;

    // make sure we can proceed
    if (!filerAvailable())
	return;

    head = filerGetHead(loggerData.logHandle);
    buf = loggerData.loggerBuf + head;

    // log header signature
    *buf++ = 'A';
    *buf++ = 'q';
    *buf++ = 'H';

    // number of fields
    *buf++ = loggerData.numFields;

    // fields and types
    memcpy(buf, loggerFields, sizeof(loggerFields));

    // calc checksum
    ckA = ckB = 0;
    buf = loggerData.loggerBuf + head + 3;
    for (i = 0; i < (1 + sizeof(loggerFields)); i++) {
	ckA += buf[i];
	ckB += ckA;
    }
    buf[i++] = ckA;
    buf[i++] = ckB;

    // block size is the actual data packet size
    filerSetHead(loggerData.logHandle, (head + loggerData.packetSize) % loggerData.bufSize);
}

void loggerDo(void) {
    int32_t head;
    char *buf;
    char ckA, ckB;
    int i;

    // make sure we can proceed
    if (!filerAvailable())
	return;

    head = filerGetHead(loggerData.logHandle);
    buf = loggerData.loggerBuf + head;

    // log header signature
    *buf++ = 'A';
    *buf++ = 'q';
    *buf++ = 'M';

    // number of fields
    for (i = 0; i < loggerData.numFields; i++)
	buf += loggerData.fp[i].copyFunc(buf, loggerData.fp[i].fieldPointer);

    ckA = ckB = 0;
    buf = loggerData.loggerBuf + head;
    for (i = 3; i < loggerData.packetSize - 2; i++) {
	ckA += buf[i];
	ckB += ckA;
    }
    buf[i++] = ckA;
    buf[i++] = ckB;

    filerSetHead(loggerData.logHandle, (head + loggerData.packetSize) % loggerData.bufSize);
}

void loggerSetup(void) {
    int i;

    loggerData.numFields = sizeof(loggerFields) / sizeof(loggerFields_t);
    loggerData.packetSize = 3 + 2;  // signature + checksum

    loggerData.fp = (fieldData_t *)aqDataCalloc(loggerData.numFields, sizeof(fieldData_t));

    for (i = 0; i < loggerData.numFields; i++) {
	switch (loggerFields[i].fieldId) {
	    case LOG_LASTUPDATE:
		loggerData.fp[i].fieldPointer = (void *)&IMU_LASTUPD;
		break;
	    case LOG_VOLTAGE0:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RAW_RATEX;
		break;
	    case LOG_VOLTAGE1:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RAW_RATEY;
		break;
	    case LOG_VOLTAGE2:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RAW_RATEZ;
		break;
	    case LOG_VOLTAGE3:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RAW_MAGX;
		break;
	    case LOG_VOLTAGE4:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RAW_MAGY;
		break;
	    case LOG_VOLTAGE5:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RAW_MAGZ;
		break;
	    case LOG_VOLTAGE6:
#ifdef HAS_AIMU
		loggerData.fp[i].fieldPointer = (void *)&adcData.voltages[6];
#endif
		break;
	    case LOG_VOLTAGE7:
#ifdef HAS_AIMU
		loggerData.fp[i].fieldPointer = (void *)&adcData.voltages[7];
#else
		loggerData.fp[i].fieldPointer = (void *)&analogData.voltages[ANALOG_VOLTS_VIN];
#endif
		break;
	    case LOG_VOLTAGE8:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RAW_ACCX;
		break;
	    case LOG_VOLTAGE9:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RAW_ACCY;
		break;
	    case LOG_VOLTAGE10:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RAW_ACCZ;
		break;
#ifdef HAS_AIMU
	    case LOG_VOLTAGE11:
		loggerData.fp[i].fieldPointer = (void *)&adcData.voltages[11];
		break;
	    case LOG_VOLTAGE12:
		loggerData.fp[i].fieldPointer = (void *)&adcData.voltages[12];
		break;
	    case LOG_VOLTAGE13:
		loggerData.fp[i].fieldPointer = (void *)&adcData.voltages[13];
		break;
	    case LOG_VOLTAGE14:
		loggerData.fp[i].fieldPointer = (void *)&adcData.voltages[14];
		break;
#endif
	    case LOG_IMU_RATEX:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RATEX;
		break;
	    case LOG_IMU_RATEY:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RATEY;
		break;
	    case LOG_IMU_RATEZ:
		loggerData.fp[i].fieldPointer = (void *)&IMU_RATEZ;
		break;
	    case LOG_IMU_ACCX:
		loggerData.fp[i].fieldPointer = (void *)&IMU_ACCX;
		break;
	    case LOG_IMU_ACCY:
		loggerData.fp[i].fieldPointer = (void *)&IMU_ACCY;
		break;
	    case LOG_IMU_ACCZ:
		loggerData.fp[i].fieldPointer = (void *)&IMU_ACCZ;
		break;
	    case LOG_IMU_MAGX:
		loggerData.fp[i].fieldPointer = (void *)&IMU_MAGX;
		break;
	    case LOG_IMU_MAGY:
		loggerData.fp[i].fieldPointer = (void *)&IMU_MAGY;
		break;
	    case LOG_IMU_MAGZ:
		loggerData.fp[i].fieldPointer = (void *)&IMU_MAGZ;
		break;
	    case LOG_GPS_PDOP:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.pDOP;
		break;
	    case LOG_GPS_HDOP:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.hDOP;
		break;
	    case LOG_GPS_VDOP:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.vDOP;
		break;
	    case LOG_GPS_TDOP:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.tDOP;
		break;
	    case LOG_GPS_NDOP:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.nDOP;
		break;
	    case LOG_GPS_EDOP:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.eDOP;
		break;
	    case LOG_GPS_ITOW:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.iTOW;
		break;
	    case LOG_GPS_POS_UPDATE:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.lastPosUpdate;
		break;
	    case LOG_GPS_LAT:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.lat;
		break;
	    case LOG_GPS_LON:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.lon;
		break;
	    case LOG_GPS_HEIGHT:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.height;
		break;
	    case LOG_GPS_HACC:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.hAcc;
		break;
	    case LOG_GPS_VACC:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.vAcc;
		break;
	    case LOG_GPS_VEL_UPDATE:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.lastVelUpdate;
		break;
	    case LOG_GPS_VELN:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.velN;
		break;
	    case LOG_GPS_VELE:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.velE;
		break;
	    case LOG_GPS_VELD:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.velD;
		break;
	    case LOG_GPS_SACC:
		loggerData.fp[i].fieldPointer = (void *)&gpsData.sAcc;
		break;
	    case LOG_ADC_PRESSURE1:
		loggerData.fp[i].fieldPointer = (void *)&AQ_PRESSURE;
		break;
#ifdef HAS_AIMU
	    case LOG_ADC_PRESSURE2:
		loggerData.fp[i].fieldPointer = (void *)&adcData.pressure2;
		break;
#endif
	    case LOG_ADC_TEMP0:
		loggerData.fp[i].fieldPointer = (void *)&IMU_TEMP;
		break;
	    case LOG_ADC_VIN:
		loggerData.fp[i].fieldPointer = (void *)&analogData.vIn;
		break;
#ifdef HAS_AIMU
	    case LOG_ADC_MAG_SIGN:
		loggerData.fp[i].fieldPointer = (void *)&adcData.magSign;
		break;
#endif
	    case LOG_UKF_Q1:
		loggerData.fp[i].fieldPointer = (void *)&UKF_Q1;
		break;
	    case LOG_UKF_Q2:
		loggerData.fp[i].fieldPointer = (void *)&UKF_Q2;
		break;
	    case LOG_UKF_Q3:
		loggerData.fp[i].fieldPointer = (void *)&UKF_Q3;
		break;
	    case LOG_UKF_Q4:
		loggerData.fp[i].fieldPointer = (void *)&UKF_Q4;
		break;
	    case LOG_UKF_POSN:
		loggerData.fp[i].fieldPointer = (void *)&UKF_POSN;
		break;
	    case LOG_UKF_POSE:
		loggerData.fp[i].fieldPointer = (void *)&UKF_POSE;
		break;
	    case LOG_UKF_POSD:
		loggerData.fp[i].fieldPointer = (void *)&UKF_POSD;
		break;
	    case LOG_UKF_PRES_ALT:
		loggerData.fp[i].fieldPointer = (void *)&UKF_PRES_ALT;
		break;
	    case LOG_UKF_ALT:
		loggerData.fp[i].fieldPointer = (void *)&ALT_POS;
		break;
	    case LOG_UKF_ALT_VEL:
		loggerData.fp[i].fieldPointer = (void *)&ALT_VEL;
		break;
	    case LOG_UKF_VELN:
		loggerData.fp[i].fieldPointer = (void *)&UKF_VELN;
		break;
	    case LOG_UKF_VELE:
		loggerData.fp[i].fieldPointer = (void *)&UKF_VELE;
		break;
	    case LOG_UKF_VELD:
		loggerData.fp[i].fieldPointer = (void *)&UKF_VELD;
		break;
	    case LOG_MOT_MOTOR0:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[0];
		break;
	    case LOG_MOT_MOTOR1:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[1];
		break;
	    case LOG_MOT_MOTOR2:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[2];
		break;
	    case LOG_MOT_MOTOR3:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[3];
		break;
	    case LOG_MOT_MOTOR4:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[4];
		break;
	    case LOG_MOT_MOTOR5:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[5];
		break;
	    case LOG_MOT_MOTOR6:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[6];
		break;
	    case LOG_MOT_MOTOR7:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[7];
		break;
	    case LOG_MOT_MOTOR8:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[8];
		break;
	    case LOG_MOT_MOTOR9:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[9];
		break;
	    case LOG_MOT_MOTOR10:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[10];
		break;
	    case LOG_MOT_MOTOR11:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[11];
		break;
	    case LOG_MOT_MOTOR12:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[12];
		break;
	    case LOG_MOT_MOTOR13:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.value[13];
		break;
	    case LOG_MOT_THROTTLE:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.throttle;
		break;
	    case LOG_MOT_PITCH:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.pitch;
		break;
	    case LOG_MOT_ROLL:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.roll;
		break;
	    case LOG_MOT_YAW:
		loggerData.fp[i].fieldPointer = (void *)&motorsData.yaw;
		break;
	    case LOG_RADIO_QUALITY:
		loggerData.fp[i].fieldPointer = (void *)&RADIO_QUALITY;
		break;
	    case LOG_RADIO_CHANNEL0:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[0];
		break;
	    case LOG_RADIO_CHANNEL1:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[1];
		break;
	    case LOG_RADIO_CHANNEL2:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[2];
		break;
	    case LOG_RADIO_CHANNEL3:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[3];
		break;
	    case LOG_RADIO_CHANNEL4:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[4];
		break;
	    case LOG_RADIO_CHANNEL5:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[5];
		break;
	    case LOG_RADIO_CHANNEL6:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[6];
		break;
	    case LOG_RADIO_CHANNEL7:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[7];
		break;
	    case LOG_RADIO_CHANNEL8:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[8];
		break;
	    case LOG_RADIO_CHANNEL9:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[9];
		break;
	    case LOG_RADIO_CHANNEL10:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[10];
		break;
	    case LOG_RADIO_CHANNEL11:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[11];
		break;
	    case LOG_RADIO_CHANNEL12:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[12];
		break;
	    case LOG_RADIO_CHANNEL13:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[13];
		break;
	    case LOG_RADIO_CHANNEL14:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[14];
		break;
	    case LOG_RADIO_CHANNEL15:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[15];
		break;
	    case LOG_RADIO_CHANNEL16:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[16];
		break;
	    case LOG_RADIO_CHANNEL17:
		loggerData.fp[i].fieldPointer = (void *)&radioData.channels[17];
		break;
	    case LOG_RADIO_ERRORS:
		loggerData.fp[i].fieldPointer = (void *)&RADIO_ERROR_COUNT;
		break;
	    case LOG_GMBL_TRIGGER:
		loggerData.fp[i].fieldPointer = (void *)&gimbalData.triggerLogVal;
		break;
	    case LOG_ACC_BIAS_X:
		loggerData.fp[i].fieldPointer = (void *)&UKF_ACC_BIAS_X;
		break;
	    case LOG_ACC_BIAS_Y:
		loggerData.fp[i].fieldPointer = (void *)&UKF_ACC_BIAS_Y;
		break;
	    case LOG_ACC_BIAS_Z:
		loggerData.fp[i].fieldPointer = (void *)&UKF_ACC_BIAS_Z;
		break;
	    case LOG_CURRENT_PDB:
		loggerData.fp[i].fieldPointer = (void *)&canSensorsData.values[CAN_SENSORS_PDB_BATA];
		break;
	    case LOG_CURRENT_EXT:
		loggerData.fp[i].fieldPointer = (void *)&analogData.extAmp;
		break;
	    case LOG_VIN_PDB:
		loggerData.fp[i].fieldPointer = (void *)&canSensorsData.values[CAN_SENSORS_PDB_BATV];
		break;
	}

	switch (loggerFields[i].fieldType) {
	    case AQ_TYPE_DBL:
		loggerData.fp[i].copyFunc = loggerCopy8;
		loggerData.packetSize += 8;
		break;
	    case AQ_TYPE_FLT:
	    case AQ_TYPE_U32:
	    case AQ_TYPE_S32:
		loggerData.fp[i].copyFunc = loggerCopy4;
		loggerData.packetSize += 4;
		break;
	    case AQ_TYPE_U16:
	    case AQ_TYPE_S16:
		loggerData.fp[i].copyFunc = loggerCopy2;
		loggerData.packetSize += 2;
		break;
	    case AQ_TYPE_U8:
	    case AQ_TYPE_S8:
		loggerData.fp[i].copyFunc = loggerCopy1;
		loggerData.packetSize += 1;
		break;
	}
    }
}

void loggerInit(void) {
    memset((void *)&loggerData, 0, sizeof(loggerData));

    loggerSetup();

    // skip the first 512 bytes (used exclusively by the USB MSC driver)
    loggerData.loggerBuf = (TCHAR *)(filerBuf + 512);
    loggerData.bufSize = ((FILER_BUF_SIZE-512) / loggerData.packetSize / FILER_FLUSH_THRESHOLD) * loggerData.packetSize * FILER_FLUSH_THRESHOLD;

    loggerData.logHandle = filerGetHandle(LOGGER_FNAME);
    filerStream(loggerData.logHandle, loggerData.loggerBuf, loggerData.bufSize);

    loggerDoHeader();
}
