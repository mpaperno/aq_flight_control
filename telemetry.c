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
#include "telemetry.h"
#include "util.h"
#include "aq_timer.h"
#include "control.h"
#include "config.h"
#include "radio.h"
#include "imu.h"
#include "gps.h"
#include "nav.h"
#include "compass.h"
#include "motors.h"
#include "rcc.h"
#include "aq_mavlink.h"
#include "nav_ukf.h"
#include "supervisor.h"
#include "analog.h"
#include "comm.h"
#include "alt_ukf.h"
#include "run.h"
#include <CoOS.h>
#include <string.h>
#include <stdio.h>

telemetryStruct_t telemetryData __attribute__((section(".ccm")));

static void telemtryChecksum(uint8_t c) {
    telemetryData.ckA += c;
    telemetryData.ckB += telemetryData.ckA;
}

static uint8_t *telemtrySendChar(uint8_t *ptr, uint8_t c) {
    *ptr++ = c;
    telemtryChecksum(c);

    return ptr;
}

static uint8_t *telemtrySendFloat(uint8_t *ptr, float f) {
    uint8_t *c = (uint8_t *)&f;
    int j;

    for (j = 0; j < sizeof(float); j++)
	ptr = telemtrySendChar(ptr, *c++);

    return ptr;
}

static uint8_t *telemtrySendInt(uint8_t *ptr, uint32_t i) {
    uint8_t *c = (uint8_t *)&i;
    uint8_t j;

    for (j = 0; j < sizeof(uint32_t); j++)
	ptr = telemtrySendChar(ptr, *c++);

    return ptr;
}

void telemetryDo(void) {
    static unsigned long lastAqUpdate;
    static unsigned long aqCounter;
    commTxBuf_t *txBuf;
    uint8_t *ptr;

    telemetryData.loops++;

    if (!(telemetryData.loops % (unsigned int)p[TELEMETRY_RATE])) {
	aqCounter = counter;

	// calculate idle time
	telemetryData.idlePercent = (aqCounter - telemetryData.lastAqCounter) * (1e6 / (IMU_LASTUPD - lastAqUpdate)) / p[TELEMETRY_RATE] / (rccClocks.SYSCLK_Frequency / minCycles) * 100.0f;
	telemetryData.lastAqCounter = aqCounter;

	if (telemetryData.telemetryEnable) {
	    txBuf = commGetTxBuf(COMM_STREAM_TYPE_TELEMETRY, 256);

	    // fail as we cannot block
	    if (txBuf != 0) {
		supervisorSendDataStart();

		ptr = &txBuf->buf;

		*ptr++ = 'A';
		*ptr++ = 'q';
		*ptr++ = 'T';

		telemetryData.ckA = telemetryData.ckB = 0;

		ptr = telemtrySendFloat(ptr, AQ_ROLL);
		ptr = telemtrySendFloat(ptr, AQ_PITCH);
		ptr = telemtrySendFloat(ptr, AQ_YAW);
		ptr = telemtrySendInt(ptr, RADIO_THROT);
		ptr = telemtrySendInt(ptr, RADIO_RUDD);
		ptr = telemtrySendInt(ptr, RADIO_PITCH);
		ptr = telemtrySendInt(ptr, RADIO_ROLL);
		ptr = telemtrySendInt(ptr, RADIO_FLAPS);
		ptr = telemtrySendInt(ptr, RADIO_AUX4);
		ptr = telemtrySendFloat(ptr, IMU_RATEX);
		ptr = telemtrySendFloat(ptr, IMU_RATEY);
		ptr = telemtrySendFloat(ptr, IMU_RATEZ);
		ptr = telemtrySendFloat(ptr, IMU_ACCX);
		ptr = telemtrySendFloat(ptr, IMU_ACCY);
		ptr = telemtrySendFloat(ptr, IMU_ACCZ);
		ptr = telemtrySendFloat(ptr, navData.holdHeading);
		ptr = telemtrySendFloat(ptr, AQ_PRESSURE);
		ptr = telemtrySendFloat(ptr, IMU_TEMP);
		ptr = telemtrySendFloat(ptr, ALTITUDE);
		ptr = telemtrySendFloat(ptr, analogData.vIn);
		ptr = telemtrySendInt(ptr, IMU_LASTUPD - gpsData.lastPosUpdate);  // us
		ptr = telemtrySendFloat(ptr, UKF_POSN);
		ptr = telemtrySendFloat(ptr, UKF_POSE);
		ptr = telemtrySendFloat(ptr, ALT_POS);
		ptr = telemtrySendFloat(ptr, gpsData.lat);
		ptr = telemtrySendFloat(ptr, gpsData.lon);
		ptr = telemtrySendFloat(ptr, gpsData.hAcc);
		ptr = telemtrySendFloat(ptr, gpsData.heading);
		ptr = telemtrySendFloat(ptr, gpsData.height);
		ptr = telemtrySendFloat(ptr, gpsData.pDOP);
		ptr = telemtrySendFloat(ptr, navData.holdCourse);
		ptr = telemtrySendFloat(ptr, navData.holdDistance);
		ptr = telemtrySendFloat(ptr, navData.holdAlt);
		ptr = telemtrySendFloat(ptr, navData.holdTiltN);
		ptr = telemtrySendFloat(ptr, navData.holdTiltE);
		ptr = telemtrySendFloat(ptr, UKF_VELN);
		ptr = telemtrySendFloat(ptr, UKF_VELE);
		ptr = telemtrySendFloat(ptr, -VELOCITYD);
		ptr = telemtrySendFloat(ptr, IMU_MAGX);
		ptr = telemtrySendFloat(ptr, IMU_MAGY);
		ptr = telemtrySendFloat(ptr, IMU_MAGZ);
		ptr = telemtrySendInt(ptr, 1e6 / (IMU_LASTUPD - lastAqUpdate));
		ptr = telemtrySendFloat(ptr, RADIO_QUALITY);
		ptr = telemtrySendFloat(ptr, motorsData.value[0]);
		ptr = telemtrySendFloat(ptr, motorsData.value[1]);
		ptr = telemtrySendFloat(ptr, motorsData.value[2]);
		ptr = telemtrySendFloat(ptr, motorsData.value[3]);
		ptr = telemtrySendFloat(ptr, telemetryData.idlePercent);
		ptr = telemtrySendFloat(ptr, UKF_ACC_BIAS_X);
		ptr = telemtrySendFloat(ptr, UKF_ACC_BIAS_Y);
		ptr = telemtrySendFloat(ptr, UKF_ACC_BIAS_Z);
		ptr = telemtrySendFloat(ptr, supervisorData.flightTimeRemaining);

		*ptr++ = telemetryData.ckA;
		*ptr++ = telemetryData.ckB;

		commSendTxBuf(txBuf, ptr - &txBuf->buf);
		supervisorSendDataStop();
	    }
	}
    }

    lastAqUpdate = IMU_LASTUPD;
}

void telemetrySendNotice(const char *s) {
    commTxBuf_t *txBuf;
    uint8_t ckA, ckB;
    uint8_t *ptr;

    txBuf = commGetTxBuf(COMM_STREAM_TYPE_TELEMETRY, 64);

    if (txBuf > 0) {
	ptr = &txBuf->buf;

	supervisorSendDataStart();

	*ptr++ = 'A';
	*ptr++ = 'q';
	*ptr++ = 'I';

	ckA = ckB = 0;
	do {
	    *ptr++ = *s;
	    ckA += *s;
	    ckB += ckA;
	} while (*(s++));

	*ptr++ = ckA;
	*ptr++ = ckB;

	commSendTxBuf(txBuf, ptr - &txBuf->buf);
	supervisorSendDataStop();
    }
}

void telemetryEnable(void) {
    telemetryData.telemetryEnable = 1;
}

void telemetryDisable(void) {
    telemetryData.telemetryEnable = 0;
}

void telemetryInit(void) {
    memset((void *)&telemetryData, 0, sizeof(telemetryData));

    commRegisterTelemFunc(telemetryDo);
    commRegisterNoticeFunc(telemetrySendNotice);
}
