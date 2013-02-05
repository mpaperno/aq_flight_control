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
#include "downlink.h"
#include "notice.h"
#include "telemetry.h"
#include "serial.h"
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
#include <CoOS.h>
#include <string.h>
#include <stdio.h>

telemetryStruct_t telemetryData __attribute__((section(".ccm")));

void telemetryDo(void) {
    static unsigned long lastAqUpdate;
    static unsigned long aqCounter;

    telemetryData.loops++;

    if (!(telemetryData.loops % (unsigned int)p[TELEMETRY_RATE])) {
	aqCounter = counter;

	// calculate idle time
	telemetryData.idlePercent = (aqCounter - telemetryData.lastAqCounter) * (1e6 / (IMU_LASTUPD - lastAqUpdate)) / p[TELEMETRY_RATE] / (rccClocks.SYSCLK_Frequency / minCycles) * 100.0f;
	telemetryData.lastAqCounter = aqCounter;

	if (telemetryData.telemetryEnable) {
	    // grab port lock
	    CoEnterMutexSection(downlinkData.serialPortMutex);

	    downlinkSendString("AqT");	// telemetry header

	    downlinkResetChecksum();
	    downlinkSendFloat(AQ_ROLL);
	    downlinkSendFloat(AQ_PITCH);
	    downlinkSendFloat(AQ_YAW);
	    downlinkSendInt(RADIO_THROT);
	    downlinkSendInt(RADIO_RUDD);
	    downlinkSendInt(RADIO_PITCH);
	    downlinkSendInt(RADIO_ROLL);
	    downlinkSendInt(RADIO_FLAPS);
	    downlinkSendInt(RADIO_AUX4);
	    downlinkSendFloat(IMU_RATEX);
	    downlinkSendFloat(IMU_RATEY);
	    downlinkSendFloat(IMU_RATEZ);
	    downlinkSendFloat(IMU_ACCX);
	    downlinkSendFloat(IMU_ACCY);
	    downlinkSendFloat(IMU_ACCZ);
	    downlinkSendFloat(navData.holdHeading);
	    downlinkSendFloat(AQ_PRESSURE);
	    downlinkSendFloat(IMU_TEMP);
	    downlinkSendFloat(UKF_ALTITUDE);
	    downlinkSendFloat(adcData.vIn);
	    downlinkSendInt(gpsData.microsPerSecond>>11);  // us
//		downlinkSendInt(AQ_LASTUPD - gpsData.lastPosUpdate);  // us
	    downlinkSendFloat(UKF_POSN);
	    downlinkSendFloat(UKF_POSE);
	    downlinkSendFloat(UKF_ALTITUDE);
	    downlinkSendFloat(gpsData.lat);
	    downlinkSendFloat(gpsData.lon);
	    downlinkSendFloat(gpsData.hAcc);
	    downlinkSendFloat(gpsData.heading);
	    downlinkSendFloat(gpsData.height);
	    downlinkSendFloat(gpsData.pDOP);
	    downlinkSendFloat(navData.holdCourse);
	    downlinkSendFloat(navData.holdDistance);
	    downlinkSendFloat(navData.holdAlt);
	    downlinkSendFloat(navData.holdTiltN);
	    downlinkSendFloat(navData.holdTiltE);
	    downlinkSendFloat(UKF_VELN);
	    downlinkSendFloat(UKF_VELE);
	    downlinkSendFloat(-UKF_VELD);
	    downlinkSendFloat(IMU_MAGX);
	    downlinkSendFloat(IMU_MAGY);
	    downlinkSendFloat(IMU_MAGZ);
	    downlinkSendInt(1e6 / (IMU_LASTUPD - lastAqUpdate));
	    downlinkSendFloat(RADIO_QUALITY);
	    downlinkSendFloat(motorsData.value[0]);
	    downlinkSendFloat(motorsData.value[1]);
	    downlinkSendFloat(motorsData.value[2]);
	    downlinkSendFloat(motorsData.value[3]);
	    downlinkSendFloat(telemetryData.idlePercent);
	    downlinkSendFloat(UKF_ACC_BIAS_X);
	    downlinkSendFloat(UKF_ACC_BIAS_Y);
	    downlinkSendFloat(UKF_ACC_BIAS_Z);
	    downlinkSendFloat(supervisorData.flightTimeRemaining);
	    downlinkSendChecksum();

	    // release serial port
	    CoLeaveMutexSection(downlinkData.serialPortMutex);
	}

    }
    lastAqUpdate = IMU_LASTUPD;
}

void telemetryEnable(void) {
    telemetryData.telemetryEnable = 1;
}

void telemetryDisable(void) {
    telemetryData.telemetryEnable = 0;
}

void telemetryInit(void) {
    memset((void *)&telemetryData, 0, sizeof(telemetryData));
}
