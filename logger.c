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
#include "logger.h"
#include "sdio.h"
#include "filer.h"
#include "notice.h"
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
#include "util.h"
#include <CoOS.h>
#include <stdio.h>
#include <string.h>

loggerStruct_t loggerData;

TCHAR loggerBuf[(sizeof(loggerOutput_t) * LOGGER_BUF_SIZE)];

void loggerDo(void) {
    loggerOutput_t *out;
    int32_t head;
    char *buf;
    char ckA, ckB;
    int i;

    head = filerGetHead(loggerData.logHandle);
    buf = loggerBuf + head;
    out = (loggerOutput_t *)(buf);

    // log signature
    out->sig[0] = 'A';
    out->sig[1] = 'q';
    out->sig[2] = 'L';

    out->lastUpdate = IMU_LASTUPD;
    for (i = 0; i < ADC_SENSORS; i++)
	out->voltages[i] = adcData.voltages[i];
    out->rate[0] = IMU_RATEX;
    out->rate[1] = IMU_RATEY;
    out->rate[2] = IMU_RATEZ;
    out->acc[0] = IMU_ACCX;
    out->acc[1] = IMU_ACCY;
    out->acc[2] = IMU_ACCZ;
    out->mag[0] = IMU_MAGX;
    out->mag[1] = IMU_MAGY;
    out->mag[2] = IMU_MAGZ;
//    out->rateAux[0] = IMU_RATEX_AUX;
//    out->rateAux[1] = IMU_RATEY_AUX;
//    out->rateAux[2] = IMU_RATEZ_AUX;
    out->rateAux[0] = gpsData.pDOP;
    out->rateAux[1] = gpsData.hDOP;
    out->rateAux[2] = gpsData.vDOP;
//    out->magAux[0] = IMU_MAGX_AUX;
//    out->magAux[1] = IMU_MAGY_AUX;
//    out->magAux[2] = IMU_MAGZ_AUX;
    out->magAux[0] = gpsData.tDOP;
    out->magAux[1] = gpsData.nDOP;
    out->magAux[2] = gpsData.eDOP;
//    out->accAux[0] = IMU_ACCX_AUX;
//    out->accAux[1] = IMU_ACCY_AUX;
//    out->accAux[2] = IMU_ACCZ_AUX;
    out->accAux[0] = motorsData.pitch;
    out->accAux[1] = motorsData.roll;
    out->accAux[2] = motorsData.yaw;
    out->pressure[0] = adcData.pressure1;
    out->pressure[1] = adcData.pressure2;
    out->temp[0] = IMU_TEMP;
    out->temp[1] = adcData.temp1;
    out->temp[2] = adcData.temp2;
//    out->temp[3] = IMU_TEMP_AUX;
    *((unsigned long *)&(out->temp[3])) = gpsData.iTOW;
    out->vIn = adcData.vIn;
    out->quat[0] = UKF_Q1;
    out->quat[1] = UKF_Q2;
    out->quat[2] = UKF_Q3;
    out->quat[3] = UKF_Q4;
    out->gpsPosUpdate = gpsData.lastPosUpdate;
    out->lat = gpsData.lat;
    out->lon = gpsData.lon;
    out->gpsAlt = gpsData.height;
    out->gpsPosAcc = gpsData.hAcc;
    out->gpsVelUpdate = gpsData.lastVelUpdate;
    out->gpsVel[0] = gpsData.velN;
    out->gpsVel[1] = gpsData.velE;
    out->gpsVel[2] = gpsData.velD;
    out->gpsVelAcc = gpsData.sAcc;
    out->pos[0] = UKF_POSN;
    out->pos[1] = UKF_POSE;
    out->pos[2] = UKF_ALTITUDE;
    out->vel[0] = UKF_VELN;
    out->vel[1] = UKF_VELE;
    out->vel[2] = -UKF_VELD;
    out->motors[0] =  motorsData.value[0];
    out->motors[1] =  motorsData.value[1];
    out->motors[2] =  motorsData.value[2];
    out->motors[3] =  motorsData.value[3];
    out->motors[4] =  motorsData.value[4];
    out->motors[5] =  motorsData.value[5];
    out->motors[6] =  motorsData.value[6];
    out->motors[7] =  motorsData.value[7];
    out->motors[8] =  motorsData.value[8];
    out->motors[9] =  motorsData.value[9];
    out->motors[10] =  motorsData.value[10];
    out->motors[11] =  motorsData.value[11];
    out->motors[12] =  motorsData.value[12];
    out->motors[13] =  motorsData.value[13];
    out->throttle = motorsData.throttle;
    out->extra[0] = adcData.magSign;
    out->extra[1] = gpsData.vAcc;
    out->extra[2] = UKF_POSD;
    out->extra[3] = UKF_PRES_ALT;

    // calc checksum
    ckA = ckB = 0;
    for (i = 3; i < sizeof(loggerOutput_t) - 2; i++) {
	ckA += buf[i];
	ckB += ckA;
    }
    out->ckA = ckA;
    out->ckB = ckB;

    filerSetHead(loggerData.logHandle, (head + sizeof(loggerOutput_t)) % sizeof(loggerBuf));
}

void loggerInit(void) {
    loggerData.logHandle = filerGetHandle(LOGGER_FNAME);
    filerStream(loggerData.logHandle, loggerBuf, sizeof(loggerBuf));
}
