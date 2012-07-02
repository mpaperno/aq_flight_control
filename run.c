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
#include "run.h"
#include "notice.h"
#include "nav_ukf.h"
#include "imu.h"
#include "gps.h"
#include "nav.h"
#include "control.h"
#include "telemetry.h"
#include "aq_mavlink.h"
#include "logger.h"
#include "supervisor.h"
#include "gimbal.h"
#include <CoOS.h>
#include <intrinsics.h>

OS_STK *runTaskStack;

runStruct_t runData __attribute__((section(".ccm")));

void runTaskCode(void *p) {
    uint32_t loops = 0;

    AQ_NOTICE("Run task started\n");

    while (1) {
	// wait for data
	CoWaitForSingleFlag(imuData.sensorFlag, 0);

	navUkfInertialUpdate();

	// record history for acc & mag & pressure readings for smoothing purposes
	// acc
	runData.sumAcc[0] -= runData.accHist[0][runData.sensorHistIndex];
	runData.sumAcc[1] -= runData.accHist[1][runData.sensorHistIndex];
	runData.sumAcc[2] -= runData.accHist[2][runData.sensorHistIndex];

	runData.accHist[0][runData.sensorHistIndex] = IMU_ACCX;
	runData.accHist[1][runData.sensorHistIndex] = IMU_ACCY;
	runData.accHist[2][runData.sensorHistIndex] = IMU_ACCZ;

	runData.sumAcc[0] += runData.accHist[0][runData.sensorHistIndex];
	runData.sumAcc[1] += runData.accHist[1][runData.sensorHistIndex];
	runData.sumAcc[2] += runData.accHist[2][runData.sensorHistIndex];

	// mag
	runData.sumMag[0] -= runData.magHist[0][runData.sensorHistIndex];
	runData.sumMag[1] -= runData.magHist[1][runData.sensorHistIndex];
	runData.sumMag[2] -= runData.magHist[2][runData.sensorHistIndex];

	runData.magHist[0][runData.sensorHistIndex] = IMU_MAGX;
	runData.magHist[1][runData.sensorHistIndex] = IMU_MAGY;
	runData.magHist[2][runData.sensorHistIndex] = IMU_MAGZ;

	runData.sumMag[0] += runData.magHist[0][runData.sensorHistIndex];
	runData.sumMag[1] += runData.magHist[1][runData.sensorHistIndex];
	runData.sumMag[2] += runData.magHist[2][runData.sensorHistIndex];

	// pressure
	runData.sumPres -= runData.presHist[runData.sensorHistIndex];
	runData.presHist[runData.sensorHistIndex] = AQ_PRESSURE;
	runData.sumPres += runData.presHist[runData.sensorHistIndex];

	runData.sensorHistIndex = (runData.sensorHistIndex + 1) % RUN_SENSOR_HIST;

	if (!(loops % 20)) {
	   simDoAccUpdate(runData.sumAcc[0]*(1.0 / (float)RUN_SENSOR_HIST), runData.sumAcc[1]*(1.0 / (float)RUN_SENSOR_HIST), runData.sumAcc[2]*(1.0 / (float)RUN_SENSOR_HIST));
	}
	else if (!((loops+7) % 20)) {
	   simDoPresUpdate(runData.sumPres*(1.0 / (float)RUN_SENSOR_HIST));
	}
	else if (!((loops+14) % 20)) {
	   simDoMagUpdate(runData.sumMag[0]*(1.0 / (float)RUN_SENSOR_HIST), runData.sumMag[1]*(1.0 / (float)RUN_SENSOR_HIST), runData.sumMag[2]*(1.0 / (float)RUN_SENSOR_HIST));
	}
	else if (CoAcceptSingleFlag(gpsData.gpsPosFlag) == E_OK) {
	    if (runData.accMask > 1.0f && gpsData.hAcc < 5.0f) {
		// 50 readings before mask is completely dropped
		runData.accMask -= RUN_ACC_MASK / 50.0f;
		if (runData.accMask < 1.0f)
		    runData.accMask = 1.0f;
	    }

	    navUkfGpsPosUpate(gpsData.lastPosUpdate, gpsData.lat, gpsData.lon, gpsData.height, gpsData.hAcc*runData.accMask, gpsData.vAcc*runData.accMask);
	    CoClearFlag(gpsData.gpsPosFlag);
	    // refine static sea level pressure based on better GPS altitude fixes
	    if (gpsData.hAcc < runData.bestHacc && gpsData.hAcc < NAV_MIN_GPS_ACC) {
		UKFPressureAdjust(gpsData.height);
		runData.bestHacc = gpsData.hAcc;
	    }
	}
	else if (CoAcceptSingleFlag(gpsData.gpsVelFlag) == E_OK) {
	    navUkfGpsVelUpate(gpsData.lastVelUpdate, gpsData.velN, gpsData.velE, gpsData.velD, gpsData.sAcc*runData.accMask);
	    CoClearFlag(gpsData.gpsVelFlag);
	}
	// observe that the rates are exactly 0 if not flying or moving
	else if (!(supervisorData.state & STATE_FLYING)) {
	    static uint32_t axis = 0;
	    float stdX, stdY, stdZ;

	    arm_std_f32(runData.accHist[0], RUN_SENSOR_HIST, &stdX);
	    arm_std_f32(runData.accHist[1], RUN_SENSOR_HIST, &stdY);
	    arm_std_f32(runData.accHist[2], RUN_SENSOR_HIST, &stdZ);

	    if ((stdX + stdY + stdZ) < (IMU_STATIC_STD*2))
		navUkfZeroRate(IMU_RATEZ, (axis++) % 3);
	}

	navUkfFinish();
	navNavigate();
	if (!(loops % 200))
	    loggerDoHeader();
	loggerDo();
	gimbalUpdate();
#ifdef USE_MAVLINK
	mavlinkDo();
#else
	telemetryDo();
#endif

	loops++;
    }
}

void runInit(void) {
    float acc[3], mag[3];
    float pres;
    int i;

    memset((void *)&runData, 0, sizeof(runData));

    runTaskStack = aqStackInit(RUN_TASK_SIZE);

    runData.runTask = CoCreateTask(runTaskCode, (void *)0, RUN_PRIORITY, &runTaskStack[RUN_TASK_SIZE-1], RUN_TASK_SIZE);

    acc[0] = IMU_ACCX;
    acc[1] = IMU_ACCY;
    acc[2] = IMU_ACCZ;

    mag[0] = IMU_MAGX;
    mag[1] = IMU_MAGY;
    mag[2] = IMU_MAGZ;

    pres = AQ_PRESSURE;

    for (i = 0; i < RUN_SENSOR_HIST; i++) {
	runData.accHist[0][i] = acc[0];
	runData.accHist[1][i] = acc[1];
	runData.accHist[2][i] = acc[2];
	runData.magHist[0][i] = mag[0];
	runData.magHist[1][i] = mag[1];
	runData.magHist[2][i] = mag[2];
	runData.presHist[i] = pres;

	runData.sumAcc[0] += acc[0];
	runData.sumAcc[1] += acc[1];
	runData.sumAcc[2] += acc[2];
	runData.sumMag[0] += mag[0];
	runData.sumMag[1] += mag[1];
	runData.sumMag[2] += mag[2];
	runData.sumPres += pres;
    }

    runData.sensorHistIndex = 0;

    runData.bestHacc = 99.9f;
    runData.accMask = RUN_ACC_MASK;
}
