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

#include "run.h"
#include "comm.h"
#include "nav_ukf.h"
#include "imu.h"
#include "gps.h"
#include "nav.h"
#include "control.h"
#include "logger.h"
#include "supervisor.h"
#include "gimbal.h"
#include "analog.h"
#include "config.h"
#include "aq_timer.h"
#include "aq_mavlink.h"
#include "calib.h"
#include "alt_ukf.h"
#include <CoOS.h>
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

OS_STK *runTaskStack;

runStruct_t runData __attribute__((section(".ccm")));

void runTaskCode(void *unused) {
    uint32_t axis = 0;
    uint32_t loops = 0;

    AQ_NOTICE("Run task started\n");

    while (1) {
	// wait for data
	CoWaitForSingleFlag(imuData.sensorFlag, 0);

	// soft start GPS accuracy
	runData.accMask *= 0.999f;

	if (runData.ukfInitFlag) {
	    runData.ukfInitFlag = false;
	    if (!(supervisorData.state & STATE_FLYING)) {
		runInitHistory();
		navUkfInitState();
	    }
	}

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

	if (!((loops+1) % 20)) {
	   simDoAccUpdate(runData.sumAcc[0]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumAcc[1]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumAcc[2]*(1.0f / (float)RUN_SENSOR_HIST));
	}
	else if (!((loops+7) % 20)) {
	   simDoPresUpdate(runData.sumPres*(1.0f / (float)RUN_SENSOR_HIST));
	}
#ifndef USE_DIGITAL_IMU
	else if (!((loops+13) % 20) && AQ_MAG_ENABLED) {
	   simDoMagUpdate(runData.sumMag[0]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumMag[1]*(1.0f / (float)RUN_SENSOR_HIST), runData.sumMag[2]*(1.0f / (float)RUN_SENSOR_HIST));
	}
#endif
	// optical flow update
	else if (navUkfData.flowCount >= 10 && !navUkfData.flowLock) {
	    navUkfFlowUpdate();
	}
	// only accept GPS updates if there is no optical flow
	else if (CoAcceptSingleFlag(gpsTaskData.gpsPosFlag) == E_OK && navUkfData.flowQuality == 0.0f && gpsData.hAcc < NAV_MIN_GPS_ACC && gpsData.tDOP != 0.0f) {
	    navUkfGpsPosUpdate(gpsData.lastPosUpdate, gpsData.lat, gpsData.lon, gpsData.height, gpsData.hAcc + runData.accMask, gpsData.vAcc + runData.accMask);
	    CoClearFlag(gpsTaskData.gpsPosFlag);
	    // refine static sea level pressure based on better GPS altitude fixes
	    if (gpsData.hAcc < runData.bestHacc && gpsData.hAcc < NAV_MIN_GPS_ACC) {
                navPressureAdjust(gpsData.height);
		runData.bestHacc = gpsData.hAcc;
	    }
	}
	else if (CoAcceptSingleFlag(gpsTaskData.gpsVelFlag) == E_OK && navUkfData.flowQuality == 0.0f && gpsData.sAcc < NAV_MIN_GPS_ACC/2 && gpsData.tDOP != 0.0f) {
	    navUkfGpsVelUpdate(gpsData.lastVelUpdate, gpsData.velN, gpsData.velE, gpsData.velD, gpsData.sAcc + runData.accMask);
	    CoClearFlag(gpsTaskData.gpsVelFlag);
	}
	// observe zero position
	else if (!((loops+4) % 20) && (gpsData.hAcc >= NAV_MIN_GPS_ACC || gpsData.tDOP == 0.0f) && navUkfData.flowQuality == 0.0f) {
	    navUkfZeroPos();
	}
	// observe zero velocity
	else if (!((loops+10) % 20) && (gpsData.sAcc >= NAV_MIN_GPS_ACC/2 || gpsData.tDOP == 0.0f) && navUkfData.flowQuality == 0.0f) {
	    navUkfZeroVel();
	}
	// observe that the rates are exactly 0 if not flying or moving
	else if (!(supervisorData.state & STATE_FLYING)) {
	    float stdX, stdY, stdZ;

	    arm_std_f32(runData.accHist[0], RUN_SENSOR_HIST, &stdX);
	    arm_std_f32(runData.accHist[1], RUN_SENSOR_HIST, &stdY);
	    arm_std_f32(runData.accHist[2], RUN_SENSOR_HIST, &stdZ);

	    if ((stdX + stdY + stdZ) < (IMU_STATIC_STD*2)) {
		if (!((axis + 0) % 3))
		    navUkfZeroRate(IMU_RATEX, 0);
		else if (!((axis + 1) % 3))
		    navUkfZeroRate(IMU_RATEY, 1);
		else
		    navUkfZeroRate(IMU_RATEZ, 2);
		axis++;
	    }
	}

        navUkfFinish();
        altUkfProcess(AQ_PRESSURE);

        // determine which altitude estimate to use
        if (gpsData.hAcc > p[NAV_ALT_GPS_ACC]) {
            runData.altPos = &ALT_POS;
            runData.altVel = &ALT_VEL;
        }
        else {
            runData.altPos = &UKF_ALTITUDE;
            runData.altVel = &UKF_VELD;
        }

	CoSetFlag(runData.runFlag);	// new state data

	navNavigate();
#ifndef HAS_AIMU
	analogDecode();
#endif
	if (!(loops % (int)(1.0f / AQ_OUTER_TIMESTEP)))
	    loggerDoHeader();
	loggerDo();
	gimbalUpdate();

#ifdef CAN_CALIB
	canTxIMUData(loops);
#endif
	if (supervisorData.state & STATE_CALIBRATION)
	    calibrate();

	loops++;
    }
}

void runInitUkf(void) {
    runData.ukfInitFlag = true;
}

void runInitHistory(void) {
    float acc[3], mag[3];
    float pres;
    int i;

    acc[0] = IMU_ACCX;
    acc[1] = IMU_ACCY;
    acc[2] = IMU_ACCZ;

    mag[0] = IMU_MAGX;
    mag[1] = IMU_MAGY;
    mag[2] = IMU_MAGZ;

    pres = AQ_PRESSURE;

    // initialize sensor history
    for (i = 0; i < 3; ++i) {
	runData.sumAcc[i] = 0.0f;
	runData.sumMag[i] = 0.0f;
    }
    runData.sumPres = 0.0f;
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

    runData.bestHacc = 1000.0f;
    runData.accMask = 1000.0f;

    // use altUkf altitude & vertical velocity estimates to start with
    runData.altPos = &ALT_POS;
    runData.altVel = &ALT_VEL;

}

void runInit(void) {
    memset((void *)&runData, 0, sizeof(runData));

    runData.runFlag = CoCreateFlag(1, 0);	    // auto reset
    runTaskStack = aqStackInit(RUN_TASK_SIZE, "RUN");

    runData.runTask = CoCreateTask(runTaskCode, (void *)0, RUN_PRIORITY, &runTaskStack[RUN_TASK_SIZE-1], RUN_TASK_SIZE);

    runInitHistory();

#ifdef USE_MAVLINK
    // configure px4flow sensor
//    mavlinkSendParameter(81, 50, "BFLOW_V_THLD", 2500.0f);
//    mavlinkSendParameter(81, 50, "BFLOW_F_THLD", 100.0f);
    mavlinkSendParameter(81, 50, "BFLOW_GYRO_COM", 0.0f);
    mavlinkSendParameter(81, 50, "USB_SEND_VIDEO", 0.0f);
#endif

}
