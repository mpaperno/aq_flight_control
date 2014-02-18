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

#include "imu.h"
#include "arm_math.h"
#include "config.h"
#include <string.h>

imuStruct_t imuData __attribute__((section(".ccm")));

// wait for lack of movement
void imuQuasiStatic(int n) {
    uint32_t lastUpdate;
    float stdX, stdY, stdZ;
    float vX[n];
    float vY[n];
    float vZ[n];
    int i, j;

    i = 0;
    j = 0;
    do {
	lastUpdate = IMU_LASTUPD;
	while (lastUpdate == IMU_LASTUPD)
	    ;

	vX[j] = IMU_ACCX;
	vY[j] = IMU_ACCY;
	vZ[j] = IMU_ACCZ;
	j = (j + 1) % n;

	if (i >= n) {
	    arm_std_f32(vX, n, &stdX);
	    arm_std_f32(vY, n, &stdY);
	    arm_std_f32(vZ, n, &stdZ);
	}

	i++;
    } while (i < (int)(1.0f / AQ_OUTER_TIMESTEP)*IMU_STATIC_TIMEOUT && (i <= n || (stdX + stdY + stdZ) > IMU_STATIC_STD));
}

void imuCalcRot(void) {
    float rotAngle;

    rotAngle = p[IMU_ROT] * DEG_TO_RAD;

    imuData.sinRot = sinf(rotAngle);
    imuData.cosRot = cosf(rotAngle);
}

void imuInit(void) {
    memset((void *)&imuData, 0, sizeof(imuData));

    imuData.dRateFlag = CoCreateFlag(1, 0);
    imuData.sensorFlag = CoCreateFlag(1, 0);

     // calculate IMU rotation
    imuCalcRot();

#ifdef HAS_AIMU
    adcInit();
#endif

#ifdef HAS_DIGITAL_IMU
    dIMUInit();
#endif	// HAS_DIGITAL_IMU
}

void imuAdcDRateReady(void) {
#ifndef USE_DIGITAL_IMU
    imuData.halfUpdates++;
    CoSetFlag(imuData.dRateFlag);
#endif
}

void imuAdcSensorReady(void) {
#ifndef USE_DIGITAL_IMU
    imuData.fullUpdates++;
    CoSetFlag(imuData.sensorFlag);
#endif
}

void imuDImuDRateReady(void) {
#ifdef USE_DIGITAL_IMU
    imuData.halfUpdates++;
    CoSetFlag(imuData.dRateFlag);
#endif	// USE_DIGITAL_IMU
}

void imuDImuSensorReady(void) {
#ifdef USE_DIGITAL_IMU
    imuData.fullUpdates++;
    CoSetFlag(imuData.sensorFlag);
#endif	// USE_DIGITAL_IMU
}
