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

#include "imu.h"
#include "arm_math.h"
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
    } while (i <= n || (stdX + stdY + stdZ) > IMU_STATIC_STD);
}

void imuInit(void) {
    memset((void *)&imuData, 0, sizeof(imuData));

    imuData.dRateFlag = CoCreateFlag(1, 0);
    imuData.sensorFlag = CoCreateFlag(1, 0);

    adcInit();
#ifdef IMU_HAS_VN100
    vn100Init();
#endif  // IMU_HAS_VN100
}

void imuAdcDRateReady(void) {
#ifndef IMU_USE_VN100
    imuData.halfUpdates++;
    CoSetFlag(imuData.dRateFlag);
#endif	// IMU_USE_VN100
}

void imuAdcSensorReady(void) {
#ifndef IMU_USE_VN100
    imuData.fullUpdates++;
    CoSetFlag(imuData.sensorFlag);
#endif	// IMU_USE_VN100
}

#ifdef IMU_HAS_VN100
void imuVN100DRateReady(void) {
#ifdef IMU_USE_VN100
    imuData.halfUpdates++;
    isr_SetFlag(imuData.dRateFlag);
#endif	// IMU_USE_VN100
}

void imuVN100SensorReady(void) {
#ifdef IMU_USE_VN100
    imuData.fullUpdates++;
    isr_SetFlag(imuData.sensorFlag);
#endif	// IMU_USE_VN100
}
#endif  // IMU_HAS_VN100
