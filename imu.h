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

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#ifndef _imu_h
#define _imu_h

#include "aq.h"
#include "adc.h"
#include "vn100.h"

#define IMU_STATIC_STD		0.05f
#define IMU_STATIC_TIMEOUT	5	// seconds

#ifdef HAS_VN100
#define IMU_HAS_VN100
#ifdef USE_VN100
#define IMU_USE_VN100
#endif	// USE_VN100
#endif	// HAS_VN100

// these define where to get certain data
#define AQ_TIMESTEP		adcData.dt
#define AQ_YAW			navUkfData.yaw
#define AQ_PITCH		navUkfData.pitch
#define AQ_ROLL			navUkfData.roll
#define AQ_PRES_ADJ		UKF_PRES_ALT
#define AQ_PRESSURE		adcData.pressure

#ifdef IMU_USE_VN100
// using VN100 as IMU
#define IMU_DRATEX		vn100Data.doubleRates[0]
#define IMU_DRATEY		vn100Data.doubleRates[1]
#define IMU_DRATEZ		vn100Data.doubleRates[2]
#define IMU_RATEX		vn100Data.rates[0]
#define IMU_RATEY		vn100Data.rates[1]
#define IMU_RATEZ		vn100Data.rates[2]
#define IMU_ACCX		vn100Data.accs[0]
#define IMU_ACCY		vn100Data.accs[1]
#define IMU_ACCZ		vn100Data.accs[2]
#define IMU_MAGX		vn100Data.mags[0]
#define IMU_MAGY		vn100Data.mags[1]
#define IMU_MAGZ		vn100Data.mags[2]
#define IMU_TEMP		vn100Data.recvBuf.parameters[9]
#define IMU_LASTUPD		vn100Data.solutionTime

#define IMU_DRATEX_AUX		adcData.dRateX
#define IMU_DRATEY_AUX		adcData.dRateY
#define IMU_DRATEZ_AUX		adcData.dRateZ
#define IMU_RATEX_AUX		adcData.rateX
#define IMU_RATEY_AUX		adcData.rateY
#define IMU_RATEZ_AUX		adcData.rateZ
#define IMU_ACCX_AUX		adcData.accX
#define IMU_ACCY_AUX		adcData.accY
#define IMU_ACCZ_AUX		adcData.accZ
#define IMU_MAGX_AUX		adcData.magX
#define IMU_MAGY_AUX		adcData.magY
#define IMU_MAGZ_AUX		adcData.magZ
#define IMU_TEMP_AUX		adcData.temperature

#else
// using ADC as IMU
#define IMU_DRATEX		adcData.dRateX
#define IMU_DRATEY		adcData.dRateY
#define IMU_DRATEZ		adcData.dRateZ
#define IMU_RATEX		adcData.rateX
#define IMU_RATEY		adcData.rateY
#define IMU_RATEZ		adcData.rateZ
#define IMU_ACCX		adcData.accX
#define IMU_ACCY		adcData.accY
#define IMU_ACCZ		adcData.accZ
#define IMU_MAGX		adcData.magX
#define IMU_MAGY		adcData.magY
#define IMU_MAGZ		adcData.magZ
#define IMU_TEMP		adcData.temperature
#define IMU_LASTUPD		adcData.lastUpdate

#ifdef IMU_HAS_VN100
#define IMU_DRATEX_AUX		vn100Data.doubleRates[0]
#define IMU_DRATEY_AUX		vn100Data.doubleRates[1]
#define IMU_DRATEZ_AUX		vn100Data.doubleRates[2]
#define IMU_RATEX_AUX		vn100Data.rates[0]
#define IMU_RATEY_AUX		vn100Data.rates[1]
#define IMU_RATEZ_AUX		vn100Data.rates[2]
#define IMU_ACCX_AUX		vn100Data.accs[0]
#define IMU_ACCY_AUX		vn100Data.accs[1]
#define IMU_ACCZ_AUX		vn100Data.accs[2]
#define IMU_MAGX_AUX		vn100Data.mags[0]
#define IMU_MAGY_AUX		vn100Data.mags[1]
#define IMU_MAGZ_AUX		vn100Data.mags[2]
#define IMU_TEMP_AUX		vn100Data.regBuf.parameters[9]
#else
#define IMU_DRATEX_AUX		(0.0f)
#define IMU_DRATEY_AUX		(0.0f)
#define IMU_DRATEZ_AUX		(0.0f)
#define IMU_RATEX_AUX		(0.0f)
#define IMU_RATEY_AUX		(0.0f)
#define IMU_RATEZ_AUX		(0.0f)
#define IMU_ACCX_AUX		(0.0f)
#define IMU_ACCY_AUX		(0.0f)
#define IMU_ACCZ_AUX		(0.0f)
#define IMU_MAGX_AUX		(0.0f)
#define IMU_MAGY_AUX		(0.0f)
#define IMU_MAGZ_AUX		(0.0f)
#define IMU_TEMP_AUX		(0.0f)
#endif	// IMU_HAS_VN100
#endif	// IMU_USE_VN100

typedef struct {
    OS_FlagID dRateFlag;
    OS_FlagID sensorFlag;
    uint32_t fullUpdates;
    uint32_t halfUpdates;
} imuStruct_t;

extern imuStruct_t imuData;

extern void imuInit(void);
extern void imuQuasiStatic(int n);
extern void imuAdcDRateReady(void);
extern void imuAdcSensorReady(void);
extern void imuVN100DRateReady(void);
extern void imuVN100SensorReady(void);

#endif
