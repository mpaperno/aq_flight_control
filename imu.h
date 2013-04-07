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
#include "d_imu.h"

#define IMU_ROOM_TEMP		20.0f
#define IMU_STATIC_STD		0.05f
#define IMU_STATIC_TIMEOUT	5	// seconds

// these define where to get certain data
#define AQ_YAW			navUkfData.yaw
#define AQ_PITCH		navUkfData.pitch
#define AQ_ROLL			navUkfData.roll
#define AQ_PRES_ADJ		UKF_PRES_ALT

#ifdef USE_VN100
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
#define AQ_TIMESTEP		adcData.dt
#define AQ_PRESSURE		adcData.pressure
#endif	// USE_VN100

#ifdef USE_DIGITAL_IMU
// using the Digital IMU as IMU
#define IMU_DRATEX		mpu6000Data.gyo[0]
#define IMU_DRATEY		mpu6000Data.gyo[1]
#define IMU_DRATEZ		mpu6000Data.gyo[2]
#define IMU_RATEX		mpu6000Data.gyo[0]
#define IMU_RATEY		mpu6000Data.gyo[1]
#define IMU_RATEZ		mpu6000Data.gyo[2]
#define IMU_ACCX		mpu6000Data.acc[0]
#define IMU_ACCY		mpu6000Data.acc[1]
#define IMU_ACCZ		mpu6000Data.acc[2]
#define IMU_MAGX		hmc5983Data.mag[0]
#define IMU_MAGY		hmc5983Data.mag[1]
#define IMU_MAGZ		hmc5983Data.mag[2]
#define IMU_TEMP		dImuData.temp
#define IMU_LASTUPD		dImuData.lastUpdate
#define AQ_TIMESTEP		DIMU_DT
#define AQ_PRESSURE		ms5611Data.pres
#endif	// USE_DIGITAL_IMU

#ifndef USE_VN100
#ifndef USE_DIGITAL_IMU
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
#define AQ_TIMESTEP		adcData.dt
#define AQ_PRESSURE		adcData.pressure
#endif
#endif

typedef struct {
    OS_FlagID dRateFlag;
    OS_FlagID sensorFlag;
    float sinRot, cosRot;
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
extern void imuDImuDRateReady(void);
extern void imuDImuSensorReady(void);

#endif
