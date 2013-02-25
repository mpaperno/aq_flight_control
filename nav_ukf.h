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

#ifndef _nav_ukf_h
#define _nav_ukf_h

#include "aq.h"
#include "srcdkf.h"

//#define UKF_LOG_BUF		(10*sizeof(float)*100)	// comment out to disable logging
//#define UKF_FNAME		"UKF"

#define SIM_S                   17		// states
#define SIM_M                   3		// max measurements
#define SIM_V                   16		// process noise
#define SIM_N                   3		// max observation noise

#define UKF_GYO_AVG_NUM		20

#define UKF_VELN		navUkfData.x[0]
#define UKF_VELE		navUkfData.x[1]
#define UKF_VELD		navUkfData.x[2]
#define UKF_POSN		navUkfData.x[3]
#define UKF_POSE		navUkfData.x[4]
#define UKF_POSD		navUkfData.x[5]
#define UKF_ACC_BIAS_X		navUkfData.x[6]
#define UKF_ACC_BIAS_Y		navUkfData.x[7]
#define UKF_ACC_BIAS_Z		navUkfData.x[8]
#define UKF_GYO_BIAS_X		navUkfData.x[9]
#define UKF_GYO_BIAS_Y		navUkfData.x[10]
#define UKF_GYO_BIAS_Z		navUkfData.x[11]
#define UKF_Q1			navUkfData.x[12]
#define UKF_Q2			navUkfData.x[13]
#define UKF_Q3			navUkfData.x[14]
#define UKF_Q4			navUkfData.x[15]
#define UKF_PRES_ALT		navUkfData.x[16]

#ifdef USE_PRES_ALT
#define UKF_ALTITUDE	UKF_PRES_ALT
#else
#define UKF_ALTITUDE	UKF_POSD
#endif

#define UKF_HIST		40

#define UKF_P0			101325.0f		    // standard static pressure at sea level

typedef struct {
    srcdkf_t *kf;
    float v0a[3];
    float v0m[3];
    double holdLat, holdLon;
    double r1, r2;
    float posN[UKF_HIST];
    float posE[UKF_HIST];
    float posD[UKF_HIST];
    float velN[UKF_HIST];
    float velE[UKF_HIST];
    float velD[UKF_HIST];
    int navHistIndex;
    float yaw, pitch, roll;
    float yawCos, yawSin;
    float mat3x3[3*3];
    float *x;			// states
    float presAltOffset;
    int logPointer;
    uint8_t logHandle;
} navUkfStruct_t;

extern navUkfStruct_t navUkfData;

extern void navUkfInit(void);
extern void navUkfInertialUpdate(void);
extern void simDoPresUpdate(float pres);
extern void simDoAccUpdate(float accX, float accY, float accZ);
extern void simDoMagUpdate(float magX, float magY, float magZ);
extern void navUkfGpsPosUpate(uint32_t gpsMicros, double lat, double lon, float alt, float hAcc, float vAcc);
extern void navUkfGpsVelUpate(uint32_t gpsMicros, float velN, float velE, float velD, float sAcc);
extern void navUkfSetGlobalPositionTarget(double lat, double lon);
extern void UKFPressureAdjust(float altitude);
extern void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll);
extern void navUkfZeroRate(float zRate, int axis);
extern void navUkfFinish(void);
extern void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q);

#endif
