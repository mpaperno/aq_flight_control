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

#include "nav_ukf.h"
#include "imu.h"
#include "nav.h"
#include "util.h"
#include "compass.h"
#include "aq_timer.h"
#include "config.h"
#include "gps.h"
#include "supervisor.h"
#include "filer.h"
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

navUkfStruct_t navUkfData;

#ifdef UKF_LOG_BUF
char ukfLog[UKF_LOG_BUF];
#endif

float navUkfPresToAlt(float pressure) {
    return (1.0f -  powf(pressure / UKF_P0, 0.19f)) * (1.0f / 22.558e-6f);
}

// reset current sea level static pressure based on better GPS estimate
void UKFPressureAdjust(float altitude) {
    navUkfData.presAltOffset = altitude - UKF_PRES_ALT;
}

void navUkfCalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    navUkfData.r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    navUkfData.r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

void navUkfCalcDistance(double lat, double lon, float *posNorth, float *posEast) {
    *posNorth = (lat - navUkfData.holdLat) * navUkfData.r1;
    *posEast = (lon - navUkfData.holdLon) * navUkfData.r2;
}

void navUkfResetPosition(float deltaN, float deltaE, float deltaD) {
    int i;

    for (i = 0; i < UKF_HIST; i++) {
	navUkfData.posN[i] += deltaN;
	navUkfData.posE[i] += deltaE;
	navUkfData.posD[i] += deltaD;
    }

    UKF_POSN += deltaN;
    UKF_POSE += deltaE;
    UKF_POSD += deltaD;

//    UKF_PRES_BIAS += deltaD;

    navResetHoldAlt(deltaD);
}

void navUkfSetGlobalPositionTarget(double lat, double lon) {
    float oldPosN, oldPosE;
    float newPosN, newPosE;

    navUkfCalcDistance(lat, lon, &oldPosN, &oldPosE);

    navUkfData.holdLat = lat;
    navUkfData.holdLon = lon;

    navUkfCalcDistance(lat, lon, &newPosN, &newPosE);

    navUkfResetPosition(newPosN - oldPosN, newPosE - oldPosE, 0.0f);
}

void navUkfNormalizeVec3(float *vr, float *v) {
    float norm;

    norm = __sqrtf(v[0]*v[0] + v[1]*v[1] + v[2]*v[2]);

    vr[0] = v[0] / norm;
    vr[1] = v[1] / norm;
    vr[2] = v[2] / norm;
}

void navUkfNormalizeQuat(float *qr, float *q) {
    float norm;

    norm = __sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

    qr[0] = q[0] / norm;
    qr[1] = q[1] / norm;
    qr[2] = q[2] / norm;
    qr[3] = q[3] / norm;
}

void crossVector3(float *vr, float *va, float *vb) {
    vr[0] = va[1] * vb[2] - vb[1] * va[2];
    vr[1] = va[2] * vb[0] - vb[2] * va[0];
    vr[2] = va[0] * vb[1] - vb[0] * va[1];
}

float dotVector3(float *va, float *vb) {
    return va[0]*vb[0] + va[1]*vb[1] + va[2]*vb[2];
}

void navUkfRotateVectorByQuat(float *vr, float *v, float *q) {
    float w, x, y, z;

    w = q[0];
    x = q[1];
    y = q[2];
    z = q[3];

    vr[0] = w*w*v[0] + 2.0f*y*w*v[2] - 2.0f*z*w*v[1] + x*x*v[0] + 2.0f*y*x*v[1] + 2.0f*z*x*v[2] - z*z*v[0] - y*y*v[0];
    vr[1] = 2.0f*x*y*v[0] + y*y*v[1] + 2.0f*z*y*v[2] + 2.0f*w*z*v[0] - z*z*v[1] + w*w*v[1] - 2.0f*x*w*v[2] - x*x*v[1];
    vr[2] = 2.0f*x*z*v[0] + 2.0f*y*z*v[1] + z*z*v[2] - 2.0f*w*y*v[0] - y*y*v[2] + 2.0f*w*x*v[1] - x*x*v[2] + w*w*v[2];
}

void navUkfRotateVectorByRevQuat(float *vr, float *v, float *q) {
    float qc[4];

    qc[0] = q[0];
    qc[1] = -q[1];
    qc[2] = -q[2];
    qc[3] = -q[3];

    navUkfRotateVectorByQuat(vr, v, qc);
}

void navUkfRotateVecByMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[0*3 + 1]*v[1] + m[0*3 + 2]*v[2];
    vr[1] = m[1*3 + 0]*v[0] + m[1*3 + 1]*v[1] + m[1*3 + 2]*v[2];
    vr[2] = m[2*3 + 0]*v[0] + m[2*3 + 1]*v[1] + m[2*3 + 2]*v[2];
}

void navUkfRotateVecByRevMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[1*3 + 0]*v[1] + m[2*3 + 0]*v[2];
    vr[1] = m[0*3 + 1]*v[0] + m[1*3 + 1]*v[1] + m[2*3 + 1]*v[2];
    vr[2] = m[0*3 + 2]*v[0] + m[1*3 + 2]*v[1] + m[2*3 + 2]*v[2];
}

void navUkfQuatToMatrix(float *m, float *q, int normalize) {
    float sqw = q[0]*q[0];
    float sqx = q[1]*q[1];
    float sqy = q[2]*q[2];
    float sqz = q[3]*q[3];
    float tmp1, tmp2;
    float invs;

    // get the invert square length
    if (normalize)
	    invs = 1.0f / (sqx + sqy + sqz + sqw);
    else
	    invs = 1.0f;

    // rotation matrix is scaled by inverse square length
    m[0*3 + 0] = ( sqx - sqy - sqz + sqw) * invs;
    m[1*3 + 1] = (-sqx + sqy - sqz + sqw) * invs;
    m[2*3 + 2] = (-sqx - sqy + sqz + sqw) * invs;

    tmp1 = q[1]*q[2];
    tmp2 = q[3]*q[0];
    m[1*3 + 0] = 2.0f * (tmp1 + tmp2) * invs;
    m[0*3 + 1] = 2.0f * (tmp1 - tmp2) * invs;

    tmp1 = q[1]*q[3];
    tmp2 = q[2]*q[0];
    m[2*3 + 0] = 2.0f * (tmp1 - tmp2) * invs;
    m[0*3 + 2] = 2.0f * (tmp1 + tmp2) * invs;

    tmp1 = q[2]*q[3];
    tmp2 = q[1]*q[0];
    m[2*3 + 1] = 2.0f * (tmp1 + tmp2) * invs;
    m[1*3 + 2] = 2.0f * (tmp1 - tmp2) * invs;
}

void navUkfMatrixExtractEuler(float *m, float *yaw, float *pitch, float *roll) {
    if (m[1*3+0] > 0.998f) { // singularity at north pole
	*pitch = atan2f(m[0*3+2], m[2*3+2]);
	*yaw = M_PI/2.0f;
	*roll = 0.0f;
    } else if (m[1*3+0] < -0.998f) { // singularity at south pole
	*pitch = atan2f(m[0*3+2] ,m[2*3+2]);
	*yaw = -M_PI/2.0f;
	*roll = 0.0f;
    }
    else {
	*pitch = atan2f(-m[2*3+0] ,m[0*3+0]);
	*yaw = asinf(m[1*3+0]);
	*roll = atan2f(-m[1*3+2], m[1*3+1]);
    }
}

void navUkfQuatExtractEuler(float *q, float *yaw, float *pitch, float *roll) {
    float q0, q1, q2, q3;

    q0 = q[1];
    q1 = q[2];
    q2 = q[3];
    q3 = q[0];

    *yaw = atan2f((2.0f * (q0 * q1 + q3 * q2)), (q3*q3 - q2*q2 - q1*q1 + q0*q0));
    *pitch = asinf(-2.0f * (q0 * q2 - q1 * q3));
    *roll = atanf((2.0f * (q1 * q2 + q0 * q3)) / (q3*q3 + q2*q2 - q1*q1 -q0*q0));
}

// result and source can be the same
void navUkfRotateQuat(float *qr, float *q, float *rate, float dt) {
    float q1[4];
    float s, t, lg;
    float qMag;

    s = __sqrtf(rate[0]*rate[0] + rate[1]*rate[1] + rate[2]*rate[2]) * 0.5f;
    t = -(0.5f * sinf(s) / s);
    rate[0] *= t;
    rate[1] *= t;
    rate[2] *= t;

    // create Lagrange factor to control quat's numerical integration errors
    qMag = q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3];
    lg = cosf(s) + (1.0f - qMag*qMag) * dt * dt;

    // rotate
    q1[0] = q[0];
    q1[1] = q[1];
    q1[2] = q[2];
    q1[3] = q[3];

    qr[0] =  lg*q1[0]      + rate[0]*q1[1] + rate[1]*q1[2] + rate[2]*q1[3];
    qr[1] = -rate[0]*q1[0] + lg*q1[1]      - rate[2]*q1[2] + rate[1]*q1[3];
    qr[2] = -rate[1]*q1[0] + rate[2]*q1[1] + lg*q1[2]      - rate[0]*q1[3];
    qr[3] = -rate[2]*q1[0] - rate[1]*q1[1] + rate[0]*q1[2] + lg*q1[3];
}

float t1, t2, t3;
void navUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt) {
    float tmp[3], acc[3];
    float rate[3];
    float mat3x3[3*3];

    // acc bias
    out[6] = in[6] + noise[0] * dt;
    out[7] = in[7] + noise[1] * dt;
    out[8] = in[8] + noise[2] * dt;

    // gbias
    out[9] = in[9] + noise[3] * dt;
    out[10] = in[10] + noise[4] * dt;
    out[11] = in[11] + noise[5] * dt;

    // rate = rate + bias + noise
    rate[0] = (u[3] + out[9]  + noise[6]) * dt;
    rate[1] = (u[4] + out[10] + noise[7]) * dt;
    rate[2] = (u[5] + out[11] + noise[8]) * dt;

    // rotate
    navUkfRotateQuat(&out[12], &in[12], rate, dt);
    navUkfQuatToMatrix(mat3x3, &out[12], 1);

    // acc
    tmp[0] = u[0] + out[6];
    tmp[1] = u[1] + out[7];
    tmp[2] = u[2] + out[8];

    // rotate acc to world frame
    navUkfRotateVecByMatrix(acc, tmp, mat3x3);
    acc[2] += GRAVITY;

    // vel
    out[0] = in[0] + acc[0] * dt + noise[10];
    out[1] = in[1] + acc[1] * dt + noise[11];
    out[2] = in[2] + acc[2] * dt + noise[12];

    // pos
    out[3] = in[3] + (in[0] + out[0]) * 0.5f * dt + noise[13];
    out[4] = in[4] + (in[1] + out[1]) * 0.5f * dt + noise[14];
    out[5] = in[5] - (in[2] + out[2]) * 0.5f * dt + noise[15];

    // pres alt
    out[16] = in[16] - (in[2] + out[2]) * 0.5f * dt + noise[9];
}

void navUkfRateUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = -x[9+(int)u[0]] + noise[0];
}

void navUkfAccUpdate(float *u, float *x, float *noise, float *y) {
    navUkfRotateVectorByRevQuat(y, navUkfData.v0a, &x[12]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}

void navUkfMagUpdate(float *u, float *x, float *noise, float *y) {
    navUkfRotateVectorByRevQuat(y, navUkfData.v0m, &x[12]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}

void navUkfPresUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[16] + noise[0]; // return altitude
}

void navUkfPresGPSAltUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[16] + noise[0];// return pres altitude
    y[1] = x[5] + noise[1]; // return GPS altitude
}

void navUkfPosUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[3] + noise[0]; // return position
    y[1] = x[4] + noise[1];
    y[2] = x[5] + noise[2];
}

void navUkfVelUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[0] + noise[0]; // return velocity
    y[1] = x[1] + noise[1];
    y[2] = x[2] + noise[2];
}

void navUkfFinish(void) {
    navUkfNormalizeQuat(&UKF_Q1, &UKF_Q1);
    navUkfQuatExtractEuler(&UKF_Q1, &navUkfData.yaw, &navUkfData.pitch, &navUkfData.roll);
    navUkfData.yaw = compassNormalize(navUkfData.yaw * RAD_TO_DEG);
    navUkfData.pitch *= RAD_TO_DEG;
    navUkfData.roll *= RAD_TO_DEG;

    //    x' = x cos f - y sin f
    //    y' = y cos f + x sin f
    navUkfData.yawCos = cosf(navUkfData.yaw * DEG_TO_RAD);
    navUkfData.yawSin = sinf(navUkfData.yaw * DEG_TO_RAD);
}

void navUkfInertialUpdate(void) {
    float u[6];

    u[0] = IMU_ACCX;
    u[1] = IMU_ACCY;
    u[2] = IMU_ACCZ;

    u[3] = IMU_RATEX;
    u[4] = IMU_RATEY;
    u[5] = IMU_RATEZ;

    srcdkfTimeUpdate(navUkfData.kf, u, AQ_TIMESTEP);

    // store history
    navUkfData.posN[navUkfData.navHistIndex] = UKF_POSN;
    navUkfData.posE[navUkfData.navHistIndex] = UKF_POSE;
    navUkfData.posD[navUkfData.navHistIndex] = UKF_POSD;

    navUkfData.velN[navUkfData.navHistIndex] = UKF_VELN;
    navUkfData.velE[navUkfData.navHistIndex] = UKF_VELE;
    navUkfData.velD[navUkfData.navHistIndex] = UKF_VELD;

    navUkfData.navHistIndex = (navUkfData.navHistIndex + 1) % UKF_HIST;
}

void navUkfZeroRate(float rate, int axis) {
    float noise[1];        // measurement variance
    float y[1];            // measurment(s)
    float u[1];		   // user data

    noise[0] = 0.00001f;
    y[0] = rate;
    u[0] = (float)axis;

    srcdkfMeasurementUpdate(navUkfData.kf, u, y, 1, 1, noise, navUkfRateUpdate);
}

void simDoPresUpdate(float pres) {
    float noise[2];        // measurement variance
    float y[2];            // measurment(s)

    noise[0] = p[UKF_ALT_N];
//    if (!(supervisorData.state & STATE_FLYING))
//	noise[0] *= 0.001f;

    noise[1] = noise[0];

    y[0] = navUkfPresToAlt(pres);
    y[1] = y[0];

    // if GPS altitude data has been available, only update pressure altitude
    if (navUkfData.presAltOffset != 0.0f)
	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 1, 1, noise, navUkfPresUpdate);
    // otherwise update pressure and GPS altitude from the single pressure reading
    else
	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 2, 2, noise, navUkfPresGPSAltUpdate);
}

void simDoAccUpdate(float accX, float accY, float accZ) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    // remove bias
    accX += UKF_ACC_BIAS_X;
    accY += UKF_ACC_BIAS_Y;
    accZ += UKF_ACC_BIAS_Z;

    // normalize vector
    norm =  __sqrtf(accX*accX + accY*accY + accZ*accZ);
    y[0] = accX / norm;
    y[1] = accY / norm;
    y[2] = accZ / norm;

    noise[0] = p[UKF_ACC_N] + fabsf(GRAVITY - norm) * p[UKF_DIST_N];
    if (!(supervisorData.state & STATE_FLYING)) {
	accX -= UKF_ACC_BIAS_X;
	accY -= UKF_ACC_BIAS_Y;
	noise[0] *= 0.001f;
    }

    noise[1] = noise[0];
    noise[2] = noise[0];

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfAccUpdate);
}

void simDoMagUpdate(float magX, float magY, float magZ) {
    float noise[3];        // measurement variance
    float y[3];            // measurement(s)
    float norm;

    noise[0] = p[UKF_MAG_N];
    if (!(supervisorData.state & STATE_FLYING))
	noise[0] *= 0.001f;

    noise[1] = noise[0];
    noise[2] = noise[0];

    // normalize vector
    norm = 1.0f / __sqrtf(magX*magX + magY*magY + magZ*magZ);
    y[0] = magX * norm;
    y[1] = magY * norm;
    y[2] = magZ * norm;

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfMagUpdate);
}

void navUkfGpsPosUpdate(uint32_t gpsMicros, double lat, double lon, float alt, float hAcc, float vAcc) {
    float y[3];
    float noise[3];
    float posDelta[3];
    int histIndex;

    if (hAcc < 4.0f && gpsData.tDOP != 0.0f) {
	if (navUkfData.holdLat == (double)0.0) {
	    navUkfData.holdLat = lat;
	    navUkfData.holdLon = lon;
	    navUkfCalcEarthRadius(lat);
	    navUkfSetGlobalPositionTarget(lat, lon);
	    navUkfResetPosition(-UKF_POSN, -UKF_POSE, alt - UKF_POSD);
	}
	else {
	    navUkfCalcDistance(lat, lon, &y[0], &y[1]);
	    y[2] = alt;

	    // determine how far back this GPS position update came from
	    histIndex = (timerMicros() - (gpsMicros + p[UKF_POS_DELAY])) / (int)(1e6f * AQ_TIMESTEP);
	    histIndex = navUkfData.navHistIndex - histIndex;
	    if (histIndex < 0)
		histIndex += UKF_HIST;
	    if (histIndex < 0 || histIndex >= UKF_HIST)
		histIndex = 0;

	    // calculate delta from current position
	    posDelta[0] = UKF_POSN - navUkfData.posN[histIndex];
	    posDelta[1] = UKF_POSE - navUkfData.posE[histIndex];
	    posDelta[2] = UKF_POSD - navUkfData.posD[histIndex];

	    // set current position state to historic data
	    UKF_POSN = navUkfData.posN[histIndex];
	    UKF_POSE = navUkfData.posE[histIndex];
	    UKF_POSD = navUkfData.posD[histIndex];

	    noise[0] = p[UKF_GPS_POS_N] + hAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.nDOP*gpsData.nDOP) * p[UKF_GPS_POS_M_N];
	    noise[1] = p[UKF_GPS_POS_N] + hAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.eDOP*gpsData.eDOP) * p[UKF_GPS_POS_M_N];
	    noise[2] = p[UKF_GPS_ALT_N] + vAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.vDOP*gpsData.vDOP) * p[UKF_GPS_ALT_M_N];

	    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);

	    // add the historic position delta back to the current state
	    UKF_POSN += posDelta[0];
	    UKF_POSE += posDelta[1];
	    UKF_POSD += posDelta[2];

#ifdef UKF_LOG_BUF
	{
	    float *log = (float *)&ukfLog[navUkfData.logPointer];

	    *(uint32_t *)&log[0] = 0xffffffff;
	    log[1] = y[0];
	    log[2] = y[1];
	    log[3] = y[2];
	    log[4] = noise[0];
	    log[5] = noise[1];
	    log[6] = noise[2];
	    log[7] = posDelta[0];
	    log[8] = posDelta[1];
	    log[9] = posDelta[2];

	    navUkfData.logPointer = (navUkfData.logPointer + 10*sizeof(float)) % UKF_LOG_BUF;
	    filerSetHead(navUkfData.logHandle, navUkfData.logPointer);
	}
#endif
	}
    }
    else {
	y[0] = 0.0f;
	y[1] = 0.0f;
	y[2] = UKF_PRES_ALT;

	if (supervisorData.state & STATE_FLYING) {
	    noise[0] = 1e1f;
	    noise[1] = 1e1f;
	    noise[2] = 1e2f;
	}
	else {
	    noise[0] = 1e-7f;
	    noise[1] = 1e-7f;
	    noise[2] = 1e2f;
	}

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);
    }
}

void navUkfGpsVelUpdate(uint32_t gpsMicros, float velN, float velE, float velD, float sAcc) {
    float y[3];
    float noise[3];
    float velDelta[3];
    int histIndex;

    if (sAcc < 2.0f && gpsData.tDOP != 0.0f) {
	y[0] = velN;
	y[1] = velE;
	y[2] = velD;

	// determine how far back this GPS velocity update came from
	histIndex = (timerMicros() - (gpsMicros + p[UKF_VEL_DELAY])) / (int)(1e6f * AQ_TIMESTEP);
	histIndex = navUkfData.navHistIndex - histIndex;
	if (histIndex < 0)
	    histIndex += UKF_HIST;
	if (histIndex < 0 || histIndex >= UKF_HIST)
	    histIndex = 0;

	// calculate delta from current position
	velDelta[0] = UKF_VELN - navUkfData.velN[histIndex];
	velDelta[1] = UKF_VELE - navUkfData.velE[histIndex];
	velDelta[2] = UKF_VELD - navUkfData.velD[histIndex];

	// set current position state to historic data
	UKF_VELN = navUkfData.velN[histIndex];
	UKF_VELE = navUkfData.velE[histIndex];
	UKF_VELD = navUkfData.velD[histIndex];

	noise[0] = p[UKF_GPS_VEL_N] + sAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.nDOP*gpsData.nDOP) * p[UKF_GPS_VEL_M_N];
	noise[1] = p[UKF_GPS_VEL_N] + sAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.eDOP*gpsData.eDOP) * p[UKF_GPS_VEL_M_N];
	noise[2] = p[UKF_GPS_VD_N]  + sAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.vDOP*gpsData.vDOP) * p[UKF_GPS_VD_M_N];

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfVelUpdate);

	// add the historic position delta back to the current state
	UKF_VELN += velDelta[0];
	UKF_VELE += velDelta[1];
	UKF_VELD += velDelta[2];

#ifdef UKF_LOG_BUF
	{
	    float *log = (float *)&ukfLog[navUkfData.logPointer];

	    *(uint32_t *)&log[0] = 0xfffffffe;
	    log[1] = y[0];
	    log[2] = y[1];
	    log[3] = y[2];
	    log[4] = noise[0];
	    log[5] = noise[1];
	    log[6] = noise[2];
	    log[7] = velDelta[0];
	    log[8] = velDelta[1];
	    log[9] = velDelta[2];

	    navUkfData.logPointer = (navUkfData.logPointer + 10*sizeof(float)) % UKF_LOG_BUF;
	    filerSetHead(navUkfData.logHandle, navUkfData.logPointer);
	}
#endif
    }
    else {
	y[0] = 0.0f;
	y[1] = 0.0f;
	y[2] = 0.0f;

	if (supervisorData.state & STATE_FLYING) {
	    noise[0] = 5.0f;
	    noise[1] = 5.0f;
	    noise[2] = 2.0f;
	}
	else {
	    noise[0] = 1e-7f;
	    noise[1] = 1e-7f;
	    noise[2] = 1e-7f;
	}

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfVelUpdate);
    }
}

void navUkfOpticalFlow(float x, float y, uint8_t quality, float ground) {
}

void navUkfResetBias(void) {
    // acc bias
    UKF_ACC_BIAS_X = 0.0f;
    UKF_ACC_BIAS_Y = 0.0f;
    UKF_ACC_BIAS_Z = 0.0f;

    // gyo bias
    UKF_GYO_BIAS_X = 0.0f;
    UKF_GYO_BIAS_Y = 0.0f;
    UKF_GYO_BIAS_Z = 0.0f;
}

void navUkfInitState(void) {
    uint32_t lastUpdate;
    float acc[3], mag[3];
    float estAcc[3], estMag[3];
//    float vX[UKF_GYO_AVG_NUM];
//    float vY[UKF_GYO_AVG_NUM];
//    float vZ[UKF_GYO_AVG_NUM];
//    float stdX, stdY, stdZ;
    float m[3*3];
    int i;//, j;

    // vel
    UKF_VELN = 0.0f;
    UKF_VELE = 0.0f;
    UKF_VELD = 0.0f;

    // pos
    UKF_POSN = 0.0f;
    UKF_POSE = 0.0f;
    UKF_POSD = navUkfPresToAlt(AQ_PRESSURE);

    // acc bias
    UKF_ACC_BIAS_X = 0.0f;
    UKF_ACC_BIAS_Y = 0.0f;
    UKF_ACC_BIAS_Z = 0.0f;

    // gyo bias
    UKF_GYO_BIAS_X = 0.0f;
    UKF_GYO_BIAS_Y = 0.0f;
    UKF_GYO_BIAS_Z = 0.0f;

    // quat
    UKF_Q1 =  1.0f;
    UKF_Q2 =  0.0f;
    UKF_Q3 =  0.0f;
    UKF_Q4 =  0.0f;

    UKF_PRES_ALT = navUkfPresToAlt(AQ_PRESSURE);

    // wait for lack of movement
    imuQuasiStatic(UKF_GYO_AVG_NUM);

    // estimate initial orientation & gyo bias
    i = 0;
//    j = 0;
    do {
	float rotError[3];

	lastUpdate = IMU_LASTUPD;
	while (lastUpdate == IMU_LASTUPD)
	    ;

//	vX[j] = -IMU_RATEX;
//	vY[j] = -IMU_RATEY;
//	vZ[j] = -IMU_RATEZ;
//	j = (j + 1) % UKF_GYO_AVG_NUM;

	mag[0] = IMU_MAGX;
	mag[1] = IMU_MAGY;
	mag[2] = IMU_MAGZ;

	acc[0] = IMU_ACCX;
	acc[1] = IMU_ACCY;
	acc[2] = IMU_ACCZ;

	navUkfNormalizeVec3(acc, acc);
	navUkfNormalizeVec3(mag, mag);

	navUkfQuatToMatrix(m, &UKF_Q1, 1);

	// rotate gravity to body frame of reference
	navUkfRotateVecByRevMatrix(estAcc, navUkfData.v0a, m);

	// rotate mags to body frame of reference
	navUkfRotateVecByRevMatrix(estMag, navUkfData.v0m, m);

	// measured error, starting with accel vector
	rotError[0] = -(acc[2] * estAcc[1] - estAcc[2] * acc[1]) * 1.0f;
	rotError[1] = -(acc[0] * estAcc[2] - estAcc[0] * acc[2]) * 1.0f;
	rotError[2] = -(acc[1] * estAcc[0] - estAcc[1] * acc[0]) * 1.0f;

	// add in mag vector
	rotError[0] += -(mag[2] * estMag[1] - estMag[2] * mag[1]) * 0.50f;
	rotError[1] += -(mag[0] * estMag[2] - estMag[0] * mag[2]) * 0.50f;
	rotError[2] += -(mag[1] * estMag[0] - estMag[1] * mag[0]) * 0.50f;

        navUkfRotateQuat(&UKF_Q1, &UKF_Q1, rotError, 0.1f);

//	if (i >= UKF_GYO_AVG_NUM) {
//	    arm_std_f32(vX, UKF_GYO_AVG_NUM, &stdX);
//	    arm_std_f32(vY, UKF_GYO_AVG_NUM, &stdY);
//	    arm_std_f32(vZ, UKF_GYO_AVG_NUM, &stdZ);
//	}

	i++;
//    } while (i < (int)(1.0f / AQ_TIMESTEP)*IMU_STATIC_TIMEOUT && (i <= UKF_GYO_AVG_NUM*5 || (stdX + stdY + stdZ) > 0.004f));
    } while (i <= UKF_GYO_AVG_NUM*5);

//    arm_mean_f32(vX, UKF_GYO_AVG_NUM, &UKF_GYO_BIAS_X);
//    arm_mean_f32(vY, UKF_GYO_AVG_NUM, &UKF_GYO_BIAS_Y);
//    arm_mean_f32(vZ, UKF_GYO_AVG_NUM, &UKF_GYO_BIAS_Z);
}

void navUkfInit(void) {
    float Q[SIM_S];		// state variance
    float V[SIM_V];		// process variance
    float mag[3];

    memset((void *)&navUkfData, 0, sizeof(navUkfData));

    navUkfData.v0a[0] = 0.0f;
    navUkfData.v0a[1] = 0.0f;
    navUkfData.v0a[2] = -1.0f;

    // calculate mag vector based on inclination
    mag[0] = cosf(p[IMU_MAG_INCL] * DEG_TO_RAD);
    mag[1] = 0.0f;
    mag[2] = -sinf(p[IMU_MAG_INCL] * DEG_TO_RAD);

    // rotate local mag vector to align with true north
    navUkfData.v0m[0] = mag[0] * cosf(p[IMU_MAG_DECL] * DEG_TO_RAD) - mag[1] * sinf(p[IMU_MAG_DECL]  * DEG_TO_RAD);
    navUkfData.v0m[1] = mag[1] * cosf(p[IMU_MAG_DECL]  * DEG_TO_RAD) + mag[0] * sinf(p[IMU_MAG_DECL]  * DEG_TO_RAD);
    navUkfData.v0m[2] = mag[2];

    navUkfData.kf = srcdkfInit(SIM_S, SIM_M, SIM_V, SIM_N, navUkfTimeUpdate);

    navUkfData.x = srcdkfGetState(navUkfData.kf);

    Q[0] = p[UKF_VEL_Q];
    Q[1] = p[UKF_VEL_Q];
    Q[2] = p[UKF_VEL_ALT_Q];
    Q[3] = p[UKF_POS_Q];
    Q[4] = p[UKF_POS_Q];
    Q[5] = p[UKF_POS_ALT_Q];
    Q[6] = p[UKF_ACC_BIAS_Q];
    Q[7] = p[UKF_ACC_BIAS_Q];
    Q[8] = p[UKF_ACC_BIAS_Q];
    Q[9] = p[UKF_GYO_BIAS_Q];
    Q[10] = p[UKF_GYO_BIAS_Q];
    Q[11] = p[UKF_GYO_BIAS_Q];
    Q[12] = p[UKF_QUAT_Q];
    Q[13] = p[UKF_QUAT_Q];
    Q[14] = p[UKF_QUAT_Q];
    Q[15] = p[UKF_QUAT_Q];
    Q[16] = p[UKF_PRES_ALT_Q];

    V[0] = p[UKF_ACC_BIAS_V];
    V[1] = p[UKF_ACC_BIAS_V];
    V[2] = p[UKF_ACC_BIAS_V];
    V[3] = p[UKF_GYO_BIAS_V];
    V[4] = p[UKF_GYO_BIAS_V];
    V[5] = p[UKF_GYO_BIAS_V];
    V[6] = p[UKF_RATE_V];
    V[7] = p[UKF_RATE_V];
    V[8] = p[UKF_RATE_V];
    V[9] = p[UKF_PRES_ALT_V];
    V[10] = p[UKF_VEL_V];
    V[11] = p[UKF_VEL_V];
    V[12] = p[UKF_ALT_VEL_V];
    V[13] = p[UKF_POS_V];
    V[14] = p[UKF_POS_V];
    V[15] = p[UKF_ALT_POS_V];

    srcdkfSetVariance(navUkfData.kf, Q, V, 0, 0);

    navUkfInitState();

#ifdef UKF_LOG_BUF
    navUkfData.logHandle = filerGetHandle(UKF_FNAME);
    filerStream(navUkfData.logHandle, ukfLog, UKF_LOG_BUF);
#endif
}
