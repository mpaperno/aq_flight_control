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

#ifdef UKF_LOG_FNAME
char ukfLog[UKF_LOG_BUF_SIZE];
#endif

float navUkfPresToAlt(float pressure) {
    return (1.0f -  powf(pressure / UKF_P0, 0.19f)) * (1.0f / 22.558e-6f);
}

static void navUkfCalcEarthRadius(double lat) {
    double sinLat2;

    sinLat2 = sin(lat * (double)DEG_TO_RAD);
    sinLat2 = sinLat2 * sinLat2;

    navUkfData.r1 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD * ((double)1.0 - (double)NAV_E_2) / pow((double)1.0 - ((double)NAV_E_2 * sinLat2), ((double)3.0 / (double)2.0));
    navUkfData.r2 = (double)NAV_EQUATORIAL_RADIUS * (double)DEG_TO_RAD / sqrt((double)1.0 - ((double)NAV_E_2 * sinLat2)) * cos(lat * (double)DEG_TO_RAD);
}

static void navUkfCalcGlobalDistance(double lat, double lon, float *posNorth, float *posEast) {
    *posNorth = (lat - navUkfData.holdLat) * navUkfData.r1;
    *posEast = (lon - navUkfData.holdLon) * navUkfData.r2;
}

static void navUkfResetPosition(float deltaN, float deltaE, float deltaD) {
    int i;

    for (i = 0; i < UKF_HIST; i++) {
	navUkfData.posN[i] += deltaN;
	navUkfData.posE[i] += deltaE;
	navUkfData.posD[i] += deltaD;
    }

    UKF_POSN += deltaN;
    UKF_POSE += deltaE;
    UKF_POSD += deltaD;

#ifndef USE_PRES_ALT
    navResetHoldAlt(deltaD);
#endif
}

void navUkfSetGlobalPositionTarget(double lat, double lon) {
    float oldPosN, oldPosE;
    float newPosN, newPosE;

    navUkfCalcGlobalDistance(lat, lon, &oldPosN, &oldPosE);

    navUkfData.holdLat = lat;
    navUkfData.holdLon = lon;

    navUkfCalcGlobalDistance(lat, lon, &newPosN, &newPosE);

    navUkfResetPosition(newPosN - oldPosN, newPosE - oldPosE, 0.0f);
}

static void navUkfCalcLocalDistance(float localPosN, float localPosE, float *posN, float *posE) {
    *posN = localPosN - (float)navUkfData.holdLat;
    *posE = localPosE - (float)navUkfData.holdLon;
}

static void navUkfSetLocalPositionTarget(float posN, float posE) {
    float oldPosN, oldPosE;
    float newPosN, newPosE;

    navUkfCalcLocalDistance(posN, posE, &oldPosN, &oldPosE);

    navUkfData.holdLat = posN;
    navUkfData.holdLon = posE;

    navUkfCalcLocalDistance(posN, posE, &newPosN, &newPosE);

    navUkfResetPosition(newPosN - oldPosN, newPosE - oldPosE, 0.0f);
}

void navUkfSetHereAsPositionTarget(void) {
    if (navUkfData.flowPosN != 0.0f && navUkfData.flowPosE != 0.0f)
	navUkfSetLocalPositionTarget(navUkfData.flowPosN, navUkfData.flowPosE);
    else
	navUkfSetGlobalPositionTarget(gpsData.lat, gpsData.lon);
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

    norm = 1.0f / __sqrtf(q[0]*q[0] + q[1]*q[1] + q[2]*q[2] + q[3]*q[3]);

    qr[0] *= norm;
    qr[1] *= norm;
    qr[2] *= norm;
    qr[3] *= norm;
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

static void navUkfRotateVecByRevMatrix(float *vr, float *v, float *m) {
    vr[0] = m[0*3 + 0]*v[0] + m[1*3 + 0]*v[1] + m[2*3 + 0]*v[2];
    vr[1] = m[0*3 + 1]*v[0] + m[1*3 + 1]*v[1] + m[2*3 + 1]*v[2];
    vr[2] = m[0*3 + 2]*v[0] + m[1*3 + 2]*v[1] + m[2*3 + 2]*v[2];
}

static void navUkfQuatToMatrix(float *m, float *q, int normalize) {
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
static void navUkfRotateQuat(float *qOut, float *qIn, float *rate) {
    float q[4];
    float r[3];

    r[0] = rate[0] * -0.5f;
    r[1] = rate[1] * -0.5f;
    r[2] = rate[2] * -0.5f;

    q[0] = qIn[0];
    q[1] = qIn[1];
    q[2] = qIn[2];
    q[3] = qIn[3];

    // rotate
    qOut[0] =       q[0] + r[0]*q[1] + r[1]*q[2] + r[2]*q[3];
    qOut[1] = -r[0]*q[0] +      q[1] - r[2]*q[2] + r[1]*q[3];
    qOut[2] = -r[1]*q[0] + r[2]*q[1] +      q[2] - r[0]*q[3];
    qOut[3] = -r[2]*q[0] - r[1]*q[1] + r[0]*q[2] +      q[3];
}

void navUkfTimeUpdate(float *in, float *noise, float *out, float *u, float dt, int n) {
    float tmp[3], acc[3];
    float rate[3];
    float mat3x3[3*3];
    float q[4];
    int i;

    // assume out == in
    out = in;

    for (i = 0; i < n; i++) {
	// pos
	out[UKF_STATE_POSN*n + i] = in[UKF_STATE_POSN*n + i] + in[UKF_STATE_VELN*n + i] * dt;
	out[UKF_STATE_POSE*n + i] = in[UKF_STATE_POSE*n + i] + in[UKF_STATE_VELE*n + i] * dt;
	out[UKF_STATE_POSD*n + i] = in[UKF_STATE_POSD*n + i] - in[UKF_STATE_VELD*n + i] * dt;

	// pres alt
	out[UKF_STATE_PRES_ALT*n + i] = in[UKF_STATE_PRES_ALT*n + i] - in[UKF_STATE_VELD*n + i] * dt;

	// create rot matrix from current quat
	q[0] = in[UKF_STATE_Q1*n + i];
	q[1] = in[UKF_STATE_Q2*n + i];
	q[2] = in[UKF_STATE_Q3*n + i];
	q[3] = in[UKF_STATE_Q4*n + i];
	navUkfQuatToMatrix(mat3x3, q, 1);

	// acc
	tmp[0] = u[0] + in[UKF_STATE_ACC_BIAS_X*n + i];
	tmp[1] = u[1] + in[UKF_STATE_ACC_BIAS_Y*n + i];
	tmp[2] = u[2] + in[UKF_STATE_ACC_BIAS_Z*n + i];

	// rotate acc to world frame
	navUkfRotateVecByMatrix(acc, tmp, mat3x3);
	acc[2] += GRAVITY;

	// vel
	out[UKF_STATE_VELN*n + i] = in[UKF_STATE_VELN*n + i] + acc[0] * dt + noise[UKF_V_NOISE_VELN*n + i];
	out[UKF_STATE_VELE*n + i] = in[UKF_STATE_VELE*n + i] + acc[1] * dt + noise[UKF_V_NOISE_VELE*n + i];
	out[UKF_STATE_VELD*n + i] = in[UKF_STATE_VELD*n + i] + acc[2] * dt + noise[UKF_V_NOISE_VELD*n + i];

	// acc bias
	out[UKF_STATE_ACC_BIAS_X*n + i] = in[UKF_STATE_ACC_BIAS_X*n + i] + noise[UKF_V_NOISE_ACC_BIAS_X*n + i] * dt;
	out[UKF_STATE_ACC_BIAS_Y*n + i] = in[UKF_STATE_ACC_BIAS_Y*n + i] + noise[UKF_V_NOISE_ACC_BIAS_Y*n + i] * dt;
	out[UKF_STATE_ACC_BIAS_Z*n + i] = in[UKF_STATE_ACC_BIAS_Z*n + i] + noise[UKF_V_NOISE_ACC_BIAS_Z*n + i] * dt;

	// rate = rate + bias + noise
	rate[0] = (u[3] + in[UKF_STATE_GYO_BIAS_X*n + i] + noise[UKF_V_NOISE_RATE_X*n + i]) * dt;
	rate[1] = (u[4] + in[UKF_STATE_GYO_BIAS_Y*n + i] + noise[UKF_V_NOISE_RATE_Y*n + i]) * dt;
	rate[2] = (u[5] + in[UKF_STATE_GYO_BIAS_Z*n + i] + noise[UKF_V_NOISE_RATE_Z*n + i]) * dt;

	// rotate quat
	navUkfRotateQuat(q, q, rate);
	out[UKF_STATE_Q1*n + i] = q[0];
	out[UKF_STATE_Q2*n + i] = q[1];
	out[UKF_STATE_Q3*n + i] = q[2];
	out[UKF_STATE_Q4*n + i] = q[3];

	// gbias
	out[UKF_STATE_GYO_BIAS_X*n + i] = in[UKF_STATE_GYO_BIAS_X*n + i] + noise[UKF_V_NOISE_GYO_BIAS_X*n + i] * dt;
	out[UKF_STATE_GYO_BIAS_Y*n + i] = in[UKF_STATE_GYO_BIAS_Y*n + i] + noise[UKF_V_NOISE_GYO_BIAS_Y*n + i] * dt;
	out[UKF_STATE_GYO_BIAS_Z*n + i] = in[UKF_STATE_GYO_BIAS_Z*n + i] + noise[UKF_V_NOISE_GYO_BIAS_Z*n + i] * dt;
    }
}

void navUkfRateUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = -x[UKF_STATE_GYO_BIAS_X+(int)u[0]] + noise[0];
}

void navUkfAccUpdate(float *u, float *x, float *noise, float *y) {
    navUkfRotateVectorByRevQuat(y, navUkfData.v0a, &x[UKF_STATE_Q1]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}

void navUkfMagUpdate(float *u, float *x, float *noise, float *y) {
    navUkfRotateVectorByRevQuat(y, navUkfData.v0m, &x[UKF_STATE_Q1]);
    y[0] += noise[0];
    y[1] += noise[1];
    y[2] += noise[2];
}

void navUkfPresUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_PRES_ALT] + noise[0]; // return altitude
}

void navUkfPresGPSAltUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_PRES_ALT] + noise[0];// return pres altitude
    y[1] = x[UKF_STATE_POSD] + noise[1]; // return GPS altitude
}

void navUkfPosUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_POSN] + noise[0]; // return position
    y[1] = x[UKF_STATE_POSE] + noise[1];
    y[2] = x[UKF_STATE_POSD] + noise[2];
}

void navUkfVelUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_VELN] + noise[0]; // return velocity
    y[1] = x[UKF_STATE_VELE] + noise[1];
    y[2] = x[UKF_STATE_VELD] + noise[2];
}

void navUkfOfVelUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_VELN] + noise[0]; // velN
    y[1] = x[UKF_STATE_VELE] + noise[1]; // velE
}

void navUkfOfPosUpdate(float *u, float *x, float *noise, float *y) {
    y[0] = x[UKF_STATE_POSN] + noise[0]; // posN
    y[1] = x[UKF_STATE_POSE] + noise[1]; // posE
    y[2] = x[UKF_STATE_POSD] + noise[2]; // alt
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

    srcdkfTimeUpdate(navUkfData.kf, u, AQ_OUTER_TIMESTEP);

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

    noise[0] = UKF_ALT_N;

    noise[1] = noise[0];

    y[0] = navUkfPresToAlt(pres);
    y[1] = y[0];

    // if GPS altitude data has been available, only update pressure altitude
    if (navData.presAltOffset != 0.0f)
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

    noise[0] = UKF_ACC_N + fabsf(GRAVITY - norm) * UKF_DIST_N;
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

    noise[0] = UKF_MAG_N;
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

void navUkfZeroPos(void) {
    float y[3];
    float noise[3];

    y[0] = 0.0f;
    y[1] = 0.0f;
    y[2] = navUkfPresToAlt(AQ_PRESSURE);

    if (supervisorData.state & STATE_FLYING) {
	noise[0] = 1e1f;
	noise[1] = 1e1f;
	noise[2] = 1e1f;
    }
    else {
	noise[0] = 1e-7f;
	noise[1] = 1e-7f;
	noise[2] = 1.0f;
    }

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);
}

void navUkfGpsPosUpdate(uint32_t gpsMicros, double lat, double lon, float alt, float hAcc, float vAcc) {
    float y[3];
    float noise[3];
    float posDelta[3];
    int histIndex;

    if (navUkfData.holdLat == (double)0.0) {
	navUkfData.holdLat = lat;
	navUkfData.holdLon = lon;
	navUkfCalcEarthRadius(lat);
	navUkfSetGlobalPositionTarget(lat, lon);
	navUkfResetPosition(-UKF_POSN, -UKF_POSE, alt - UKF_POSD);
    }
    else {
	navUkfCalcGlobalDistance(lat, lon, &y[0], &y[1]);
	y[2] = alt;

	// determine how far back this GPS position update came from
	histIndex = (timerMicros() - (gpsMicros + UKF_POS_DELAY)) / (int)(1e6f * AQ_OUTER_TIMESTEP);
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

	noise[0] = UKF_GPS_POS_N + hAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.nDOP*gpsData.nDOP) * UKF_GPS_POS_M_N;
	noise[1] = UKF_GPS_POS_N + hAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.eDOP*gpsData.eDOP) * UKF_GPS_POS_M_N;
	noise[2] = UKF_GPS_ALT_N + vAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.vDOP*gpsData.vDOP) * UKF_GPS_ALT_M_N;

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfPosUpdate);

	// add the historic position delta back to the current state
	UKF_POSN += posDelta[0];
	UKF_POSE += posDelta[1];
	UKF_POSD += posDelta[2];

#ifdef UKF_LOG_FNAME
    {
	float *log = (float *)&ukfLog[navUkfData.logPointer];
	int i = 0;

	*(uint32_t *)&log[i++] = 0xffffffff;
	log[i++] = y[0];
	log[i++] = y[1];
	log[i++] = y[2];
	log[i++] = noise[0];
	log[i++] = noise[1];
	log[i++] = noise[2];
	log[i++] = posDelta[0];
	log[i++] = posDelta[1];
	log[i++] = posDelta[2];

	navUkfData.logPointer = (navUkfData.logPointer + UKF_LOG_SIZE) % UKF_LOG_BUF_SIZE;
	filerSetHead(navUkfData.logHandle, navUkfData.logPointer);
    }
#endif
    }
}

void navUkfZeroVel(void) {
    float y[3];
    float noise[3];

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

void navUkfGpsVelUpdate(uint32_t gpsMicros, float velN, float velE, float velD, float sAcc) {
    float y[3];
    float noise[3];
    float velDelta[3];
    int histIndex;

    y[0] = velN;
    y[1] = velE;
    y[2] = velD;

    // determine how far back this GPS velocity update came from
    histIndex = (timerMicros() - (gpsMicros + UKF_VEL_DELAY)) / (int)(1e6f * AQ_OUTER_TIMESTEP);
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

    noise[0] = UKF_GPS_VEL_N + sAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.nDOP*gpsData.nDOP) * UKF_GPS_VEL_M_N;
    noise[1] = UKF_GPS_VEL_N + sAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.eDOP*gpsData.eDOP) * UKF_GPS_VEL_M_N;
    noise[2] = UKF_GPS_VD_N  + sAcc * __sqrtf(gpsData.tDOP*gpsData.tDOP + gpsData.vDOP*gpsData.vDOP) * UKF_GPS_VD_M_N;

    srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfVelUpdate);

    // add the historic position delta back to the current state
    UKF_VELN += velDelta[0];
    UKF_VELE += velDelta[1];
    UKF_VELD += velDelta[2];

#ifdef UKF_LOG_FNAME
    {
	float *log = (float *)&ukfLog[navUkfData.logPointer];
	int i = 0;

	*(uint32_t *)&log[i++] = 0xffffffff;
	log[i++] = y[0];
	log[i++] = y[1];
	log[i++] = y[2];
	log[i++] = noise[0];
	log[i++] = noise[1];
	log[i++] = noise[2];
	log[i++] = velDelta[0];
	log[i++] = velDelta[1];
	log[i++] = velDelta[2];

	navUkfData.logPointer = (navUkfData.logPointer + UKF_LOG_SIZE) % UKF_LOG_BUF_SIZE;
	filerSetHead(navUkfData.logHandle, navUkfData.logPointer);
    }
#endif
}

/*
    We take raw pixel movement data reported by the sensor and combine
    it with our own estimates of ground height and rotational rates
    to calculate the x/y velocity estimates.  Along with reported sonar
    altitudes, they are fed to the UKF as observations.
*/
void navUkfFlowUpdate(void) {
    static float oldPitch, oldRoll;
    float flowX, flowY;
    float xT, yT;
    float dt;
    float flowAlt = 0.0f;
    float y[3];
    float noise[3];

    // set default noise levels
    noise[0] = 0.001f;
    noise[1] = 0.001f;
    noise[2] = 100.0f;

    // valid altitudes?
    if (navUkfData.flowAltCount > 0) {
	flowAlt = navUkfData.flowSumAlt / navUkfData.flowAltCount;
	if (fabsf(navUkfData.flowAlt - flowAlt) < 1.0f)
	    noise[2] = 0.0025f;

	navUkfData.flowAlt = flowAlt;
    }

    // first valid flow update ?
    if (navUkfData.flowInit == 0) {
	// only allow init if we have a valid altitude
	if (navUkfData.flowAltCount > 0) {
	    navPressureAdjust(navUkfData.flowAlt);
	    UKF_POSD = navUkfData.flowAlt;
	    navUkfData.flowInit = 1;
	}
    }
    else {
	// scaled, average quality
	navUkfData.flowQuality = (float)navUkfData.flowSumQuality / navUkfData.flowCount * (1.0f / 255.0f);

	// first rotate sensor data to craft frame around Z axis
	flowX = navUkfData.flowSumX * navUkfData.flowRotCos + navUkfData.flowSumY * navUkfData.flowRotSin;
	flowY = navUkfData.flowSumY * navUkfData.flowRotCos - navUkfData.flowSumX * navUkfData.flowRotSin;

	// adjust for pitch/roll rotations during measurement
	flowX -= (AQ_PITCH - oldPitch) * DEG_TO_RAD * UKF_FOCAL_PX;
	flowY += (AQ_ROLL  - oldRoll)  * DEG_TO_RAD * UKF_FOCAL_PX;

	// next, rotate flow to world frame
	xT = flowX * navUkfData.yawCos - flowY * navUkfData.yawSin;
	yT = flowY * navUkfData.yawCos + flowX * navUkfData.yawSin;

	// convert to distance covered based on focal length and height above ground
	flowX = xT * (1.0f / UKF_FOCAL_PX) * UKF_POSD;
	flowY = yT * (1.0f / UKF_FOCAL_PX) * UKF_POSD;

	// integrate for absolute position
	navUkfData.flowPosN += flowX;
	navUkfData.flowPosE += flowY;

	// time delta - assume each count is from two readings @ 5ms each
	dt = (float)navUkfData.flowCount * (5.0f * 2.0f) / 1000.0f;

	// differentiate for velocity
	navUkfData.flowVelX = flowX / dt;
	navUkfData.flowVelY = flowY / dt;

	// TODO: properly estimate noise
	noise[0] = (UKF_GPS_POS_N + UKF_GPS_POS_M_N) * 0.5f;
	noise[1] = noise[0];

	navUkfCalcLocalDistance(navUkfData.flowPosN, navUkfData.flowPosE, &y[0], &y[1]);
	y[2] = navUkfData.flowAlt;

	srcdkfMeasurementUpdate(navUkfData.kf, 0, y, 3, 3, noise, navUkfOfPosUpdate);
#ifdef UKF_LOG_FNAME
	{
	    float *log = (float *)&ukfLog[navUkfData.logPointer];
	    int i = 0;

	    *(uint32_t *)&log[i++] = 0xffffffff;
	    log[i++] = flowX;
	    log[i++] = flowY;
	    log[i++] = navUkfData.flowVelX;
	    log[i++] = navUkfData.flowVelY;
	    log[i++] = navUkfData.flowPosN;
	    log[i++] = navUkfData.flowPosE;
	    log[i++] = flowAlt;
	    log[i++] = UKF_POSN;
	    log[i++] = UKF_POSE;
	    log[i++] = UKF_POSD;
	    log[i++] = UKF_PRES_ALT;
	    log[i++] = UKF_VELD;
	    log[i++] = navUkfData.flowQuality;
	    log[i++] = (AQ_PITCH - oldPitch);
	    log[i++] = (AQ_ROLL  - oldRoll);
	    log[i++] = dt;

	    navUkfData.logPointer = (navUkfData.logPointer + UKF_LOG_SIZE) % UKF_LOG_BUF_SIZE;
	    filerSetHead(navUkfData.logHandle, navUkfData.logPointer);
	}
#endif
    }

    navUkfData.flowSumX = 0.0f;
    navUkfData.flowSumY = 0.0f;
    navUkfData.flowSumQuality = 0;
    navUkfData.flowSumAlt = 0.0f;
    navUkfData.flowCount = 0;
    navUkfData.flowAltCount = 0;

    oldRoll = AQ_ROLL;
    oldPitch = AQ_PITCH;
}

void navUkfOpticalFlow(int16_t x, int16_t y, uint8_t quality, float ground) {
    // since this is called from a low priority thread,
    // this flag just lets high priority threads know we
    // are in the middle of things here.
    navUkfData.flowLock = 1;

    // valid sonar reading?
    if (ground > 0.4f && ground < 4.50f) {
	navUkfData.flowSumAlt += ground;
	navUkfData.flowAltCount++;
    }
    // valid flow?
    if (quality > 0) {
	navUkfData.flowSumX += (x * -0.1f);
	navUkfData.flowSumY += (y * -0.1f);
	navUkfData.flowSumQuality += quality;
	navUkfData.flowCount++;
    }

    navUkfData.flowLock = 0;
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

void navUkfResetVels(void) {
    UKF_VELN = 0.0f;
    UKF_VELE = 0.0f;
    UKF_VELD = 0.0f;
}

void navUkfInitState(void) {
    uint32_t lastUpdate;
    float acc[3], mag[3];
    float estAcc[3], estMag[3];
    float m[3*3];
    int i;

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

    // estimate initial orientation
    i = 0;
    do {
	float rotError[3];

	lastUpdate = IMU_LASTUPD;
	while (lastUpdate == IMU_LASTUPD)
	    yield(1);

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

	// measured error, starting with ACC vector
	rotError[0] = -(acc[2] * estAcc[1] - estAcc[2] * acc[1]) * 1.0f;
	rotError[1] = -(acc[0] * estAcc[2] - estAcc[0] * acc[2]) * 1.0f;
	rotError[2] = -(acc[1] * estAcc[0] - estAcc[1] * acc[0]) * 1.0f;

	// add in MAG vector
	if (AQ_MAG_ENABLED) {
	    rotError[0] += -(mag[2] * estMag[1] - estMag[2] * mag[1]) * 1.0f;
	    rotError[1] += -(mag[0] * estMag[2] - estMag[0] * mag[2]) * 1.0f;
	    rotError[2] += -(mag[1] * estMag[0] - estMag[1] * mag[0]) * 1.0f;
	}

        navUkfRotateQuat(&UKF_Q1, &UKF_Q1, rotError);
        navUkfNormalizeQuat(&UKF_Q1, &UKF_Q1);

	i++;
    } while (i <= UKF_GYO_AVG_NUM*5);
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
    navUkfData.v0m[1] = mag[1] * cosf(p[IMU_MAG_DECL] * DEG_TO_RAD) + mag[0] * sinf(p[IMU_MAG_DECL]  * DEG_TO_RAD);
    navUkfData.v0m[2] = mag[2];

    navUkfData.kf = srcdkfInit(SIM_S, SIM_M, SIM_V, SIM_N, navUkfTimeUpdate);

    navUkfData.x = srcdkfGetState(navUkfData.kf);

    Q[0] = UKF_VEL_Q;
    Q[1] = UKF_VEL_Q;
    Q[2] = UKF_VEL_ALT_Q;
    Q[3] = UKF_POS_Q;
    Q[4] = UKF_POS_Q;
    Q[5] = UKF_POS_ALT_Q;
    Q[6] = UKF_ACC_BIAS_Q;
    Q[7] = UKF_ACC_BIAS_Q;
    Q[8] = UKF_ACC_BIAS_Q;
    Q[9] = UKF_GYO_BIAS_Q;
    Q[10] = UKF_GYO_BIAS_Q;
    Q[11] = UKF_GYO_BIAS_Q;
    Q[12] = UKF_QUAT_Q;
    Q[13] = UKF_QUAT_Q;
    Q[14] = UKF_QUAT_Q;
    Q[15] = UKF_QUAT_Q;
    Q[16] = UKF_PRES_ALT_Q;

    V[UKF_V_NOISE_ACC_BIAS_X] = UKF_ACC_BIAS_V;
    V[UKF_V_NOISE_ACC_BIAS_Y] = UKF_ACC_BIAS_V;
    V[UKF_V_NOISE_ACC_BIAS_Z] = UKF_ACC_BIAS_V;
    V[UKF_V_NOISE_GYO_BIAS_X] = UKF_GYO_BIAS_V;
    V[UKF_V_NOISE_GYO_BIAS_Y] = UKF_GYO_BIAS_V;
    V[UKF_V_NOISE_GYO_BIAS_Z] = UKF_GYO_BIAS_V;
    V[UKF_V_NOISE_RATE_X] = UKF_RATE_V;
    V[UKF_V_NOISE_RATE_Y] = UKF_RATE_V;
    V[UKF_V_NOISE_RATE_Z] = UKF_RATE_V;
    V[UKF_V_NOISE_VELN] = UKF_VEL_V;
    V[UKF_V_NOISE_VELE] = UKF_VEL_V;
    V[UKF_V_NOISE_VELD] = UKF_ALT_VEL_V;

    srcdkfSetVariance(navUkfData.kf, Q, V, 0, 0);

    navUkfInitState();

    navUkfData.flowRotCos = cosf(UKF_FLOW_ROT * DEG_TO_RAD);
    navUkfData.flowRotSin = sinf(UKF_FLOW_ROT * DEG_TO_RAD);

#ifdef UKF_LOG_FNAME
    navUkfData.logHandle = filerGetHandle(UKF_LOG_FNAME);
    filerStream(navUkfData.logHandle, ukfLog, UKF_LOG_BUF_SIZE);
#endif
}
