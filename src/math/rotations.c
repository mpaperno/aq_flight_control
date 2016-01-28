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

#include "aq.h"
#include "aq_math.h"

void quatMultiply(float32_t *qr, float32_t *q1, float32_t *q2) {
    qr[0] = q2[0]*q1[0] - q2[1]*q1[1] - q2[2]*q1[2] - q2[3]*q1[3];
    qr[1] = q2[0]*q1[1] + q2[1]*q1[0] - q2[2]*q1[3] + q2[3]*q1[2];
    qr[2] = q2[0]*q1[2] + q2[1]*q1[3] + q2[2]*q1[0] - q2[3]*q1[1];
    qr[3] = q2[0]*q1[3] - q2[1]*q1[2] + q2[2]*q1[1] + q2[3]*q1[0];
}

void eulerToQuatYPR(float32_t *q, float32_t yaw, float32_t pitch, float32_t roll) {
    float32_t cy, cp, cr;
    float32_t sy, sp, sr;

    yaw *= DEG_TO_RAD * 0.5f;
    pitch *= DEG_TO_RAD * 0.5f;
    roll *= DEG_TO_RAD * 0.5f;

    cy = cosf(yaw);
    cp = cosf(pitch);
    cr = cosf(roll);

    sy = sinf(yaw);
    sp = sinf(pitch);
    sr = sinf(roll);

    q[0] = cy*cp*cr + sy*sp*sr;
    q[1] = cy*cp*sr - sy*sp*cr;
    q[2] = cy*sp*cr + sy*cp*sr;
    q[3] = sy*cp*cr - cy*sp*sr;
}

void eulerToQuatRPY(float32_t *q, float32_t roll, float32_t pitch, float32_t yaw) {
    float32_t cy, cp, cr;
    float32_t sy, sp, sr;

    yaw *= DEG_TO_RAD * 0.5f;
    pitch *= DEG_TO_RAD * 0.5f;
    roll *= DEG_TO_RAD * 0.5f;

    cy = cosf(yaw);
    cp = cosf(pitch);
    cr = cosf(roll);

    sy = sinf(yaw);
    sp = sinf(pitch);
    sr = sinf(roll);

    q[0] = cr*cp*cy - sr*sp*sy;
    q[1] = cr*sp*sy + sr*cp*cy;
    q[2] = cr*sp*cy - sr*cp*sy;
    q[3] = cr*cp*sy + sr*sp*cy;
}

void nlerp(float32_t *r, float32_t *a, float32_t *b, float32_t t) {
    float32_t dp;
    float32_t f1, f2;

    f1 = 1.0f - t;
    f2 = t;

    dp = (a[0]*b[0] + a[1]*b[1] + a[2]*b[2] + a[3]*b[3]);

    // rotate the shorter distance
    if (dp >= 0.0f) {
	r[0] = a[0]*f1 + b[0]*f2;
	r[1] = a[1]*f1 + b[1]*f2;
	r[2] = a[2]*f1 + b[2]*f2;
	r[3] = a[3]*f1 + b[3]*f2;
    }
    else {
	r[0] = a[0]*f1 - b[0]*f2;
	r[1] = a[1]*f1 - b[1]*f2;
	r[2] = a[2]*f1 - b[2]*f2;
	r[3] = a[3]*f1 - b[3]*f2;
    }

    vectorNormalize(r, 4);
}

void quatFilterReset(quatFilter_t *f, float32_t *q) {
    f->qz1[0] = q[0];
    f->qz1[1] = q[1];
    f->qz1[2] = q[2];
    f->qz1[3] = q[3];
}

void quatFilterReset3(quatFilter_t *f, float32_t *q) {
    quatFilterReset(&f[0], q);
    quatFilterReset(&f[1], q);
    quatFilterReset(&f[2], q);
}

void quatFilterInit(quatFilter_t *f, float32_t dt, float32_t tau, float32_t *q) {
    f->tc = dt / tau;
    quatFilterReset(f, q);
}

void quatFilterInit3(quatFilter_t *f, float32_t dt, float32_t tau, float32_t *q) {
    quatFilterInit(&f[0], dt, tau, q);
    quatFilterInit(&f[1], dt, tau, q);
    quatFilterInit(&f[2], dt, tau, q);
}

float32_t *quatFilter(quatFilter_t *f, float32_t *b) {
    nlerp(f->qz1, f->qz1, b, f->tc);

    return f->qz1;
}

float32_t *quatFilter3(quatFilter_t *f, float32_t *b) {
    return quatFilter(&f[2], quatFilter(&f[1], quatFilter(&f[0], b)));
}
