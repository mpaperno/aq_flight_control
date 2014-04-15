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

#ifndef _aq_math_h
#define _aq_math_h

#include "arm_math.h"

// first order quat filter
typedef struct {
    float tc;
    float qz1[4];
} quatFilter_t;

extern void matrixInit(arm_matrix_instance_f32 *m, int rows, int cols);
extern void matrixFree(arm_matrix_instance_f32 *m);
extern void matrixDump(char *name, arm_matrix_instance_f32 *m);
extern int qrDecompositionT_f32(arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R);
extern void matrixDiv_f32(arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R, arm_matrix_instance_f32 *AQ);
extern void quatMultiply(float32_t *qr, float32_t *q1, float32_t *q2);
extern void eulerToQuatYPR(float32_t *q, float32_t yaw, float32_t pitch, float32_t roll);
extern void eulerToQuatRPY(float32_t *q, float32_t roll, float32_t pitch, float32_t yaw);
extern void vectorNormalize(float32_t *v, int n);
extern void nlerp(float32_t *r, float32_t *a, float32_t *b, float32_t t);
extern void quatFilterReset(quatFilter_t *f, float32_t *q);
extern void quatFilterReset3(quatFilter_t *f, float32_t *q);
extern void quatFilterInit(quatFilter_t *f, float32_t dt, float32_t tau, float32_t *q);
extern void quatFilterInit3(quatFilter_t *f, float32_t dt, float32_t tau, float32_t *q);
extern float32_t *quatFilter(quatFilter_t *f, float32_t *b);
extern float32_t *quatFilter3(quatFilter_t *f, float32_t *b);
extern int cholF(float32_t *U);
extern void svd(float32_t *A, float32_t *S2, int n);

#endif
