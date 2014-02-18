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

#include "srcdkf.h"
#include "aq_math.h"
#include "util.h"
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

float *srcdkfGetState(srcdkf_t *f) {
    return f->x.pData;
}

void srcdkfSetVariance(srcdkf_t *f, float32_t *q, float32_t *v, float32_t *n, int nn) {
	float32_t *Sx = f->Sx.pData;
	float32_t *Sv = f->Sv.pData;
	float32_t *Sn = f->Sn.pData;
	int i;

	// state variance
	if (q)
		for (i = 0; i < f->S; i++)
			Sx[i*f->S + i] = __sqrtf(fabsf(q[i]));

	// process noise
	if (v)
		for (i = 0; i < f->V; i++)
			Sv[i*f->V + i] = __sqrtf(fabsf(v[i]));

	// observation noise
	if (n && nn) {
		// resize Sn
		f->Sn.numRows = nn;
		f->Sn.numCols = nn;

		for (i = 0; i < nn; i++)
			Sn[i*nn + i] = __sqrtf(fabsf(n[i]));
	}
}

void srcdkfGetVariance(srcdkf_t *f, float32_t *q) {
	float32_t *Sx = f->Sx.pData;
	int i;

	// state variance
	if (q)
		for (i = 0; i < f->S; i++) {
			q[i] = Sx[i*f->S + i];
			q[i] = q[i]*q[i];
		}
}

// states, max observations, process noise, max observation noise
srcdkf_t *srcdkfInit(int s, int m, int v, int n, SRCDKFTimeUpdate_t *timeUpdate) {
	srcdkf_t *f;
	int maxN = MAX(v, n);

	f = (srcdkf_t *)aqDataCalloc(1, sizeof(srcdkf_t));

	f->S = s;
	f->V = v;

	matrixInit(&f->Sx, s, s);
	matrixInit(&f->SxT, s, s);
	matrixInit(&f->Sv, v, v);
	matrixInit(&f->Sn, n, n);
	matrixInit(&f->x, s, 1);
	matrixInit(&f->Xa, s+maxN, 1+(s+maxN)*2);

	matrixInit(&f->qrTempS, s, (s+v)*2);
	matrixInit(&f->y, m, 1);
	matrixInit(&f->Y, m, 1+(s+n)*2);
	matrixInit(&f->qrTempM, m, (s+n)*2);
	matrixInit(&f->Sy, m, m);
	matrixInit(&f->SyT, m, m);
	matrixInit(&f->SyC, m, m);
	matrixInit(&f->Pxy, s, m);
	matrixInit(&f->C1, m, s);
	matrixInit(&f->C1T, s, m);
	matrixInit(&f->C2, m, n);
	matrixInit(&f->D, m, s+n);
	matrixInit(&f->K, s, m);
	matrixInit(&f->inov, m, 1);
	matrixInit(&f->xUpdate, s, 1);
	matrixInit(&f->qrFinal, s, 2*s + 2*n);
	matrixInit(&f->Q, s, s+n);	// scratch
	matrixInit(&f->R, n, n);	// scratch
	matrixInit(&f->AQ, s, n);	// scratch

	f->xOut = (float32_t *)aqDataCalloc(s, sizeof(float32_t));
	f->xNoise = (float32_t *)aqDataCalloc(maxN, sizeof(float32_t));
	f->xIn = (float32_t *)aqDataCalloc(s, sizeof(float32_t));

	f->h = SRCDKF_H;
	f->hh = f->h*f->h;
//	f->w0m = (f->hh - (float32_t)s) / f->hh;	// calculated in process
	f->wim = 1.0f / (2.0f * f->hh);
	f->wic1 = __sqrtf(1.0f / (4.0f * f->hh));
	f->wic2 = __sqrtf((f->hh - 1.0f) / (4.0f * f->hh*f->hh));

        f->timeUpdate = timeUpdate;

	return f;
}

// given noise matrix
static void srcdkfCalcSigmaPoints(srcdkf_t *f, arm_matrix_instance_f32 *Sn) {
	int S = f->S;			// number of states
	int N = Sn->numRows;		// number of noise variables
	int A = S+N;			// number of agumented states
	int L = 1+A*2;			// number of sigma points
	float32_t *x = f->x.pData;	// state
	float32_t *Sx = f->Sx.pData;	// state covariance
	float32_t *Xa = f->Xa.pData;	// augmented sigma points
	int i, j;

	// set the number of sigma points
	f->L = L;

	// resize output matrix
	f->Xa.numRows = A;
	f->Xa.numCols = L;

	//	-	   -
	// Sa =	| Sx	0  |
	//	| 0	Sn |
	//	-	   -
	// xa = [ x 	0  ]
	// Xa = [ xa  (xa + h*Sa)  (xa - h*Sa) ]
	//
	for (i = 0; i < A; i++) {
		int rOffset = i*L;
		float32_t base = (i < S) ? x[i] : 0.0f;

		Xa[rOffset + 0] = base;

		for (j = 1; j <= A; j++) {
			float32_t t = 0.0f;

			if (i < S && j < S+1)
				t = Sx[i*S + (j-1)]*f->h;

			if (i >= S && j >= S+1)
				t = Sn->pData[(i-S)*N + (j-S-1)]*f->h;

			Xa[rOffset + j]     = base + t;
			Xa[rOffset + j + A] = base - t;
		}
	}
}

void srcdkfTimeUpdate(srcdkf_t *f, float32_t *u, float32_t dt) {
	int S = f->S;			// number of states
	int V = f->V;			// number of noise variables
	int L;				// number of sigma points
	float32_t *x = f->x.pData;	// state estimate
	float32_t *Xa = f->Xa.pData;	// augmented sigma points
//	float32_t *xIn = f->xIn;	// callback buffer
//	float32_t *xOut = f->xOut;	// callback buffer
//	float32_t *xNoise = f->xNoise;	// callback buffer
	float32_t *qrTempS = f->qrTempS.pData;
	int i, j;

	srcdkfCalcSigmaPoints(f, &f->Sv);
	L = f->L;

	// Xa = f(Xx, Xv, u, dt)
//	for (i = 0; i < L; i++) {
//		for (j = 0; j < S; j++)
//			xIn[j] = Xa[j*L + i];
//
//		for (j = 0; j < V; j++)
//			xNoise[j] = Xa[(S+j)*L + i];
//
//		f->timeUpdate(xIn, xNoise, xOut, u, dt);
//
//		for (j = 0; j < S; j++)
//			Xa[j*L + i] = xOut[j];
//	}
	f->timeUpdate(&Xa[0], &Xa[S*L], &Xa[0], u, dt, L);

	// sum weighted resultant sigma points to create estimated state
	f->w0m = (f->hh - (float32_t)(S+V)) / f->hh;
	for (i = 0; i < S; i++) {
		int rOffset = i*L;

		x[i] = Xa[rOffset + 0] * f->w0m;

		for (j = 1; j < L; j++)
			x[i] += Xa[rOffset + j] * f->wim;
	}

	// update state covariance
	for (i = 0; i < S; i++) {
		int rOffset = i*(S+V)*2;

		for (j = 0; j < S+V; j++) {
			qrTempS[rOffset + j] = (Xa[i*L + j + 1] - Xa[i*L + S+V + j + 1]) * f->wic1;
			qrTempS[rOffset + S+V + j] = (Xa[i*L + j + 1] + Xa[i*L + S+V + j + 1] - 2.0f*Xa[i*L + 0]) * f->wic2;
		}
	}

	qrDecompositionT_f32(&f->qrTempS, NULL, &f->SxT);   // with transposition
	arm_mat_trans_f32(&f->SxT, &f->Sx);
}

void srcdkfMeasurementUpdate(srcdkf_t *f, float32_t *u, float32_t *ym, int M, int N, float32_t *noise, SRCDKFMeasurementUpdate_t *measurementUpdate) {
	int S = f->S;				// number of states
	float32_t *Xa = f->Xa.pData;			// sigma points
	float32_t *xIn = f->xIn;			// callback buffer
	float32_t *xNoise = f->xNoise;		// callback buffer
	float32_t *xOut = f->xOut;			// callback buffer
	float32_t *Y = f->Y.pData;			// measurements from sigma points
	float32_t *y = f->y.pData;			// measurement estimate
	float32_t *Sn = f->Sn.pData;			// observation noise covariance
	float32_t *qrTempM = f->qrTempM.pData;
	float32_t *C1 = f->C1.pData;
	float32_t *C1T = f->C1T.pData;
	float32_t *C2 = f->C2.pData;
	float32_t *D = f->D.pData;
	float32_t *inov = f->inov.pData;		// M x 1 matrix
	float32_t *xUpdate = f->xUpdate.pData;	// S x 1 matrix
	float32_t *x = f->x.pData;			// state estimate
	float32_t *Sx = f->Sx.pData;
	float32_t *Q = f->Q.pData;
	float32_t *qrFinal = f->qrFinal.pData;
	int L;					// number of sigma points
	int i, j;

	// make measurement noise matrix if provided
	if (noise) {
		f->Sn.numRows = N;
		f->Sn.numCols = N;
		arm_fill_f32(0.0f, f->Sn.pData, N*N);
		for (i = 0; i < N; i++)
			arm_sqrt_f32(fabsf(noise[i]), &Sn[i*N + i]);
	}

	// generate sigma points
	srcdkfCalcSigmaPoints(f, &f->Sn);
	L = f->L;

	// resize all N and M based storage as they can change each iteration
	f->y.numRows = M;
	f->Y.numRows = M;
	f->Y.numCols = L;
	f->qrTempM.numRows = M;
	f->qrTempM.numCols = (S+N)*2;
	f->Sy.numRows = M;
	f->Sy.numCols = M;
	f->SyT.numRows = M;
	f->SyT.numCols = M;
	f->SyC.numRows = M;
	f->SyC.numCols = M;
	f->Pxy.numCols = M;
	f->C1.numRows = M;
	f->C1T.numCols = M;
	f->C2.numRows = M;
	f->C2.numCols = N;
	f->D.numRows = M;
	f->D.numCols = S+N;
	f->K.numCols = M;
	f->inov.numRows = M;
	f->qrFinal.numCols = 2*S + 2*N;

	// Y = h(Xa, Xn)
	for (i = 0; i < L; i++) {
		for (j = 0; j < S; j++)
			xIn[j] = Xa[j*L + i];

		for (j = 0; j < N; j++)
			xNoise[j] = Xa[(S+j)*L + i];

		measurementUpdate(u, xIn, xNoise, xOut);

		for (j = 0; j < M; j++)
			Y[j*L + i] = xOut[j];
	}

	// sum weighted resultant sigma points to create estimated measurement
	f->w0m = (f->hh - (float32_t)(S+N)) / f->hh;
	for (i = 0; i < M; i++) {
		int rOffset = i*L;

		y[i] = Y[rOffset + 0] * f->w0m;

		for (j = 1; j < L; j++)
			y[i] += Y[rOffset + j] * f->wim;
	}

	// calculate measurement covariance components
	for (i = 0; i < M; i++) {
		int rOffset = i*(S+N)*2;

		for (j = 0; j < S+N; j++) {
			float32_t c, d;

			c = (Y[i*L + j + 1] - Y[i*L + S+N + j + 1]) * f->wic1;
			d = (Y[i*L + j + 1] + Y[i*L + S+N + j + 1] - 2.0f*Y[i*L]) * f->wic2;

			qrTempM[rOffset + j] = c;
			qrTempM[rOffset + S+N + j] = d;

			// save fragments for future operations
			if (j < S) {
				C1[i*S + j] = c;
				C1T[j*M + i] = c;
			}
			else {
				C2[i*N + (j-S)] = c;
			}
			D[i*(S+N) + j] = d;
		}
	}

	qrDecompositionT_f32(&f->qrTempM, NULL, &f->SyT);	// with transposition

	arm_mat_trans_f32(&f->SyT, &f->Sy);
	arm_mat_trans_f32(&f->SyT, &f->SyC);		// make copy as later Div is destructive

	// create Pxy
	arm_mat_mult_f32(&f->Sx, &f->C1T, &f->Pxy);

	// K = (Pxy / SyT) / Sy
	matrixDiv_f32(&f->K, &f->Pxy, &f->SyT, &f->Q, &f->R, &f->AQ);
	matrixDiv_f32(&f->K, &f->K, &f->Sy, &f->Q, &f->R, &f->AQ);

	// x = x + k(ym - y)
	for (i = 0; i < M; i++)
		inov[i] = ym[i] - y[i];
	arm_mat_mult_f32(&f->K, &f->inov, &f->xUpdate);

	for (i = 0; i < S; i++)
		x[i] += xUpdate[i];

	// build final QR matrix
	//	rows = s
	//	cols = s + n + s + n
	//	use Q as temporary result storage

	f->Q.numRows = S;
	f->Q.numCols = S;
	arm_mat_mult_f32(&f->K, &f->C1, &f->Q);
	for (i = 0; i < S; i++) {
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < S; j++)
			qrFinal[rOffset + j] = Sx[i*S + j] - Q[i*S + j];
	}

	f->Q.numRows = S;
	f->Q.numCols = N;
	arm_mat_mult_f32(&f->K, &f->C2, &f->Q);
	for (i = 0; i < S; i++) {
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < N; j++)
			qrFinal[rOffset + S+j] = Q[i*N + j];
	}

	f->Q.numRows = S;
	f->Q.numCols = S+N;
	arm_mat_mult_f32(&f->K, &f->D, &f->Q);
	for (i = 0; i < S; i++) {
		int rOffset = i*(2*S + 2*N);

		for (j = 0; j < S+N; j++)
			qrFinal[rOffset + S+N+j] = Q[i*(S+N) + j];
	}

	// Sx = qr([Sx-K*C1 K*C2 K*D]')
	// this method is not susceptable to numeric instability like the Cholesky is
	qrDecompositionT_f32(&f->qrFinal, NULL, &f->SxT);	// with transposition
	arm_mat_trans_f32(&f->SxT, &f->Sx);
}

void paramsrcdkfSetVariance(srcdkf_t *f, float32_t *v, float32_t *n) {
	float32_t *rDiag = f->rDiag.pData;
	int i;

	srcdkfSetVariance(f, v, v, n, f->N);

	for (i = 0; i < f->S; i++)
		rDiag[i] = 0.0;
}

void paramsrcdkfGetVariance(srcdkf_t *f, float32_t *v, float32_t *n) {
	float32_t *Sx = f->Sx.pData;
	float32_t *Sn = f->Sn.pData;
	int i;

	// artificial parameter variance
	if (v)
		for (i = 0; i < f->S; i++) {
			v[i] = Sx[i*f->S + i];
			v[i] = v[i]*v[i];
		}

	if (n)
		for (i = 0; i < f->N; i++) {
			n[i] = Sn[i*f->N + i];
			n[i] = n[i]*n[i];
		}
}

void paramsrcdkfSetRM(srcdkf_t *f, float32_t rm) {
	f->rm = rm;
}

// parameters, outputs, output noise, map func
srcdkf_t *paramsrcdkfInit(int w, int d, int n, SRCDKFMeasurementUpdate_t *map) {
	srcdkf_t *f;

	f = srcdkfInit(w, d, w, n, (void(*)(float32_t*, float32_t*, float32_t*, float32_t*, float32_t, int))0);

	f->M = d;
	f->N = n;
	f->map = map;
	f->rm = SRCDKF_RM;

	matrixInit(&f->KT, d, w);
	matrixInit(&f->inovT, 1, d);
	matrixInit(&f->rDiag, w, 1);

	return f;
}

void paramsrcdkfUpdate(srcdkf_t *f, float32_t *u, float32_t *d) {
	int S = f->S;
	float32_t *Sx = f->Sx.pData;
	float32_t *Sv = f->Sv.pData;
	float32_t *rDiag = f->rDiag.pData;
	int i;

	srcdkfMeasurementUpdate(f, u, d, f->M, f->N, 0, f->map);

	if (f->rm) {
		// Robbins-Monro innovation estimation for Rr
		// Rr = (1 - SRCDKF_RM)*Rr + SRCDKF_RM * K * [dk - g(xk, wk)] * [dk - g(xk, wk)]^T * K^T

		arm_mat_trans_f32(&f->K, &f->KT);
		arm_mat_trans_f32(&f->inov, &f->inovT);

		// xUpdate == K*inov
		arm_mat_mult_f32(&f->xUpdate, &f->inovT, &f->K);
		arm_mat_mult_f32(&f->K, &f->KT, &f->Sv);

		for (i = 0; i < S; i++) {
			rDiag[i] = (1.0f - f->rm)*rDiag[i] + f->rm * Sv[i*S + i];
			Sx[i*S + i] = sqrt(Sx[i*S + i] * Sx[i*S + i] + rDiag[i]);
		}
	}
}
