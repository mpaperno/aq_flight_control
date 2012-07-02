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

#include "aq_math.h"
#include "util.h"
#include <intrinsics.h>

#define MIN(a, b) ((a < b) ? a : b)
#define MAX(a, b) ((a > b) ? a : b)

void matrixInit(arm_matrix_instance_f32 *m, int rows, int cols) {
    float32_t *d;

    d = (float32_t *)aqDataCalloc(rows*cols, sizeof(float32_t));

    arm_mat_init_f32(m, rows, cols, d);
    arm_fill_f32(0, d, rows*cols);
}

void matrixFree(arm_matrix_instance_f32 *m) {
    if (m && m->pData)
	free(m->pData);
}

//char matBuf[1024];
//char matBufTmp[64];
//
//#include "aq.h"
//#include "notice.h"
//#include "stdio.h"
//void matrixDump(char *name, arm_matrix_instance_f32 *m) {
//    int i, j;
//    char *p = matBuf;
//
//    p += sprintf(p, "%s  = [\n", name);
//    for (i = 0; i < m->numRows; i++) {
//	for (j = 0; j < m->numCols; j++) {
//	    p += sprintf(p, "%+.5e ", m->pData[i*m->numCols + j]);
//	}
//	p += sprintf(p, ";\n", name);
//    }
//    sprintf(p, "]\n", name);
//    AQ_NOTICE(matBuf);
//}

// Calculates the QR decomposition of the given matrix A Transposed (decomp's A', not A)
//      notes:  A matrix is modified
//      Adapted from Java code originaly written by Joni Salonen
//
void qrDecompositionT_f32(arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R) {
        int minor;
        int row, col;
        int m = A->numCols;
        int n = A->numRows;
        int min;

        // clear R
        arm_fill_f32(0, R->pData, R->numRows*R->numCols);

        min = MIN(m, n);

        /*
        * The QR decomposition of a matrix A is calculated using Householder
        * reflectors by repeating the following operations to each minor
        * A(minor,minor) of A:
        */
        for (minor = 0; minor < min; minor++) {
                float xNormSqr = 0.0f;
                float a;

                /*
                * Let x be the first column of the minor, and a^2 = |x|^2.
                * x will be in the positions A[minor][minor] through A[m][minor].
                * The first column of the transformed minor will be (a,0,0,..)'
                * The sign of a is chosen to be opposite to the sign of the first
                * component of x. Let's find a:
                */
                for (row = minor; row < m; row++)
                        xNormSqr += A->pData[minor*m + row]*A->pData[minor*m + row];

		a = __sqrtf(xNormSqr);
                if (A->pData[minor*m + minor] > 0.0f)
                        a = -a;

                R->pData[minor*R->numCols + minor] = a;

                if (a != 0.0f) {
                        /*
                        * Calculate the normalized reflection vector v and transform
                        * the first column. We know the norm of v beforehand: v = x-ae
                        * so |v|^2 = <x-ae,x-ae> = <x,x>-2a<x,e>+a^2<e,e> =
                        * a^2+a^2-2a<x,e> = 2a*(a - <x,e>).
                        * Here <x, e> is now A[minor][minor].
                        * v = x-ae is stored in the column at A:
                        */
                        A->pData[minor*m + minor] -= a; // now |v|^2 = -2a*(A[minor][minor])

                        /*
                        * Transform the rest of the columns of the minor:
                        * They will be transformed by the matrix H = I-2vv'/|v|^2.
                        * If x is a column vector of the minor, then
                        * Hx = (I-2vv'/|v|^2)x = x-2vv'x/|v|^2 = x - 2<x,v>/|v|^2 v.
                        * Therefore the transformation is easily calculated by
                        * subtracting the column vector (2<x,v>/|v|^2)v from x.
                        *
                        * Let 2<x,v>/|v|^2 = alpha. From above we have
                        * |v|^2 = -2a*(A[minor][minor]), so
                        * alpha = -<x,v>/(a*A[minor][minor])
                        */
                        for (col = minor+1; col < n; col++) {
                                float alpha = 0.0f;

                                for (row = minor; row < m; row++)
                                        alpha -= A->pData[col*m + row]*A->pData[minor*m + row];

                                alpha /= a*A->pData[minor*m + minor];

                                // Subtract the column vector alpha*v from x.
                                for (row = minor; row < m; row++)
                                        A->pData[col*m + row] -= alpha*A->pData[minor*m + row];
                        }
                }
        }

        // Form the matrix R of the QR-decomposition.
        //      R is supposed to be m x n, but only calculate n x n
        // copy the upper triangle of A
        for (row = min-1; row >= 0; row--)
                for (col = row+1; col < n; col++)
                        R->pData[row*R->numCols + col] = A->pData[col*m + row];

        // Form the matrix Q of the QR-decomposition.
        //      Q is supposed to be m x m

        // only compute Q if requested
        if (Q) {
                arm_fill_f32(0, Q->pData, Q->numRows*Q->numCols);

                /*
                * Q = Q1 Q2 ... Q_m, so Q is formed by first constructing Q_m and then
                * applying the Householder transformations Q_(m-1),Q_(m-2),...,Q1 in
                * succession to the result
                */
                for (minor = m-1; minor >= min; minor--)
                        Q->pData[minor*m + minor] = 1.0f;

                for (minor = min-1; minor >= 0; minor--) {
                        Q->pData[minor * m + minor] = 1.0f;

                        if (A->pData[minor*m + minor] != 0.0f) {
                                for (col = minor; col < m; col++) {
                                        float alpha = 0.0f;

                                        for (row = minor; row < m; row++)
                                                alpha -= Q->pData[row*m + col]*A->pData[minor*m + row];

                                        alpha /= R->pData[minor*R->numCols + minor]*A->pData[minor*m + minor];

                                        for (row = minor; row < m; row++)
                                                Q->pData[row*m + col] -= alpha*A->pData[minor*m + row];
                                }
                        }
                }
        }
}

// Solves m sets of n equations A * X = B using QR decomposition and backsubstitution
void matrixDiv_f32(arm_matrix_instance_f32 *X, arm_matrix_instance_f32 *A, arm_matrix_instance_f32 *B, arm_matrix_instance_f32 *Q, arm_matrix_instance_f32 *R, arm_matrix_instance_f32 *AQ) {
        int i, j, k;
        int m, n;

        // this is messy (going into a class's private data structure),
        // but it is better than malloc/free
        Q->numRows = B->numRows;
        Q->numCols = B->numRows;
        R->numRows = B->numRows;
        R->numCols = B->numCols;
        AQ->numRows = A->numRows;
        AQ->numCols = B->numRows;

        m = A->numRows;
        n = B->numCols;

        qrDecompositionT_f32(B, Q, R);
	arm_mat_mult_f32(A, Q, AQ);

        // solve for X by backsubstitution
        for (i = 0; i < m; i++) {
                for (j = n-1; j >= 0; j--) {
                        for (k = j+1; k < n; k++)
                                AQ->pData[i*n + j] -= R->pData[j*n + k] * X->pData[i*n + k];
                        X->pData[i*n + j] = AQ->pData[i*n + j] / R->pData[j*n + j];
                }
        }
}

