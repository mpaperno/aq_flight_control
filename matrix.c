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

#include "arm_math.h"
#include <stdlib.h>

void matrixInit(arm_matrix_instance_f32 *m, int rows, int cols) {
    float32_t *d;

    d = (float32_t *)calloc(rows*cols, sizeof(float32_t));

    arm_mat_init_f32(m, rows, cols, d);
}

void matrixFree(arm_matrix_instance_f32 *m) {
    if (m && m->pData)
	free(m->pData);
}
