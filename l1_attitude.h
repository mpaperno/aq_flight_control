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

#ifndef _l1_attitude_h
#define _l1_attitude_h

#include "aq_math.h"

extern void l1AttitudeInit(void);
extern void l1AttitudeReset(float32_t *quat);
extern void l1Attitude(float32_t *quatD);
extern void l1AttitudePowerDistribution(float32_t hoverThrot);

#endif
