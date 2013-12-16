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

#ifndef _quatos_h
#define _quatos_h

#include "aq_math.h"

extern void quatosInit(void);
extern void quatosReset(float32_t *quat);
extern void quatos(float32_t *quatD, float32_t *rates, uint16_t *override);
extern void quatosRate(float32_t *rates);
extern void quatosPowerDistribution(float32_t hoverThrot);

#endif
