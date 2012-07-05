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

#ifndef _pwm_h
#define _pwm_h

enum pwmDirections {
    PWM_OUTPUT,
    PWM_INPUT
};

typedef struct {
    volatile uint32_t *ccr;
    volatile uint32_t *cnt;
} pwmStruct_t;

extern pwmStruct_t *pwmInit(uint8_t pwmPort, uint32_t period, uint8_t direction, uint32_t inititalValue, int8_t ESC32Mode);

#endif
