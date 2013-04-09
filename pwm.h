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

#ifndef _pwm_h
#define _pwm_h

enum pwmDirections {
    PWM_OUTPUT = 1,
    PWM_INPUT
};

typedef void pwmCallback_t(uint32_t, uint8_t);

typedef struct {
    volatile uint32_t *ccr;
    volatile uint32_t *cnt;
    pwmCallback_t *callback;
    uint32_t period;
    int8_t direction;
} pwmPortStruct_t;

extern pwmPortStruct_t *pwmInitOut(uint8_t pwmPort, uint32_t period, uint32_t inititalValue, int8_t ESC32Mode);
extern pwmPortStruct_t *pwmInitIn(uint8_t pwmPort, int16_t polarity, uint32_t period, pwmCallback_t callback);
extern uint16_t pwmCheckTimer(uint8_t pwmPort);
extern void pwmZeroTimers(void);

#endif
