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

#ifndef _pwm_h
#define _pwm_h

#include "aq.h"

#define pwmDigitalHi(p)		{ p->port->BSRRL = p->pin; }
#define pwmDigitalLo(p)		{ p->port->BSRRH = p->pin; }
#define pwmDigitalGet(p)	((p->port->ODR & p->pin) != 0)

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
    GPIO_TypeDef* port;
    uint16_t pin;
} pwmPortStruct_t;

extern pwmPortStruct_t *pwmInitOut(uint8_t pwmPort, uint32_t resolution, uint32_t freq, uint32_t inititalValue, int8_t ESC32Mode);
extern pwmPortStruct_t *pwmInitDigitalOut(uint8_t pwmPort);
extern pwmPortStruct_t *pwmInitIn(uint8_t pwmPort, int16_t polarity, uint32_t period, pwmCallback_t callback);
extern uint16_t pwmCheckTimer(uint8_t pwmPort);
extern void pwmZeroTimers(void);
extern void pwmDigitalToggle(pwmPortStruct_t *p);

#endif
