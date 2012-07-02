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

enum pwmESC32configParameters {
    ESC32_CONFIG_VERSION = 0,
    ESC32_STARTUP_MODE,
    ESC32_BAUD_RATE,
    ESC32_PTERM,
    ESC32_ITERM,
    ESC32_FF1TERM,
    ESC32_FF2TERM,
    ESC32_CL1TERM,
    ESC32_CL2TERM,
    ESC32_CL3TERM,
    ESC32_CL4TERM,
    ESC32_CL5TERM,
    ESC32_SHUNT_RESISTANCE,
    ESC32_MIN_PERIOD,
    ESC32_MAX_PERIOD,
    ESC32_BLANKING_MICROS,
    ESC32_ADVANCE,
    ESC32_START_VOLTAGE,
    ESC32_GOOD_DETECTS_START,
    ESC32_BAD_DETECTS_DISARM,
    ESC32_MAX_CURRENT,
    ESC32_SWITCH_FREQ,
    ESC32_MOTOR_POLES,
    ESC32_PWM_MIN_PERIOD,
    ESC32_PWM_MAX_PERIOD,
    ESC32_PWM_MIN_VALUE,
    ESC32_PWM_LO_VALUE,
    ESC32_PWM_HI_VALUE,
    ESC32_PWM_MAX_VALUE,
    ESC32_PWM_MIN_START,
    ESC32_PWM_RPM_SCALE,
    ESC32_FET_BRAKING,
    ESC32_CONFIG_NUM_PARAMS
};

typedef struct {
    volatile uint32_t *ccr;
    volatile uint32_t *cnt;
} pwmStruct_t;


extern pwmStruct_t *pwmInit(uint8_t pwmPort, uint32_t period, uint8_t direction, uint32_t inititalValue, int8_t ESC32Mode);

#endif
