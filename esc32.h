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

#ifndef _esc32_h
#define _esc32_h

#define ESC32_FILE_NAME			"esc32.txt"
#define ESC32_FILE_BUF_SIZE		512
#define ESC32_LINE_BUF_SIZE		128

enum esc32configParameters {
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

extern float esc32ReadParam(uint8_t paramId);
extern float esc32WriteParam(uint8_t paramId, float value);
extern uint8_t esc32Mode(uint8_t mode);
extern void esc32Setup(const GPIO_TypeDef *port, const uint16_t pin, uint8_t mode);

#endif
