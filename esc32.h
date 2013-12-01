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

#ifndef _esc32_h
#define _esc32_h

#include "can.h"

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
    ESC32_PNFAC,
    ESC32_INFAC,
    ESC32_THR1TERM,
    ESC32_THR2TERM,
    ESC32_START_ALIGN_TIME,
    ESC32_START_ALIGN_VOLTAGE,
    ESC32_START_STEPS_NUM,
    ESC32_START_STEPS_PERIOD,
    ESC32_START_STEPS_ACCEL,
    ESC32_PWM_LOWPASS,
    ESC32_RPM_MEAS_LP,
    ESC32_SERVO_DUTY,
    ESC32_SERVO_P,
    ESC32_SERVO_D,
    ESC32_SERVO_MAX_RATE,
    ESC32_SERVO_SCALE,
    ESC32_ESC_ID,
    ESC32_DIRECTION,
    ESC32_CONFIG_NUM_PARAMS
};

enum esc32States {
    ESC32_STATE_DISARMED = 0,
    ESC32_STATE_STOPPED,
    ESC32_STATE_NOCOMM,
    ES32C_STATE_STARTING,
    ESC32_STATE_RUNNING
};

typedef struct {
    unsigned int state :    3;
    unsigned int vin :	    12;	// x 100
    unsigned int amps :	    14;	// x 100
    unsigned int rpm :	    15;
    unsigned int duty :	    8;	// x (255/100)
    unsigned int errors :   9;
    unsigned int errCode :  3;
} esc32CanStatus_t;

extern void esc32SetupOw(const GPIO_TypeDef *port, const uint16_t pin, uint8_t mode);
extern void esc32SetupCan(canNodes_t *canNode, uint8_t mode);

#endif
