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

#include "aq.h"
#include "gimbal.h"
#include "imu.h"
#include "nav_ukf.h"
#include "radio.h"
#include "util.h"
#include "config.h"
#include "aq_timer.h"

gimbalStruct_t gimbalData;

void gimbalInit(void) {
#ifdef GIMBAL_PORT_PITCH
    gimbalData.pitch = pwmInit(GIMBAL_PORT_PITCH, 5000, PWM_OUTPUT, p[GMBL_NTRL_PITCH]);
#endif
#ifdef GIMBAL_PORT_ROLL
    gimbalData.roll = pwmInit(GIMBAL_PORT_ROLL, 5000, PWM_OUTPUT, p[GMBL_NTRL_ROLL]);
#endif
}

void gimbalUpdate(void) {
    uint16_t pwm;

    if (gimbalData.pitch) {
	if (RADIO_AUX3 != gimbalData.radioPitch) {
	    gimbalData.radioPitch -= (gimbalData.radioPitch - RADIO_AUX3) * p[GMBL_SLEW_RATE];
	}

	pwm = constrainInt(-AQ_PITCH * (2000 - 1000) * p[GMBL_SCAL_PITCH] + p[GMBL_NTRL_PITCH] + gimbalData.radioPitch, p[GMBL_PWM_MIN], p[GMBL_PWM_MAX]);
	if (timerMicros() > 20000000)
	    *gimbalData.pitch->ccr = pwm;
    }

    if (gimbalData.roll) {
	pwm = constrainInt(-AQ_ROLL * (2000 - 1000) * p[GMBL_SCAL_ROLL] + p[GMBL_NTRL_ROLL], p[GMBL_PWM_MIN], p[GMBL_PWM_MAX]);
	if (timerMicros() > 20000000)
	    *gimbalData.roll->ccr = pwm;
    }
}
