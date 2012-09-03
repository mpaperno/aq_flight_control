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
#include "nav.h"
#include <string.h>

gimbalStruct_t gimbalData __attribute__((section(".ccm")));

void gimbalInit(void) {
    memset((void *)&gimbalData, 0, sizeof(gimbalData));

    if (p[GMBL_PITCH_PORT] >= 1.0f)
	gimbalData.pitch = pwmInitOut((uint8_t)(p[GMBL_PITCH_PORT] - 1.0f), 5000, p[GMBL_NTRL_PITCH], -1);

    if (p[GMBL_ROLL_PORT] >= 1.0f)
	gimbalData.roll = pwmInitOut((uint8_t)(p[GMBL_ROLL_PORT] - 1.0f), 5000, p[GMBL_NTRL_ROLL], -1);
}

void gimbalUpdate(void) {
    uint16_t pwm;
    float tilt;

    if (gimbalData.pitch) {
	tilt = (RADIO_AUX3 / p[GMBL_SCAL_PITCH]) + navData.poiAngle * (2000 - 1000);

	// smooth
	if (tilt != gimbalData.tilt)
	    gimbalData.tilt -= (gimbalData.tilt - tilt) * p[GMBL_SLEW_RATE];

	pwm = constrainInt((-AQ_PITCH * (2000 - 1000)  + gimbalData.tilt )* p[GMBL_SCAL_PITCH] + p[GMBL_NTRL_PITCH], p[GMBL_PWM_MIN], p[GMBL_PWM_MAX]);
	if (timerMicros() > 20000000)
	    *gimbalData.pitch->ccr = pwm;
    }

    if (gimbalData.roll) {
	pwm = constrainInt(-AQ_ROLL * (2000 - 1000) * p[GMBL_SCAL_ROLL] + p[GMBL_NTRL_ROLL], p[GMBL_PWM_MIN], p[GMBL_PWM_MAX]);
	if (timerMicros() > 20000000)
	    *gimbalData.roll->ccr = pwm;
    }
}
