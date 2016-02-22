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

#include "aq.h"
#include "gimbal.h"
#include "imu.h"
#include "nav_ukf.h"
#include "rc.h"
#include "util.h"
#include "config.h"
#include "aq_timer.h"
#include "nav.h"
#include "comm.h"
#include "gps.h"
#include "supervisor.h"
#include <string.h>

gimbalStruct_t gimbalData __attribute__((section(".ccm")));

void gimbalInit(void) {
    int8_t chkTim, initPitchPort, initRollPort, initTiltPort, initTrigPort, initPsthrPort;

    AQ_NOTICE("Gimbal init\n");

    memset((void *)&gimbalData, 0, sizeof(gimbalData));

    gimbalData.trigger = 0;
    gimbalData.triggerCount = 0;
    gimbalData.triggerLogVal = 0;
    gimbalData.triggerTimer = 0;
    gimbalData.triggerLastTime = 0;
    gimbalData.triggerLastLat = (double)0.0f;
    gimbalData.triggerLastLon = (double)0.0f;

    if (!p[GMBL_PWM_FREQ]) {
	AQ_NOTICE("Gimbal functions disabled.\n");
	return;
    }

    chkTim = ((int)p[GMBL_PWM_FREQ] != MOTORS_PWM_FREQ);

    initPitchPort = (p[GMBL_PITCH_PORT] && (!chkTim || !pwmCheckTimer(p[GMBL_PITCH_PORT]-1)));
    initRollPort = (p[GMBL_ROLL_PORT] && (!chkTim || !pwmCheckTimer(p[GMBL_ROLL_PORT]-1)));
    initTiltPort = (p[GMBL_TILT_PORT] && p[GMBL_TILT_PORT] != p[GMBL_PITCH_PORT] && (!chkTim || !pwmCheckTimer(p[GMBL_TILT_PORT]-1)));
    initTrigPort = (p[GMBL_TRIG_PORT] && (!chkTim || !pwmCheckTimer(p[GMBL_TRIG_PORT]-1)));
    initPsthrPort = (p[GMBL_PSTHR_PORT] && (!chkTim || !pwmCheckTimer(p[GMBL_PSTHR_PORT]-1)) && p[GMBL_PSTHR_CHAN] && (int)p[GMBL_PSTHR_CHAN] <= RADIO_MAX_CHANNELS);

    if (initPitchPort) {
	gimbalData.pitchPort = pwmInitOut(p[GMBL_PITCH_PORT]-1, PWM_RESOLUTION, (int)p[GMBL_PWM_FREQ], GMBL_PITCH_NTRL_PWM, -1);
	if (gimbalData.pitchPort) {
	    if (p[GMBL_TILT_PORT] == p[GMBL_PITCH_PORT])
		AQ_NOTICE("Gimbal combined PITCH & TILT port initialized.\n");
	    else
		AQ_NOTICE("Gimbal PITCH stabilization port initialized.\n");
	    yield(100);
	}
    }
    if (initRollPort) {
	gimbalData.rollPort = pwmInitOut(p[GMBL_ROLL_PORT]-1, PWM_RESOLUTION, (int)p[GMBL_PWM_FREQ], GMBL_ROLL_NTRL_PWM, -1);
	if (gimbalData.rollPort) {
	    AQ_NOTICE("Gimbal ROLL stabilization port initialized.\n");
	}
    }
    if (initTiltPort) {
	gimbalData.tiltPort = pwmInitOut(p[GMBL_TILT_PORT]-1, PWM_RESOLUTION, (int)p[GMBL_PWM_FREQ], GMBL_PITCH_NTRL_PWM, -1);
	if (gimbalData.tiltPort) {
	    AQ_NOTICE("Gimbal TILT control port initialized.\n");
	}
    }
    if (initTrigPort) {
	gimbalData.triggerPort = pwmInitOut(p[GMBL_TRIG_PORT]-1, PWM_RESOLUTION, (int)p[GMBL_PWM_FREQ], GMBL_TRIG_NTRL_PWM, -1);
	if (gimbalData.triggerPort) {
	    AQ_NOTICE("Gimbal TRIGGER control port initialized.\n");
	}
    }
    if (initPsthrPort) {
	gimbalData.passthroughPort = pwmInitOut(p[GMBL_PSTHR_PORT]-1, PWM_RESOLUTION, (int)p[GMBL_PWM_FREQ], GMBL_TRIG_NTRL_PWM, -1);
	if (gimbalData.passthroughPort) {
	    AQ_NOTICE("Gimbal radio PASSTHROUGH port initialized.\n");
	}
    }
}

void gimbalUpdate(void) {
    uint16_t pwm;
    float tmp_f;

    if (timerMicros() < 5e6f)
	return;

    // calculate manual/PoI tilt angle override
    if (p[GMBL_TILT_PORT]) {
	tmp_f = rcGetControlValue(GMBL_CTRL_TILT) + navData.poiAngle * GMBL_DEGREES_TO_PW * GMBL_PITCH_SCALE;
	// smooth
	if (tmp_f != gimbalData.tilt)
	    gimbalData.tilt -= (gimbalData.tilt - tmp_f) * configGetParamValue(GMBL_SLEW_RATE);
    }

    // stabilized pitch axis output
    if (gimbalData.pitchPort) {
	tmp_f = -AQ_PITCH * GMBL_DEGREES_TO_PW * GMBL_PITCH_SCALE;
	// stabilization AND manual/PoI override output to pitchPort
	if (p[GMBL_TILT_PORT] == p[GMBL_PITCH_PORT])
	    tmp_f += gimbalData.tilt;
	pwm = constrainInt(tmp_f + GMBL_PITCH_NTRL_PWM, p[GMBL_PWM_MIN_PT], p[GMBL_PWM_MAX_PT]);
	*gimbalData.pitchPort->ccr = pwm;
    }

    // unstabilized cameara tilt (pitch) output with manual and PoI override
    if (gimbalData.tiltPort) {
	pwm = constrainInt(gimbalData.tilt + GMBL_PITCH_NTRL_PWM, p[GMBL_PWM_MIN_PT], p[GMBL_PWM_MAX_PT]);
	*gimbalData.tiltPort->ccr = pwm;
    }

    // stabilized roll axis output
    if (gimbalData.rollPort) {
	tmp_f = configGetParamValue(GMBL_ROLL_EXPO);
	if (tmp_f)
	    tmp_f = (-AQ_ROLL * (fabsf(AQ_ROLL) / (100 / tmp_f))) * GMBL_DEGREES_TO_PW * GMBL_ROLL_SCALE + GMBL_ROLL_NTRL_PWM;
	else
	    tmp_f = -AQ_ROLL * GMBL_DEGREES_TO_PW * GMBL_ROLL_SCALE + GMBL_ROLL_NTRL_PWM;
	pwm = constrainInt(tmp_f, p[GMBL_PWM_MIN_RL], p[GMBL_PWM_MAX_RL]);
	*gimbalData.rollPort->ccr = pwm;
    }

    // trigger output, manual/passthrough or automatic modes
    if (gimbalData.triggerPort) {
	pwm = rcGetControlValue(GMBL_CTRL_TRG_ON) + GMBL_TRIG_NTRL_PWM;

	// manual trigger active
	if (rcIsSwitchActive(GMBL_CTRL_TRG_ON)) {
	    gimbalData.trigger = 0;		// cancel automatic trigger, if any
	    gimbalData.triggerTimer = 0;
	    if (!gimbalData.triggerLogVal)	// first activation after channel had returned to neutral
		gimbalData.triggerLogVal = ++gimbalData.triggerCount;
	}
	// automated trigger detection if trigger channel at neutral position
	else {
	    // if trigger is not already active
	    if (!gimbalData.triggerTimer) {
		gimbalData.triggerLogVal = 0;	// trigger should be off at this point (auto or manual)

		if (!gimbalData.trigger && (supervisorData.state & STATE_FLYING)) {
		    tmp_f = configGetParamValue(GMBL_TRIG_TIME);
		    if (tmp_f && timerMicros() - gimbalData.triggerLastTime >= tmp_f * 1e6) {
			gimbalData.trigger = 1;
			gimbalData.triggerLastTime = timerMicros();
			AQ_NOTICE("Time trigger activated.\n");
		    }
		    tmp_f = configGetParamValue(GMBL_TRIG_DIST);
		    if (tmp_f && (gimbalData.triggerLastLat != gpsData.lat || gimbalData.triggerLastLon != gpsData.lon) &&
			    navCalcDistance(gimbalData.triggerLastLat, gimbalData.triggerLastLon, gpsData.lat, gpsData.lon) >= tmp_f) {
			gimbalData.trigger = 1;
			gimbalData.triggerLastLat = gpsData.lat;
			gimbalData.triggerLastLon = gpsData.lon;
			AQ_NOTICE("Distance trigger activated.\n");
		    }
		}

		if (gimbalData.trigger) {
		    gimbalData.triggerTimer = timerMicros();
		    gimbalData.triggerLogVal = ++gimbalData.triggerCount;
		    gimbalData.trigger = 0;
		}
	    }

	    // if trigger is active, keep correct pwm output until timeout
	    if (gimbalData.triggerTimer) {
		if (timerMicros() - gimbalData.triggerTimer > p[GMBL_TRIG_ON_TIM] * 1e3)
		    gimbalData.triggerTimer = 0;
		else
		    pwm = p[GMBL_TRIG_ON_PWM];
	    }
	} // end auto trigger

	pwm = constrainInt(pwm, GMBL_TRIG_MIN_PWM, GMBL_TRIG_MAX_PWM);
	*gimbalData.triggerPort->ccr = pwm;
    }

    // simple passthrough output of any channel
    if (gimbalData.passthroughPort) {
	pwm = constrainInt(rcGetControlValue(GMBL_PSTHR_CHAN) + GMBL_TRIG_NTRL_PWM, GMBL_TRIG_MIN_PWM, GMBL_TRIG_MAX_PWM);
	*gimbalData.passthroughPort->ccr = pwm;
    }
}
