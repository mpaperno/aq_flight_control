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
#include "motors.h"
#include "radio.h"
#include "util.h"
#include "config.h"
#include "comm.h"
#include "aq_timer.h"
#include "rcc.h"
#include "analog.h"
#include "aq_mavlink.h"
#include <math.h>
#include <string.h>
#include <stdlib.h>

motorsStruct_t motorsData __attribute__((section(".ccm")));

void motorsSendValues(void) {
    int i;

    for (i = 0; i < PWM_NUM_PORTS; i++)
	if (motorsData.active[i]) {
	    // ensure motor output is constrained
	    motorsData.value[i] = constrainInt(motorsData.value[i], p[MOT_START], p[MOT_MAX]);
	    if (motorsData.pwm[i])
		*motorsData.pwm[i]->ccr = motorsData.value[i];
	}
}

void motorsOff(void) {
    int i;

    for (i = 0; i < PWM_NUM_PORTS; i++)
	if (motorsData.active[i]) {
	    motorsData.value[i] = p[MOT_MIN];
	    if (motorsData.pwm[i])
		*motorsData.pwm[i]->ccr = motorsData.value[i];
    }

    motorsData.throttle = 0;
    motorsData.throttleLimiter = 0.0f;
}

void motorsCommands(float throtCommand, float pitchCommand, float rollCommand, float ruddCommand) {
    float throttle;
    float expFactor;
    float voltageFactor;
    float value;
    float nominalBatVolts;
    int i;

    // throttle limiter to prevent control saturation
    throttle = constrainFloat(throtCommand - motorsData.throttleLimiter, 0.0f, p[MOT_MAX]);

    // scale commands based on throttle setting
    expFactor = constrainFloat((p[MOT_HOV_THROT] - throttle) * p[MOT_EXP_FACT], p[MOT_EXP_MIN], p[MOT_EXP_MAX]);
    pitchCommand += (pitchCommand * expFactor);
    rollCommand += (rollCommand * expFactor);
    ruddCommand += (ruddCommand * expFactor);

    // calculate voltage factor
    nominalBatVolts = MOTORS_CELL_VOLTS*analogData.batCellCount;
    voltageFactor = 1.0f + (nominalBatVolts - analogData.vIn) / nominalBatVolts;

    // calculate and set each motor value
    for (i = 0; i < PWM_NUM_PORTS; i++) {
	if (motorsData.active[i]) {
	    motorsPowerStruct_t *d = &motorsData.distribution[i];

	    value = 0.0f;
	    value += (throttle * d->throttle * 0.01f);
	    value += (pitchCommand * d->pitch * 0.01f);
	    value += (rollCommand * d->roll * 0.01f);
	    value += (ruddCommand * d->yaw * 0.01f);

	    motorsData.value[i] = value*voltageFactor + p[MOT_START];

	    // check for over throttle
	    if (motorsData.value[i] == p[MOT_MAX])
		motorsData.throttleLimiter += MOTORS_THROTTLE_LIMITER;
	}
    }

    motorsSendValues();

    // decay throttle limit
    motorsData.throttleLimiter = constrainFloat(motorsData.throttleLimiter - MOTORS_THROTTLE_LIMITER, 0.0f, p[MOT_MAX]/2);

    motorsData.pitch = pitchCommand;
    motorsData.roll = rollCommand;
    motorsData.yaw = ruddCommand;
    motorsData.throttle = throttle;
}

void motorsInit(void) {
    float sumPitch, sumRoll, sumYaw;
    int i;

    AQ_NOTICE("Motors init\n");

    memset((void *)&motorsData, 0, sizeof(motorsData));

    if ( p[MOT_FRAME] > 0.01f && p[MOT_FRAME] < 4.01f ) {
	AQ_NOTICE("Motors: ERROR! Predefined frame types are no longer supported.\n");
	return;
    }

    motorsData.distribution = (motorsPowerStruct_t *)&p[MOT_PWRD_01_T];

    sumPitch = 0.0f;
    sumRoll = 0.0f;
    sumYaw = 0.0f;

    for (i = 0; i < PWM_NUM_PORTS; i++) {
	motorsPowerStruct_t *d = &motorsData.distribution[i];

	if (d->throttle != 0.0f || d->pitch != 0.0f || d->roll != 0.0f || d->yaw != 0.0f) {

#ifdef USE_L1_ATTITUDE
	    motorsData.pwm[i] = pwmInitOut(i, 2500, p[MOT_START], 1);	    // closed loop RPM mode
#else
	    motorsData.pwm[i] = pwmInitOut(i, 2500, p[MOT_START], 0);	    // open loop mode
#endif
	    motorsData.active[i] = 1;

	    sumPitch += d->pitch;
	    sumRoll += d->roll;
	    sumYaw += d->yaw;
	}
    }

    if (fabsf(sumPitch) > 0.01f)
	AQ_NOTICE("Motors: Warning pitch control imbalance\n");

    if (fabsf(sumRoll) > 0.01f)
	AQ_NOTICE("Motors: Warning roll control imbalance\n");

    if (fabsf(sumYaw) > 0.01f)
	AQ_NOTICE("Motors: Warning yaw control imbalance\n");

    motorsOff();
}

