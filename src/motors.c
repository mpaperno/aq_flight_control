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
#include "motors.h"
#include "radio.h"
#include "util.h"
#include "config.h"
#include "comm.h"
#include "control.h"
#include "aq_timer.h"
#include "rcc.h"
#include "analog.h"
#include "aq_mavlink.h"
#include "can.h"
#include "esc32.h"
#include "supervisor.h"
#include "imu.h"
#ifndef __CC_ARM
#include <intrinsics.h>
#endif
#include <math.h>
#include <string.h>
#include <stdlib.h>

motorsStruct_t motorsData __attribute__((section(".ccm")));

#ifdef MOTORS_CAN_LOGGING
#include "filer.h"

uint8_t motorsLogBuf[MOTORS_CAN_LOGGING * sizeof(motorsLog_t)];

void motorsSetupLogging(void) {
    // setup logging
    motorsData.logHandle = filerGetHandle("ESC");
    filerStream(motorsData.logHandle, motorsLogBuf, sizeof(motorsLogBuf));
}
#endif

void motorsReceiveTelem(uint8_t canId, uint8_t doc, void *p) {
    uint32_t *data = (uint32_t *)p;
    uint32_t *storage = (uint32_t *)&motorsData.canStatus[canId-1];
    uint32_t micros = timerMicros();
#ifdef MOTORS_CAN_LOGGING
    motorsLog_t *buf = (motorsLog_t *)(motorsLogBuf + motorsData.head);

    buf->sync = 0xff;
    buf->escId = 0xc0 | canId;  // 2 MSB are high as part of sync bits
    buf->micros = micros;
    buf->data[0] = data[0];
    buf->data[1] = data[1];

    motorsData.head = (motorsData.head + sizeof(motorsLog_t)) % sizeof(motorsLogBuf);

    filerSetHead(motorsData.logHandle, motorsData.head);
#endif

    // copy status data to our storage (8 bytes)
    storage[0] = data[0];
    storage[1] = data[1];

    // record reception time
    motorsData.canStatusTime[canId-1] = micros;
}

static float motorsThrust2Value(float thrust) {
    return (-p[MOT_VALUE2T_A1] + __sqrtf(p[MOT_VALUE2T_A1]*p[MOT_VALUE2T_A1] + p[MOT_VALUE2T_A2] * 4.0f * thrust)) * (1.0f / (2.0f * p[MOT_VALUE2T_A2]));
}

float motorsPwm2Thrust(float pwm) {
    float value;

    value = p[MOT_VALUE_SCAL] * pwm / MOTORS_SCALE;

    return p[MOT_VALUE2T_A1]*value + p[MOT_VALUE2T_A2]*value*value;
}

float motorsMax(void) {
    return MOTORS_SCALE;
}

float motorsVFactor() {
    // calculate voltage factor
    float nominalBatVolts = MOTORS_CELL_VOLTS * analogData.batCellCount;
    return 1.0f + (nominalBatVolts - analogData.vIn) / nominalBatVolts;
}

static void motorsCanSendGroups(void) {
    int i;

    for (i = 0; i < motorsData.numGroups; i++)
	canCommandSetpoint16(i+1, (uint8_t *)&motorsData.canGroups[i]);
}

static void motorsCanRequestTelem(int motorId) {
#if MOTORS_CAN_TELEM_RATE > 0
    // request telemetry
    canSetTelemetryValueNoWait(CAN_TT_NODE, motorsData.can[motorId]->networkId, 0, CAN_TELEM_STATUS);
    canSetTelemetryRateNoWait(CAN_TT_NODE, motorsData.can[motorId]->networkId, MOTORS_CAN_TELEM_RATE);

    motorsData.canTelemReqTime[motorId] = timerMicros();
#endif
}

static void motorsCheckCanStatus(int motorId) {
#if MOTORS_CAN_TELEM_RATE > 0
    // no status report within the last second?
    if ((timerMicros() - motorsData.canStatusTime[motorId]) > 1e6f) {
        // has it been more than 1 second since our last request?
        if ((timerMicros() - motorsData.canTelemReqTime[motorId]) > 1e6f) {
            // clear status information
            uint32_t *storage = (uint32_t *)&motorsData.canStatus[motorId];
            storage[0] = 0;
            storage[1] = 0;

            motorsCanRequestTelem(motorId);
        }
    }
    // if ESC is reporting as being disarmed (and should not be)
    else if (motorsData.canStatus[motorId].state == ESC32_STATE_DISARMED && (supervisorData.state & STATE_ARMED)) {
	// send an arm command
	canCommandArm(CAN_TT_NODE, motorsData.can[motorId]->networkId);
    }
#endif
}

void motorsSendValues(void) {
    int i;

    if (motorsData.disableAllPorts)
	return;

    for (int j = 0; j < motorsData.numActive; ++j) {
	i = motorsData.activeList[j];
	// ensure motor output is constrained
	motorsData.value[i] = constrainInt(motorsData.value[i], 0, MOTORS_SCALE);

	// PWM
	if (i < PWM_NUM_PORTS && motorsData.pwm[i]) {
	    if (supervisorData.state & STATE_ARMED) {
#ifdef HAS_ONBOARD_ESC
		if (MOTORS_ESC_TYPE == ESC_TYPE_ONBOARD_PWM)
		    *motorsData.pwm[i]->ccr = constrainInt((float)motorsData.value[i] * (ONBOARD_ESC_PWM_MAX - ONBOARD_ESC_PWM_MIN) / MOTORS_SCALE + ONBOARD_ESC_PWM_MIN, ONBOARD_ESC_PWM_START, ONBOARD_ESC_PWM_MAX);
		else
#endif
		    *motorsData.pwm[i]->ccr = constrainInt((float)motorsData.value[i] * (p[MOT_MAX] - p[MOT_MIN]) / MOTORS_SCALE + p[MOT_MIN], p[MOT_START], p[MOT_MAX]);
	    } else
		*motorsData.pwm[i]->ccr = 0;
	}
	// CAN
	else if (motorsData.can[i]) {
	    motorsCheckCanStatus(i);

	    if (supervisorData.state & STATE_ARMED)
		// convert to 16 bit
		*motorsData.canPtrs[i] = constrainInt(motorsData.value[i], MOTORS_SCALE * 0.1f, MOTORS_SCALE)<<4;
	    else
		*motorsData.canPtrs[i] = 0;
	}
    }

    motorsCanSendGroups();
}

// thrust in gram-force
void motorsSendThrust(void) {
    float value, v;
    int i;

    for (int j = 0; j < motorsData.numActive; ++j) {
	i = motorsData.activeList[j];
	value = motorsThrust2Value(motorsData.thrust[i]);

	// using open-loop PWM ESC?
	if (i < PWM_NUM_PORTS && motorsData.pwm[i] && (uint8_t)p[MOT_ESC_TYPE] != ESC_TYPE_ESC32) {
	    // preload the request to accelerate setpoint changes
	    if (motorsData.oldValues[i] != value) {
		v = (value -  motorsData.oldValues[i]);

		// increase
		if (v > 0.0f)
		    value += v * MOTORS_COMP_PRELOAD_PTERM;
		// decrease
		else
		    value += v * MOTORS_COMP_PRELOAD_PTERM * MOTORS_COMP_PRELOAD_NFACT;

		// slowly follow setpoint
		motorsData.oldValues[i] += v * MOTORS_COMP_PRELOAD_TAU;
	    }

	    // adjust for voltage factor
	    value *= motorsVFactor();
	}

	motorsData.value[i] = constrainInt(value * MOTORS_SCALE / p[MOT_VALUE_SCAL], 0, MOTORS_SCALE);
    }

    motorsSendValues();
}

void motorsOff(void) {
    int i;

    for (int j = 0; j < motorsData.numActive; ++j) {
	i = motorsData.activeList[j];
	motorsData.value[i] = 0;

	// PWM
	if (i < PWM_NUM_PORTS && motorsData.pwm[i]) {
#ifdef HAS_ONBOARD_ESC
	    if (MOTORS_ESC_TYPE == ESC_TYPE_ONBOARD_PWM)
		*motorsData.pwm[i]->ccr = (supervisorData.state & STATE_ARMED) ? ONBOARD_ESC_PWM_ARM : 0;
	    else
#endif
		*motorsData.pwm[i]->ccr = (supervisorData.state & STATE_ARMED) ? p[MOT_ARM] : 0;
	}
	// CAN
	else if (motorsData.can[i]) {
	    motorsCheckCanStatus(i);
	    *motorsData.canPtrs[i] = 0;
	}
    }
    motorsCanSendGroups();

    motorsData.throttle = 0;
    motorsData.throttleLimiter = 0.0f;
}

void motorsCommands(float throtCommand, float pitchCommand, float rollCommand, float ruddCommand) {
    float throttle;
    float value;
    int i;

    // throttle limiter to prevent control saturation
    throttle = constrainFloat(throtCommand - motorsData.throttleLimiter, 0.0f, MOTORS_SCALE);

    // calculate and set each motor value
    for (int j = 0; j < motorsData.numActive; ++j) {
	i = motorsData.activeList[j];
	motorsPowerStruct_t *d = &motorsData.distribution[i];

	value = 0.0f;
	value += (throttle * d->throttle * 0.01f);
	value += (pitchCommand * d->pitch * 0.01f);
	value += (rollCommand * d->roll * 0.01f);
	value += (ruddCommand * d->yaw * 0.01f);

	// adjust for voltage factor
	value *= motorsVFactor();

	// check for over throttle
	if (value >= MOTORS_SCALE)
	    motorsData.throttleLimiter += MOTORS_THROTTLE_LIMITER;

	motorsData.value[i] = constrainInt(value, 0, MOTORS_SCALE);
    }

    motorsSendValues();

    // decay throttle limit
    motorsData.throttleLimiter = constrainFloat(motorsData.throttleLimiter - MOTORS_THROTTLE_LIMITER, 0.0f, MOTORS_SCALE/4);

    motorsData.pitch = pitchCommand;
    motorsData.roll = rollCommand;
    motorsData.yaw = ruddCommand;
    motorsData.throttle = throttle;
}

static int motorsCanInit(int i) {
    uint8_t numTry = CAN_RETRIES;
    uint8_t motorId = i;
    int ret = 1;
    float escVer;

    // canId's 17-32 map to motorId's 1-16
    if (motorId >= 16)
        motorId -= 16;

    while (numTry-- && (motorsData.can[motorId] = canFindNode(CAN_TYPE_ESC, i+1)) == 0)
        yield(100);

    if (motorsData.can[motorId] == 0) {
	AQ_PRINTF("Motors: cannot find CAN id [%d]\n", i+1);
	ret = 0;
    }
    else {
	escVer = esc32SetupCan(motorsData.can[motorId], USE_QUATOS);
	motorsData.esc32Version[motorId] = (uint16_t)(escVer / 0.01f);
    	motorsCanRequestTelem(motorId);
    }

    return ret;
}

static int motorsPwmInit(int i) {
    uint32_t res = PWM_RESOLUTION;
    uint32_t freq = MOTORS_PWM_FREQ;
    int8_t  esc32Mode = -1;
    int ret = 1;

    if (i >= PWM_NUM_PORTS) {
	AQ_PRINTF("Motors: PWM port number %d does not exist!\n", i);
	ret = 0;
    }
    else if (MOTORS_ESC_TYPE == ESC_TYPE_ONBOARD_PWM) {
#if defined(HAS_ONBOARD_ESC)
	res = ONBOARD_ESC_PWM_RESOLUTION;
	freq = MOTORS_ONBOARD_PWM_FREQ;
#else
	AQ_NOTICE("Motors: Onboard ESC not supported on this hardware!\n");
	ret = 0;
#endif
    }
    else if (MOTORS_ESC_TYPE == ESC_TYPE_ESC32) {
	esc32Mode = USE_QUATOS;
    }

    if (ret)
	motorsData.pwm[i] = pwmInitOut(i, res, freq, 0, esc32Mode);

    return ret;
}

int motorsArm(void) {
    int tries = 1;
    int i;

    if (motorsData.disableAllPorts)
	return tries;

    // group arm
    for (i = 0; i < motorsData.numGroups; i++)
	canCommandArm(CAN_TT_GROUP, i+1);

    // wait for all to arm
    for (i = 0; i < MOTORS_NUM; i++)
	if (motorsData.can[i]) {
            tries = 3;
	    while (--tries && *canGetState(motorsData.can[i]->networkId) == ESC32_STATE_DISARMED)
		yield(1);

            if (!tries)
                break;
        }

    return tries;
}

void motorsDisarm(void) {
    int i;

    // group disarm
    for (i = 0; i < motorsData.numGroups; i++)
	canCommandDisarm(CAN_TT_GROUP, i+1);
}

static void motorsSetCanGroup(void) {
    int group;
    int subGroup;
    int i;

    group = 0;
    subGroup = 0;
    for (i = 0; i < MOTORS_NUM; i++) {
	if (motorsData.can[i]) {
	    canSetGroup(motorsData.can[i]->networkId, group+1, subGroup+1);

	    switch (subGroup) {
		case 0:
		    motorsData.canPtrs[i] = &motorsData.canGroups[group].value1;
		    motorsData.numGroups++;
		    break;
		case 1:
		    motorsData.canPtrs[i] = &motorsData.canGroups[group].value2;
		    break;
		case 2:
		    motorsData.canPtrs[i] = &motorsData.canGroups[group].value3;
		    break;
		case 3:
		    motorsData.canPtrs[i] = &motorsData.canGroups[group].value4;
		    break;
	    }

	    subGroup++;
	    if (subGroup == MOTORS_CAN_GROUP_SIZE) {
		group++;
		subGroup = 0;
	    }
	}
    }
}

void motorsPwmToAll(float pwmValue) {
    int i;
    for (int j = 0; j < motorsData.numActive; ++j) {
	i = motorsData.activeList[j];
	if (i < PWM_NUM_PORTS && motorsData.pwm[i])
	    *motorsData.pwm[i]->ccr = constrainInt(pwmValue, p[MOT_MIN], p[MOT_MAX]);
    }
}

void motorsEscPwmCalibration(void) {
    // check esc type and calibration request bit
    if (MOTORS_ESC_TYPE != ESC_TYPE_STD_PWM || !((uint32_t)p[MOT_ESC_TYPE] & MOT_ESC_TYPE_CALIB_ENABLE))
	return;

    // reset calibration request bit
    p[MOT_ESC_TYPE] = (uint32_t)p[MOT_ESC_TYPE] & ~(MOT_ESC_TYPE_CALIB_ENABLE);
    // store new param, or fail
    if (!configSaveParamsToFlash())
	return;

    for (int i = 5; i > 0; --i) {
	AQ_PRINTF("Warning: ESC calibration, full throttle in %d sec!\n", i);
	yield(1000);
    }
    AQ_NOTICE("ESC Calibration: Setting maximum.\n");
    motorsPwmToAll(p[MOT_MAX]);
    yield(4000);
    AQ_NOTICE("ESC Calibration: Setting minimum.\n");
    motorsPwmToAll(p[MOT_MIN]);
    yield(3000);
    AQ_NOTICE("ESC Calibration: Finished. Restart system now.\n");
}

void motorsInit(void) {
    float sumPitch, sumRoll, sumYaw;
    int i, initOk;

    AQ_NOTICE("Motors init\n");

    memset((void *)&motorsData, 0, sizeof(motorsData));

    if (p[MOT_FRAME] > 0.01f && p[MOT_FRAME] < 4.01f) {
	AQ_NOTICE("Motors: ERROR! Predefined frame types are no longer supported.\n");
	return;
    }

    motorsData.distribution = (motorsPowerStruct_t *)configGetParamPtr(MOT_PWRD_01_T);

    sumPitch = 0.0f;
    sumRoll = 0.0f;
    sumYaw = 0.0f;

    for (i = 0; i < MOTORS_NUM; i++) {
	motorsPowerStruct_t *d = &motorsData.distribution[i];

	if (d->throttle != 0.0f || d->pitch != 0.0f || d->roll != 0.0f || d->yaw != 0.0f) {

	    // CAN LO
	    if (((uint32_t)p[MOT_CANL]) & (1<<i))
		initOk = motorsCanInit(i);
            // CAN HI (PDB)
	    else if (((uint32_t)p[MOT_CANH]) & (1<<i))
		initOk = motorsCanInit(i+16);
	    // PWM
	    else
		initOk = motorsPwmInit(i);

	    if (initOk) {
		motorsData.activeList[motorsData.numActive++] = i;

		sumPitch += d->pitch;
		sumRoll += d->roll;
		sumYaw += d->yaw;
	    }
	}
    }

    if (fabsf(sumPitch) > 0.01f)
	AQ_NOTICE("Motors: Warning pitch control imbalance\n");

    if (fabsf(sumRoll) > 0.01f)
	AQ_NOTICE("Motors: Warning roll control imbalance\n");

    if (fabsf(sumYaw) > 0.01f)
	AQ_NOTICE("Motors: Warning yaw control imbalance\n");

#ifdef MOTORS_CAN_LOGGING
    if (p[MOT_CANL] != 0.0f || p[MOT_CANH] != 0.0f)
        motorsSetupLogging();
#endif

    motorsEscPwmCalibration();
    motorsSetCanGroup();
    motorsOff();
    canTelemRegister(motorsReceiveTelem, CAN_TYPE_ESC);

    if ((uint32_t)p[MOT_ESC_TYPE] & MOT_ESC_TYPE_CALIB_ENABLE) {
	AQ_NOTICE("Motors: ERROR! ESC calibration bit still set.\n");
	motorsData.numActive = 0;
    }
}
