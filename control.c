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
#include "telemetry.h"
#include "pid.h"
#include "util.h"
#include "notice.h"
#include "motors.h"
#include "control.h"
#include "config.h"
#include "nav.h"
#include "compass.h"
#include "aq_timer.h"
#include "imu.h"
#include "radio.h"
#include "nav_ukf.h"
#include "aq_mavlink.h"
#include "supervisor.h"
#include "gps.h"
#ifdef USE_BACKSTEP_CONTROLLER
#include "backstep.h"
#endif
#include <CoOS.h>
#include <string.h>
#include <math.h>

controlStruct_t controlData;
OS_STK controlTaskStack[TASK_STACK_SIZE];

void controlTaskCode(void *unused) {
    float pitch, roll, yaw;
    float throttle;
#ifdef USE_BACKSTEP_CONTROLLER
#if USE_BACKSTEP_CONTROLLER == 2
    float quat[4];
#endif	// USE_BACKSTEP_CONTROLLER == 2
#else
    float pitchCommand, rollCommand, ruddCommand;
#endif	// USE_BACKSTEP_CONTROLLER

    AQ_NOTICE("Control task started...\n");

    while (1) {
	// wait for work
	CoWaitForSingleFlag(imuData.dRateFlag, 0);

	// this needs to be done ASAP with the freshest of data
	if (supervisorData.state & STATE_ARMED) {
	    if (RADIO_THROT > p[CTRL_MIN_THROT] || navData.mode > NAV_STATUS_MANUAL) {
		supervisorThrottleUp(1);

		// are we in altitude hold mode?
		if (navData.mode > NAV_STATUS_MANUAL) {
		    // override throttle with nav's request
		    throttle = pidUpdate(navData.altSpeedPID, navData.holdSpeedAlt, -UKF_VELD);
		}
		else {
		    throttle = (RADIO_THROT - p[CTRL_MIN_THROT]) * p[CTRL_FACT_THRO];
		}
		throttle = constrainInt(throttle, 1, 1400 * p[CTRL_FACT_THRO]);

		// if motors are not yet running, use this heading as hold heading
		if (motorsData.throttle == 0) {
		    navData.holdHeading = AQ_YAW;
		    controlData.yaw = navData.holdHeading;

		    // Reset all PIDs
		    pidZeroIntegral(controlData.pitchRatePID, 0.0f, 0.0f);
		    pidZeroIntegral(controlData.rollRatePID, 0.0f, 0.0f);
		    pidZeroIntegral(controlData.yawRatePID, 0.0f, 0.0f);

		    pidZeroIntegral(controlData.pitchAnglePID, 0.0f, 0.0f);
		    pidZeroIntegral(controlData.rollAnglePID, 0.0f, 0.0f);
		    pidZeroIntegral(controlData.yawAnglePID, 0.0f, 0.0f);

		    // also set this position as hold position
		    if (navData.mode == NAV_STATUS_POSHOLD)
			navUkfSetGlobalPositionTarget(gpsData.lat, gpsData.lon);
		}

		// implement rudder dead zone
		if (RADIO_RUDD > p[CTRL_DEAD_BAND] || RADIO_RUDD < -p[CTRL_DEAD_BAND]) {
		    // remove dead band first
		    if (RADIO_RUDD > p[CTRL_DEAD_BAND])
			yaw = (RADIO_RUDD - p[CTRL_DEAD_BAND]) * p[CTRL_FACT_RUDD];
		    else
			yaw = (RADIO_RUDD + p[CTRL_DEAD_BAND]) * p[CTRL_FACT_RUDD];

		    // don't allow desired yaw angle to lead attitude yaw angle by more than 90 deg
		    if (fabsf(compassDifference(navData.holdHeading + yaw, AQ_YAW) < 90.0f)) {
			controlData.yaw = compassNormalize(controlData.yaw + yaw);
			navData.holdHeading = compassNormalize(navData.holdHeading + yaw);
		    }
		}

		// constrict nav (only) yaw rates
		yaw = compassDifference(controlData.yaw, navData.holdHeading);
		yaw = constrainFloat(yaw, -p[CTRL_NAV_YAW_RT]/400.0f, +p[CTRL_NAV_YAW_RT]/400.0f);
		controlData.yaw = compassNormalize(controlData.yaw + yaw);

		// DVH overrides direct user pitch / roll requests
		if (navData.mode != NAV_STATUS_DVH) {
		    controlData.userPitchTarget = RADIO_PITCH * p[CTRL_FACT_PITC];
		    controlData.userRollTarget = RADIO_ROLL * p[CTRL_FACT_ROLL];
		}
		else {
		    controlData.userPitchTarget = 0.0f;
		    controlData.userRollTarget = 0.0f;
		}

		// navigation requests
		if (navData.mode > NAV_STATUS_ALTHOLD) {
		    controlData.navPitchTarget = navData.holdTiltN;
		    controlData.navRollTarget = navData.holdTiltE;
		}
		else {
		    controlData.navPitchTarget = 0.0f;
		    controlData.navRollTarget = 0.0f;
		}
#if USE_BACKSTEP_CONTROLLER != 2
		// smooth
		controlData.userPitchTarget = utilFilter3(controlData.userPitchFilter, controlData.userPitchTarget);
		controlData.userRollTarget = utilFilter3(controlData.userRollFilter, controlData.userRollTarget);

		// smooth
		controlData.navPitchTarget = utilFilter3(controlData.navPitchFilter, controlData.navPitchTarget);
		controlData.navRollTarget = utilFilter3(controlData.navRollFilter, controlData.navRollTarget);

		// rotate nav's NE frame of reference to our craft's local frame of reference
		pitch = controlData.navPitchTarget * navUkfData.yawCos - controlData.navRollTarget * navUkfData.yawSin;
		roll  = controlData.navRollTarget * navUkfData.yawCos + controlData.navPitchTarget * navUkfData.yawSin;

		// combine nav & user requests (both are already smoothed)
		controlData.pitch = pitch + controlData.userPitchTarget;
		controlData.roll = roll + controlData.userRollTarget;
#endif

#ifdef USE_BACKSTEP_CONTROLLER
		// calculate required thrust for each axis + throttle
#if USE_BACKSTEP_CONTROLLER == 1
		// reset controller on startup
		if (motorsData.throttle == 0)
		    backStepResetEuler(controlData.roll, controlData.pitch, controlData.yaw);

		backStepEuler(controlData.roll, controlData.pitch, controlData.yaw);
#else
		// determine which frame of reference to control from
		if (navData.mode <= NAV_STATUS_ALTHOLD)
		    // craft frame - manual
		    eulerToQuatYPR(quat, controlData.yaw, controlData.userPitchTarget, controlData.userRollTarget);
		else
		    // world frame - autonomous
		    eulerToQuatRPY(quat, controlData.navRollTarget, controlData.navPitchTarget, controlData.yaw);

		// reset controller on startup
		if (motorsData.throttle == 0) {
		    quat[0] = UKF_Q1;
		    quat[1] = UKF_Q2;
		    quat[2] = UKF_Q3;
		    quat[3] = UKF_Q4;
		    backStepResetQuat(quat);
		}

		backStepQuat(quat);
#endif  // USE_BACKSTEP_CONTROLLER == 1
		backStepPowerDistribution(throttle);
		motorsSendValues();
		motorsData.throttle = throttle;
#else

		if (p[CTRL_PID_TYPE] == 0) {
		    // pitch angle
		    pitchCommand = pidUpdate(controlData.pitchAnglePID, controlData.pitch, AQ_PITCH);
		    // rate
		    pitchCommand += pidUpdate(controlData.pitchRatePID, 0.0f, IMU_DRATEY);

		    // roll angle
		    rollCommand = pidUpdate(controlData.rollAnglePID, controlData.roll, AQ_ROLL);
		    // rate
		    rollCommand += pidUpdate(controlData.rollRatePID, 0.0f, IMU_DRATEX);
		}
		else if (p[CTRL_PID_TYPE] == 1) {
		    // pitch rate from angle
		    pitchCommand = pidUpdate(controlData.pitchRatePID, pidUpdate(controlData.pitchAnglePID, controlData.pitch, AQ_PITCH), IMU_DRATEY);

		    // roll rate from angle
		    rollCommand = pidUpdate(controlData.rollRatePID, pidUpdate(controlData.rollAnglePID, controlData.roll, AQ_ROLL), IMU_DRATEX);
		}
		else {
		    pitchCommand = 0.0f;
		    rollCommand = 0.0f;
		    ruddCommand = 0.0f;
		}

		// seek a 0 deg difference between hold heading and actual yaw
		ruddCommand = pidUpdate(controlData.yawRatePID, pidUpdate(controlData.yawAnglePID, 0.0f, compassDifference(controlData.yaw, AQ_YAW)), IMU_DRATEZ);

		rollCommand = constrainFloat(rollCommand, -p[CTRL_MAX], p[CTRL_MAX]);
		pitchCommand = constrainFloat(pitchCommand, -p[CTRL_MAX], p[CTRL_MAX]);
		ruddCommand = constrainFloat(ruddCommand, -p[CTRL_MAX], p[CTRL_MAX]);

		motorsCommands(throttle, pitchCommand, rollCommand, ruddCommand);
#endif

	    }
	    // no throttle input
	    else {
		supervisorThrottleUp(0);

		motorsOff();
	    }
	}
	// not armed
	else {
	    motorsOff();
	}

	controlData.lastUpdate = IMU_LASTUPD;
	controlData.loops++;
    }
}

void controlInit(void) {
    int i;

    AQ_NOTICE("Control init... ");

#ifdef USE_BACKSTEP_CONTROLLER
    backStepInit();
#endif

    for (i = 0; i < 3; i++) {
	utilFilterInit(&controlData.userPitchFilter[i], 1.0f/400.0f, 0.1f, 0.0f);
	utilFilterInit(&controlData.userRollFilter[i], 1.0f/400.0f, 0.1f, 0.0f);
	utilFilterInit(&controlData.navPitchFilter[i], 1.0f/400.0f, 0.125f, 0.0f);
	utilFilterInit(&controlData.navRollFilter[i], 1.0f/400.0f, 0.125f, 0.0f);
    }

    controlData.pitchRatePID = pidInit(&p[CTRL_TLT_RTE_P], &p[CTRL_TLT_RTE_I], &p[CTRL_TLT_RTE_D], &p[CTRL_TLT_RTE_F], &p[CTRL_TLT_RTE_PM], &p[CTRL_TLT_RTE_IM], &p[CTRL_TLT_RTE_DM], &p[CTRL_TLT_RTE_OM], 0, 0, 0, 0);
    controlData.rollRatePID = pidInit(&p[CTRL_TLT_RTE_P], &p[CTRL_TLT_RTE_I], &p[CTRL_TLT_RTE_D], &p[CTRL_TLT_RTE_F], &p[CTRL_TLT_RTE_PM], &p[CTRL_TLT_RTE_IM], &p[CTRL_TLT_RTE_DM], &p[CTRL_TLT_RTE_OM], 0, 0, 0, 0);
    controlData.yawRatePID = pidInit(&p[CTRL_YAW_RTE_P], &p[CTRL_YAW_RTE_I], &p[CTRL_YAW_RTE_D], &p[CTRL_YAW_RTE_F], &p[CTRL_YAW_RTE_PM], &p[CTRL_YAW_RTE_IM], &p[CTRL_YAW_RTE_DM], &p[CTRL_YAW_RTE_OM], 0, 0, 0, 0);

    controlData.pitchAnglePID = pidInit(&p[CTRL_TLT_ANG_P], &p[CTRL_TLT_ANG_I], &p[CTRL_TLT_ANG_D], &p[CTRL_TLT_ANG_F], &p[CTRL_TLT_ANG_PM], &p[CTRL_TLT_ANG_IM], &p[CTRL_TLT_ANG_DM], &p[CTRL_TLT_ANG_OM], 0, 0, 0, 0);
    controlData.rollAnglePID = pidInit(&p[CTRL_TLT_ANG_P], &p[CTRL_TLT_ANG_I], &p[CTRL_TLT_ANG_D], &p[CTRL_TLT_ANG_F], &p[CTRL_TLT_ANG_PM], &p[CTRL_TLT_ANG_IM], &p[CTRL_TLT_ANG_DM], &p[CTRL_TLT_ANG_OM], 0, 0, 0, 0);
    controlData.yawAnglePID = pidInit(&p[CTRL_YAW_ANG_P], &p[CTRL_YAW_ANG_I], &p[CTRL_YAW_ANG_D], &p[CTRL_YAW_ANG_F], &p[CTRL_YAW_ANG_PM], &p[CTRL_YAW_ANG_IM], &p[CTRL_YAW_ANG_DM], &p[CTRL_YAW_ANG_OM], 0, 0, 0, 0);

    controlData.controlTask = CoCreateTask(controlTaskCode, (void *)0, 5, &controlTaskStack[TASK_STACK_SIZE-1], TASK_STACK_SIZE);
}
