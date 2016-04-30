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
    Copyright 2013-2016 Maxim Paperno
*/

#include "supervisor.h"

#include "analog.h"
#include "aq_mavlink.h"
#include "aq_timer.h"
#include "calib.h"
#include "canSensors.h"
#include "comm.h"
#include "config.h"
#include "control.h"
#include "d_imu.h"
#include "gps.h"
#include "motors.h"
#include "nav_ukf.h"
#include "nav.h"
#include "radio.h"
#include "rc.h"
#include "run.h"
#include "signaling.h"
#include "util.h"

#include <stdlib.h>
#include <string.h>

supervisorStruct_t supervisorData __attribute__((section(".ccm")));

OS_STK *supervisorTaskStack;

static void supervisorSetSystemStatus(void) {

    // set the base status (these are mutually exclusive)
    if (supervisorData.state & STATE_FLYING)
	supervisorData.systemStatus = SPVR_AQ_STATUS_ACTIVE;
    else if (supervisorData.state & STATE_ARMED)
	supervisorData.systemStatus = SPVR_AQ_STATUS_STANDBY;
    else if (supervisorData.state & STATE_CALIBRATION)
	supervisorData.systemStatus =  SPVR_AQ_STATUS_CALIBRATING;
    else
	supervisorData.systemStatus = SPVR_AQ_STATUS_READY;

    // rest of the status flags are cumulative

    if ((supervisorData.state & STATE_RADIO_LOSS1))
	supervisorData.systemStatus |= SPVR_AQ_STATUS_NO_RC;

    if ((supervisorData.state & STATE_RADIO_LOSS2))
	supervisorData.systemStatus |= SPVR_AQ_STATUS_FAILSAFE;

    if (supervisorData.state & STATE_LOW_BATTERY2)
	supervisorData.systemStatus |=  SPVR_AQ_STATUS_FUEL_CRITICAL;
    else if (supervisorData.state & STATE_LOW_BATTERY1)
	supervisorData.systemStatus |=  SPVR_AQ_STATUS_FUEL_LOW;

    if (supervisorData.state & STATE_SIM_ENABLED)
	supervisorData.systemStatus |= SPVR_AQ_STATUS_SIM;

    if (navData.ceilingAlt) {
	supervisorData.systemStatus |= SPVR_AQ_STATUS_CEILING;
	if (navData.setCeilingReached)
	    supervisorData.systemStatus |= SPVR_AQ_STATUS_CEILING_REACHED;
    }

    if (controlData.controlMode == CTRL_MODE_LTD_RATE)
	supervisorData.systemStatus |= SPVR_AQ_STATUS_LTD_RATE_MODE;
    else if (controlData.controlMode == CTRL_MODE_RATE)
	supervisorData.systemStatus |= SPVR_AQ_STATUS_FUL_RATE_MODE;

    if (navData.headFreeMode == NAV_HEADFREE_DYNAMIC)
	supervisorData.systemStatus |= SPVR_AQ_STATUS_HF_DYNAMIC;
    else if (navData.headFreeMode == NAV_HEADFREE_LOCKED)
	supervisorData.systemStatus |= SPVR_AQ_STATUS_HF_LOCKED;

    if (navData.mode <= NAV_STATUS_MANUAL)
	return;

    switch(navData.mode) {
	case NAV_STATUS_ALTHOLD:
	    supervisorData.systemStatus |= SPVR_AQ_STATUS_ALTHOLD;
	    break;

	case NAV_STATUS_POSHOLD:
	    supervisorData.systemStatus |= SPVR_AQ_STATUS_ALTHOLD | SPVR_AQ_STATUS_POSHOLD;
	    break;

	case NAV_STATUS_DVH:
	    supervisorData.systemStatus |= SPVR_AQ_STATUS_ALTHOLD | SPVR_AQ_STATUS_POSHOLD | SPVR_AQ_STATUS_DVH;
	    break;

	case NAV_STATUS_MISSION:
	    supervisorData.systemStatus |= SPVR_AQ_STATUS_MISSION;
	    break;
    }

    if (navData.verticalOverride)
	supervisorData.systemStatus |= SPVR_AQ_STATUS_DAO;

    // FIXME: we need a better indicator of whether we're actually RingTH
    if (rcIsSwitchActive(NAV_CTRL_HOM_GO))
	supervisorData.systemStatus |= SPVR_AQ_STATUS_RTH;

}

void supervisorArm(void) {
    uint8_t rcStat = rcCheckValidController();
    if (rcStat) {
	AQ_NOTICE("Error: Can't arm due to RC error(s):\n");
	rcReportAllErrors(rcStat);
    }
    else if (rcIsSwitchActive(NAV_CTRL_AH) || rcIsSwitchActive(NAV_CTRL_PH) || rcIsSwitchActive(NAV_CTRL_MISN))
	AQ_NOTICE("Error: Can't arm, not in manual flight mode.\n");
    else if (rcIsSwitchActive(NAV_CTRL_HOM_SET) || rcIsSwitchActive(NAV_CTRL_HOM_GO))
	AQ_NOTICE("Error: Can't arm, home command active.\n");
    else if (rcIsSwitchActive(NAV_CTRL_HF_SET) /*|| rcIsSwitchActive(NAV_CTRL_HF_LOCK)*/)
	AQ_NOTICE("Error: Can't arm, heading-free mode active.\n");
    else if (motorsArm()) {
	supervisorData.state = STATE_ARMED | (supervisorData.state & (STATE_LOW_BATTERY1 | STATE_LOW_BATTERY2 | STATE_SIM_ENABLED));
	AQ_NOTICE("Armed\n");
	signalingOnetimeEvent(SIG_EVENT_OT_ARMING);
    }
    else {
	motorsDisarm();
	AQ_NOTICE("Error: Arm motors failed - disarmed.\n");
    }
}

void supervisorDisarm(void) {
    motorsDisarm();
    calibDeinit();
    signalingLEDsOff();
    supervisorData.state = STATE_DISARMED | (supervisorData.state & (STATE_LOW_BATTERY1 | STATE_LOW_BATTERY2 | STATE_SIM_ENABLED));
    AQ_NOTICE("Disarmed\n");
    signalingOnetimeEvent(SIG_EVENT_OT_DISARMING);
}

void supervisorCalibrate(void) {
    if ((supervisorData.state & STATE_ARMED) || (supervisorData.state & STATE_CALIBRATION))
	return;
    supervisorData.state |= STATE_CALIBRATION;
    AQ_NOTICE("Starting MAG calibration mode.\n");
    calibInit();
}

void supervisorTare(void) {
    if ((supervisorData.state & STATE_ARMED))
	return;
#ifdef USE_DIGITAL_IMU
    signalingLEDsOn();
    dIMUTare();
    AQ_NOTICE("Level calibration complete.\n");
    signalingLEDsOff();
#else
    AQ_NOTICE("Cannot perform AIMU tare.\n");
#endif // HAS_DIGITAL_IMU
}

void supervisorCheckPendingActionRequest() {
    uint8_t req = supervisorData.configActionRequest;
    supervisorData.configActionRequest = SPVR_ACT_REQ_NONE;
    if ((supervisorData.state & STATE_FLYING) || req == SPVR_ACT_REQ_NONE || supervisorData.stickCmdTimer)
	return;

    switch (req) {
	case SPVR_ACT_REQ_CFG_READ_FLASH:
	    configLoadParamsFromFlash();
	    break;

	case SPVR_ACT_REQ_CFG_WRITE_FLASH:
	    configSaveParamsToFlash();
	    break;

	case SPVR_ACT_REQ_CFG_READ_FILE:
	    configLoadParamsFromFile();
	    break;

	case SPVR_ACT_REQ_CFG_WRITE_FILE:
	    configSaveParamsToFile();
	    break;

	case SPVR_ACT_REQ_CFG_DEFAULTS:
	    configLoadParamsFromDefault();
	    break;

#ifdef HAS_DIGITAL_IMU
	case SPVR_ACT_REQ_DIMU_CFG_READ:
	    if (!(supervisorData.state & STATE_ARMED))
		dIMURequestCalibRead();
	    break;

	case SPVR_ACT_REQ_DIMU_CFG_WRITE:
	    if (!(supervisorData.state & STATE_ARMED))
		dIMURequestCalibWrite();
	    break;
#endif

	case SPVR_ACT_REQ_CALIB_ACC:
	    if (!(supervisorData.state & STATE_ARMED))
		supervisorTare();
	    break;

	case SPVR_ACT_REQ_CALIB_MAG:
	    if (!(supervisorData.state & STATE_ARMED) && !(supervisorData.state & STATE_CALIBRATION))
		supervisorCalibrate();
	    break;

	case SPVR_ACT_REQ_SYSTEM_RESET:
	    if (!(supervisorData.state & STATE_ARMED)) {
		AQ_NOTICE("Flight controller restarting...");
		yield(500);
		NVIC_SystemReset();
	    }
	    break;
    }
}

void supervisorTaskCode(void *unused) {
    unsigned long lastAqCounter = 0;  // used for idle time calc
    uint32_t count = 0;

    AQ_NOTICE("Supervisor task started\n");

    // wait for ADC vIn data
    while (analogData.batCellCount == 0)
	yield(100);

    supervisorData.vInLPF = analogData.vIn;

    if (analogData.extAmp > 0.0f)
	supervisorData.currentSenseValPtr = &analogData.extAmp;
    else if (canSensorsData.values[CAN_SENSORS_PDB_BATA] > 0.0f)
	supervisorData.currentSenseValPtr = &canSensorsData.values[CAN_SENSORS_PDB_BATA];
    else
	supervisorData.aOutLPF = SUPERVISOR_INVALID_AMPSOUT_VALUE;

    if (supervisorData.currentSenseValPtr)
	supervisorData.aOutLPF = *supervisorData.currentSenseValPtr;

    while (1) {
	yield(1000/SUPERVISOR_RATE);

	// check for stick commands
	if ((supervisorData.state & STATE_DISARMED) && !(supervisorData.state & STATE_CALIBRATION)) {

	    // Attempt to arm if throttle down and yaw full right for 2sec.
	    if (RADIO_VALID && RADIO_THROT < p[CTRL_MIN_THROT] && RADIO_RUDD > +500) {
		if (!supervisorData.armTime) {
		    supervisorData.armTime = timerMicros();
		}
		else if ((timerMicros() - supervisorData.armTime) > SUPERVISOR_DISARM_TIME) {
		    supervisorArm();
		    supervisorData.armTime = 0;
		}
	    }
	    else {
		supervisorData.armTime = 0;
	    }

	    // various functions while disarmed
	    if (RADIO_VALID && RADIO_THROT < p[CTRL_MIN_THROT] && RADIO_RUDD < -500) {
		if (!supervisorData.stickCmdTimer) {
		    supervisorData.stickCmdTimer = timerMicros();
		} else if ((timerMicros() - supervisorData.stickCmdTimer) > SUPERVISOR_STICK_CMD_TIME) {

		    // tare function (lower left)
		    if (RADIO_ROLL < -500 && RADIO_PITCH > +500)
			supervisorTare();
		    // calibration mode (upper left)
		    else if (RADIO_ROLL < -500 && RADIO_PITCH < -500)
			supervisorCalibrate();
		    // clear waypoints (lower right with WP Record switch active)
		    else if (RADIO_ROLL > 500 && RADIO_PITCH > 500 && rcIsSwitchActive(NAV_CTRL_WP_REC))
			navClearWaypoints();
		    // config write (upper right)
		    else if (RADIO_ROLL > +500 && RADIO_PITCH < -500) {
			signalingLEDsOn();
			configSaveParamsToFlash();
#ifdef HAS_DIGITAL_IMU
			dIMURequestCalibWrite();
#endif
			signalingLEDsOff();
		    }

		    supervisorData.stickCmdTimer = 0;
		} // end stick timer check
	    }
	    // no stick commands detected
	    else
		supervisorData.stickCmdTimer = 0;

	} // end if disarmed
	else if ((supervisorData.state & STATE_ARMED) || (supervisorData.state & STATE_CALIBRATION)) {
	    // Disarm only if in manual mode
	    if (RADIO_THROT < p[CTRL_MIN_THROT] && RADIO_RUDD < -500 && navData.mode == NAV_STATUS_MANUAL) {
		if (!supervisorData.armTime) {
		    supervisorData.armTime = timerMicros();
		}
		else if ((timerMicros() - supervisorData.armTime) > SUPERVISOR_DISARM_TIME) {
		    supervisorDisarm();
		    supervisorData.armTime = 0;
		}
	    }
	    else {
		supervisorData.armTime = 0;
	    }
	}
	// end of stick command checks

	// radio loss
	if ((supervisorData.state & STATE_FLYING) && (navData.mode < NAV_STATUS_MISSION || (supervisorData.state & STATE_RADIO_LOSS2))) {
	    // regained?
	    if (RADIO_QUALITY > 1.0f) {
		supervisorData.lastGoodRadioMicros = timerMicros();

		if (supervisorData.state & STATE_RADIO_LOSS1)
		    AQ_NOTICE("Warning: radio signal regained\n");

		navData.spvrModeOverride = 0;
		supervisorData.state &= ~(STATE_RADIO_LOSS1 | STATE_RADIO_LOSS2);
	    }
	    // loss 1
	    else if (!(supervisorData.state & STATE_RADIO_LOSS1) && !(supervisorData.state & STATE_RADIO_LOSS2) && (timerMicros() - supervisorData.lastGoodRadioMicros) > SUPERVISOR_RADIO_LOSS1) {
		supervisorData.state |= STATE_RADIO_LOSS1;
		AQ_NOTICE("Warning: Radio loss stage 1 detected\n");

		// hold position
		navData.spvrModeOverride = NAV_STATUS_POSHOLD;
		RADIO_PITCH = 0;    // center sticks
		RADIO_ROLL = 0;
		RADIO_RUDD = 0;
		RADIO_THROT = RADIO_MID_THROTTLE;  // center throttle
	    }
	    // loss 2
	    else if (!(supervisorData.state & STATE_RADIO_LOSS2) && (timerMicros() - supervisorData.lastGoodRadioMicros) > SUPERVISOR_RADIO_LOSS2) {
		supervisorData.state |= STATE_RADIO_LOSS2;
		AQ_NOTICE("Warning: Radio loss stage 2! Initiating recovery.\n");

		// only available with GPS
		if (navData.navCapable) {
		    navMission_t *wp;
		    uint8_t wpi = 0;
		    uint8_t fsOption = (uint8_t)p[SPVR_FS_RAD_ST2];

		    navClearWaypoints();
		    wp = navGetWaypoint(wpi++);

		    if (fsOption > SPVR_OPT_FS_RAD_ST2_LAND && navCalcDistance(gpsData.lat, gpsData.lon, navData.homeLeg.targetLat, navData.homeLeg.targetLon) > SUPERVISOR_HOME_POS_DETECT_RADIUS) {
			float targetAltitude;

			// ascend
			if (fsOption == SPVR_OPT_FS_RAD_ST2_ASCEND && ALTITUDE < navData.homeLeg.targetAlt + p[SPVR_FS_ADD_ALT]) {
			    // the home leg's altitude is recorded without pressure offset
			    targetAltitude = navData.homeLeg.targetAlt + p[SPVR_FS_ADD_ALT] + navData.presAltOffset;

			    wp->type = NAV_LEG_GOTO;
			    wp->relativeAlt = 0;
			    wp->targetAlt = targetAltitude;
			    wp->targetLat = gpsData.lat;
			    wp->targetLon = gpsData.lon;
			    wp->targetRadius = SUPERVISOR_HOME_ALT_DETECT_MARGIN;
			    wp->maxHorizSpeed = navData.homeLeg.maxHorizSpeed;
			    wp->maxVertSpeed = navData.homeLeg.maxVertSpeed;
			    wp->poiHeading = navData.homeLeg.poiHeading;
			    wp->loiterTime = 0;
			    wp->poiAltitude = 0.0f;

			    wp = navGetWaypoint(wpi++);
			}
			else {
			    // the greater of our current altitude or home's altitude
			    targetAltitude = ((ALTITUDE > navData.homeLeg.targetAlt) ? ALTITUDE : navData.homeLeg.targetAlt) + navData.presAltOffset;
			}

			// go home with previously determined altitude
			wp->type = NAV_LEG_GOTO;
			wp->relativeAlt = 0;
			wp->targetAlt = targetAltitude;
			wp->targetLat = navData.homeLeg.targetLat;
			wp->targetLon = navData.homeLeg.targetLon;
			wp->targetRadius = SUPERVISOR_HOME_POS_DETECT_RADIUS;
			wp->maxHorizSpeed = navData.homeLeg.maxHorizSpeed;
			wp->maxVertSpeed = navData.homeLeg.maxVertSpeed;
			wp->poiHeading = navData.homeLeg.poiHeading;
			wp->loiterTime = 0;
			wp->poiAltitude = 0.0f;

			wp = navGetWaypoint(wpi++);

			// decend to home
			wp->type = NAV_LEG_HOME;
			wp->targetRadius = SUPERVISOR_HOME_POS_DETECT_RADIUS;
			wp->loiterTime = 0;
			wp->poiAltitude = 0.0f;

			wp = navGetWaypoint(wpi++);
		    }

		    // land
		    wp->type = NAV_LEG_LAND;
		    wp->maxVertSpeed = NAV_DFLT_LND_SPEED;
		    wp->maxHorizSpeed = 0.0f;
		    wp->poiAltitude = 0.0f;

		    // go
		    navData.missionLeg = 0;
		    navData.tempMissionLoaded = 1;
		    navData.spvrModeOverride = NAV_STATUS_MISSION;
		}
		// no GPS, slow decent in PH mode
		else {
		    navData.spvrModeOverride = NAV_STATUS_POSHOLD;
		    RADIO_PITCH = 0;    // center sticks
		    RADIO_ROLL = 0;
		    RADIO_RUDD = 0;
		    RADIO_THROT = RADIO_MID_THROTTLE * 3 / 4;  // 1/4 max decent
		}
	    }
	}
	// end radio loss check

	// check for configuration or calibration requests from other threads
	// (this is done in part to centralize memory use for potentially expensive printf/scanf and I/O operations)
	supervisorCheckPendingActionRequest();

	// calculate idle time
	supervisorData.idlePercent = (counter - lastAqCounter) * minCycles * 100.0f / ((1e6f / SUPERVISOR_RATE) * rccClocks.SYSCLK_Frequency / 1e6f);
	lastAqCounter = counter;

	// smooth vIn readings
	supervisorData.vInLPF += (analogData.vIn - supervisorData.vInLPF) * (0.1f / SUPERVISOR_RATE);

	// smooth current flow readings, if any
	if (supervisorData.currentSenseValPtr)
	    supervisorData.aOutLPF += (*supervisorData.currentSenseValPtr - supervisorData.aOutLPF) * (0.1f / SUPERVISOR_RATE);

	//calculate remaining battery % based on configured low batt stg 2 level -- ASSumes 4.2v/cell maximum
	supervisorData.battRemainingPrct = (supervisorData.vInLPF - p[SPVR_LOW_BAT2] * analogData.batCellCount) / ((4.2f - p[SPVR_LOW_BAT2]) * analogData.batCellCount) * 100;

	// low battery
	if (!(supervisorData.state & STATE_LOW_BATTERY1) && supervisorData.vInLPF < (p[SPVR_LOW_BAT1]*analogData.batCellCount)) {
	    supervisorData.state |= STATE_LOW_BATTERY1;
	    AQ_NOTICE("Warning: Low battery stage 1\n");

	    // TODO: something
	}
	else if (!(supervisorData.state & STATE_LOW_BATTERY2) && supervisorData.vInLPF < (p[SPVR_LOW_BAT2]*analogData.batCellCount)) {
	    supervisorData.state |= STATE_LOW_BATTERY2;
	    AQ_NOTICE("Warning: Low battery stage 2\n");

	    // TODO: something
	}
	// end battery level checks

	supervisorSetSystemStatus();

	if (supervisorData.state & STATE_FLYING)
	    // count flight time in seconds
	    supervisorData.flightTime += (1.0f / SUPERVISOR_RATE);

	count++;

	signalingEvent(count);
    }
}

void supervisorInitComplete(void) {
    supervisorDisarm();
}

void supervisorDiskWait(uint8_t waiting) {
    supervisorData.diskWait = waiting;
}

void supervisorThrottleUp(uint8_t throttle) {
    if (throttle)
	supervisorData.state |= STATE_FLYING;
    else
	supervisorData.state &= ~STATE_FLYING;
}

void supervisorSendDataStart(void) {
    signalingDataStream();
}

void supervisorSendDataStop(void) {
    signalingDataStream();
}

void supervisorConfigRead(void) {
    supervisorData.configRead = 1;
}

void supervisorToggleSimState(bool enable) {
    if (enable) {
	supervisorData.state |= STATE_SIM_ENABLED;
	motorsData.disableAllPorts = true;  // disable all motor port output until restart
    } else
	supervisorData.state &= ~STATE_SIM_ENABLED;
}

bool supervisorRequestConfigAction(uint8_t act) {
    if (act >= SPVR_ACT_REQ_ENUM_END || supervisorData.configActionRequest > SPVR_ACT_REQ_NONE) {
	AQ_NOTICE("Error: Unknown or currently pending action.");
	return false;
    }

    if ((supervisorData.state & STATE_FLYING)) {
	AQ_NOTICE("Error: Must be landed to perform this action.");
	return false;
    }

    if ((supervisorData.state & STATE_ARMED) && act > SPVR_ACT_REQ_DISARMED_ENUM_END) {
	AQ_NOTICE("Error: Must be disarmed to perform this action.");
	return false;
    }

#ifndef HAS_DIGITAL_IMU
    if (act == SPVR_ACT_REQ_DIMU_CFG_READ || act == SPVR_ACT_REQ_DIMU_CFG_WRITE || act == SPVR_ACT_REQ_CALIB_ACC) {
	AQ_NOTICE("Error: DIMU required to perform this action.");
	return false;
    }
#endif

    supervisorData.configActionRequest = act;
    return true;
}

void supervisorInit(void) {
    memset((void *)&supervisorData, 0, sizeof(supervisorData));

    supervisorData.state = STATE_INITIALIZING;
    supervisorData.systemStatus = SPVR_AQ_STATUS_INIT;
    supervisorData.configActionRequest = SPVR_ACT_REQ_NONE;
    supervisorTaskStack = aqStackInit(SUPERVISOR_STACK_SIZE, "SUPERVISOR");

    supervisorData.supervisorTask = CoCreateTask(supervisorTaskCode, (void *)0, SUPERVISOR_PRIORITY, &supervisorTaskStack[SUPERVISOR_STACK_SIZE-1], SUPERVISOR_STACK_SIZE);
}
