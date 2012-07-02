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

#include "nav.h"
#include "nav_ukf.h"
#include "aq.h"
#include "util.h"
#include "notice.h"
#include "aq_timer.h"
#include "control.h"
#include "pid.h"
#include "config.h"
#include "radio.h"
#include "gps.h"
#include "compass.h"
#include "imu.h"
#include "nav_ukf.h"
#include "motors.h"
#include "run.h"
#include "aq_mavlink.h"
#include <CoOS.h>
#include <string.h>
#include <math.h>
#include <intrinsics.h>
#include <stdio.h>

navStruct_t navData;

void navResetHoldAlt(float delta) {
    navData.holdAlt += delta;
}

void navSetHoldAlt(float alt) {
    navData.holdAlt = alt;
}

void navSetHomeCurrent(void) {
    navData.homeLegs[0].type = NAV_LEG_GOTO;
    navData.homeLegs[0].targetAlt = UKF_ALTITUDE;
    navData.homeLegs[0].targetLat = gpsData.lat;
    navData.homeLegs[0].targetLon = gpsData.lon;
    navData.homeLegs[0].maxSpeed = p[NAV_MAX_SPEED];
    navData.homeLegs[0].poiHeading = AQ_YAW;
}

void navLoadLeg(unsigned char leg) {
    if (navData.missionLegs[leg].type == NAV_LEG_HOME) {
	navSetHoldAlt(navData.homeLegs[0].targetAlt);
	navUkfSetGlobalPositionTarget(navData.homeLegs[0].targetLat, navData.homeLegs[0].targetLon);
	navData.holdHeading = navData.homeLegs[0].poiHeading;
	navData.holdMaxSpeed = navData.homeLegs[0].maxSpeed;
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_GOTO) {
	navSetHoldAlt(navData.missionLegs[leg].targetAlt - navUkfData.presAltOffset);
	navUkfSetGlobalPositionTarget(navData.missionLegs[leg].targetLat, navData.missionLegs[leg].targetLon);
	navData.holdHeading = navData.missionLegs[leg].poiHeading;
	navData.holdMaxSpeed = navData.missionLegs[leg].maxSpeed;
    }
    // invalid type
    else
	return;

    if (navData.holdMaxSpeed == 0.0f)
	navData.holdMaxSpeed = p[NAV_MAX_SPEED];

    navData.loiterCompleteTime = 0;

    navData.missionLeg = leg;

#ifdef USE_MAVLINK
    // notify ground
    mavlinkWpAnnounceCurrent(leg);
#endif
}

void navNavigate(void) {
    unsigned long currentTime;
    unsigned char leg = navData.missionLeg;
    float tmp;

    currentTime = IMU_LASTUPD;

    // de-activate GPS LED
    digitalLo(gpsData.gpsLed);

    // do we have a sufficient, recent fix?
    if ((currentTime - gpsData.lastPosUpdate) < NAV_MAX_FIX_AGE && (gpsData.hAcc/* * runData.accMask*/) < NAV_MIN_GPS_ACC) {
	// activate GPS LED
	digitalHi(gpsData.gpsLed);
	navData.navCapibal = 1;
    }
    // do not drop out of mission due to (hopefully) temporary GPS degradation
    else if (navData.mode < NAV_STATUS_POSHOLD) {
	// Cannot Navigate
	navData.navCapibal = 0;
    }

    // Can we navigate && do we want to be in mission mode?
    if (navData.navCapibal && RADIO_FLAPS > 250) {
	//  are we currently in position hold mode && do we have a clear mission ahead of us?
	if (navData.mode == NAV_STATUS_POSHOLD && leg < NAV_MAX_MISSION_LEGS && navData.missionLegs[leg].type > 0) {
	    navLoadLeg(leg);
	    navData.mode = NAV_STATUS_MISSION;
	}
    }
    // do we want to be in position hold mode?
    else if (RADIO_FLAPS > -250) {
	// always allow alt hold
	if (navData.mode < NAV_STATUS_ALTHOLD) {
	    // record this altitude as the hold altitude
	    navSetHoldAlt(UKF_ALTITUDE);

	    // set integral to current RC throttle setting
	    pidZeroIntegral(navData.altSpeedPID, -UKF_VELD, RADIO_THROT * p[CTRL_FACT_THRO]);
	    pidZeroIntegral(navData.altPosPID, UKF_ALTITUDE, 0.0f);

	    navData.mode = NAV_STATUS_ALTHOLD;
	}

	// are we not in position hold mode now?
	if (navData.navCapibal && navData.mode != NAV_STATUS_POSHOLD && navData.mode != NAV_STATUS_DVH) {
	    // store this position as hold position
	    navUkfSetGlobalPositionTarget(gpsData.lat, gpsData.lon);

	    // set this position as home if we have none
	    if (navData.homeLegs[0].targetLat == 0.0 || navData.homeLegs[0].targetLon == 0.0)
		navSetHomeCurrent();

	    // only zero bias if coming from lower mode
	    if (navData.mode < NAV_STATUS_POSHOLD) {
		navData.holdTiltN = 0.0f;
		navData.holdTiltE = 0.0f;

		// speed
		pidZeroIntegral(navData.speedNPID, UKF_VELN, 0.0f);
		pidZeroIntegral(navData.speedEPID, UKF_VELE, 0.0f);

		// pos
		pidZeroIntegral(navData.distanceNPID, UKF_POSN, 0.0f);
		pidZeroIntegral(navData.distanceEPID, UKF_POSE, 0.0f);

		navData.holdMaxSpeed = p[NAV_MAX_SPEED];
	    }

	    // activate pos hold
	    navData.mode = NAV_STATUS_POSHOLD;
	}
	// DVH
	else if (navData.navCapibal && (
	    RADIO_PITCH > p[CTRL_DEAD_BAND] ||
	    RADIO_PITCH < -p[CTRL_DEAD_BAND] ||
	    RADIO_ROLL > p[CTRL_DEAD_BAND] ||
	    RADIO_ROLL < -p[CTRL_DEAD_BAND])) {
		    navData.mode = NAV_STATUS_DVH;
	}
	else if (navData.navCapibal && navData.mode == NAV_STATUS_DVH) {
	    // allow speed to drop before holding position
	    if (UKF_VELN < +0.1f && UKF_VELN > -0.1f && UKF_VELE < +0.1f && UKF_VELE > -0.1f) {
		navUkfSetGlobalPositionTarget(gpsData.lat, gpsData.lon);

		navData.mode = NAV_STATUS_POSHOLD;
	    }
	}
    }
    else {
	// switch to manual mode
	navData.mode = NAV_STATUS_MANUAL;
	// reset mission legs
	navData.missionLeg = leg = 0;
	// keep up with changing altitude
	navSetHoldAlt(UKF_ALTITUDE);
    }

    // home set
    if (RADIO_AUX2 > 250) {
	navSetHomeCurrent();
    }
    // recall home
    else if (RADIO_AUX2 < -250) {
	navUkfSetGlobalPositionTarget(navData.homeLegs[0].targetLat, navData.homeLegs[0].targetLon);
	navSetHoldAlt(navData.homeLegs[0].targetAlt);
	navData.holdMaxSpeed = navData.homeLegs[0].maxSpeed;
	navData.holdHeading = navData.homeLegs[0].poiHeading;

	if (navData.holdMaxSpeed > p[NAV_MAX_SPEED] || navData.holdMaxSpeed == 0.0f)
	    navData.holdMaxSpeed = p[NAV_MAX_SPEED];
    }

    if (UKF_POSN != 0.0f || UKF_POSE != 0.0f) {
	navData.holdCourse = compassNormalize(atan2f(-UKF_POSE, -UKF_POSN) * RAD_TO_DEG);
	navData.holdDistance = __sqrtf(UKF_POSN*UKF_POSN + UKF_POSE*UKF_POSE);
    }
    else {
	navData.holdCourse = 0.0f;
	navData.holdDistance = 0.0f;
    }

    if (navData.mode == NAV_STATUS_MISSION) {
	// have we arrived yet?
	if (navData.loiterCompleteTime == 0) {
	    // are we close enough (distance and altitude)?
	    if (navData.holdDistance < navData.missionLegs[leg].targetRadius && fabsf(navData.holdAlt - UKF_ALTITUDE) < navData.missionLegs[leg].targetRadius) {
		navData.loiterCompleteTime = currentTime + navData.missionLegs[leg].loiterTime;
#ifdef USE_MAVLINK
		// notify ground
		mavlinkWpReached(leg);
#endif
	    }
	}
	// have we loitered long enough?
	else if (currentTime > navData.loiterCompleteTime) {
	    leg++;
	    // next leg
	    if (leg < NAV_MAX_MISSION_LEGS && navData.missionLegs[leg].type > 0) {
		navLoadLeg(leg);
	    }
	    else {
		navData.mode = NAV_STATUS_POSHOLD;
	    }
	}
    }

    // DVH
    if (navData.mode == NAV_STATUS_DVH) {
	float factor = (1.0f / 500.0f) * navData.holdMaxSpeed;
	float x = 0.0f;
	float y = 0.0f;

	if (RADIO_PITCH > p[CTRL_DEAD_BAND])
	    x = -(RADIO_PITCH - p[CTRL_DEAD_BAND]) * factor;
	if (RADIO_PITCH < -p[CTRL_DEAD_BAND])
	    x = -(RADIO_PITCH + p[CTRL_DEAD_BAND]) * factor;
	if (RADIO_ROLL > p[CTRL_DEAD_BAND])
	    y = +(RADIO_ROLL - p[CTRL_DEAD_BAND]) * factor;
	if (RADIO_ROLL < -p[CTRL_DEAD_BAND])
	    y = +(RADIO_ROLL + p[CTRL_DEAD_BAND]) * factor;

	// rotate to earth frame
	navData.holdSpeedN = x * navUkfData.yawCos - y * navUkfData.yawSin;
	navData.holdSpeedE = y * navUkfData.yawCos + x * navUkfData.yawSin;
    }
    else {
	// distance => speed
	navData.holdSpeedN = pidUpdate(navData.distanceNPID, 0.0f, UKF_POSN);
	navData.holdSpeedE = pidUpdate(navData.distanceEPID, 0.0f, UKF_POSE);
    }

    // normalize N/E speed requests to fit below max nav speed
    tmp = __sqrtf(navData.holdSpeedN*navData.holdSpeedN + navData.holdSpeedE*navData.holdSpeedE);
    if (tmp > navData.holdMaxSpeed) {
	navData.holdSpeedN = (navData.holdSpeedN / tmp) * navData.holdMaxSpeed;
	navData.holdSpeedE = (navData.holdSpeedE / tmp) * navData.holdMaxSpeed;
    }

    // speed => tilt
    navData.holdTiltN = -pidUpdate(navData.speedNPID, navData.holdSpeedN, UKF_VELN);
    navData.holdTiltE = +pidUpdate(navData.speedEPID, navData.holdSpeedE, UKF_VELE);

    if (navData.mode > NAV_STATUS_MANUAL) {
	float vertStick = RADIO_THROT - 700;

	// Throttle controls vertical speed
	if (vertStick > p[CTRL_DEAD_BAND] || vertStick < -p[CTRL_DEAD_BAND]) {
	    // altitude speed proportional to throttle stick
	    navData.targetHoldSpeedAlt = vertStick * (1.0f / 700.0f) * p[NAV_ALT_POS_OM];

	    // limit descent to 1 m/s
	    navData.targetHoldSpeedAlt = constrainFloat(navData.targetHoldSpeedAlt, -1.0f, +2.0f);
	    navData.holdSpeedAlt = navData.targetHoldSpeedAlt;

	    // set new hold altitude to wherever we are during vertical speed overrides
	    if (navData.mode != NAV_STATUS_MISSION)
		navSetHoldAlt(UKF_ALTITUDE);
	}
	else {
	    navData.targetHoldSpeedAlt = pidUpdate(navData.altPosPID, navData.holdAlt, UKF_ALTITUDE);

	    // limit descent to 1 m/s
	    navData.targetHoldSpeedAlt = constrainFloat(navData.targetHoldSpeedAlt, -1.0f, 999.0f);
	    navData.holdSpeedAlt += (navData.targetHoldSpeedAlt - navData.holdSpeedAlt) * 0.005f;
	}
    }

    navData.lastUpdate = currentTime;
}

void navInit(void) {
    AQ_NOTICE("Nav init... ");

    navData.speedNPID = pidInit(&p[NAV_SPEED_P], &p[NAV_SPEED_I], 0, 0, &p[NAV_SPEED_PM], &p[NAV_SPEED_IM], 0, &p[NAV_SPEED_OM], 0, 0, 0, 0);
    navData.speedEPID = pidInit(&p[NAV_SPEED_P], &p[NAV_SPEED_I], 0, 0, &p[NAV_SPEED_PM], &p[NAV_SPEED_IM], 0, &p[NAV_SPEED_OM], 0, 0, 0, 0);
    navData.distanceNPID = pidInit(&p[NAV_DIST_P], &p[NAV_DIST_I], 0, 0, &p[NAV_DIST_PM], &p[NAV_DIST_IM], 0, &p[NAV_DIST_OM], 0, 0, 0, 0);
    navData.distanceEPID = pidInit(&p[NAV_DIST_P], &p[NAV_DIST_I], 0, 0, &p[NAV_DIST_PM], &p[NAV_DIST_IM], 0, &p[NAV_DIST_OM], 0, 0, 0, 0);
    navData.altSpeedPID = pidInit(&p[NAV_ATL_SPED_P], &p[NAV_ATL_SPED_I], 0, 0, &p[NAV_ATL_SPED_PM], &p[NAV_ATL_SPED_IM], 0, &p[NAV_ATL_SPED_OM], 0, 0, 0, 0);
    navData.altPosPID = pidInit(&p[NAV_ALT_POS_P], &p[NAV_ALT_POS_I], 0, 0, &p[NAV_ALT_POS_PM], &p[NAV_ALT_POS_IM], 0, &p[NAV_ALT_POS_OM], 0, 0, 0, 0);

    navData.mode = NAV_STATUS_MANUAL;
    navData.holdHeading = AQ_YAW;
    navData.holdAlt = UKF_ALTITUDE;

    // HOME
    navData.missionLegs[0].type = NAV_LEG_HOME;
}

unsigned int navGetWaypointCount(void) {
    int i;

    for (i = 0; i < NAV_MAX_MISSION_LEGS; i++)
	if (navData.missionLegs[i].type == 0)
	    break;

    return i;
}

unsigned char navClearWaypoints(void) {
    unsigned char ack = 0;
    int i;

    if (navData.mode != NAV_STATUS_MISSION) {
	for (i = 0; i < NAV_MAX_MISSION_LEGS; i++)
	    navData.missionLegs[i].type = 0;
	ack = 1;
    }

    return ack;
}

navMission_t *navGetWaypoint(int seqId) {
    return &navData.missionLegs[seqId];
}

navMission_t *navGetHomeWaypoint(void) {
    return &navData.homeLegs[0];
}
