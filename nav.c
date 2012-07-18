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
#include "supervisor.h"
#include "aq_mavlink.h"
#include <CoOS.h>
#include <string.h>
#include <math.h>
#include <intrinsics.h>
#include <stdio.h>

navStruct_t navData __attribute__((section(".ccm")));

void navResetHoldAlt(float delta) {
    navData.holdAlt += delta;
}

void navSetHoldAlt(float alt, uint8_t relative) {
    if (relative)
	navData.holdAlt += alt;
    else
	navData.holdAlt = alt;
}

void navSetHoldHeading(float targetHeading) {
    // signbit() returns true if negative sign
    if (signbit(targetHeading))
	// use targetHeading as relative to bearing to target
	navData.holdHeading = compassNormalize(navData.holdCourse + targetHeading);
    else
	// use targetHeading as absolute heading
	navData.holdHeading = targetHeading;
}

void navSetHomeCurrent(void) {
    navData.homeLeg.type = NAV_LEG_GOTO;
    navData.homeLeg.relativeAlt = 0;
    navData.homeLeg.targetAlt = UKF_ALTITUDE;
    navData.homeLeg.targetLat = gpsData.lat;
    navData.homeLeg.targetLon = gpsData.lon;
    navData.homeLeg.maxHorizSpeed = p[NAV_MAX_SPEED];
    navData.homeLeg.poiHeading = AQ_YAW;
}

void navRecallHome(void) {
    navUkfSetGlobalPositionTarget(navData.homeLeg.targetLat, navData.homeLeg.targetLon);
    navSetHoldAlt(navData.homeLeg.targetAlt, navData.homeLeg.relativeAlt);
    navData.holdMaxHorizSpeed = navData.homeLeg.maxHorizSpeed;
    navData.holdMaxVertSpeed = navData.homeLeg.maxVertSpeed;
    navSetHoldHeading(navData.homeLeg.poiHeading);

    if (navData.holdMaxHorizSpeed == 0.0f)
	navData.holdMaxHorizSpeed = p[NAV_MAX_SPEED];
    if (navData.holdMaxVertSpeed == 0.0f)
	navData.holdMaxVertSpeed = p[NAV_ALT_POS_OM];
}

void navLoadLeg(uint8_t leg) {
    // invalid type?
    if (!navData.missionLegs[leg].type || navData.missionLegs[leg].type >= NAV_NUM_LEG_TYPES)
	return;

    // common
    if (navData.missionLegs[leg].relativeAlt)
	navSetHoldAlt(navData.missionLegs[leg].targetAlt, navData.missionLegs[leg].relativeAlt);
    else
	navSetHoldAlt(navData.missionLegs[leg].targetAlt - navUkfData.presAltOffset, navData.missionLegs[leg].relativeAlt);

    navData.holdMaxHorizSpeed = navData.missionLegs[leg].maxHorizSpeed;
    navData.holdMaxVertSpeed = navData.missionLegs[leg].maxVertSpeed;

    // type specific
    if (navData.missionLegs[leg].type == NAV_LEG_HOME) {
	navSetHoldAlt(navData.homeLeg.targetAlt, navData.homeLeg.relativeAlt);
	navUkfSetGlobalPositionTarget(navData.homeLeg.targetLat, navData.homeLeg.targetLon);
	navData.targetHeading = navData.homeLeg.poiHeading;

	navData.holdMaxHorizSpeed = navData.homeLeg.maxHorizSpeed;
	navData.holdMaxVertSpeed = navData.homeLeg.maxVertSpeed;
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_GOTO) {
	if (navData.missionLegs[leg].targetLat != 0.0 && navData.missionLegs[leg].targetLon != 0.0)
	    navUkfSetGlobalPositionTarget(navData.missionLegs[leg].targetLat, navData.missionLegs[leg].targetLon);
	navData.targetHeading = navData.missionLegs[leg].poiHeading;
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_ORBIT) {
	if (navData.missionLegs[leg].targetLat != 0.0 && navData.missionLegs[leg].targetLon != 0.0)
	    navUkfSetGlobalPositionTarget(navData.missionLegs[leg].targetLat, navData.missionLegs[leg].targetLon);
	navData.targetHeading = navData.missionLegs[leg].poiHeading;
	navData.holdMaxHorizSpeed = p[NAV_MAX_SPEED];
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_TAKEOFF) {
	// store this position as the takeoff position
	navUkfSetGlobalPositionTarget(gpsData.lat, gpsData.lon);
	navData.targetHeading = AQ_YAW;

	if (navData.missionLegs[leg].maxVertSpeed == 0.0f)
	    navData.holdMaxVertSpeed = NAV_LANDING_VEL;
	else
	    navData.holdMaxVertSpeed = navData.missionLegs[leg].maxVertSpeed;

	// set the launch location as home
	navSetHomeCurrent();
	navData.homeLeg.targetAlt = navData.holdAlt;
	navData.homeLeg.poiHeading = -0.0f;		// relative
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_LAND) {
	if (navData.missionLegs[leg].maxVertSpeed == 0.0f)
	    navData.holdMaxVertSpeed = NAV_LANDING_VEL;
	else
	    navData.holdMaxVertSpeed = navData.missionLegs[leg].maxVertSpeed;
    }

    if (navData.holdMaxHorizSpeed == 0.0f)
	navData.holdMaxHorizSpeed = p[NAV_MAX_SPEED];
    if (navData.holdMaxVertSpeed == 0.0f)
	navData.holdMaxVertSpeed = p[NAV_ALT_POS_OM];

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
	navData.navCapable = 1;
    }
    // do not drop out of mission due to (hopefully) temporary GPS degradation
    else if (navData.mode < NAV_STATUS_POSHOLD) {
	// Cannot Navigate
	navData.navCapable = 0;
    }

    // Can we navigate && do we want to be in mission mode?
    if (navData.navCapable && RADIO_FLAPS > 250) {
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
	    navSetHoldAlt(UKF_ALTITUDE, 0);

	    // set integral to current RC throttle setting
	    pidZeroIntegral(navData.altSpeedPID, -UKF_VELD, motorsData.throttle);
	    pidZeroIntegral(navData.altPosPID, UKF_ALTITUDE, 0.0f);

	    navData.mode = NAV_STATUS_ALTHOLD;
	    navData.holdSpeedAlt = navData.targetHoldSpeedAlt = -UKF_VELD;
	}

	// are we not in position hold mode now?
	if (navData.navCapable && navData.mode != NAV_STATUS_POSHOLD && navData.mode != NAV_STATUS_DVH) {
	    // store this position as hold position
	    navUkfSetGlobalPositionTarget(gpsData.lat, gpsData.lon);

	    // set this position as home if we have none
	    if (navData.homeLeg.targetLat == 0.0 || navData.homeLeg.targetLon == 0.0)
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
	    }

	    navData.holdMaxHorizSpeed = p[NAV_MAX_SPEED];
	    navData.holdMaxVertSpeed = p[NAV_ALT_POS_OM];

	    // activate pos hold
	    navData.mode = NAV_STATUS_POSHOLD;
	}
	// DVH
	else if (navData.navCapable && (
	    RADIO_PITCH > p[CTRL_DEAD_BAND] ||
	    RADIO_PITCH < -p[CTRL_DEAD_BAND] ||
	    RADIO_ROLL > p[CTRL_DEAD_BAND] ||
	    RADIO_ROLL < -p[CTRL_DEAD_BAND])) {
		    navData.mode = NAV_STATUS_DVH;
	}
	else if (navData.navCapable && navData.mode == NAV_STATUS_DVH) {
	    // allow speed to drop before holding position (or if RTH engaged)
	    if ((UKF_VELN < +0.1f && UKF_VELN > -0.1f && UKF_VELE < +0.1f && UKF_VELE > -0.1f) || RADIO_AUX2 < -250) {
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
	navSetHoldAlt(UKF_ALTITUDE, 0);
    }

    // home set
    if (RADIO_AUX2 > 250) {
	navSetHomeCurrent();
    }
    // recall home
    else if (RADIO_AUX2 < -250) {
	navRecallHome();
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
	    // goto/home test
	    if (((navData.missionLegs[leg].type == NAV_LEG_GOTO || navData.missionLegs[leg].type == NAV_LEG_HOME) &&
		navData.holdDistance < navData.missionLegs[leg].targetRadius &&
		fabsf(navData.holdAlt - UKF_ALTITUDE) < navData.missionLegs[leg].targetRadius) ||
	    // orbit test
		(navData.missionLegs[leg].type == NAV_LEG_ORBIT &&
		fabsf(navData.holdDistance - navData.missionLegs[leg].targetRadius) +
		fabsf(navData.holdAlt - UKF_ALTITUDE) < 2.0f)  ||
	    // takeoff test
		(navData.missionLegs[leg].type == NAV_LEG_TAKEOFF &&
		navData.holdDistance < navData.missionLegs[leg].targetRadius &&
		fabsf(navData.holdAlt - UKF_ALTITUDE) < navData.missionLegs[leg].targetRadius)
		) {
		    // start the loiter clock
		    navData.loiterCompleteTime = currentTime + navData.missionLegs[leg].loiterTime;
#ifdef USE_MAVLINK
		    // notify ground
		    mavlinkWpReached(leg);
#endif
	    }
	}
	// have we loitered long enough?
	else if (currentTime > navData.loiterCompleteTime && navData.missionLegs[leg].type != NAV_LEG_LAND) {
	    // next leg
	    if (++leg < NAV_MAX_MISSION_LEGS && navData.missionLegs[leg].type > 0) {
		navLoadLeg(leg);
	    }
	    else {
		navData.mode = NAV_STATUS_POSHOLD;
	    }
	}
    }

    // DVH
    if (navData.mode == NAV_STATUS_DVH) {
	float factor = (1.0f / 500.0f) * navData.holdMaxHorizSpeed;
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
    // orbit POI
    else if (navData.mode == NAV_STATUS_MISSION && navData.missionLegs[leg].type == NAV_LEG_ORBIT) {
	float velX, velY;

	// maintain orbital radius
	velX = -pidUpdate(navData.distanceNPID, navData.missionLegs[leg].targetRadius, navData.holdDistance);

	// maintain orbital velocity (clock wise)
	velY = -navData.missionLegs[leg].maxHorizSpeed;

	// rotate to earth frame
	navData.holdSpeedN = velX * navUkfData.yawCos - velY * navUkfData.yawSin;
	navData.holdSpeedE = velY * navUkfData.yawCos + velX * navUkfData.yawSin;
    }
    else {
	// distance => velocity
	navData.holdSpeedN = pidUpdate(navData.distanceNPID, 0.0f, UKF_POSN);
	navData.holdSpeedE = pidUpdate(navData.distanceEPID, 0.0f, UKF_POSE);
    }

    // normalize N/E speed requests to fit below max nav speed
    tmp = __sqrtf(navData.holdSpeedN*navData.holdSpeedN + navData.holdSpeedE*navData.holdSpeedE);
    if (tmp > navData.holdMaxHorizSpeed) {
	navData.holdSpeedN = (navData.holdSpeedN / tmp) * navData.holdMaxHorizSpeed;
	navData.holdSpeedE = (navData.holdSpeedE / tmp) * navData.holdMaxHorizSpeed;
    }

    // velocity => tilt
    navData.holdTiltN = -pidUpdate(navData.speedNPID, navData.holdSpeedN, UKF_VELN);
    navData.holdTiltE = +pidUpdate(navData.speedEPID, navData.holdSpeedE, UKF_VELE);

    if (navData.mode > NAV_STATUS_MANUAL) {
	float vertStick;

	// Throttle controls vertical speed
	vertStick = RADIO_THROT - 700;
	if (vertStick > p[CTRL_DEAD_BAND] || vertStick < -p[CTRL_DEAD_BAND]) {
	    // altitude velocity proportional to throttle stick
	    if (vertStick > 0.0f)
		navData.targetHoldSpeedAlt = (vertStick - p[CTRL_DEAD_BAND]) * p[NAV_ALT_POS_OM] * (1.0f / 700.0f);
	    else
		navData.targetHoldSpeedAlt = (vertStick + p[CTRL_DEAD_BAND]) * p[NAV_MAX_DECENT] * (1.0f / 700.0f);

	    // set new hold altitude to wherever we are during vertical speed overrides
	    if (navData.mode != NAV_STATUS_MISSION)
		navSetHoldAlt(UKF_ALTITUDE, 0);
	}
	// are we trying to land?
	else if (navData.mode == NAV_STATUS_MISSION && navData.missionLegs[leg].type == NAV_LEG_LAND) {
	    navData.targetHoldSpeedAlt = -navData.holdMaxVertSpeed;
	}
	else {
	    navData.targetHoldSpeedAlt = pidUpdate(navData.altPosPID, navData.holdAlt, UKF_ALTITUDE);
	}

	// constrain vertical velocity
	navData.targetHoldSpeedAlt = constrainFloat(navData.targetHoldSpeedAlt, (navData.holdMaxVertSpeed < p[NAV_MAX_DECENT]) ? -navData.holdMaxVertSpeed : -p[NAV_MAX_DECENT], navData.holdMaxVertSpeed);

	// smooth vertical velocity changes
	navData.holdSpeedAlt += (navData.targetHoldSpeedAlt - navData.holdSpeedAlt) * 0.01f;
    }

    // calculate POI angle
    if (navData.mode == NAV_STATUS_MISSION && navData.missionLegs[leg].poiAltitude != 0.0f) {
	float a, b, c;

	a = navData.holdDistance;
	b = UKF_ALTITUDE - navData.missionLegs[leg].poiAltitude;
	c = __sqrtf(a*a + b*b);

	navData.poiAngle = asinf(a/c) * RAD_TO_DEG - 90.0f;
    }
    else {
	navData.poiAngle = 0.0f;
    }

    if (navData.mode == NAV_STATUS_MISSION) {
	// recalculate autonomous heading
	navSetHoldHeading(navData.targetHeading);

	// watch for bump if landing
	if (navData.missionLegs[leg].type == NAV_LEG_LAND && IMU_ACCZ < NAV_LANDING_DECEL)
	    // shut everything down (sure hope we are really on the ground :)
	    supervisorDisarm();
    }

    navData.lastUpdate = currentTime;
}

void navInit(void) {
    int i = 0;

    AQ_NOTICE("Nav init\n");

    memset((void *)&navData, 0, sizeof(navData));

    navData.speedNPID = pidInit(&p[NAV_SPEED_P], &p[NAV_SPEED_I], 0, 0, &p[NAV_SPEED_PM], &p[NAV_SPEED_IM], 0, &p[NAV_SPEED_OM], 0, 0, 0, 0);
    navData.speedEPID = pidInit(&p[NAV_SPEED_P], &p[NAV_SPEED_I], 0, 0, &p[NAV_SPEED_PM], &p[NAV_SPEED_IM], 0, &p[NAV_SPEED_OM], 0, 0, 0, 0);
    navData.distanceNPID = pidInit(&p[NAV_DIST_P], &p[NAV_DIST_I], 0, 0, &p[NAV_DIST_PM], &p[NAV_DIST_IM], 0, &p[NAV_DIST_OM], 0, 0, 0, 0);
    navData.distanceEPID = pidInit(&p[NAV_DIST_P], &p[NAV_DIST_I], 0, 0, &p[NAV_DIST_PM], &p[NAV_DIST_IM], 0, &p[NAV_DIST_OM], 0, 0, 0, 0);
    navData.altSpeedPID = pidInit(&p[NAV_ATL_SPED_P], &p[NAV_ATL_SPED_I], 0, 0, &p[NAV_ATL_SPED_PM], &p[NAV_ATL_SPED_IM], 0, &p[NAV_ATL_SPED_OM], 0, 0, 0, 0);
    navData.altPosPID = pidInit(&p[NAV_ALT_POS_P], &p[NAV_ALT_POS_I], 0, 0, &p[NAV_ALT_POS_PM], &p[NAV_ALT_POS_IM], 0, &p[NAV_ALT_POS_OM], 0, 0, 0, 0);

    navData.mode = NAV_STATUS_MANUAL;
    navSetHoldHeading(AQ_YAW);
    navSetHoldAlt(UKF_ALTITUDE, 0);

    // HOME
    navData.missionLegs[i].type = NAV_LEG_HOME;
    navData.missionLegs[i].targetRadius = 0.10f;
    navData.missionLegs[i].loiterTime = (uint32_t)0.0e6f;
    navData.missionLegs[i].poiAltitude = UKF_ALTITUDE;
    i++;

    // land
    navData.missionLegs[i].type = NAV_LEG_LAND;
    navData.missionLegs[i].maxHorizSpeed = 1.0f;
    navData.missionLegs[i].poiAltitude = 0.0f;
    i++;
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
    return &navData.homeLeg;
}
