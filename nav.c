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

#include "nav.h"
#include "nav_ukf.h"
#include "aq.h"
#include "util.h"
#include "comm.h"
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
#ifdef USE_SIGNALING
   #include "signaling.h"
#endif
#include <CoOS.h>
#include <string.h>
#include <math.h>
#include <stdio.h>
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

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
    if (p[NAV_CEILING])
	navData.ceilingAlt = (UKF_ALTITUDE + p[NAV_CEILING]);    // home altitude + x meter as ceiling

    AQ_NOTICE("Home position set\n");
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

// set reference frame angle for heading-free mode
void navSetHfReference(uint8_t refType) {
    if (refType == 0) {
	// use current heading as reference
	navData.hfReferenceCos = navUkfData.yawCos;
	navData.hfReferenceSin = navUkfData.yawSin;
	navData.hfUseStoredReference = 1;
    } else if (navData.fixType) {
	// use current bearing from home position
	float homeBearing = navCalcBearing(navData.homeLeg.targetLat, navData.homeLeg.targetLon, gpsData.lat, gpsData.lon);
	if (fabsf(homeBearing - navData.bearingToHome) > NAV_HF_HOME_BRG_D_MAX) {
	    navData.hfReferenceCos = cosf(homeBearing);
	    navData.hfReferenceSin = sinf(homeBearing);
	    navData.bearingToHome = homeBearing;
	}
	navData.hfUseStoredReference = 1;
    }
}

// set headfree mode based on radio command
void navSetHeadFreeMode(void) {
    int16_t hfChan = radioData.channels[(int)(p[NAV_HDFRE_CHAN]-1)];

    // HF switch to set/dynamic position
    // when disarmed one can also set the orientation heading in this position (for 2-pos control)
    if (hfChan > 250 || (hfChan < -250 && !(supervisorData.state & STATE_ARMED))) {
	if (navData.headFreeMode != NAV_HEADFREE_DYNAMIC) {
	    if (navData.headFreeMode != NAV_HEADFREE_SETTING && navData.headFreeMode != NAV_HEADFREE_DYN_DELAY) {
		navData.headFreeMode = NAV_HEADFREE_SETTING;
		navData.hfDynamicModeTimer = timerMicros();
		AQ_NOTICE("Set new reference heading for Heading-Free mode\n");
#ifdef USE_SIGNALING
		signalingOnetimeEvent(SIG_EVENT_OT_HF_SET);
#endif
	    }
	    else if ((supervisorData.state & STATE_ARMED) && timerMicros() - navData.hfDynamicModeTimer > NAV_HF_DYNAMIC_DELAY) {
		navData.headFreeMode = NAV_HEADFREE_DYNAMIC;
		AQ_NOTICE("Now in dynamic Heading-Free DVH mode\n");
	    }
	    else if (navData.headFreeMode == NAV_HEADFREE_SETTING)
		navData.headFreeMode = NAV_HEADFREE_DYN_DELAY;
	}

    }
    // HF switch to locked/on position
    else if (hfChan < -250) {
	if (navData.headFreeMode != NAV_HEADFREE_LOCKED) {
	    AQ_NOTICE("Now in locked Heading-Free DVH mode\n");
	    navData.headFreeMode = NAV_HEADFREE_LOCKED;
	}
    }
    // HF channel at off position
    else {
	if (navData.headFreeMode > NAV_HEADFREE_DYN_DELAY)
	    AQ_NOTICE("Now in normal DVH mode\n");
	navData.headFreeMode = NAV_HEADFREE_OFF;
    }
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
	if (navData.missionLegs[leg].targetLat != (double)0.0 && navData.missionLegs[leg].targetLon != (double)0.0)
	    navUkfSetGlobalPositionTarget(navData.missionLegs[leg].targetLat, navData.missionLegs[leg].targetLon);
	navData.targetHeading = navData.missionLegs[leg].poiHeading;
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_ORBIT) {
	if (navData.missionLegs[leg].targetLat != (double)0.0 && navData.missionLegs[leg].targetLon != (double)0.0)
	    navUkfSetGlobalPositionTarget(navData.missionLegs[leg].targetLat, navData.missionLegs[leg].targetLon);
	navData.targetHeading = navData.missionLegs[leg].poiHeading;
	navData.holdMaxHorizSpeed = p[NAV_MAX_SPEED];
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_TAKEOFF) {
	// store this position as the takeoff position
	navUkfSetGlobalPositionTarget(gpsData.lat, gpsData.lon);
	navData.targetHeading = AQ_YAW;

	if (navData.missionLegs[leg].maxVertSpeed == 0.0f)
	    navData.holdMaxVertSpeed = p[NAV_LANDING_VEL];
	else
	    navData.holdMaxVertSpeed = navData.missionLegs[leg].maxVertSpeed;

	// set the launch location as home
	navSetHomeCurrent();
	navData.homeLeg.targetAlt = navData.holdAlt;
	navData.homeLeg.poiHeading = -0.0f;		// relative
    }
    else if (navData.missionLegs[leg].type == NAV_LEG_LAND) {
	if (navData.missionLegs[leg].maxVertSpeed == 0.0f)
	    navData.holdMaxVertSpeed = p[NAV_LANDING_VEL];
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
    if (supervisorData.state > STATE_DISARMED && navData.navCapable && RADIO_FLAPS > 250) {
	//  are we currently in position hold mode && do we have a clear mission ahead of us?
	if (navData.mode == NAV_STATUS_POSHOLD && leg < NAV_MAX_MISSION_LEGS && navData.missionLegs[leg].type > 0) {
	    navLoadLeg(leg);
	    navData.mode = NAV_STATUS_MISSION;
	}
    }
    // do we want to be in position hold mode?
    else if (supervisorData.state > STATE_DISARMED && RADIO_FLAPS > -250) {
	// always allow alt hold
	if (navData.mode < NAV_STATUS_ALTHOLD) {
	    // record this altitude as the hold altitude
	    navSetHoldAlt(UKF_ALTITUDE, 0);

	    // set integral to current RC throttle setting
	    pidZeroIntegral(navData.altSpeedPID, -UKF_VELD, motorsData.throttle);
	    pidZeroIntegral(navData.altPosPID, UKF_ALTITUDE, 0.0f);

	    navData.holdSpeedAlt = navData.targetHoldSpeedAlt = -UKF_VELD;
	    navData.holdMaxVertSpeed = p[NAV_ALT_POS_OM];
	    navData.mode = NAV_STATUS_ALTHOLD;

            // notify ground
            AQ_NOTICE("Altitude Hold engaged\n");
	}

	// are we not in position hold mode now?
	if (navData.navCapable && navData.mode != NAV_STATUS_POSHOLD && navData.mode != NAV_STATUS_DVH) {
	    // store this position as hold position
	    navUkfSetGlobalPositionTarget(gpsData.lat, gpsData.lon);

	    // set this position as home if we have none
	    if (navData.homeLeg.targetLat == (double)0.0 || navData.homeLeg.targetLon == (double)0.0)
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

	    // notify ground
            AQ_NOTICE("Position Hold engaged\n");
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

    // Ceiling
    if (navData.ceilingAlt) { // ceiling set ?, 0 is disable
	if ( UKF_ALTITUDE > navData.ceilingAlt && !navData.setCeilingFlag ) { // ceiling reached 1st time
	    navData.setCeilingFlag = 1;
	    navData.ceilingTimer = timerMicros();
	} else if ( navData.setCeilingFlag && UKF_ALTITUDE > navData.ceilingAlt && (timerMicros() - navData.ceilingTimer) > (1e6 * 5) ) { // ceiling still reached for 5 seconds
	    navData.ceilingTimer = timerMicros();
	    if (!navData.setCeilingReached)
		AQ_NOTICE("Warning: Altitude ceiling reached.");
	    navData.setCeilingReached = 1;
	} else if ( (navData.setCeilingFlag || navData.setCeilingReached) && UKF_ALTITUDE <= navData.ceilingAlt ) {
	    if (navData.setCeilingReached)
		AQ_NOTICE("Notice: Altitude returned to within ceiling.");
	    navData.setCeilingFlag = 0;
	    navData.setCeilingReached = 0;
	}
    }

    // home set
    if (supervisorData.state > STATE_DISARMED && RADIO_AUX2 > 250) {
	if (!navData.homeActionFlag) {
	    navSetHomeCurrent();
	    navData.homeActionFlag = 1;
	}
    }
    // recall home
    else if (supervisorData.state > STATE_DISARMED && RADIO_AUX2 < -250) {
	if (!navData.homeActionFlag) {
	    navRecallHome();
	    AQ_NOTICE("Returning to home position\n");
	    navData.homeActionFlag = 1;
	}
    }
    // switch to middle, clear action flag
    else {
	navData.homeActionFlag = 0;
    }

    // heading-free mode
    if ((int)p[NAV_HDFRE_CHAN] > 0 && (int)p[NAV_HDFRE_CHAN] <= RADIO_MAX_CHANNELS) {

	navSetHeadFreeMode();

	// set/maintain headfree frame reference
	if (!navData.homeActionFlag && ( navData.headFreeMode == NAV_HEADFREE_SETTING ||
		(navData.headFreeMode == NAV_HEADFREE_DYNAMIC && navData.mode == NAV_STATUS_DVH) )) {
	    uint8_t dfRefTyp = 0;
	    if ((supervisorData.state & STATE_FLYING) && navData.homeLeg.targetLat != (double)0.0f && navData.homeLeg.targetLon != (double)0.0f) {
		if (NAV_HF_HOME_DIST_D_MIN && NAV_HF_HOME_DIST_FREQ && (currentTime - navData.homeDistanceLastUpdate) > (AQ_US_PER_SEC / NAV_HF_HOME_DIST_FREQ)) {
		    navData.distanceToHome = navCalcDistance(gpsData.lat, gpsData.lon, navData.homeLeg.targetLat, navData.homeLeg.targetLon);
		    navData.homeDistanceLastUpdate = currentTime;
		}
		if (!NAV_HF_HOME_DIST_D_MIN || navData.distanceToHome > NAV_HF_HOME_DIST_D_MIN)
		    dfRefTyp = 1;
	    }
	    navSetHfReference(dfRefTyp);
	}
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

	// do we want to ignore rotation of craft (headfree/carefree mode)?
	if (navData.headFreeMode > NAV_HEADFREE_OFF) {
	    if (navData.hfUseStoredReference) {
		// rotate to stored frame
		navData.holdSpeedN = x * navData.hfReferenceCos - y * navData.hfReferenceSin;
		navData.holdSpeedE = y * navData.hfReferenceCos + x * navData.hfReferenceSin;
	    }
	    else {
		// don't rotate to any frame (pitch/roll move to N/S/E/W)
		navData.holdSpeedN = x;
		navData.holdSpeedE = y;
	    }
	}
	else {
	// rotate to earth frame
	navData.holdSpeedN = x * navUkfData.yawCos - y * navUkfData.yawSin;
	navData.holdSpeedE = y * navUkfData.yawCos + x * navUkfData.yawSin;
	}
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
	if ((vertStick > p[CTRL_DBAND_THRO]  && !navData.setCeilingReached)  || vertStick < -p[CTRL_DBAND_THRO]) {
	    // altitude velocity proportional to throttle stick
	    navData.targetHoldSpeedAlt = (vertStick - p[CTRL_DBAND_THRO] * (vertStick > 0.0f ? 1.0f : -1.0f)) * p[NAV_ALT_POS_OM] * (1.0f / 700.0f);

	    navData.verticalOverride = 1;
	}
	// are we trying to land?
	else if (navData.mode == NAV_STATUS_MISSION && navData.missionLegs[leg].type == NAV_LEG_LAND) {
	    navData.targetHoldSpeedAlt = -navData.holdMaxVertSpeed;
	}
	// coming out of vertical override?
	else if (navData.verticalOverride) {
	    navData.targetHoldSpeedAlt = 0.0f;

	    // slow down before trying to hold altitude
	    if (fabsf(UKF_VELD) < 0.025f)
		navData.verticalOverride = 0;

	    // set new hold altitude to wherever we are while still in override
	    if (navData.mode != NAV_STATUS_MISSION)
		navSetHoldAlt(UKF_ALTITUDE, 0);
	}
	// PID has the throttle
	else {
	    navData.targetHoldSpeedAlt = pidUpdate(navData.altPosPID, navData.holdAlt, UKF_ALTITUDE);
	}

	// constrain vertical velocity
	navData.targetHoldSpeedAlt = constrainFloat(navData.targetHoldSpeedAlt, (navData.holdMaxVertSpeed < p[NAV_MAX_DECENT]) ? -navData.holdMaxVertSpeed : -p[NAV_MAX_DECENT], navData.holdMaxVertSpeed);

	// smooth vertical velocity changes
	navData.holdSpeedAlt += (navData.targetHoldSpeedAlt - navData.holdSpeedAlt) * 0.01f;
    }
    else {
	navData.verticalOverride = 0;
    }

    // calculate POI angle (used for tilt in gimbal function)
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

    if (gpsData.lat != (double)0.0 && gpsData.lon != (double)0.0) {
	if (navData.navCapable)
	    navData.fixType = 3;
	else
	    navData.fixType = 2;
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
    navData.ceilingAlt = 0.0f;
    navData.setCeilingFlag = 0;
    navData.setCeilingReached = 0;
    navData.homeActionFlag = 0;
    navData.distanceToHome = 0.0f;
    navData.headFreeMode = NAV_HEADFREE_OFF;
    navData.hfUseStoredReference = 0;

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

// input lat/lon in degrees, returns distance in meters
float navCalcDistance(double lat1, double lon1, double lat2, double lon2) {
    float n = (lat1 - lat2) * navUkfData.r1;
    float e = (lon1 - lon2) * navUkfData.r2;
    return __sqrtf(n*n + e*e);
}

// input lat/lon in degrees, returns bearing in radians
float navCalcBearing(double lat1, double lon1, double lat2, double lon2) {
    float n = (float)((lat2 - lat1) * (double)DEG_TO_RAD * navUkfData.r1);
    float e = (float)((lon2 - lon1) * (double)DEG_TO_RAD * navUkfData.r2);
    float ret = atan2f(e, n);

    if (!isfinite(ret))
	ret = 0.0f;

    return ret;
}
