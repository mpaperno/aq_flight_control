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
#include "alt_ukf.h"
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

// reset current sea level static pressure based on better GPS estimate
void navPressureAdjust(float altitude) {
    navData.presAltOffset = altitude - ALTITUDE;
}

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
    navData.homeLeg.targetAlt = ALTITUDE;
    navData.homeLeg.targetLat = gpsData.lat;
    navData.homeLeg.targetLon = gpsData.lon;
    navData.homeLeg.maxHorizSpeed = p[NAV_MAX_SPEED];
    navData.homeLeg.poiHeading = AQ_YAW;
    if (p[NAV_CEILING])
        navData.ceilingAlt = (ALTITUDE + p[NAV_CEILING]);    // home altitude + x meter as ceiling

    // notify ground
    AQ_NOTICE("Home position set\n");
#ifdef USE_MAVLINK
    mavlinkAnnounceHome();
#endif
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
    } else if (navData.navCapable) {
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

navMission_t *navLoadLeg(uint8_t leg) {
    navMission_t *curLeg = &navData.missionLegs[leg];

    // invalid type?
    if (!curLeg->type || curLeg->type >= NAV_NUM_LEG_TYPES)
        return &navData.missionLegs[0];

    // common
    if (curLeg->relativeAlt)
        navSetHoldAlt(curLeg->targetAlt, curLeg->relativeAlt);
    else
        navSetHoldAlt(curLeg->targetAlt - navData.presAltOffset, curLeg->relativeAlt);

    navData.holdMaxHorizSpeed = curLeg->maxHorizSpeed;
    navData.holdMaxVertSpeed = curLeg->maxVertSpeed;

    // type specific
    if (curLeg->type == NAV_LEG_HOME) {
        navSetHoldAlt(navData.homeLeg.targetAlt, navData.homeLeg.relativeAlt);
        navUkfSetGlobalPositionTarget(navData.homeLeg.targetLat, navData.homeLeg.targetLon);
        navData.targetHeading = navData.homeLeg.poiHeading;

        navData.holdMaxHorizSpeed = navData.homeLeg.maxHorizSpeed;
        navData.holdMaxVertSpeed = navData.homeLeg.maxVertSpeed;
    }
    else if (curLeg->type == NAV_LEG_GOTO) {
        if (curLeg->targetLat != (double)0.0 && curLeg->targetLon != (double)0.0)
            navUkfSetGlobalPositionTarget(curLeg->targetLat, curLeg->targetLon);
        navData.targetHeading = curLeg->poiHeading;
    }
    else if (curLeg->type == NAV_LEG_ORBIT) {
        if (curLeg->targetLat != (double)0.0 && curLeg->targetLon != (double)0.0)
            navUkfSetGlobalPositionTarget(curLeg->targetLat, curLeg->targetLon);
        navData.targetHeading = curLeg->poiHeading;
        navData.holdMaxHorizSpeed = p[NAV_MAX_SPEED];
    }
    else if (curLeg->type == NAV_LEG_TAKEOFF) {
        // store this position as the takeoff position
        navUkfSetHereAsPositionTarget();
        navData.targetHeading = AQ_YAW;

        if (curLeg->maxVertSpeed == 0.0f)
            navData.holdMaxVertSpeed = p[NAV_LANDING_VEL];
        else
            navData.holdMaxVertSpeed = curLeg->maxVertSpeed;

        // set the launch location as home
        navSetHomeCurrent();
        navData.homeLeg.targetAlt = navData.holdAlt;
        navData.homeLeg.poiHeading = -0.0f;                // relative
    }
    else if (curLeg->type == NAV_LEG_LAND) {
        if (curLeg->maxVertSpeed == 0.0f)
            navData.holdMaxVertSpeed = p[NAV_LANDING_VEL];
        else
            navData.holdMaxVertSpeed = curLeg->maxVertSpeed;
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

    return curLeg;
}

// Set fixType as a rough indication of GPS data quality
void navSetFixType(void) {
    unsigned long posTimeDlta = IMU_LASTUPD - gpsData.lastPosUpdate;
    float hAccMsk = gpsData.hAcc + runData.accMask;

    if (posTimeDlta < NAV_MAX_GPS_AGE && hAccMsk < NAV_MIN_GPS_ACC)
        navData.fixType = 3;
    else if (posTimeDlta < NAV_MAX_FIX_AGE && hAccMsk < NAV_MIN_FIX_ACC)
        navData.fixType = 2;
    else
        navData.fixType = 0;

    if (navData.fixType == 3) {
	if (!(supervisorData.state & STATE_CALIBRATION))
	    digitalHi(supervisorData.gpsLed);
    } else {
	if (!(supervisorData.state & STATE_CALIBRATION))
	    digitalLo(supervisorData.gpsLed);
    }
}

void navNavigate(void) {
    unsigned long currentTime;
    unsigned char leg = navData.missionLeg;
    navMission_t *curLeg = &navData.missionLegs[leg];
    float tmp;

    currentTime = IMU_LASTUPD;

    navSetFixType();

    // is there sufficient position accuracy to navigate?
    if (navData.fixType == 3)
        navData.navCapable = 1;
    // do not drop out of mission due to (hopefully) temporary GPS degradation
    else if (navData.mode < NAV_STATUS_POSHOLD)
        navData.navCapable = 0;

    // Can we navigate && do we want to be in mission mode?
    if ((supervisorData.state & STATE_ARMED) && navData.navCapable && RADIO_FLAPS > 250) {
        //  are we currently in position hold mode && do we have a clear mission ahead of us?
        if ((navData.mode == NAV_STATUS_POSHOLD || navData.mode == NAV_STATUS_DVH) && leg < NAV_MAX_MISSION_LEGS && curLeg->type > 0) {
            curLeg = navLoadLeg(leg);
            navData.mode = NAV_STATUS_MISSION;
        }
    }
    // do we want to be in position hold mode?
    else if ((supervisorData.state & STATE_ARMED) && RADIO_FLAPS > -250) {
        // always allow alt hold
        if (navData.mode < NAV_STATUS_ALTHOLD) {
            // record this altitude as the hold altitude
            navSetHoldAlt(ALTITUDE, 0);

            // set integral to current RC throttle setting
            pidZeroIntegral(navData.altSpeedPID, -VELOCITYD, motorsData.throttle * RADIO_MID_THROTTLE / MOTORS_SCALE);
            pidZeroIntegral(navData.altPosPID, ALTITUDE, 0.0f);

            navData.holdSpeedAlt = navData.targetHoldSpeedAlt = -VELOCITYD;
            navData.holdMaxVertSpeed = p[NAV_ALT_POS_OM];
            navData.mode = NAV_STATUS_ALTHOLD;

            // notify ground
            AQ_NOTICE("Altitude Hold engaged\n");
        }

        // are we not in position hold mode now?
        if ((navData.navCapable || navUkfData.flowQuality > 0.0f) && navData.mode != NAV_STATUS_POSHOLD && navData.mode != NAV_STATUS_DVH) {
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

            // store this position as hold position
            navUkfSetHereAsPositionTarget();

            if (navData.navCapable) {
                // set this position as home if we have none
                if (navData.homeLeg.targetLat == (double)0.0 || navData.homeLeg.targetLon == (double)0.0)
                    navSetHomeCurrent();
            }

            // activate pos hold
            navData.mode = NAV_STATUS_POSHOLD;

            navData.holdSpeedN = 0.0f;
            navData.holdSpeedE = 0.0f;

            navData.holdMaxHorizSpeed = p[NAV_MAX_SPEED];
            navData.holdMaxVertSpeed = p[NAV_ALT_POS_OM];

            // notify ground
            AQ_NOTICE("Position Hold engaged\n");
        }
        // DVH
        else if ((navData.navCapable || navUkfData.flowQuality > 0.0f) && (
            RADIO_PITCH > p[CTRL_DEAD_BAND] ||
            RADIO_PITCH < -p[CTRL_DEAD_BAND] ||
            RADIO_ROLL > p[CTRL_DEAD_BAND] ||
            RADIO_ROLL < -p[CTRL_DEAD_BAND])) {
                    navData.mode = NAV_STATUS_DVH;
        }
        else if (navData.mode == NAV_STATUS_DVH) {
            // allow speed to drop before holding position (or if RTH engaged)
            if ((UKF_VELN < +0.1f && UKF_VELN > -0.1f && UKF_VELE < +0.1f && UKF_VELE > -0.1f) || RADIO_AUX2 < -250) {
                navUkfSetHereAsPositionTarget();

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
        navSetHoldAlt(ALTITUDE, 0);
    }

    // ceiling set ?, 0 is disable
    if (navData.ceilingAlt) {
        // ceiling reached 1st time
        if (ALTITUDE > navData.ceilingAlt && !navData.setCeilingFlag) {
            navData.setCeilingFlag = 1;
            navData.ceilingTimer = timerMicros();
        }
        // ceiling still reached for 5 seconds
        else if (navData.setCeilingFlag && ALTITUDE > navData.ceilingAlt && (timerMicros() - navData.ceilingTimer) > (1e6 * 5) ) {
            navData.ceilingTimer = timerMicros();
            if (!navData.setCeilingReached)
                AQ_NOTICE("Warning: Altitude ceiling reached.");
            navData.setCeilingReached = 1;
        }
        else if ((navData.setCeilingFlag || navData.setCeilingReached) && ALTITUDE <= navData.ceilingAlt) {
            if (navData.setCeilingReached)
                AQ_NOTICE("Notice: Altitude returned to within ceiling.");
            navData.setCeilingFlag = 0;
            navData.setCeilingReached = 0;
        }
    }

    // home set
    if ((supervisorData.state & STATE_ARMED) && RADIO_AUX2 > 250) {
        if (!navData.homeActionFlag) {
            navSetHomeCurrent();
            navData.homeActionFlag = 1;
        }
    }
    // recall home
    else if ((supervisorData.state & STATE_ARMED) && RADIO_AUX2 < -250) {
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
            if (((curLeg->type == NAV_LEG_GOTO || curLeg->type == NAV_LEG_HOME) &&
                navData.holdDistance < curLeg->targetRadius &&
                fabsf(navData.holdAlt - ALTITUDE) < curLeg->targetRadius) ||
            // orbit test
                (curLeg->type == NAV_LEG_ORBIT &&
                fabsf(navData.holdDistance - curLeg->targetRadius) +
                fabsf(navData.holdAlt - ALTITUDE) < 2.0f)  ||
            // takeoff test
                (curLeg->type == NAV_LEG_TAKEOFF &&
                navData.holdDistance < curLeg->targetRadius &&
                fabsf(navData.holdAlt - ALTITUDE) < curLeg->targetRadius)
                ) {
                    // freeze heading unless orbiting
                    if (curLeg->type != NAV_LEG_ORBIT)
                        navSetHoldHeading(AQ_YAW);

                    // start the loiter clock
                    navData.loiterCompleteTime = currentTime + curLeg->loiterTime;
#ifdef USE_MAVLINK
                    // notify ground
                    mavlinkWpReached(leg);
#endif
            }
        }
        // have we loitered long enough?
        else if (currentTime > navData.loiterCompleteTime && curLeg->type != NAV_LEG_LAND) {
            // next leg
            if (++leg < NAV_MAX_MISSION_LEGS && navData.missionLegs[leg].type > 0) {
                curLeg = navLoadLeg(leg);
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
    else if (navData.mode == NAV_STATUS_MISSION && curLeg->type == NAV_LEG_ORBIT) {
        float velX, velY;

        // maintain orbital radius
        velX = -pidUpdate(navData.distanceNPID, curLeg->targetRadius, navData.holdDistance);

        // maintain orbital velocity (clock wise)
        velY = -curLeg->maxHorizSpeed;

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
        vertStick = RADIO_THROT - RADIO_MID_THROTTLE;
        if ((vertStick > p[CTRL_DBAND_THRO]  && !navData.setCeilingReached)  || vertStick < -p[CTRL_DBAND_THRO]) {
            // altitude velocity proportional to throttle stick
            navData.targetHoldSpeedAlt = (vertStick - p[CTRL_DBAND_THRO] * (vertStick > 0.0f ? 1.0f : -1.0f)) * p[NAV_ALT_POS_OM] * (1.0f / RADIO_MID_THROTTLE);

            navData.verticalOverride = 1;
        }
        // are we trying to land?
        else if (navData.mode == NAV_STATUS_MISSION && curLeg->type == NAV_LEG_LAND) {
            navData.targetHoldSpeedAlt = -navData.holdMaxVertSpeed;
        }
        // coming out of vertical override?
        else if (navData.verticalOverride) {
            navData.targetHoldSpeedAlt = 0.0f;

            // slow down before trying to hold altitude
            if (fabsf(VELOCITYD) < 0.025f)
                navData.verticalOverride = 0;

            // set new hold altitude to wherever we are while still in override
            if (navData.mode != NAV_STATUS_MISSION)
                navSetHoldAlt(ALTITUDE, 0);
        }
        // PID has the throttle
        else {
            navData.targetHoldSpeedAlt = pidUpdate(navData.altPosPID, navData.holdAlt, ALTITUDE);
        }

        // constrain vertical velocity
        navData.targetHoldSpeedAlt = constrainFloat(navData.targetHoldSpeedAlt, (navData.holdMaxVertSpeed < p[NAV_MAX_DECENT]) ? -navData.holdMaxVertSpeed : -p[NAV_MAX_DECENT], navData.holdMaxVertSpeed);

        // smooth vertical velocity changes
        navData.holdSpeedAlt += (navData.targetHoldSpeedAlt - navData.holdSpeedAlt) * 0.05f;
    }
    else {
        navData.verticalOverride = 0;
    }

    // calculate POI angle (used for tilt in gimbal function)
    if (navData.mode == NAV_STATUS_MISSION && curLeg->poiAltitude != 0.0f) {
        float a, b, c;

        a = navData.holdDistance;
        b = ALTITUDE - curLeg->poiAltitude;
        c = __sqrtf(a*a + b*b);

        navData.poiAngle = asinf(a/c) * RAD_TO_DEG - 90.0f;
    }
    else {
        navData.poiAngle = 0.0f;
    }

    if (navData.mode == NAV_STATUS_MISSION) {
        // recalculate autonomous heading
        navSetHoldHeading(navData.targetHeading);

        // wait for low throttle if landing
        if (curLeg->type == NAV_LEG_LAND && motorsData.throttle <= 1)
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
    navData.altSpeedPID = pidInit(&p[NAV_ALT_SPED_P], &p[NAV_ALT_SPED_I], 0, 0, &p[NAV_ALT_SPED_PM], &p[NAV_ALT_SPED_IM], 0, &p[NAV_ALT_SPED_OM], 0, 0, 0, 0);
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
    navSetHoldAlt(ALTITUDE, 0);

    // HOME
    navData.missionLegs[i].type = NAV_LEG_HOME;
    navData.missionLegs[i].targetRadius = 0.10f;
    navData.missionLegs[i].loiterTime = 0;
    navData.missionLegs[i].poiAltitude = ALTITUDE;
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
