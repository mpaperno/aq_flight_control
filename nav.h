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

#ifndef _nav_h
#define _nav_h

#include <CoOS.h>
#include "pid.h"

#define NAV_MIN_GPS_ACC		3.0f					    // minimum gps hAcc needed to enter auto nav modes, in meters
#define NAV_MAX_GPS_AGE		1e6					    // maximum age of position update needed to enter auto nav modes, in microseconds
#define NAV_MIN_FIX_ACC		4.0f					    // minimum gps hAcc still considered a valid "2D" fix, in meters
#define NAV_MAX_FIX_AGE		10e6					    // maximum age of position update still considered a valid "2D" fix, in microseconds

#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters
#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))

#define NAV_HF_HOME_DIST_D_MIN	2.0f						// do not compute dynamic bearing when closer than this to home position (zero to never compute)
#define NAV_HF_HOME_DIST_FREQ	4						// update distance to home at this Hz, should be > 0 and <= 400
#define NAV_HF_HOME_BRG_D_MAX	1.0f * DEG_TO_RAD				// re-compute headfree reference angles when bearing to home changes by this many degrees (zero to always re-compute)
#define NAV_HF_DYNAMIC_DELAY	((int)3e6f)					// delay micros before entering dynamic mode after switch it toggled high

enum navStatusTypes {
    NAV_STATUS_MANUAL = 0,              // full manual control
    NAV_STATUS_ALTHOLD,                 // altitude hold only
    NAV_STATUS_POSHOLD,                 // altitude & position hold
    NAV_STATUS_DVH,                     // dynamic velocity hold cut through
    NAV_STATUS_MISSION,                 // autonomous mission
};

enum navLegTypes {
    NAV_LEG_HOME = 1,
    NAV_LEG_TAKEOFF,
    NAV_LEG_GOTO,
    NAV_LEG_ORBIT,
    NAV_LEG_LAND,
    NAV_NUM_LEG_TYPES
};

enum headFreeActiveModes {
    NAV_HEADFREE_OFF = 0,               // not active
    NAV_HEADFREE_SETTING,               // active, setting reference point
    NAV_HEADFREE_DYN_DELAY,             // active, waiting for timer to enable dynamic mode
    NAV_HEADFREE_LOCKED,                // active with locked frame reference
    NAV_HEADFREE_DYNAMIC                // active with continually adjusting frame reference
};

#define NAV_MAX_MISSION_LEGS	25

typedef struct {
    double targetLat;
    double targetLon;
    float targetAlt;			// either relative or absolute - if absolute, GPS altitude is used
    float targetRadius;			// achievement threshold for GOTO or orbit radius for ORBIT
    uint32_t loiterTime;		// us
    float maxHorizSpeed;		// m/s
    float maxVertSpeed;			// m/s
    float poiHeading;			// POI heading (>= 0 is absolute, < 0 is relative to target bearing)
    float poiAltitude;			// altitude of POI - used for camera tilting
    uint8_t relativeAlt;		// 0 == absolute, 1 == relative
    uint8_t type;
} navMission_t;

typedef struct {
    float poiAngle;			// pitch angle for gimbal to center POI
    float holdAlt;			// altitude to hold
    float targetHeading;		// navigation heading target (>= 0 is absolute, < 0 is relative to target bearing)
    float holdHeading;			// heading to hold
    float holdCourse;			// course to hold position
    float holdDistance;			// distance to hold position (straight line)
    float holdMaxHorizSpeed;		// maximum N/E speed allowed to achieve position
    float holdMaxVertSpeed;		// maximum UP/DOWN speed allowed to achieve altitude
    float holdSpeedN;			// required speed (North/South)
    float holdSpeedE;			// required speed (East/West)
    float holdTiltN;			// required tilt (North/South)
    float holdTiltE;			// required tilt (East/West)
    float holdSpeedAlt;			// required speed (Up/Down)
    float targetHoldSpeedAlt;
    float presAltOffset;

    pidStruct_t *speedNPID;		// PID to control N/S speed - output tilt in degrees
    pidStruct_t *speedEPID;		// PID to control E/W speed - output tilt in degrees
    pidStruct_t *distanceNPID;		// PID to control N/S distance - output speed in m/s
    pidStruct_t *distanceEPID;		// PID to control E/W distance - output speed in m/s
    pidStruct_t *altSpeedPID;		// PID to control U/D speed - output speed in m/s
    pidStruct_t *altPosPID;		// PID to control U/D distance - output error in meters

    navMission_t missionLegs[NAV_MAX_MISSION_LEGS];
    navMission_t homeLeg;
    uint32_t loiterCompleteTime;

    uint32_t lastUpdate;

    uint8_t mode;			// navigation mode
    uint8_t navCapable;
    uint8_t missionLeg;
    uint8_t fixType;                    // GPS fix type, 0 = no fix, 2 = 2D, 3 = 3D (navCapable)
    uint8_t homeActionFlag;		// flag to avoid repeating set/recall home actions until switch is moved back to midpoint
    float distanceToHome;		// current distance to home position in m, if set (only updated when in headfree mode)
    float bearingToHome;		// current bearing to home position in rad, if set (only updated when in headfree mode)
    uint32_t homeDistanceLastUpdate;	// timestamp of last home position update (only updated when in headfree mode)

    uint8_t headFreeMode;		// headfree/carefree mode status
    uint32_t hfDynamicModeTimer;	// track how long switch is active before entering dynamic HF mode
    float hfReferenceCos;		// stored reference heading for HF mode
    float hfReferenceSin;
    uint8_t hfUseStoredReference;	// set to true to use stored reference in HF mode instead of N/E

    uint8_t setCeilingFlag;
    uint8_t setCeilingReached;
    uint8_t ceilingTimer;
    uint8_t verticalOverride;
    float ceilingAlt;
} navStruct_t;

extern navStruct_t navData;

extern void navInit(void);
extern void navAccelUpdate(void);
extern void navGpsUpdate(void);
extern float navGetVel(char direction);
extern float navGetPos(char direction);
extern unsigned int navGetWaypointCount(void);
extern unsigned char navClearWaypoints(void);
extern navMission_t *navGetWaypoint(int seqId);
extern navMission_t *navGetHomeWaypoint(void);
extern void navSetHomeCurrent(void);
extern navMission_t *navLoadLeg(unsigned char leg);
extern void navNavigate(void);
extern void navResetHoldAlt(float delta);
extern float navCalcDistance(double lat1, double lon1, double lat2, double lon2);
extern float navCalcBearing(double lat1, double lon1, double lat2, double lon2);
extern void navPressureAdjust(float altitude);

#endif
