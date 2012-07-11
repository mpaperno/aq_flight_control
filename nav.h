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

#ifndef _nav_h
#define _nav_h

#include <CoOS.h>
#include "pid.h"

#define NAV_MIN_GPS_ACC		3.0f
#define NAV_MAX_FIX_AGE		((int)1e6f)				    // 1 second

#define NAV_LANDING_VEL		0.33f					    // default landing/takeoff vertical velocity
#define NAV_LANDING_DECEL	(-1.5f * GRAVITY)			    // deceleration needed to indicate a landing (1.5g)

#define NAV_EQUATORIAL_RADIUS	(6378.137 * 1000.0)			    // meters
#define NAV_FLATTENING		(1.0 / 298.257223563)			    // WGS-84
#define NAV_E_2			(NAV_FLATTENING * (2.0 - NAV_FLATTENING))

#define NAV_STATUS_MANUAL	0x00					    // full manual control
#define NAV_STATUS_ALTHOLD	0x01					    // altitude hold only
#define NAV_STATUS_POSHOLD	0x02					    // altitude & position hold
#define NAV_STATUS_DVH		0x03					    // dynamic velocity hold cut through
#define NAV_STATUS_MISSION	0x04					    // autonomous mission

enum navLegTypes {
    NAV_LEG_HOME = 1,
    NAV_LEG_TAKEOFF,
    NAV_LEG_GOTO,
    NAV_LEG_ORBIT,
    NAV_LEG_LAND,
    NAV_NUM_LEG_TYPES
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
} navStruct_t;

extern navStruct_t navData;

extern void navInit(void);
extern void navAccelUpdate(void);
extern void navGpsUpdate(void);
extern void navUpdateAlt(float altitude);
extern float navGetVel(char direction);
extern float navGetPos(char direction);
extern unsigned int navGetWaypointCount(void);
extern unsigned char navClearWaypoints(void);
extern navMission_t *navGetWaypoint(int seqId);
extern navMission_t *navGetHomeWaypoint(void);
extern void navSetHomeCurrent(void);
extern void navLoadLeg(unsigned char leg);
extern void navNavigate(void);
extern void navResetHoldAlt(float delta);

#endif
