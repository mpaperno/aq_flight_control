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

#ifndef _supervisor_h
#define _supervisor_h

#include <CoOS.h>
#include "aq.h"
#include "digital.h"

#define SUPERVISOR_STACK_SIZE	    240             // must be evenly divisible by 8
#define SUPERVISOR_PRIORITY	    34

#define SUPERVISOR_RATE		    20		    // Hz
#define SUPERVISOR_DISARM_TIME	    (1e6*2)	    // 2 seconds delay for arm/disarm stick commands
#define SUPERVISOR_STICK_CMD_TIME   (1e6*2)	    // 2 seconds delay for other stick commands
#define SUPERVISOR_RADIO_LOSS1	    ((uint32_t)(p[SPVR_FS_RAD_TO1] * 1e6f))	    // seconds to micros
#define SUPERVISOR_RADIO_LOSS2	    ((uint32_t)(p[SPVR_FS_RAD_TO2] * 1e6f))	    // seconds to micros

// Radio loss stage 2 failsafe settings:
#define SUPERVISOR_HOME_POS_DETECT_RADIUS	2.0f 	// start descending when within this radius of home position in meters
#define SUPERVISOR_HOME_ALT_DETECT_MARGIN	2.0f	// start moving to home lat/lon when within this altitude of home position in meters

#define SUPERVISOR_SOC_TABLE_SIZE               100
#define SUPERVISOR_INVALID_AMPSOUT_VALUE        -9999.0f // value of supervisorData.aOutLPF when current flow is unknown

enum supervisorStates {
    STATE_INITIALIZING	= 0x0000,
    STATE_CALIBRATION	= 0x0001,
    STATE_DISARMED	= 0x0002,
    STATE_ARMED		= 0x0004,
    STATE_FLYING	= 0x0008,
    STATE_RADIO_LOSS1	= 0x0010,
    STATE_RADIO_LOSS2	= 0x0020,
    STATE_LOW_BATTERY1	= 0x0040,
    STATE_LOW_BATTERY2	= 0x0080,
    STATE_SIM_ENABLED	= 0x0100
};

enum supervisorFailsafeStg2Options {
    SPVR_OPT_FS_RAD_ST2_LAND = 0,
    SPVR_OPT_FS_RAD_ST2_RTH,
    SPVR_OPT_FS_RAD_ST2_ASCEND
};

// Some telemetry streams (like Mavlink) use this as a summary of system status and flight modes.
// These bits may be used by external programs and should not be modified lightly (some are published in official Mavlnk library).
// Some status bits may be mutually exclusive, most can be concurrent. See supervisorSetSystemStatus() for details.
enum supervisorSystemStatusBits {
    SPVR_AQ_STATUS_INIT            = 0,          // System is initializing
    SPVR_AQ_STATUS_STANDBY         = 0x00000001, // b00 System is *armed* standing by, with no throttle input and no autonomous mode
    SPVR_AQ_STATUS_ACTIVE          = 0x00000002, // b01 Active/flying, assumed under full manual angle-mode control unless other mode bits are set
    SPVR_AQ_STATUS_ALTHOLD         = 0x00000004, // b02 Altitude hold engaged
    SPVR_AQ_STATUS_POSHOLD         = 0x00000008, // b03 Position hold engaged
    //SPVR_AQ_STATUS_RESRVD        = 0x00000010, // b04 reserved
    SPVR_AQ_STATUS_MISSION         = 0x00000020, // b05 Autonomous mission execution mode
    SPVR_AQ_STATUS_LTD_RATE_MODE   = 0x00000040, // b06 Manual limited rate-control mode is active
    SPVR_AQ_STATUS_FUL_RATE_MODE   = 0x00000080, // b07 Manual full rate-control "acro" mode is active

    SPVR_AQ_STATUS_READY           = 0x00000100, // b08 Ready but *not armed*
    SPVR_AQ_STATUS_CALIBRATING     = 0x00000200, // b09 Calibration mode active

    SPVR_AQ_STATUS_NO_RC           = 0x00001000, // b12 No valid control input (eg. no radio link)
    SPVR_AQ_STATUS_FUEL_LOW        = 0x00002000, // b13 Battery is low (stage 1 warning)
    SPVR_AQ_STATUS_FUEL_CRITICAL   = 0x00004000, // b14 battery is depleted (stage 2 warning)

    SPVR_AQ_STATUS_SIM             = 0x00800000, // b23 HIL/SIL Simulator mode active
    SPVR_AQ_STATUS_DVH             = 0x01000000, // b24 Dynamic Velocity Hold is active (PH with proportional manual direction override)
    SPVR_AQ_STATUS_DAO             = 0x02000000, // b25 Dynamic Altitude Override is active (AH with proportional manual adjustment)
    SPVR_AQ_STATUS_CEILING_REACHED = 0x04000000, // b26 Craft is at ceiling altitude
    SPVR_AQ_STATUS_CEILING         = 0x08000000, // b27 Ceiling altitude is set
    SPVR_AQ_STATUS_HF_DYNAMIC      = 0x10000000, // b28 Heading-Free dynamic mode active
    SPVR_AQ_STATUS_HF_LOCKED       = 0x20000000, // b29 Heading-Free locked mode active
    SPVR_AQ_STATUS_RTH             = 0x40000000, // b30 Automatic Return to Home is active
    SPVR_AQ_STATUS_FAILSAFE        = 0x80000000, // b31 System is in failsafe recovery mode
};

enum supervisorActionRequests {
    SPVR_ACT_REQ_NONE = 0,
    SPVR_ACT_REQ_CFG_READ_FLASH,
    SPVR_ACT_REQ_CFG_WRITE_FLASH,
    SPVR_ACT_REQ_CFG_READ_FILE,
    SPVR_ACT_REQ_CFG_WRITE_FILE,
    SPVR_ACT_REQ_CFG_DEFAULTS,
    SPVR_ACT_REQ_DISARMED_ENUM_END,
    SPVR_ACT_REQ_DIMU_CFG_READ,
    SPVR_ACT_REQ_DIMU_CFG_WRITE,
    SPVR_ACT_REQ_CALIB_ACC,
    SPVR_ACT_REQ_CALIB_MAG,
    SPVR_ACT_REQ_SYSTEM_RESET,
    SPVR_ACT_REQ_ENUM_END
};

typedef struct {
    float *currentSenseValPtr;

    float flightTime;		    // seconds
    float idlePercent;
    float vInLPF;
    float aOutLPF;

    uint32_t armTime;
    uint32_t stickCmdTimer;
    uint32_t lastGoodRadioMicros;
    uint32_t systemStatus;          // supervisorSystemStatusBits
    uint32_t lastIdleCounter;

    uint16_t state;
    uint8_t battRemainingPrct;
    uint8_t diskWait;
    uint8_t configActionRequest;   // one of supervisorActionRequests

    OS_TID supervisorTask;

    bool configRead;
} supervisorStruct_t __attribute__((aligned));

extern supervisorStruct_t supervisorData;

extern void supervisorInit(void);
extern void supervisorInitComplete(void);
extern void supervisorDiskWait(uint8_t waiting);
extern void supervisorThrottleUp(uint8_t throttle);
extern void supervisorSendDataStart(void);
extern void supervisorSendDataStop(void);
extern void supervisorConfigRead(void);
extern void supervisorArm(void);
extern void supervisorDisarm(void);
extern void supervisorToggleSimState(bool enable);
extern bool supervisorRequestConfigAction(uint8_t act);

#endif
