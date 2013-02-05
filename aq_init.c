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
#include "aq_init.h"
#include "aq_timer.h"
#include "rtc.h"
#include "filer.h"
#include "config.h"
#include "serial.h"
#ifdef MAVLINK
    #include "mavlink.h"
#else
    #include "downlink.h"
    #include "notice.h"
    #include "telemetry.h"
    #include "command.h"
#endif
#include "motors.h"
#include "radio.h"
#include "imu.h"
#include "control.h"
#include "gps.h"
#include "nav.h"
#include "logger.h"
#include "run.h"
#include "nav_ukf.h"
#include "aq_mavlink.h"
#include "util.h"
#include "supervisor.h"
#include "gimbal.h"
#include "sdio.h"
#include "can.h"
#include <CoOS.h>

OS_STK *aqInitStack;

void aqInit(void *pdata) {
    rtcInit();	    // have to do this first as it requires our microsecond timer to calibrate
    timerInit();    // now setup the microsecond timer before everything else
    sdioLowLevelInit();
    filerInit();
    supervisorInit();
    configInit();
#ifdef USE_MAVLINK
    mavlinkInit();
#else
    downlinkInit();
    noticeInit();
    telemetryInit();
#endif
    motorsInit();
    gimbalInit();
    imuInit();
    navUkfInit();
    radioInit();
    controlInit();
    gpsInit();
    navInit();
#ifndef USE_MAVLINK
    commandInit();
#endif
#ifdef USE_CAN
    canInit();
#endif
    loggerInit();
    runInit();

    info();

    supervisorInitComplete();

    AQ_NOTICE("Initialization complete, READY.\n");

#ifndef USE_MAVLINK
    // startup complete, reduce notice task priority
    CoSetPriority(noticeData.noticeTask, NOTICE_PRIORITY);
#endif

// start telemetry
#ifndef USE_MAVLINK
    telemetryEnable();
#endif

    // reset idle loop calibration now that everything is running
    minCycles = 999999999;

    CoExitTask();
}
