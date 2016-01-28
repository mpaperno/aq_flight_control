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

#include "aq.h"
#include "aq_init.h"
#include "aq_timer.h"
#include "rtc.h"
#include "filer.h"
#include "config.h"
#include "serial.h"
#include "comm.h"
#include "telemetry.h"
#include "command.h"
#include "motors.h"
#include "radio.h"
#include "imu.h"
#include "control.h"
#include "gps.h"
#include "nav.h"
#include "logger.h"
#include "alt_ukf.h"
#include "nav_ukf.h"
#include "aq_mavlink.h"
#include "util.h"
#include "supervisor.h"
#include "gimbal.h"
#include "run.h"
#include "sdio.h"
#include "can.h"
#include "analog.h"
#include "canCalib.h"
#ifdef USE_SIGNALING
   #include "signaling.h"
#endif
#include <CoOS.h>

digitalPin *tp;

OS_STK *aqInitStack;

void aqInit(void *pdata) {
#ifdef EEPROM_CS_PORT
    // TODO: clean this up
    digitalPin *eepromCS = digitalInit(EEPROM_CS_PORT, EEPROM_CS_PIN, 1);
#endif
#ifdef DAC_TP_PORT
    tp = digitalInit(DAC_TP_PORT, DAC_TP_PIN, 0);
#endif
    rtcInit();	    // have to do this first as it requires our microsecond timer to calibrate
    timerInit();    // now setup the microsecond timer before everything else
    sdioLowLevelInit();
    filerInit();
    supervisorInit();
    configInit();
    commInit();
#ifdef USE_MAVLINK
    mavlinkInit();
#endif
    telemetryInit();
    imuInit();
    analogInit();
    navUkfInit();
    altUkfInit();
    radioInit();
    gpsInit();
    navInit();
    commandInit();
#ifdef USE_SIGNALING
    signalingInit();
#endif
    loggerInit();
#ifdef CAN_CALIB
    canCalibInit();
#else
    canInit();
#endif
    motorsInit();
    controlInit();
    gimbalInit();
    runInit();

    info();

    supervisorInitComplete();

    // allow tasks to startup
    yield(10);

    AQ_NOTICE("Initialization complete, READY.\n");

    // startup complete, reduce comm task priority
    if (commData.commTask)
	CoSetPriority(commData.commTask, COMM_PRIORITY);

    // start telemetry
    telemetryEnable();

    // reset idle loop calibration now that everything is running
    minCycles = 999999999;

    CoExitTask();
}
