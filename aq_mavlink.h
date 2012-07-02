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

#ifndef _aq_mavlink_h
#define _aq_mavlink_h

#include "aq.h"
#ifdef USE_MAVLINK
#include "serial.h"
#include "digital.h"
#include "config.h"
#include "../mavlink_types.h"

#define MAVLINK_HEARTBEAT_INTERVAL	    1e6	    //  1Hz
#define MAVLINK_PARAM_INTERVAL		    2e4	    // 50Hz
#define MAVLINK_WP_TIMEOUT		    1e6	    // 1 second
#define MAVLINK_LED_PORT		    GPIOD
#define MAVLINK_LED_PIN			    GPIO_Pin_11
#define MAVLINK_USART			    USART1
#define MAVLINK_NOTICE_DEPTH		    20
#define MAVLINK_PARAMID_LEN		    16
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS

typedef struct {
    OS_TID sendTask;
    OS_TID recvTask;
    OS_MutexID serialPortMutex;
    OS_EventID notices;
    void *noticeQueue[MAVLINK_NOTICE_DEPTH];

    serialPort_t *serialPort;
    unsigned long nextHeartbeat;
    unsigned long nextParam;
    unsigned int currentParam;

    // TODO: pull actual count from header files
    unsigned long streamInterval[13];
    unsigned long streamNext[13];

    // this is a temporary implementation until we adopt mavlink completely
    int numParams;

    uint16_t packetDrops;
    uint16_t idlePercent;
    uint8_t mode;
    uint8_t nav_mode;
    uint8_t status;
    uint8_t wpTargetSysId;
    uint8_t wpTargetCompId;
    uint8_t wpCount;
    uint8_t wpCurrent;
    uint32_t wpNext;

    unsigned long lastCounter;
} mavlinkStruct_t;

extern mavlinkStruct_t mavlinkData;
extern mavlink_system_t mavlink_system;

extern void mavlinkInit(void);
extern void mavlinkNotice(const char *s);
extern void mavlinkWpReached(uint16_t seqId);
extern void mavlinkWpAnnounceCurrent(uint16_t seqId);
extern void comm_send_ch(mavlink_channel_t chan, uint8_t ch);
extern void mavlinkDo(void);

#endif
#endif	// USE_MAVLINK
