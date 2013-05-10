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

#ifndef _comm_h
#define _comm_h

#include "serial.h"

#define COMM_NUM_PORTS		3

//#define COMM_DISABLE_FLOW_CONTROL1
//#define COMM_DISABLE_FLOW_CONTROL2
//#define COMM_DISABLE_FLOW_CONTROL3

#define COMM_RX_BUF_SIZE1	256
#define COMM_TX_BUF_SIZE1	1024

#define COMM_RX_BUF_SIZE2	256
#define COMM_TX_BUF_SIZE2	1024

#define COMM_RX_BUF_SIZE3	256
#define COMM_TX_BUF_SIZE3	1024

#define COMM_MAX_PROTOCOLS	2

enum commModes {
    COMM_UNUSED = 0,
    COMM_SINGLEPLEX,
    COMM_MULTIPLEX
};

enum commStreamTypes {
    COMM_TYPE_MAVLINK,
    COMM_TYPE_TELEMETRY,
    COMM_TYPE_UBLOX,
    COMM_TYPE_FILEIO,
    COMM_TYPE_CLI,
    COMM_TYPE_OMAP_CONSOLE,
    COMM_TYPE_OMAP_PPP
};

typedef void commNoticeCallback_t(const char *s);
typedef void commTelemCallback_t(void);

typedef struct {
    serialPort_t *serialPorts[COMM_NUM_PORTS];
    uint8_t portMode[COMM_NUM_PORTS];
    commNoticeCallback_t *noticeFuncs[COMM_MAX_PROTOCOLS];
    commTelemCallback_t *telemFuncs[COMM_MAX_PROTOCOLS];
} commStruct_t;

extern commStruct_t commData;

extern void commInit(void);
extern serialPort_t *commAcquirePort(int port);
extern void commRegisterNoticeFunc(commNoticeCallback_t *func);
extern void commRegisterTelemFunc(commTelemCallback_t *func);
extern void commSendNotice(const char *s);
extern void commDoTelem(void);

#endif