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

#ifndef _comm_h
#define _comm_h

#include "serial.h"
#include <stdio.h>

#define COMM_STACK_SIZE		320   // must be evenly divisible by 8
#define COMM_PRIORITY		40
#define COMM_NOTICE_DEPTH	36    // must be power of 2
#define COMM_NOTICE_LENGTH	64

#define COMM_LOG_BUF_SIZE	512
#define COMM_LOG_FNAME		"MSG"		// comment out to disable logging

#ifdef HAS_USB
#define COMM_NUM_PORTS		8
#define COMM_USB_PORT		7
#else
#define COMM_NUM_PORTS		7
#endif
#define COMM_CAN_PORT		4

#define COMM_DISABLE_FLOW_CONTROL1
#define COMM_DISABLE_FLOW_CONTROL2
#define COMM_DISABLE_FLOW_CONTROL3
#define COMM_DISABLE_FLOW_CONTROL4

#define COMM_RX_BUF_SIZE	512			// maximum rx payload

#define COMM_STACK_DEPTH	32			// number of outstanding requests allowed (per port)
#define COMM_TX_NUM_SIZES	6

#define COMM_MAX_CONSUMERS	5

#define AQ_NOTICE		commNotice
#define AQ_PRINTF(fmt, args...)	{char *sTemp = commGetNoticeBuf(); snprintf(sTemp, COMM_NOTICE_LENGTH, fmt, args); commNotice(sTemp);}

// 8 bits maximum
enum commStreamTypes {
    COMM_STREAM_TYPE_NONE	    = 0,
    COMM_STREAM_TYPE_MULTIPLEX	    = (1<<0),	// future use for combining stream types
    COMM_STREAM_TYPE_MAVLINK	    = (1<<1),	// MAVLink telemetry format
    COMM_STREAM_TYPE_TELEMETRY	    = (1<<2),	// AQ custom telemetry format
    COMM_STREAM_TYPE_GPS	    = (1<<3),	// AQ custom format data from GPS Rx
    COMM_STREAM_TYPE_RC_RX_TELEM    = (1<<4),	// RC receiver telemetry, eg. FrSky/SmartPort
    COMM_STREAM_TYPE_HILSIM	    = (1<<5),	// HIL simulator I/O
    COMM_STREAM_TYPE_OMAP_CONSOLE   = (1<<6),   // rest are reserved for future use
    COMM_STREAM_TYPE_OMAP_PPP	    = (1<<7)
};

enum commPortTypes {
    COMM_PORT_TYPE_NONE     = 0,
    COMM_PORT_TYPE_SERIAL,
    COMM_PORT_TYPE_CAN,
    COMM_PORT_TYPE_USB
};

enum commTxBufferStatus {
    COMM_TX_BUF_FREE	    = 0,
    COMM_TX_BUF_ALLOCATED,
    COMM_TX_BUF_SENDING
};

typedef struct {
    volatile uint8_t status;				    // this packet's status
    uint8_t sync[2];					    // sync bytes for multiplex
    uint8_t ck[2];					    // packet checksum
    uint8_t seq;					    // seq id
    uint8_t type;					    // protocol type
    uint16_t len;					    // payload length
    uint8_t buf;
} __attribute__((packed)) commTxBuf_t;

#define COMM_HEADER_SIZE	8			    // TODO: hard-coded for now

typedef struct {
    commTxBuf_t *txBuf;					    // pointer to tx packet
    uint8_t *memory;					    // actual memory address to send
    uint16_t size;					    // number of bytes to send
    uint8_t port;					    // port this stack element belongs to
} commTxStack_t;

typedef struct {
    uint8_t port;
} commRcvrStruct_t;

typedef void commNoticeCallback_t(const char *s);
typedef void commTelemCallback_t(void);
typedef void commRcvrCallback_t(commRcvrStruct_t *r);

typedef struct {
    void *noticeQueue[COMM_NOTICE_DEPTH*2];
    char noticeStrings[COMM_NOTICE_DEPTH][COMM_NOTICE_LENGTH]; //(576w)

    void *portHandles[COMM_NUM_PORTS];			    // serial port handles

    commNoticeCallback_t *noticeFuncs[COMM_MAX_CONSUMERS];  // notice callbacks
    commTelemCallback_t *telemFuncs[COMM_MAX_CONSUMERS];    // telemetry callbacks
    commRcvrCallback_t *rcvrFuncs[COMM_MAX_CONSUMERS];

    commTxStack_t txStack[COMM_NUM_PORTS][COMM_STACK_DEPTH]; // tx stack for each port (768w)

    uint8_t portStreams[COMM_NUM_PORTS]; //(64b)	    // stream assignments for each port
    uint8_t portTypes[COMM_NUM_PORTS]; //(64b)              // type of port (serial, CAN, USB)

    uint8_t txStackHeads[COMM_NUM_PORTS]; //(64b) 	    // stack heads
    volatile uint8_t txStackTails[COMM_NUM_PORTS]; //(64b)  // stack tails

    uint32_t txStackOverruns[COMM_NUM_PORTS];		    // overflow counter
    uint32_t txBufStarved;				    // number of times we ran out of tx packets buffers
    uint32_t txBufUpgrades[COMM_TX_NUM_SIZES];		    // number of times we needed to up size

    uint16_t txPacketBufSizes[COMM_TX_NUM_SIZES]; //(96b)   // list of packet sizes
    void *txPacketBufs[COMM_TX_NUM_SIZES];		    // pointers to start of block for each buffer size
    uint32_t txPacketSizeHits[COMM_TX_NUM_SIZES];

    int logPointer;

    uint8_t txPacketBufNum[COMM_TX_NUM_SIZES]; //(48b)	    // list of number of buffers per size
    uint8_t streamRcvrs[COMM_MAX_CONSUMERS]; // (40b)

    uint8_t typesUsed;					    // types configured
    uint8_t noticePointer;
    int8_t noticeQueueInit;
    uint8_t logHandle;

    OS_TID commTask;
    OS_MutexID txBufferMutex;
    OS_EventID notices;

} commStruct_t __attribute__((aligned));

extern commStruct_t commData;

extern void commNoticesInit(void);
extern void commInit(void);
extern void commRegisterNoticeFunc(commNoticeCallback_t *func);
extern void commRegisterTelemFunc(commTelemCallback_t *func);
extern void commRegisterRcvrFunc(uint8_t streamType, commRcvrCallback_t *func);
extern void commRegisterCanUart(void *ptr, uint8_t type, uint8_t n);
extern void commNotice(const char *s);
extern char *commGetNoticeBuf(void);
extern commTxBuf_t *commGetTxBuf(uint8_t streamType, uint16_t maxSize);
extern void commSendTxBuf(commTxBuf_t *txBuf, uint16_t size);
extern uint8_t commAvailable(commRcvrStruct_t *r);
extern uint8_t commReadChar(commRcvrStruct_t *r);
extern uint8_t commStreamUsed(uint8_t streamType);
extern void commTxFinished(void *param);
extern void commSetStreamType(uint8_t port, uint8_t type);

#endif
