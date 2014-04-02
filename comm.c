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
#include "config.h"
#include "comm.h"
#include "run.h"
#include "util.h"
#include "filer.h"
#include "can.h"
#include "canUart.h"
#include "usb.h"
#include <CoOS.h>
#include <string.h>

OS_STK *commTaskStack;

commStruct_t commData __attribute__((section(".ccm")));

#ifdef COMM_LOG_FNAME
char commLog[COMM_LOG_BUF_SIZE];
#endif

char *commGetNoticeBuf(void) {
    uint8_t p;

    UTIL_ISR_DISABLE;
    p = (commData.noticePointer++ % COMM_NOTICE_DEPTH);
    UTIL_ISR_ENABLE;

    return (commData.noticeStrings[p]);
}

void commNotice(const char *s) {
    // post message and leave
    if (commData.initialized)
        CoPostQueueMail(commData.notices, (void *)s);
}

static void commTriggerSchedule(void) {
    // another stolen interrupt
    NVIC->STIR = CRYP_IRQn;
}

uint8_t commStreamUsed(uint8_t streamType) {
    return (commData.typesUsed & streamType);
}

void commRegisterNoticeFunc(commNoticeCallback_t *func) {
    int i;

    for (i = 0; i < COMM_MAX_CONSUMERS; i++) {
        if (commData.noticeFuncs[i] == 0) {
            commData.noticeFuncs[i] = func;
            break;
        }
    }
}

void commRegisterTelemFunc(commTelemCallback_t *func) {
    int i;

    for (i = 0; i < COMM_MAX_CONSUMERS; i++) {
        if (commData.telemFuncs[i] == 0) {
            commData.telemFuncs[i] = func;
            break;
        }
    }
}

void commRegisterRcvrFunc(uint8_t streamType, commRcvrCallback_t *func) {
    int i;

    for (i = 0; i < COMM_MAX_CONSUMERS; i++) {
        if (commData.streamRcvrs[i] == 0) {
            commData.streamRcvrs[i] = streamType;
            commData.rcvrFuncs[i] = func;
            break;
        }
    }
}

uint8_t commReadChar(commRcvrStruct_t *r) {
    uint8_t port = r->port;

    switch (commData.portTypes[port]) {
        case COMM_PORT_TYPE_SERIAL:
            if (commData.portHandles[port])
                return serialRead(commData.portHandles[port]);
            break;

        case COMM_PORT_TYPE_CAN:
            if (commData.portHandles[port])
                return canUartReadChar(commData.portHandles[port]);
            break;

#ifdef HAS_USB
        case COMM_PORT_TYPE_USB:
            return usbRx();
            break;
#endif
    }

    return 0;
}

uint8_t commAvailable(commRcvrStruct_t *r) {
    uint8_t port = r->port;

    switch (commData.portTypes[port]) {
        case COMM_PORT_TYPE_SERIAL:
            if (commData.portHandles[port])
                return serialAvailable(commData.portHandles[port]);
            break;

        case COMM_PORT_TYPE_CAN:
            if (commData.portHandles[port])
                return canUartAvailable(commData.portHandles[port]);
            break;

#ifdef HAS_USB
        case COMM_PORT_TYPE_USB:
            return usbAvailable();
            break;
#endif
    }

    return 0;
}

// return 0 if none are available
commTxBuf_t *commGetTxBuf(uint8_t streamType, uint16_t maxSize) {
    commTxBuf_t *txBuf = 0;
    commTxBuf_t *tmp;
    int i, j;

    // is this stream type even active?
    if (commData.typesUsed & streamType) {
        // look for smallest size that request will fit in
        for (i = 0; i < COMM_TX_NUM_SIZES; i++)
            if (commData.txPacketBufSizes[i] >= maxSize)
                break;

        CoEnterMutexSection(commData.txBufferMutex);

        commTopOfSearch:

        // not too big?
        if (i < COMM_TX_NUM_SIZES) {
            // look for free buffer in this block
            for (j = 0; j < commData.txPacketBufNum[i]; j++) {
                tmp = (commTxBuf_t *)(commData.txPacketBufs[i] + (commData.txPacketBufSizes[i] + COMM_HEADER_SIZE) * j);

                if (tmp->status == COMM_TX_BUF_FREE) {
                    txBuf = tmp;
                    txBuf->status = COMM_TX_BUF_ALLOCATED;
                    txBuf->type = streamType;
                    break;
                }
            }

            // need an upgrade?
            if (j == commData.txPacketBufNum[i]) {
                // make a note of this
                commData.txBufUpgrades[i]++;
                // next larger size
                i++;

                // try again
                goto commTopOfSearch;
            }
        }

        CoLeaveMutexSection(commData.txBufferMutex);

        if (txBuf == 0)
            commData.txBufStarved++;
        else
            commData.txPacketSizeHits[i]++;
    }

    return txBuf;
}

static void _commSchedule(uint8_t port) {
    uint8_t tail = commData.txStackTails[port];
    commTxStack_t *stack = &commData.txStack[port][tail];

    if (commData.txStackHeads[port] != tail)
        switch (commData.portTypes[port]) {
            case COMM_PORT_TYPE_SERIAL:
                if (!((serialPort_t *)(commData.portHandles[port]))->txDmaRunning && _serialStartTxDMA(commData.portHandles[port], stack->memory, stack->size, commTxFinished, stack))
                    commData.txStackTails[port] = (tail + 1) % COMM_STACK_DEPTH;
                break;

            case COMM_PORT_TYPE_CAN:
                if (((canUartStruct_t *)(commData.portHandles[port]))->txTail == ((canUartStruct_t *)(commData.portHandles[port]))->txHead) {
                    canUartTxBuf(commData.portHandles[port], stack->memory, stack->size, commTxFinished, stack);
                    commData.txStackTails[port] = (tail + 1) % COMM_STACK_DEPTH;
                }
                break;
        }
}

static void commSchedule(void) {
    int i;

    for (i = 0; i < COMM_NUM_PORTS; i++) {
        if (commData.portHandles[i])
            _commSchedule(i);
    }
}

void commTxFinished(void *param) {
    commTxStack_t *txStackPtr = (commTxStack_t *)param;
    commTxBuf_t *txBuf = (commTxBuf_t *)txStackPtr->txBuf;

    // if no pending tx's for this buffer, free it
    if (__sync_sub_and_fetch(&txBuf->status, 1) == COMM_TX_BUF_SENDING)
        txBuf->status = COMM_TX_BUF_FREE;

    // re-schedule
    _commSchedule(txStackPtr->port);
}

void commSendTxBuf(commTxBuf_t *txBuf, uint16_t size) {
    uint8_t head;
    uint8_t toBeScheduled[COMM_NUM_PORTS];
    uint8_t newHeads[COMM_NUM_PORTS];
    uint8_t sent = 0;
    int i;

    if (txBuf) {
        // reset status to sending
        txBuf->status = COMM_TX_BUF_SENDING;

        CoEnterMutexSection(commData.txBufferMutex);

        // look for any ports that want this stream
        for (i = 0; i < COMM_NUM_PORTS; i++) {
            toBeScheduled[i] = 0;

            // singleplex case
            if (commData.portStreams[i] == txBuf->type && commData.portTypes[i] != COMM_PORT_TYPE_USB && commData.portTypes[i] != COMM_PORT_TYPE_NONE) {
                head = commData.txStackHeads[i];
                newHeads[i] = (head + 1) % COMM_STACK_DEPTH;

                // check for stack overruns
                if (newHeads[i] == commData.txStackTails[i]) {
                    // record incident
                    commData.txStackOverruns[i]++;
                }
                else {
                    txBuf->status++;

                    // prepare to send
                    commData.txStack[i][head].port = i;
                    commData.txStack[i][head].txBuf = txBuf;
                    commData.txStack[i][head].memory = &txBuf->buf;
                    commData.txStack[i][head].size = size;

                    toBeScheduled[i] = 1;
                    sent = 1;
                }
            }
            // multiplex case
            else if (commData.portStreams[i] & txBuf->type) {
                // TODO
            }
        }

        if (!sent) {
            // release buffer
            txBuf->status = COMM_TX_BUF_FREE;
        }
        else {
            for (i = 0; i < COMM_NUM_PORTS; i++) {
                if (toBeScheduled[i])
                    commData.txStackHeads[i] = newHeads[i];
                commTriggerSchedule();
            }
        }

        CoLeaveMutexSection(commData.txBufferMutex);

#ifdef COMM_USB_PORT
        if (commData.portStreams[COMM_USB_PORT] == txBuf->type)
            usbTx(&txBuf->buf, size);
#endif
    }
}

static void commCheckNotices(void) {
    StatusType result;
    char *s;

    s = (char *)CoAcceptQueueMail(commData.notices, &result);

    if (s) {
        int i;
#ifdef COMM_LOG_FNAME
        // write to disk
        i = 0;
        while (s[i] != 0) {
            if (s[i] != '\n') {
                commLog[commData.logPointer] = s[i];
                commData.logPointer = (commData.logPointer + 1) % COMM_LOG_BUF_SIZE;
            }
            i++;
        }
        commLog[commData.logPointer] = '\n';
        commData.logPointer = (commData.logPointer + 1) % COMM_LOG_BUF_SIZE;

        filerSetHead(commData.logHandle, commData.logPointer);
#endif

        for (i = 0; i < COMM_MAX_CONSUMERS; i++)
            if (commData.noticeFuncs[i])
                commData.noticeFuncs[i](s);
    }
}

static void commCheckTelem(void) {
    int i;

    if (CoAcceptSingleFlag(runData.runFlag) == E_OK)
        for (i = 0; i < COMM_MAX_CONSUMERS; i++)
            if (commData.telemFuncs[i])
                commData.telemFuncs[i]();
}

static void commCheckRcvr(void) {
    commRcvrStruct_t r;
    int i, j;

    for (i = 0; i < COMM_NUM_PORTS; i++) {
        r.port = i;
        if (commAvailable(&r) && commData.portStreams[i] > COMM_STREAM_TYPE_NONE) {
            for (j = 0; j < COMM_MAX_CONSUMERS; j++) {
                if (commData.streamRcvrs[j] == commData.portStreams[i]) {
                    commData.rcvrFuncs[j](&r);
                    break;
                }
            }
        }
    }
}

void commTaskCode(void *unused) {
    uint32_t loops = 0;

    while (1) {
        yield(1);

        commCheckNotices();
        commCheckTelem();
        canCheckMessage(loops);
        commCheckRcvr();
        canUartStream();

        loops++;
    }
}

void commSetTypesUsed(void) {
    uint8_t typesUsed = 0;
    int i;

    for (i = 0; i < COMM_NUM_PORTS; i++)
        typesUsed |= commData.portStreams[i];

    commData.typesUsed = typesUsed;
}

void commSetStreamType(uint8_t port, uint8_t type) {
    commData.portStreams[port] = type;
    commSetTypesUsed();
}

void commInit(void) {
    NVIC_InitTypeDef NVIC_InitStructure;
    uint16_t flowControl;
    int i;

    memset((void *)&commData, 0, sizeof(commData));

#ifdef COMM_LOG_FNAME
    commData.logHandle = filerGetHandle(COMM_LOG_FNAME);
    filerStream(commData.logHandle, commLog, COMM_LOG_BUF_SIZE);
#endif

#ifdef COMM_PORT1
#ifdef COMM_DISABLE_FLOW_CONTROL1
    flowControl = USART_HardwareFlowControl_None;
#else
    flowControl = USART_HardwareFlowControl_RTS_CTS;
#endif
    commData.portHandles[0] = serialOpen(COMM_PORT1, p[COMM_BAUD1], flowControl, COMM_RX_BUF_SIZE, 0);
    commData.portStreams[0] = (uint8_t)p[COMM_STREAM_TYP1];
    commData.portTypes[0] = COMM_PORT_TYPE_SERIAL;
#endif

#ifdef COMM_PORT2
#ifdef COMM_DISABLE_FLOW_CONTROL2
    flowControl = USART_HardwareFlowControl_None;
#else
    flowControl = USART_HardwareFlowControl_RTS_CTS;
#endif
    commData.portHandles[1] = serialOpen(COMM_PORT2, p[COMM_BAUD2], flowControl, COMM_RX_BUF_SIZE, 0);
    commData.portStreams[1] = (uint8_t)p[COMM_STREAM_TYP2];
    commData.portTypes[1] = COMM_PORT_TYPE_SERIAL;
#endif

#ifdef COMM_PORT3
#ifdef COMM_DISABLE_FLOW_CONTROL3
    flowControl = USART_HardwareFlowControl_None;
#else
    flowControl = USART_HardwareFlowControl_RTS_CTS;
#endif
    commData.portHandles[2] = serialOpen(COMM_PORT3, p[COMM_BAUD3], flowControl, COMM_RX_BUF_SIZE, 0);
    commData.portStreams[2] = (uint8_t)p[COMM_STREAM_TYP3];
    commData.portTypes[2] = COMM_PORT_TYPE_SERIAL;
#endif

#ifdef COMM_PORT4
#ifdef COMM_DISABLE_FLOW_CONTROL4
    flowControl = USART_HardwareFlowControl_None;
#else
    flowControl = USART_HardwareFlowControl_RTS_CTS;
#endif
    commData.portHandles[3] = serialOpen(COMM_PORT4, p[COMM_BAUD4], flowControl, COMM_RX_BUF_SIZE, 0);
    commData.portStreams[3] = (uint8_t)p[COMM_STREAM_TYP4];
    commData.portTypes[3] = COMM_PORT_TYPE_SERIAL;
#endif

    // record which stream types that we are working with
    commSetTypesUsed();

    // perhaps make this dynamic later
    commData.txPacketBufSizes[0] = 16;
    commData.txPacketBufSizes[1] = 32;
    commData.txPacketBufSizes[2] = 64;
    commData.txPacketBufSizes[3] = 128;
    commData.txPacketBufSizes[4] = 256;
    commData.txPacketBufSizes[5] = 512;

    commData.txPacketBufNum[0] = 16;
    commData.txPacketBufNum[1] = 16;
    commData.txPacketBufNum[2] = 8;
    commData.txPacketBufNum[3] = 4;
    commData.txPacketBufNum[4] = 6;
    commData.txPacketBufNum[5] = 4;

    // allocate transmission buffers' memory
    for (i = 0; i < COMM_TX_NUM_SIZES; i++)
        commData.txPacketBufs[i] = aqCalloc(commData.txPacketBufNum[i], commData.txPacketBufSizes[i] + COMM_HEADER_SIZE);

    // Enable CRYP interrupt (for our stack management)
    NVIC_InitStructure.NVIC_IRQChannel = CRYP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // setup mutex
    commData.txBufferMutex = CoCreateMutex();

    // notice queue
    commData.notices = CoCreateQueue(commData.noticeQueue, COMM_NOTICE_DEPTH, EVENT_SORT_TYPE_FIFO);

    commTaskStack = aqStackInit(COMM_STACK_SIZE, "COMM");

    commData.commTask = CoCreateTask(commTaskCode, (void *)0, 5, &commTaskStack[COMM_STACK_SIZE-1], COMM_STACK_SIZE);

#ifdef COMM_USB_PORT
    usbInit();
    commData.portTypes[COMM_USB_PORT] = COMM_PORT_TYPE_USB;
#endif
    commData.initialized = 1;
}

void commRegisterCanUart(void *ptr, uint8_t type, uint8_t n) {
    commData.portHandles[COMM_CAN_PORT+n] = ptr;
    commData.portTypes[COMM_CAN_PORT+n] = type;
    commData.portStreams[COMM_CAN_PORT+n] = (uint8_t)p[COMM_STREAM_TYP5+n];

    commSetTypesUsed();
}

void CRYP_IRQHandler(void) {
    commSchedule();
}
