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

#include "aq.h"
#include "config.h"
#include "comm.h"
#include <string.h>

commStruct_t commData __attribute__((section(".ccm")));

serialPort_t *commAcquirePort(int port) {
    serialPort_t *ret = 0;

    if (port > 0 && port <= COMM_NUM_PORTS) {
	if (commData.portMode[port-1] == COMM_UNUSED) {
	    ret = commData.serialPorts[port-1];
	    commData.portMode[port-1] = COMM_SINGLEPLEX;
	}
    }

    return ret;
}

void commRegisterNoticeFunc(commNoticeCallback_t *func) {
    int i;

    for (i = 0; i < COMM_MAX_PROTOCOLS; i++) {
	if (commData.noticeFuncs[i] == 0) {
	    commData.noticeFuncs[i] = func;
	    break;
	}
    }
}

void commRegisterTelemFunc(commTelemCallback_t *func) {
    int i;

    for (i = 0; i < COMM_MAX_PROTOCOLS; i++) {
	if (commData.telemFuncs[i] == 0) {
	    commData.telemFuncs[i] = func;
	    break;
	}
    }
}

void commSendNotice(const char *s) {
    int i;

    for (i = 0; i < COMM_MAX_PROTOCOLS; i++) {
	if (commData.noticeFuncs[i])
	    commData.noticeFuncs[i](s);
    }
}

void commDoTelem(void) {
    int i;

    for (i = 0; i < COMM_MAX_PROTOCOLS; i++) {
	if (commData.telemFuncs[i])
	    commData.telemFuncs[i]();
    }
}

void commInit(void) {
    uint16_t flowControl = USART_HardwareFlowControl_RTS_CTS;

    memset((void *)&commData, 0, sizeof(commData));

#ifdef COMM_PORT1
#ifdef COMM_DISABLE_FLOW_CONTROL1
    flowControl = USART_HardwareFlowControl_None;
#endif
    commData.serialPorts[0] = serialOpen(COMM_PORT1, p[COMM_BAUD1], flowControl, COMM_RX_BUF_SIZE1, COMM_TX_BUF_SIZE1);
#endif

#ifdef COMM_PORT2
#ifdef COMM_DISABLE_FLOW_CONTROL2
    flowControl = USART_HardwareFlowControl_None;
#endif
    commData.serialPorts[1] = serialOpen(COMM_PORT2, p[COMM_BAUD2], flowControl, COMM_RX_BUF_SIZE2, COMM_TX_BUF_SIZE2);
#endif

#ifdef COMM_PORT3
#ifdef COMM_DISABLE_FLOW_CONTROL3
    flowControl = USART_HardwareFlowControl_None;
#endif
    commData.serialPorts[2] = serialOpen(COMM_PORT3, p[COMM_BAUD3], flowControl, COMM_RX_BUF_SIZE3, COMM_TX_BUF_SIZE3);
#endif
}