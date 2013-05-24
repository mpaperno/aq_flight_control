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
#include "notice.h"
#include "aq_mavlink.h"
#include "util.h"
#include "config.h"
#include "comm.h"
#include "supervisor.h"
#include <CoOS.h>
#include <string.h>

noticeStruct_t noticeData __attribute__((section(".ccm")));

OS_STK *noticeTaskStack;

void noticeSend(const char *s) {
    // post message and leave
    if (noticeData.initialized)
	CoPostQueueMail(noticeData.notices, (void *)s);
}

void noticeTaskCode(void *unused) {
    StatusType result;
    commTxBuf_t *txBuf;
    uint8_t ckA, ckB;
    char *c;
    uint8_t *ptr;

    AQ_NOTICE("Notice task started\n");

    while (1) {
	c = (char *)CoPendQueueMail(noticeData.notices, 0, &result);

	// wait until a txn buffer is available
	txBuf = commGetTxBuf(COMM_TYPE_TELEMETRY, 64);
	while (txBuf == 0) {
	    yield(1);
	    txBuf = commGetTxBuf(COMM_TYPE_TELEMETRY, 64);
	}
	ptr = &txBuf->buf;

	supervisorSendDataStart();

	*ptr++ = 'A';
	*ptr++ = 'q';
	*ptr++ = 'I';

	ckA = ckB = 0;
	do {
	    *ptr++ = *c;
	    ckA += *c;
	    ckB += ckA;
	} while (*(c++));

	*ptr++ = ckA;
	*ptr++ = ckB;

	commSendTxBuf(txBuf, ptr - &txBuf->buf);
	supervisorSendDataStop();
    }
}

void noticeInit(void) {
    memset((void *)&noticeData, 0, sizeof(noticeData));

    if (commStreamUsed(COMM_TYPE_TELEMETRY)) {
	noticeData.notices = CoCreateQueue(noticeData.noticeQueue, NOTICE_QUEUE_DEPTH, EVENT_SORT_TYPE_FIFO);
	noticeTaskStack = aqStackInit(NOTICE_STACK_SIZE, "NOTICE");

	// start notice thread with high priority so that it can process startup notices - will drop priority later
	noticeData.noticeTask = CoCreateTask(noticeTaskCode, (void *)0, 5, &noticeTaskStack[NOTICE_STACK_SIZE-1], NOTICE_STACK_SIZE);

	// register notice function with comm module
	commRegisterNoticeFunc(noticeSend);

	noticeData.initialized = 1;
    }
}
