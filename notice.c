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
#include "downlink.h"
#include "notice.h"
#include "aq_mavlink.h"
#include <CoOS.h>
#include <string.h>

noticeStruct_t noticeData;

OS_STK noticeTaskStack[TASK_STACK_SIZE];

void noticeSend(const char *s) {
    // post message and leave
    CoPostQueueMail(noticeData.notices, (void *)s);
}

void noticeTaskCode(void *unused) {
    StatusType result;
    char *c;

    AQ_NOTICE("Notice task started...\n");

    while (1) {
	c = (char *)CoPendQueueMail(noticeData.notices, 0, &result);

	// grab port lock
	CoEnterMutexSection(downlinkData.serialPortMutex);

	downlinkSendString("AqI"); // info header

	downlinkResetChecksum();
	do {
	    downlinkSendChar(*c);
	} while (*c++);

	downlinkSendChecksum();

	// release serial port
	CoLeaveMutexSection(downlinkData.serialPortMutex);
    }
}

void noticeInit(void) {
    noticeData.notices = CoCreateQueue(noticeData.noticeQueue, NOTICE_QUEUE_DEPTH, EVENT_SORT_TYPE_FIFO);

    // start notice thread with high priority so that it can process startup notices - will drop priority later
    noticeData.noticeTask = CoCreateTask(noticeTaskCode, (void *)0, 5, &noticeTaskStack[TASK_STACK_SIZE-1], TASK_STACK_SIZE);
}
