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

#ifndef _notice_h
#define _notice_h

#include "aq_mavlink.h"
#include <CoOS.h>

#define NOTICE_STACK_SIZE	64
#define NOTICE_PRIORITY		50

#define NOTICE_QUEUE_DEPTH	20

typedef struct {
    OS_TID noticeTask;
    OS_EventID notices;
    void *noticeQueue[NOTICE_QUEUE_DEPTH];
    int8_t initialized;
} noticeStruct_t;

extern noticeStruct_t noticeData;

extern void noticeInit(void);
extern void noticeSend(const char *s);

#endif
