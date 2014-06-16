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

#ifndef _filer_h
#define _filer_h

#include "ff.h"
#include <CoOS.h>

#define FILER_PRIORITY		62
#define FILER_STACK_SIZE	200

#define FILER_SESS_FNAME	"session.txt"
#define FILER_MAX_FILES		5
#define FILER_STREAM_SYNC	200		// ~ 1s
#define FILER_BUF_SIZE		((1<<16)-512)	// <64KB
#define FILER_FLUSH_THRESHOLD	4

#define FILER_FUNC_NONE		0x00
#define FILER_FUNC_READ		0x01
#define FILER_FUNC_WRITE	0x02
#define FILER_FUNC_STREAM	0x03
#define FILER_FUNC_SYNC		0x04
#define FILER_FUNC_CLOSE	0x05

enum {
    FILER_STATE_MSC_DISABLE = 0,
    FILER_STATE_MSC_EJECT,
    FILER_STATE_MSC_REQUEST,
    FILER_STATE_MSC_ACTIVE
};

#define filerEnableMSC()	{if (filerData.mscState == FILER_STATE_MSC_DISABLE) filerData.mscState = FILER_STATE_MSC_REQUEST;}
#define filerEjectMSC()		{filerData.mscState = FILER_STATE_MSC_EJECT;}
#define filerGetMSCState()	(filerData.mscState)

typedef struct {
    OS_FlagID completeFlag;

    char fileName[32];
    FIL fp;
    void *buf;
    int32_t seek;
    uint32_t length;
    int32_t status;
    volatile int32_t head, tail;
    uint8_t open;
    uint8_t function;
    uint8_t allocated;
} filerFileStruct_t;

typedef struct {
    OS_TID filerTask;
    OS_FlagID filerFlag;

    filerFileStruct_t files[FILER_MAX_FILES];
    FATFS fs;
    DIR dir;
    FIL sess;
    char buf[64];
    uint32_t session;
    uint32_t loops;
    uint8_t initialized;
    volatile uint8_t mscState;
} filerStruct_t;

extern filerStruct_t filerData;
extern uint8_t filerBuf[FILER_BUF_SIZE] ;

extern void filerInit(void);
extern int8_t filerGetHandle(char *fileName);
extern int32_t filerRead(int8_t handle, void *buf, int32_t seek, uint32_t length);
extern int32_t filerWrite(int8_t handle, void *buf, int32_t seek, uint32_t length);
extern int32_t filerStream(int8_t handle, void *buf, uint32_t length);
extern int32_t filerGetHead(int8_t handle);
extern void filerSetHead(int8_t handle, int32_t head);
extern int32_t filerSync(int8_t handle);
extern int32_t filerClose(int8_t handle);
extern int8_t filerAvailable(void);

#endif
