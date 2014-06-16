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
#include "filer.h"
#include "diskio.h"
#include "comm.h"
#include "aq_mavlink.h"
#include "util.h"
#include "supervisor.h"
#include "sdio.h"
#include "usb.h"
#include <string.h>
#include <stdio.h>

filerStruct_t filerData;
OS_STK *filerTaskStack;
// buffer used by logger and USB MSC drivers
uint8_t filerBuf[FILER_BUF_SIZE] __attribute__ ((aligned (16)));

static int32_t filerProcessWrite(filerFileStruct_t *f) {
    uint32_t res;
    UINT bytes;

    if (!f->open) {
	res = f_open(&f->fp, f->fileName, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK)
	    return -1;

	f->open = 1;
    }

    if (f->seek >= 0) {
	res = f_lseek(&f->fp, f->seek);
	if (res != FR_OK) {
	    f->open = 0;
	    return -1;
	}
    }

    res = f_write(&f->fp, f->buf, f->length, &bytes);
    if (res != FR_OK) {
	f->open = 0;
	return -1;
    }

    return bytes;
}

static int32_t filerProcessRead(filerFileStruct_t *f) {
    uint32_t res;
    UINT bytes;

    if (!f->open) {
	res = f_open(&f->fp, f->fileName, FA_OPEN_EXISTING | FA_READ);
	if (res != FR_OK)
	    return -1;

	f->open = 1;
    }

    if (f->seek >= 0) {
	res = f_lseek(&f->fp, f->seek);
	if (res != FR_OK) {
	    f->open = 0;
	    return -1;
	}
    }

    res = f_read(&f->fp, f->buf, f->length, &bytes);
    if (res != FR_OK) {
	f->open = 0;
	return -1;
    }

    return bytes;
}

static int32_t filerProcessSync(filerFileStruct_t *f) {
    uint32_t res;

    if (!f->open) {
	return -1;
    }

    res = f_sync(&f->fp);
    if (res != FR_OK) {
	f->open = 0;
	return -1;
    }

    return 0;
}

static int32_t filerProcessStream(filerFileStruct_t *f, uint8_t final) {
    uint32_t res;
    UINT bytes = 0;
    uint32_t size;

    if (!f->open) {
	sprintf(filerData.buf, "%03d-%s.LOG", filerData.session, f->fileName);
	res = f_open(&f->fp, filerData.buf, FA_CREATE_ALWAYS | FA_WRITE);
	if (res != FR_OK)
	    return -1;

	f->open = 1;
    }

    // enough new to write?
    while (f->tail > f->head || (f->head - f->tail) >= f->length/FILER_FLUSH_THRESHOLD || ((f->length <= 512 || final) && f->head != f->tail)) {
	if (f->head > f->tail)
	    size = f->head - f->tail;
	else
	    size = f->length - f->tail;

	res = f_write(&f->fp, f->buf + f->tail, size, &bytes);
	f->tail = (f->tail + bytes) % f->length;

	if (res != FR_OK)
	    return -1;
    }

    return bytes;
}

static int32_t filerProcessClose(filerFileStruct_t *f) {
    uint32_t res = 0;;

    if (f->open) {
	res = f_close(&f->fp);
	f->open = 0;
    }

    if (res != FR_OK)
	return -1;
    else
	return 0;
}

static void filerProcessRequest(filerFileStruct_t *f) {
    if (f->function == FILER_FUNC_STREAM)
	f->status = filerProcessStream(f, 0);
    else if (f->function == FILER_FUNC_READ)
	f->status = filerProcessRead(f);
    else if (f->function == FILER_FUNC_WRITE)
	f->status = filerProcessWrite(f);
    else if (f->function == FILER_FUNC_SYNC)
	f->status = filerProcessSync(f);
    else if (f->function == FILER_FUNC_CLOSE)
	f->status = filerProcessClose(f);

    if (f->function != FILER_FUNC_STREAM) {
	f->function = FILER_FUNC_NONE;
	CoSetFlag(f->completeFlag);
    }
}

void filerDebug(char *s, int r) {
    AQ_PRINTF("filer: %s [%d]\n", s, r);
}

// open filesystem, format if necessary, update session file
int32_t filerInitFS(void) {
    uint32_t res;
    UINT bytes;

    res = f_opendir(&filerData.dir, "/");
    if (res == FR_NOT_READY || res == FR_DISK_ERR)
	return -1;

    if (res != RES_OK) {
	filerDebug("cannot open root dir, formatting card", res);
	if ((res=f_mkfs(0, 0, 0)) != FR_OK) {
	    filerDebug("cannot create filesystem", res);
	    return -1;
	}
	filerData.session = 0;
    }

    res = f_open(&filerData.sess, FILER_SESS_FNAME, FA_OPEN_EXISTING | FA_READ);
    if (res == FR_OK) {
	f_read(&filerData.sess, filerData.buf, sizeof(filerData.buf), &bytes);
	if (bytes > 1) {
	    if (sscanf(filerData.buf, "%d\n", &filerData.session) < 1)
		filerData.session = 0;
	}
	f_close(&filerData.sess);
    }

    if (++filerData.session > 999)
	filerData.session = 0;

    res = f_open(&filerData.sess, FILER_SESS_FNAME, FA_CREATE_ALWAYS | FA_WRITE);
    if (res != FR_OK) {
	filerDebug("cannot create config file", res);
	return -1;
    }
    sprintf(filerData.buf, "%d\n", filerData.session);
    res = f_write(&filerData.sess, filerData.buf, strlen(filerData.buf), &bytes);

    f_close(&filerData.sess);

    if (res != FR_OK) {
	return -1;
    }
    else {
	filerDebug("created new session", filerData.session);

	return 0;
    }
}

void filerTaskCode(void *p) {
    uint32_t res;
    int i;

    AQ_NOTICE("Filer task started\n");

    filerRestart:

#ifdef HAS_USB
    // does USB MSC want or have the uSD card?
    if (filerData.mscState >= FILER_STATE_MSC_REQUEST) {
	if (filerData.mscState == FILER_STATE_MSC_REQUEST) {
	    if (disk_status(0) != SD_OK) {
		if (disk_initialize(0) == SD_OK)
		    filerData.mscState = FILER_STATE_MSC_ACTIVE;
	    }
	    else {
		filerData.mscState = FILER_STATE_MSC_ACTIVE;
	    }
	}

	// check for USB suspend - TODO: clean this up
	if (usbIsSuspend())
	    // reset MSC state
	    filerData.mscState = FILER_STATE_MSC_DISABLE;

	yield(1000);
	goto filerRestart;
    }
#endif

    filerData.initialized = 0;
    supervisorDiskWait(0);

    memset(&filerData.fs, 0, sizeof(FIL));
    memset(&filerData.dir, 0, sizeof(DIR));

    // setup fatfs
    f_mount(0, 0);
    if ((res = f_mount(0, &filerData.fs)) != RES_OK) {
	filerDebug("cannot register work area, aborting", res);
	CoExitTask();
	return;
    }

    // reset file table
    for (i = 0; i < FILER_MAX_FILES; i++) {
	if (filerData.files[i].allocated) {
	    memset(&filerData.files[i].fp, 0, sizeof(FIL));
	    filerData.files[i].open = 0;
	    CoSetFlag(filerData.files[i].completeFlag);
	}
    }

    while (1) {
#ifdef HAS_USB
	// have we been disabled for USB MSC?
	if (filerData.mscState == FILER_STATE_MSC_REQUEST) {
	    if (filerData.initialized) {
		AQ_NOTICE("USB MSC detected - closing all files\n");

		// sync & close all open files
		for (i = 0; i < FILER_MAX_FILES; i++) {
		    if (filerData.files[i].function == FILER_FUNC_STREAM)
			filerData.files[i].status = filerProcessStream(&filerData.files[i], 1);
		    filerProcessClose(&filerData.files[i]);
		    filerData.files[i].head = 0;
		    filerData.files[i].tail = 0;
		}

		supervisorDiskWait(0);
		filerData.initialized = 0;
	    }

	    goto filerRestart;
	}
#endif

	if (!filerData.initialized) {
	    if (filerInitFS() < 0) {
#ifdef HAS_USB
		// probably no card, reset MSC state
		filerData.mscState = FILER_STATE_MSC_DISABLE;
#endif
		yield(1000);
		goto filerRestart;
	    }
	    else {
		filerData.initialized = 1;
	    }
	}

        supervisorDiskWait(1);

	filerData.loops++;

	for (i = 0; i < FILER_MAX_FILES; i++) {
	    if (filerData.files[i].function > FILER_FUNC_NONE) {
		filerProcessRequest(&filerData.files[i]);

		if (filerData.files[i].function == FILER_FUNC_STREAM) {
		    if (filerData.files[i].status < 0) {
			filerDebug("session write error, aborting", filerData.files[i].status);
			goto filerRestart;
		    }
		    else if (!((filerData.loops+i) % FILER_STREAM_SYNC)) {
			filerProcessSync(&filerData.files[i]);
		    }
		}
	    }
	}

	CoClearFlag(filerData.filerFlag);
	CoWaitForSingleFlag(filerData.filerFlag, 5);	// run at least every 5ms if possible
    }
}

void filerInit(void) {
    memset((void *)&filerData, 0, sizeof(filerData));

    filerData.filerFlag = CoCreateFlag(0, 0);   // manual reset
    filerTaskStack = aqStackInit(FILER_STACK_SIZE, "FILER");

    filerData.filerTask = CoCreateTask(filerTaskCode, (void *)0, FILER_PRIORITY, &filerTaskStack[FILER_STACK_SIZE-1], FILER_STACK_SIZE);

    // wait for card to startup
    yield(500);
}

// returns file handle or -1 upon failure
int8_t filerGetHandle(char *fileName) {
    int i;

    for (i = 0; i < FILER_MAX_FILES; i++) {
	if (!filerData.files[i].allocated || !(strcmp(filerData.files[i].fileName, fileName))) {
	    filerData.files[i].allocated = 1;
	    strcpy(filerData.files[i].fileName, fileName);
	    filerData.files[i].completeFlag = CoCreateFlag(0, 0);

	    return i;
	}
    }

    // too many files open
    return -1;
}

int32_t filerReadWrite(filerFileStruct_t *f, void *buf, int32_t seek, uint32_t length, uint8_t function) {
    // handle allocated yet?
    if (!f->allocated || !filerData.initialized)
	return -1;

    f->buf = buf;
    f->function = function;
    f->seek = seek;
    f->length = length;

    CoSetFlag(filerData.filerFlag);
    CoClearFlag(f->completeFlag);
    CoWaitForSingleFlag(f->completeFlag, 0);

    return f->status;
}

// no seek if seek == -1
int32_t filerRead(int8_t handle, void *buf, int32_t seek, uint32_t length) {
    return filerReadWrite(&filerData.files[handle], buf, seek, length, FILER_FUNC_READ);
}

// no seek if seek == -1
int32_t filerWrite(int8_t handle, void *buf, int32_t seek, uint32_t length) {
    return filerReadWrite(&filerData.files[handle], buf, seek, length, FILER_FUNC_WRITE);
}

int32_t filerSync(int8_t handle) {
    filerFileStruct_t *f = &filerData.files[handle];

    // handle allocated yet?
    if (!f->allocated)
	    return -1;

    if (f->open) {
	f->function = FILER_FUNC_SYNC;

	CoSetFlag(filerData.filerFlag);
	CoClearFlag(f->completeFlag);
	CoWaitForSingleFlag(f->completeFlag, 0);
    }

    return f->status;
}

int32_t filerClose(int8_t handle) {
    filerFileStruct_t *f = &filerData.files[handle];

    // handle allocated yet?
    if (!f->allocated)
	    return -1;

    if (f->open) {
	f->function = FILER_FUNC_CLOSE;

	CoSetFlag(filerData.filerFlag);
	CoClearFlag(f->completeFlag);
	CoWaitForSingleFlag(f->completeFlag, 0);
    }

    f->allocated = 0;

    return f->status;
}

int32_t filerGetHead(int8_t handle) {
    return filerData.files[handle].head;
}

void filerSetHead(int8_t handle, int32_t head) {
    filerData.files[handle].head = head;
}

int32_t filerStream(int8_t handle, void *buf, uint32_t length) {
    filerFileStruct_t *f = &filerData.files[handle];

    // handle allocated yet?
    if (!f->allocated)
	    return -1;

    f->function = FILER_FUNC_STREAM;
    f->buf = buf;
    f->length = length;
    f->status = 0;

    return 1;
}

int8_t filerAvailable(void) {
    return filerData.initialized;
}
