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

    Copyright © 2014  Bill Nesbitt
*/

#ifndef _cyrf_h
#define _cyrf_h

#include "digital.h"
#include "spi.h"

#define CYRF_WRITE_BIT          (0x01<<7)
#define CYRF_INCR_BIT           (0x01<<6)

#define CYRF_STACK_SIZE         24

typedef void cyrfCallback_t(int x);

enum cyrfCommandTypes {
    CYRF_COMMAND_NOP = 0,
    CYRF_COMMAND_READ,
    CYRF_COMMAND_WRITE,
    CYRF_COMMAND_BATCH,
    CYRF_COMMAND_EXEC
};

typedef struct {
    void *target;
    uint8_t command;
    uint8_t len;
} cyrfStack_t;

typedef struct {
	uint32_t irqTime;
    cyrfCallback_t *isrCallback, *timeoutCallback;
    uint8_t isrCallbackParam, timeoutCallbackParam;
    digitalPin *rst;
    uint32_t *isrAddr;
    spiClient_t *spi;
    volatile uint32_t spiFlag;
    cyrfStack_t slots[CYRF_STACK_SIZE];
    volatile uint8_t head, tail;
    uint8_t isTx;
    uint8_t rxIrqStatus;
    uint8_t txIrqStatus;
    uint8_t initialized;
    uint8_t running;
} cyrfStruct_t;

extern cyrfStruct_t cyrfData;

extern uint8_t cyrfInit(void);
extern void cyrfTimerSet(uint16_t micros);
extern void cyrfTimerCancel(void);
extern void cyrfQueueBatch(const uint8_t batch[][2]);
extern void cyrfQueueExec(cyrfCallback_t *func, uint8_t x);
extern void cyrfQueueReadReg(uint8_t reg, uint8_t *data);
extern void cyrfQueueReadBlock(uint8_t reg, uint8_t len, uint8_t *data);
extern void cyrfQueueWriteReg(uint8_t reg, uint8_t data);
extern void cyrfQueueWriteBlock(uint8_t reg, uint8_t len, uint8_t *data);
extern void cyrfProcessStack(void);
extern void cyrfQueueClear(void);
extern void cyrfSetIsrCallback(cyrfCallback_t *callback, uint8_t param);
extern void cyrfTimeoutSetRelative(uint16_t micros, cyrfCallback_t *callback, uint8_t param);
extern void cyrfTimeoutSet(uint16_t micros, cyrfCallback_t *callback, uint8_t param);
extern void cyrfTimeoutCancel(void);
extern void cyrfTimeoutReset(void);
extern uint8_t cyrfGetIsrState(void);

#endif
