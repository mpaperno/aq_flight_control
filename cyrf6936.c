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

#include "cyrf6936.h"
#include "cyrf6936_regs.h"
#include "util.h"
#include "comm.h"
#include "rcc.h"
#include "aq_timer.h"
#include "ext_irq.h"
#include <string.h>

#ifdef CYRF_SPI

cyrfStruct_t cyrfData;

static uint32_t cyrfRxBuf[5];
static uint32_t cyrfTxBuf[CYRF_STACK_SIZE][5];

static uint8_t cyrfGetReg(uint8_t reg) {
    ((uint8_t *)&cyrfTxBuf)[0] = reg;

    cyrfData.spiFlag = 0;
    spiTransaction(cyrfData.spi, &cyrfRxBuf, &cyrfTxBuf, 2);

    while (!cyrfData.spiFlag)
        ;

    return ((uint8_t *)&cyrfRxBuf)[1];
}

static void cyrfSetReg(uint8_t reg, uint8_t val) {
    ((uint8_t *)&cyrfTxBuf)[0] = CYRF_WRITE_BIT | reg;
    ((uint8_t *)&cyrfTxBuf)[1] = val;

    cyrfData.spiFlag = 0;
    spiTransaction(cyrfData.spi, &cyrfRxBuf, &cyrfTxBuf, 2);

    while (!cyrfData.spiFlag)
		;
}

void cyrfQueueClear(void) {
	cyrfData.tail = cyrfData.head = 0;
	cyrfData.slots[0].command = CYRF_COMMAND_NOP;
}

uint8_t cyrfProcessBatch(const uint8_t batch[][2], uint8_t index, uint8_t slotNo) {
    uint8_t *txBuf = (uint8_t *)cyrfTxBuf[slotNo];

    if (batch[index][0] != 0xff) {
        txBuf[0] = batch[index][0];
        txBuf[1] = batch[index][1];

        cyrfData.spiFlag = 0;
        spiTransaction(cyrfData.spi, cyrfRxBuf, txBuf, 2);

        return 1;
    }
    else {
        return 0;
    }
}

void cyrfProcessStack(void) {
    static uint8_t inProcess = 0;
    uint8_t tail;
    cyrfStack_t *slot;

    if (inProcess)
        return;

    processStackTop:

    if (!cyrfData.running) {
        cyrfData.tail = cyrfData.head = 0;
        return;
    }

    tail = cyrfData.tail;
    slot = &cyrfData.slots[tail];

    // only if not in the middle of a txn
    if (tail != cyrfData.head && cyrfData.spiFlag != 0) {
        switch (slot->command) {
            case CYRF_COMMAND_READ:
            case CYRF_COMMAND_WRITE:
                cyrfData.spiFlag = 0;
                spiTransaction(cyrfData.spi, cyrfRxBuf, cyrfTxBuf[tail], slot->len + 1);
                break;

            case CYRF_COMMAND_BATCH:
                if (cyrfProcessBatch(slot->target, slot->len, tail) == 0) {
                    cyrfData.tail = (tail + 1) % CYRF_STACK_SIZE;
                    goto processStackTop;
                }
                else {
                    slot->len++;
                }
                break;

            case CYRF_COMMAND_EXEC:
                inProcess = 1;
                if (slot->target)
                    ((cyrfCallback_t *)slot->target)(slot->len);
                inProcess = 0;

                cyrfData.tail = (tail + 1) % CYRF_STACK_SIZE;
                goto processStackTop;
                break;

            default:
                break;
        }
    }
}

void cyrfQueueWriteBlock(uint8_t reg, uint8_t len, uint8_t *data) {
    uint8_t head = cyrfData.head;
    cyrfStack_t *slot = &cyrfData.slots[head];
    uint8_t *txBuf = (uint8_t *)cyrfTxBuf[head];
    int i;

    slot->command = CYRF_COMMAND_WRITE;
    slot->len = len;

    txBuf[0] = CYRF_WRITE_BIT | reg;
    for (i = 0; i < len; i++)
        txBuf[i+1] = data[i];

    cyrfData.head = (head + 1) % CYRF_STACK_SIZE;
}

void cyrfQueueWriteReg(uint8_t reg, uint8_t data) {
    uint8_t head = cyrfData.head;
    cyrfStack_t *slot = &cyrfData.slots[head];
    uint8_t *txBuf = (uint8_t *)cyrfTxBuf[head];

    slot->command = CYRF_COMMAND_WRITE;
    slot->len = 1;

    txBuf[0] = CYRF_WRITE_BIT | reg;
    txBuf[1] = data;

    cyrfData.head = (head + 1) % CYRF_STACK_SIZE;
}

void cyrfQueueReadBlock(uint8_t reg, uint8_t len, uint8_t *data) {
    uint8_t head = cyrfData.head;
    cyrfStack_t *slot = &cyrfData.slots[head];
    uint8_t *txBuf = (uint8_t *)cyrfTxBuf[head];

    slot->command = CYRF_COMMAND_READ;
    slot->len = len;
    slot->target = data;

    txBuf[0] = reg;

    cyrfData.head = (head + 1) % CYRF_STACK_SIZE;
}

void cyrfQueueReadReg(uint8_t reg, uint8_t *data) {
    uint8_t head = cyrfData.head;
    cyrfStack_t *slot = &cyrfData.slots[head];
    uint8_t *txBuf = (uint8_t *)cyrfTxBuf[head];

    slot->command = CYRF_COMMAND_READ;
    slot->len = 1;
    slot->target = data;

    txBuf[0] = reg;

    cyrfData.head = (head + 1) % CYRF_STACK_SIZE;
}

void cyrfQueueExec(cyrfCallback_t *func, uint8_t x) {
    uint8_t head = cyrfData.head;
    cyrfStack_t *slot = &cyrfData.slots[head];

    slot->command = CYRF_COMMAND_EXEC;
    slot->len = x;
    slot->target = func;

    cyrfData.head = (head + 1) % CYRF_STACK_SIZE;
}

void cyrfQueueBatch(const uint8_t batch[][2]) {
    uint8_t head = cyrfData.head;
    cyrfStack_t *slot = &cyrfData.slots[head];

    slot->command = CYRF_COMMAND_BATCH;
    slot->len = 0;
    slot->target = (void *)batch;

    cyrfData.head = (head + 1) % CYRF_STACK_SIZE;
}

void cyrfTxnComplete(int unused) {
    uint8_t tail = cyrfData.tail;
    cyrfStack_t *slot = &cyrfData.slots[tail];
    uint8_t *target = (uint8_t *)slot->target;
    int i;

    switch (slot->command) {
        case CYRF_COMMAND_READ:
            for (i = 0; i < slot->len; i++)
                target[i] = ((uint8_t *)cyrfRxBuf)[i+1];
            tail++;
            break;

        case CYRF_COMMAND_WRITE:
            tail++;

        default:
            break;
    }

    cyrfData.tail = tail % CYRF_STACK_SIZE;

    if (cyrfData.tail != cyrfData.head)
        cyrfProcessStack();
}

void cyrfSetIsrCallback(cyrfCallback_t *callback, uint8_t param) {
    cyrfData.isrCallback = callback;
    cyrfData.isrCallbackParam = param;
}

void cyrfTimeoutSet(uint16_t micros, cyrfCallback_t *callback, uint8_t param) {
    cyrfData.timeoutCallback = callback;
    cyrfData.timeoutCallbackParam = param;
    CYRF_TIMER->ARR = micros;
    CYRF_TIMER->SR &= (uint16_t)~TIM_IT_Update;
    CYRF_TIMER->DIER |= (uint16_t)TIM_IT_Update;
}

void cyrfTimeoutSetRelative(uint16_t micros, cyrfCallback_t *callback, uint8_t param) {
    cyrfData.timeoutCallback = callback;
    cyrfData.timeoutCallbackParam = param;
    CYRF_TIMER->ARR = CYRF_TIMER->CNT + micros;
    CYRF_TIMER->SR &= (uint16_t)~TIM_IT_Update;
    CYRF_TIMER->DIER |= (uint16_t)TIM_IT_Update;
}

void cyrfTimeoutCancel(void) {
    CYRF_TIMER->CNT = 0;
    CYRF_TIMER->DIER &= (uint16_t)~TIM_IT_Update;
    CYRF_TIMER->SR &= (uint16_t)~TIM_IT_Update;
}

void cyrfTimeoutReset(void) {
    CYRF_TIMER->CNT = 0;
    CYRF_TIMER->SR &= (uint16_t)~TIM_IT_Update;
}

uint8_t cyrfGetIsrState(void) {
    return *cyrfData.isrAddr;
}

void cyrfTimerInit(void) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // Enable the TIMER_TIM global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = CYRF_TIMER_IRQ_CH;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

	// stop timer when core halted (debug)
	DBGMCU_APB1PeriphConfig(CYRF_TIMER_DBG, ENABLE);

    /* Time base configuration for 1MHz (us)*/
    TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
    TIM_TimeBaseStructure.TIM_Prescaler = (CYRF_TIMER_CLOCK / 1000000) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(CYRF_TIMER, &TIM_TimeBaseStructure);

    TIM_ARRPreloadConfig(CYRF_TIMER, DISABLE);

    // reset
    TIM_SetCounter(CYRF_TIMER, 0);

    // go...
    TIM_Cmd(CYRF_TIMER, ENABLE);
}

void cyrfIntHandler(void) {
    cyrfData.irqTime = timerMicros();

    if (cyrfData.isTx)
        // get the TX IRQ status
        cyrfQueueReadReg(CYRF_TX_IRQ_STATUS, &cyrfData.txIrqStatus);
    else
        // get the RX IRQ status
        cyrfQueueReadReg(CYRF_RX_IRQ_STATUS, &cyrfData.rxIrqStatus);

    if (cyrfData.isrCallback)
        cyrfQueueExec(cyrfData.isrCallback, cyrfData.isrCallbackParam);

    cyrfProcessStack();
}

uint8_t cyrfInit(void) {
    int i;

    if (cyrfData.spi)
        spiClientFree(cyrfData.spi);

    memset((void *)&cyrfData, 0, sizeof(cyrfData));

    cyrfData.spi = spiClientInit(CYRF_SPI, CYRF_SPI_BAUD, 1, CYRF_CS_PORT, CYRF_CS_PIN, &cyrfData.spiFlag, 0);

    cyrfTimerInit();

#ifdef CYRF_RST_PORT
    cyrfData.rst = digitalInit(CYRF_RST_PORT, CYRF_RST_PIN, 1);
    delayMicros(100);
    digitalLo(cyrfData.rst);
    delayMicros(100);
#endif

    i = 10;
    do {
        cyrfSetReg(CYRF_XACT_CFG, 0x82);
        delayMicros(100);
        i--;
    }
    while (cyrfGetReg(CYRF_XACT_CFG) != 0x82 && i);

    if (i) {
        uint16_t pin = CYRF_IRQ_PIN;
        uint8_t bit = 0;

        while (pin >>= 1)
            bit++;

        if (bit < 8)
            cyrfData.isrAddr = PERIPH2BB((uint32_t)CYRF_IRQ_PORT + 0x10, bit);
        else
            cyrfData.isrAddr = PERIPH2BB((uint32_t)CYRF_IRQ_PORT + 0x11, bit-8);

        // External Interrupt line
        extRegisterCallback(CYRF_IRQ_PORT, CYRF_IRQ_PIN, EXTI_Trigger_Falling, 1, GPIO_PuPd_NOPULL, cyrfIntHandler);

        cyrfData.initialized = 1;

        spiChangeCallback(cyrfData.spi, cyrfTxnComplete);
    }
    else {
        spiClientFree(cyrfData.spi);
    }

    return i;
}


void CYRF_TIMER_ISR(void) {
    CYRF_TIMER->SR &= (uint16_t)~TIM_IT_Update;

    if (cyrfData.timeoutCallback)
        cyrfData.timeoutCallback(cyrfData.timeoutCallbackParam);
}
#endif