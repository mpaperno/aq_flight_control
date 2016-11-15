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

#ifndef _can_uart_h
#define _can_uart_h

#include "can.h"

#define CAN_UART_NUM            3
#define CAN_UART_BUF_SIZE       128
#define CAN_UART_BLOCK_SIZE     16      // max number of packets to send at once

typedef void canUartTxCallback_t(void *ptr);

typedef struct {
    canNodes_t *node;
    uint8_t *txBuf;
    volatile int16_t txHead, txTail;
    canUartTxCallback_t *txCallback;
    void *txCallbackParam;
    uint8_t rxBuf[CAN_UART_BUF_SIZE];
    int16_t rxHead, rxTail;
} canUartStruct_t;

extern void canUartInit(void);
extern uint8_t canUartAvailable(canUartStruct_t *ptr);
extern uint8_t canUartReadChar(canUartStruct_t *ptr);
extern void canUartRxChar(uint8_t canId, uint8_t n, uint8_t *data);
extern void canUartTxBuf(canUartStruct_t *ptr, uint8_t *buf, uint16_t n, canUartTxCallback_t *callback, void *param);
extern void canUartStream(void);

#endif