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

#include "canUart.h"
#include "comm.h"
#include "config.h"

canUartStruct_t canUartData[CAN_UART_NUM];

uint8_t canUartAvailable(canUartStruct_t *ptr) {
    return (ptr->rxHead != ptr->rxTail);
}

uint8_t canUartReadChar(canUartStruct_t *ptr) {
    uint8_t c;

    c = ptr->rxBuf[ptr->rxTail];

    ptr->rxTail = (ptr->rxTail + 1) % CAN_UART_BUF_SIZE;

    return c;
}

void canUartRxChar(uint8_t canId, uint8_t n, uint8_t *data) {
    canUartStruct_t *ptr = &canUartData[canId - 1];
    int i;

    for (i = 0; i < n; i++)
        ptr->rxBuf[(ptr->rxHead+i) & (CAN_UART_BUF_SIZE-1)] = data[i];

    ptr->rxHead = (ptr->rxHead + n) % CAN_UART_BUF_SIZE;
}

static int16_t canUartTx(canUartStruct_t *ptr) {
    int16_t tail = ptr->txTail;
    int n;

    n = ptr->txHead - tail;
    if (n > 8)
        n = 8;

    canSendBulk(CAN_LCC_INFO | CAN_TT_NODE | CAN_FID_CMD | (CAN_CMD_STREAM<<19), ptr->node->networkId, n, &ptr->txBuf[tail]);

    return n;
}

void canUartStream(void) {
    int needFlush, finished;
    int16_t sent;
    canUartTxCallback_t *txCallback;
    void *txCallbackParam;
    int i;

    needFlush = 0;
    finished = 0;
    while (finished < CAN_UART_NUM) {
        // try to prevent overflows
        if ((canData.txTailLo + 1) == canData.txHeadLo)
            break;

        // round robin
        for (i = 0; i < CAN_UART_NUM; i++) {
            // anything to send?
            if (canUartData[i].txTail != canUartData[i].txHead) {
                sent = canUartTx(&canUartData[i]);
                needFlush = 1;

                // this stream finished?
                if (canUartData[i].txTail + sent == canUartData[i].txHead) {
                    canSendBulkFinish();
                    needFlush = 0;

                    txCallbackParam = canUartData[i].txCallbackParam;
                    txCallback = canUartData[i].txCallback;
                    canUartData[i].txTail = canUartData[i].txHead;

                    txCallback(txCallbackParam);
                }
                else {
                    canUartData[i].txTail += sent;
                }
            }
            else {
                finished++;
            }
        }
    }

    if (needFlush)
        canSendBulkFinish();
}

void canUartTxBuf(canUartStruct_t *ptr, uint8_t *buf, uint16_t n, canUartTxCallback_t *callback, void *param) {
    ptr->txBuf = buf;
    ptr->txHead = n;
    ptr->txTail = 0;
    ptr->txCallback = callback;
    ptr->txCallbackParam = param;
}

void canUartInit(void) {
    int i;

    for (i = 0; i < CAN_UART_NUM; i++) {
        if ((canUartData[i].node = canFindNode(CAN_TYPE_UART, i+1)) != 0) {
            commRegisterCanUart((void *)&canUartData[i], COMM_PORT_TYPE_CAN, i);
            canSetParamByName(canUartData[i].node->networkId, (uint8_t *)"BAUD_RATE", p[COMM_BAUD5+i]);
            canCommandArm(CAN_TT_NODE, canUartData[i].node->networkId);
        }
    }
}
