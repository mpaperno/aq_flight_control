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
#include "config.h"
#include "can.h"
#include "canSensors.h"
#include "canUart.h"
#include "supervisor.h"
#include "imu.h"
#include "aq_timer.h"
#include "util.h"
#include "comm.h"

canStruct_t canData;

const char *canTypeStrings[] = {
    "NONE",
    "ESC",
    "SERVO",
    "SENSOR",
    "LED",
    "OSD",
    "UART",
    "HUB"
};

static uint32_t canGetSeqId(void) {
    uint32_t seqId;

    seqId = canData.seqId;
    canData.seqId = (canData.seqId + 1) & 0x3f;

    return seqId;
}

static int8_t canGetFreeMailbox(void) {
    int8_t mailbox;

    if ((CANx->TSR&CAN_TSR_TME0) == CAN_TSR_TME0)
	mailbox = 0;
    else if ((CANx->TSR&CAN_TSR_TME1) == CAN_TSR_TME1)
	mailbox = 1;
    else if ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)
	mailbox = 2;
    else
	mailbox = -1;

    return mailbox;
}

static int8_t _canSend(uint32_t id, uint8_t tid, uint8_t n, void *data) {
    uint32_t seqId;
    uint32_t *d = data;
    canTxBuf_t *txPtr;

    if ((id & CAN_LCC_MASK) < CAN_LCC_NORMAL)
        txPtr = &canData.txMsgsHi[canData.txHeadHi];
    else
        txPtr = &canData.txMsgsLo[canData.txHeadLo];

    seqId = canGetSeqId();

    txPtr->TIR = id | ((tid & 0x1f)<<9) | (seqId<<3) | CAN_Id_Extended;

    n = n & 0xf;
    txPtr->TDTR = n;

    if (n) {
        txPtr->TDLR = *d++;
        txPtr->TDHR = *d;
    }

    canData.responses[seqId] = 0;

    if ((id & CAN_LCC_MASK) < CAN_LCC_NORMAL)
        canData.txHeadHi = (canData.txHeadHi + 1) % CAN_BUF_SIZE;
    else
        canData.txHeadLo = (canData.txHeadLo + 1) % CAN_BUF_SIZE;

    return (int8_t)seqId;
}

void canSendBulkFinish(void) {
    // trigger transmit ISR
    NVIC->STIR = CAN_TX_IRQ;
}

static int8_t canSend(uint32_t id, uint8_t tid, uint8_t n, void *data) {
    uint8_t seqId;

    seqId = _canSend(id, tid, n, data);

    canSendBulkFinish();

    return (int8_t)seqId;
}

int8_t canSendBulk(uint32_t id, uint8_t tid, uint8_t n, void *data) {
    uint8_t seqId;

    seqId = _canSend(id, tid, n, data);

    return (int8_t)seqId;
}

static uint8_t *canSendWaitResponse(uint32_t extId, uint8_t tid, uint8_t n, uint8_t *data) {
    int16_t seqId;
    int timeout = CAN_TIMEOUT;

    seqId = canSend(extId, tid, n, data);

    if (seqId >= 0) {
	do {
	    yield(1);
	    timeout--;
	} while (timeout && canData.responses[seqId] == 0);

	if (timeout != 0)
	    return &canData.responseData[seqId*8];
    }

    return 0;
}

char *canGetVersion(uint8_t tid) {
    return (char *)canSendWaitResponse(CAN_LCC_NORMAL | CAN_TT_NODE | CAN_FID_GET | (CAN_DATA_VERSION<<19), tid, 0, 0);
}

uint8_t *canGetState(uint8_t tid) {
    return (uint8_t *)canSendWaitResponse(CAN_LCC_NORMAL | CAN_TT_NODE | CAN_FID_GET | (CAN_DATA_STATE<<19), tid, 0, 0);
}

float *canGetParam(uint8_t tid, uint16_t paramId) {
    return (float *)canSendWaitResponse(CAN_LCC_NORMAL | CAN_TT_NODE | CAN_FID_GET | (CAN_DATA_PARAM<<19), tid, 2, (uint8_t *)&paramId);
}

uint8_t *canSetRunMode(uint32_t tt, uint8_t tid, uint8_t mode) {
    return canSendWaitResponse(CAN_LCC_NORMAL | tt | CAN_FID_SET | (CAN_DATA_RUN_MODE<<19), tid, 1, &mode);
}

uint8_t *canSetGroup(uint8_t tid, uint8_t gid, uint8_t sgid) {
    uint8_t data[2];

    canData.nodes[tid-1].groupId = data[0] = gid;
    canData.nodes[tid-1].subgroupId = data[1] = sgid;
    return canSendWaitResponse(CAN_LCC_NORMAL | CAN_TT_NODE | CAN_FID_SET | (CAN_DATA_GROUP<<19), tid, 2, &canData.nodes[tid-1].groupId);
}

uint8_t *canSetParam(uint32_t tt, uint8_t tid, uint16_t paramId, float value) {
    uint32_t data[2];
    float *fPtr = (float *)&data[1];

    data[0] = paramId;
    *fPtr = value;

    return canSendWaitResponse(CAN_LCC_NORMAL | tt | CAN_FID_SET | (CAN_DATA_PARAM<<19), tid, 8, (uint8_t *)&data);
}

uint8_t *canCommandBeep(uint32_t tt, uint8_t tid, uint16_t freq, uint16_t dur) {
    uint16_t data[2];

    data[0] = freq;
    data[1] = dur;

    return canSendWaitResponse(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_BEEP<<19), tid, 4, (uint8_t *)&data);
}

void canCommandArm(uint32_t tt, uint8_t tid) {
   canSend(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_ARM<<19), tid, 0, 0);
}

void canCommandDisarm(uint32_t tt, uint8_t tid) {
    canSend(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_DISARM<<19), tid, 0, 0);
}

uint8_t *canCommandStart(uint32_t tt, uint8_t tid) {
    return canSendWaitResponse(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_START<<19), tid, 0, 0);
}

uint8_t *canCommandStop(uint32_t tt, uint8_t tid) {
    return canSendWaitResponse(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_STOP<<19), tid, 0, 0);
}

uint8_t *canCommandConfigWrite(uint32_t tt, uint8_t tid) {
    return canSendWaitResponse(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_CFG_WRITE<<19), tid, 0, 0);
}

uint8_t *canSetTelemetryValue(uint32_t tt, uint8_t tid, uint8_t index, uint8_t value) {
    uint8_t data[2];

    data[0] = index;
    data[1] = value;

    return canSendWaitResponse(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_TELEM_VALUE<<19), tid, 2, data);
}

uint8_t *canSetTelemetryRate(uint32_t tt, uint8_t tid, uint16_t rate) {
    uint16_t data;

    data = rate;

    return canSendWaitResponse(CAN_LCC_NORMAL | tt | CAN_FID_CMD | (CAN_CMD_TELEM_RATE<<19), tid, 2, (uint8_t *)&data);
}

void canCommandSetpoint16(uint8_t tid, uint8_t *data) {
    canSend(CAN_LCC_HIGH | CAN_TT_GROUP | CAN_FID_CMD | (CAN_CMD_SETPOINT16<<19), tid, 8, data);
}

void canCommandPos(uint8_t tid, float angle) {
    canSend(CAN_LCC_HIGH | CAN_TT_NODE | CAN_FID_CMD | (CAN_CMD_POS<<19), tid, 4, (uint8_t *)&angle);
}

static void canResetBus(void) {
    canSend(CAN_LCC_EXCEPTION | CAN_TT_GROUP | CAN_FID_RESET_BUS, 0, 0, 0);
}

static void canGrantAddr(CanRxMsg *rx) {
    uint32_t *uuidPtr = (uint32_t *)&rx->Data[0];
    uint32_t uuid;
    int i;

    uuid = *uuidPtr;

    // look for this UUID in our address table
    for (i = 0; i < canData.nextNodeSlot; i++)
	if (canData.nodes[i].uuid == uuid)
	    break;

    if (i == canData.nextNodeSlot)
	canData.nextNodeSlot++;

    if (i < (CAN_TID_MASK>>9)) {
	// store in table
	canData.nodes[i].networkId = i+1;
	canData.nodes[i].uuid = uuid;
	canData.nodes[i].type = rx->Data[4];
	canData.nodes[i].canId = rx->Data[5];

	// send groupId & subgroupId in bytes 5 & 6
	rx->Data[4] = canData.nodes[i].groupId;
	rx->Data[5] = canData.nodes[i].subgroupId;

	// respond
	canSend(CAN_LCC_NORMAL | CAN_TT_NODE | CAN_FID_GRANT_ADDR, i+1, 6, rx->Data);
    }
}

static void canProcessCmd(uint8_t doc, uint8_t canId, uint32_t *data, uint8_t n) {
    switch (doc) {
        case CAN_CMD_STREAM:
            canUartRxChar(canId, n, (uint8_t *)data);
            break;
    }
}

static void canProcessMessage(CanRxMsg *rx) {
    uint32_t id = rx->ExtId<<3;
    uint32_t *data = (uint32_t *)&rx->Data;
    uint16_t seqId = (id & CAN_SEQ_MASK)>>3;
    uint32_t *ptr = (uint32_t *)&canData.responseData[seqId*8];
    uint8_t sid = (id & CAN_SID_MASK)>>14;
    uint8_t doc = (id & CAN_DOC_MASK)>>19;

    switch (id & CAN_FID_MASK) {
	case CAN_FID_REQ_ADDR:
	    canGrantAddr(rx);
	    break;

	// telemetry callbacks
	case CAN_FID_TELEM:
	    if (canData.telemFuncs[canData.nodes[sid-1].type])
		canData.telemFuncs[canData.nodes[sid-1].type](canData.nodes[sid-1].canId, data);
	    break;

        case CAN_FID_CMD:
            canProcessCmd(doc, canData.nodes[sid-1].canId, data, rx->DLC);
            break;

	case CAN_FID_ACK:
	case CAN_FID_NACK:
	case CAN_FID_REPLY:
	    canData.responses[seqId] = (id & CAN_FID_MASK)>>25;
	    *ptr++ = *data++;
	    *ptr = *data;
	    break;
    }
}

int canCheckMessage(void) {
    CanRxMsg *rx;
    int ret = 0;

    if (canData.initialized) {
	while (canData.rxHead != canData.rxTail) {
	    rx = &canData.rxMsgs[canData.rxTail];

	    // ignore standard id messages
	    if (rx->IDE != CAN_Id_Standard) {
		canProcessMessage(rx);
		ret = 1;
	    }

	    canData.rxTail = (canData.rxTail + 1) % CAN_BUF_SIZE;
	}
    }

    return ret;
}

canNodes_t *canFindNode(uint8_t type, uint8_t canId) {
    int i;

    for (i = 0; i <= (CAN_TID_MASK>>9); i++)
	if (canData.nodes[i].type == type && canData.nodes[i].canId == canId)
	    return &canData.nodes[i];

    return 0;
}

void canTelemRegister(canTelemCallback_t *func, uint8_t type) {
    canData.telemFuncs[type] = func;
}

void canLowLevelInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    // Connect CAN pins to AF
    GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_RX_SOURCE, CAN_AF_PORT);
    GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_TX_SOURCE, CAN_AF_PORT);

    // Configure CAN RX and TX pins
    GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN | CAN_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

    // CAN register init
    CAN_DeInit(CANx);

    // CAN cell init
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;

    // CAN Baudrate = ~1 Mbps (CAN clocked at 42 MHz)
    CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
    CAN_InitStructure.CAN_Prescaler = 3;
    CAN_Init(CANx, &CAN_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = CAN_RX0_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = CAN_TX_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable FIFO 0 message pending Interrupt
    CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);

    // Enable TX FIFO empty Interrupt
    CAN_ITConfig(CANx, CAN_IT_TME, ENABLE);
}

void canDiscoverySummary(void) {
    int num;
    int i, j;

    for (i = 1; i < CAN_TYPE_NUM; i++) {
	num = 0;

	for (j = 0; j < (CAN_TID_MASK>>9)+1; j++)
	    if (canData.nodes[j].type == i)
		num++;

	if (num > 0)
	    AQ_PRINTF("CAN: Found %d node type %s\n", num, canTypeStrings[i]);
    }
}

void canInit(void) {
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    uint32_t micros;

    canLowLevelInit();

    // only packets targeted at us (bus master)
    if (CANx == CAN1)
	CAN_FilterInitStructure.CAN_FilterNumber = 0;
    else
	CAN_FilterInitStructure.CAN_FilterNumber = 14;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = CAN_TID_MASK;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    canResetBus();

    canData.initialized = 1;

    // wait 50ms for nodes to report in
    micros = timerMicros();
    while (timerMicros() - micros < 50000)
	// extend wait period if more nodes found
	if (canCheckMessage())
	    micros = timerMicros();

    canDiscoverySummary();
    canSensorsInit();
    canUartInit();
}

void CAN_RX0_HANDLER(void) {
    CAN_Receive(CANx, CAN_FIFO0, &canData.rxMsgs[canData.rxHead]);
    canData.rxHead = (canData.rxHead + 1) % CAN_BUF_SIZE;
}

static void canTxMsg(canTxBuf_t *tx, uint8_t mailbox) {
    CANx->sTxMailBox[mailbox].TDTR = tx->TDTR;

    CANx->sTxMailBox[mailbox].TDLR = tx->TDLR;
    CANx->sTxMailBox[mailbox].TDHR = tx->TDHR;

    // go
    CANx->sTxMailBox[mailbox].TIR = tx->TIR | 0x1;
}

void CAN_TX_HANDLER(void) {
    int8_t mailbox;

    CAN_ClearITPendingBit(CANx, CAN_IT_TME);

    // high priority - pick of any mailbox
    while (canData.txHeadHi != canData.txTailHi && (mailbox = canGetFreeMailbox()) >= 0) {
        canTxMsg(&canData.txMsgsHi[canData.txTailHi], mailbox);

        canData.txTailHi = (canData.txTailHi + 1) % CAN_BUF_SIZE;
    }

    // low priority - only if box 3 is available
    if (canData.txHeadLo != canData.txTailLo && ((CANx->TSR&CAN_TSR_TME2) == CAN_TSR_TME2)) {
        canTxMsg(&canData.txMsgsLo[canData.txTailLo], 2);

        canData.txTailLo = (canData.txTailLo + 1) % CAN_BUF_SIZE;
    }
}
