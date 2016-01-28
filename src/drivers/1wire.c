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

#include "1wire.h"
#include "aq_timer.h"
#include "digital.h"
#include "util.h"
#include <CoOS.h>
#include <string.h>

owStruct_t owData __attribute__((section(".ccm")));

void owPinOut(void) {
    owHi();
    owData.owGPIO.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_Init(owData.port, &owData.owGPIO);

    digitalHi(owData.dp);
}

void owPinIn(void) {
    owData.owGPIO.GPIO_Mode = GPIO_Mode_IN;
    GPIO_Init(owData.port, &owData.owGPIO);
    digitalLo(owData.dp);
}

uint8_t owTransaction(int8_t write, int8_t read) {
    owData.writeBytes = write;
    owData.readBytes = read;

    owTransactionISR(OW_STATE_0);
    while (owData.status == OW_STATUS_BUSY)
	;

    return owData.status;
}

void owInit(GPIO_TypeDef* port, const uint16_t pin) {
    memset((void *)&owData, 0, sizeof(owData));

    owData.dp = digitalInit(port, pin, 0);
    owData.port = port;
    owData.pin = pin;

    GPIO_StructInit(&owData.owGPIO);
    owData.owGPIO.GPIO_Pin = pin;
    owData.owGPIO.GPIO_Speed = GPIO_Speed_50MHz;
    owData.owGPIO.GPIO_Mode = GPIO_Mode_OUT;
    owData.owGPIO.GPIO_PuPd = GPIO_PuPd_UP;

    owPinOut();
    yield(1);

    owData.status = OW_STATUS_UNINITIALIZED;
}

void owResetISR(int state) {
    switch (state) {
	case OW_STATE_0:
	    owLo();
	    timerSetAlarm1(490, owResetISR, OW_STATE_1);
	break;

	case OW_STATE_1:
	    owPinIn();
	    timerSetAlarm1(60, owResetISR, OW_STATE_2);
	break;

	case OW_STATE_2:
	    owData.present = !(owRead() & 0x01);
	    timerSetAlarm1(60, owResetISR, OW_STATE_3);
	break;

	case OW_STATE_3:
	    owPinOut();
	break;
    }
}

void owReadBitISR(int state) {
    switch (state) {
	case OW_STATE_0:
	    owLo();
	    timerSetAlarm1(2, owReadBitISR, OW_STATE_1);
	break;

	case OW_STATE_1:
	    owPinIn();
	    timerSetAlarm1(10, owReadBitISR, OW_STATE_2);
	break;

	case OW_STATE_2:
	    owData.bitValue = owRead();
	    timerSetAlarm1(25, owReadBitISR, OW_STATE_3);
	break;

	case OW_STATE_3:
	    owPinOut();
	break;
    }
}

void owWriteBit1ISR(int state) {
    switch (state) {
	case OW_STATE_0:
	    owLo();
	    timerSetAlarm1(2, owWriteBit1ISR, OW_STATE_1);
	break;

	case OW_STATE_1:
	    owHi();
	    timerSetAlarm1(60, owWriteBit1ISR, OW_STATE_2);
	break;

	case OW_STATE_2:
	break;
    }
}

void owWriteBit0ISR(int state) {
    switch (state) {
	case OW_STATE_0:
	    owLo();
	    timerSetAlarm1(60, owWriteBit0ISR, OW_STATE_1);
	break;

	case OW_STATE_1:
	    owHi();
	break;
    }
}

void owReadByteISR(int bit) {
    if (bit < 0) {
	owData.byteValue = 0;
	owReadBitISR(OW_STATE_0);
	timerSetAlarm2(65, owReadByteISR, bit+1);
    }
    else {
	owData.byteValue |= (owData.bitValue<<bit);
	if (bit < 7) {
	    owReadBitISR(OW_STATE_0);
	    timerSetAlarm2(65, owReadByteISR, bit+1);
	}
    }
}

void owWriteByteISR(int bit) {
    bit++;
    if (bit < 8) {
	if ((owData.byteValue>>bit) & 0x01)
	    owWriteBit1ISR(OW_STATE_0);
	else
	    owWriteBit0ISR(OW_STATE_0);

	timerSetAlarm2(65, owWriteByteISR, bit);
    }
}

void owTransactionISR(int state) {
    if (state == OW_STATE_0) {
	owData.present = 0;
	owData.status = OW_STATUS_BUSY;
	owResetISR(OW_STATE_0);
	timerSetAlarm3(1000, owTransactionISR, OW_STATE_1);
    }
    else if (state == OW_STATE_1) {
	if (!owData.present) {
	    owData.status = OW_STATUS_NO_PRESENSE;
	    return;
	}
	else {
	    owData.byteValue = OW_ROM_SKIP;
	    owData.ptr = owData.buf;
	    owWriteByteISR(-1);
	    timerSetAlarm3(600, owTransactionISR, OW_STATE_2);
	}
    }
    else if (owData.writeBytes > 0) {
	owData.byteValue = *owData.ptr++;
	owData.writeBytes--;
	owWriteByteISR(-1);
	timerSetAlarm3(600, owTransactionISR, OW_STATE_2);

	if (owData.writeBytes == 0)
	    owData.ptr = owData.buf;
    }
    else if (owData.readBytes > -1) {
	owData.readBytes--;
	if (state == OW_STATE_3) {
	    *owData.ptr++ = owData.byteValue;
	}
	if (owData.readBytes > -1) {
	    owReadByteISR(-1);
	    timerSetAlarm3(600, owTransactionISR, OW_STATE_3);
	}
	else {
	    owData.status = OW_STATUS_COMPLETE;
	}
    }
    else {
	owData.status = OW_STATUS_COMPLETE;
    }
}
