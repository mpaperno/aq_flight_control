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
#include "radio.h"
#include "spektrum.h"
#include "notice.h"
#include "aq_timer.h"
#include "imu.h"
#include "aq_mavlink.h"
#include "util.h"
#include "config.h"
#include <string.h>
#include <CoOS.h>

spektrumStruct_t spektrumData;

OS_STK spektrumTaskStack[TASK_STACK_SIZE];

void spektrumDecode(void) {
    int addr, val;

    // 10 bit
    if (p[RADIO_TYPE] == 1) {
	addr = (spektrumData.rawBuf[0]>>2) & 0x0f;
	val = (((spektrumData.rawBuf[0] & 0x03)<<8) | spektrumData.rawBuf[1])<<1;
    }
    // 11 bit
    else {
	addr = (spektrumData.rawBuf[0]>>3) & 0x0f;
	val = ((spektrumData.rawBuf[0] & 0x07)<<8) | spektrumData.rawBuf[1];
    }

    // throttle
    if (addr == 0)
	val -= 338;
    else
	val -= 1024;

    radioData.channels[addr] = val;
}

unsigned char spektrumCharIn(int c) {
    unsigned long receiveTime = timerMicros();

    // top of frame if it's been more than 7.5ms
    if (receiveTime - spektrumData.lastCharReceived > 7500)
	    spektrumData.state = SPEKTRUM_ERR_COUNT1;

    spektrumData.lastCharReceived = timerMicros();

    switch (spektrumData.state) {
    case SPEKTRUM_ERR_COUNT1:
	spektrumData.rawBuf[0] = c;
	spektrumData.state = SPEKTRUM_ERR_COUNT2;
	break;

    case SPEKTRUM_ERR_COUNT2:
	radioData.errorCount = (spektrumData.rawBuf[0]<<8) | c;
	spektrumData.state = SPEKTRUM_CHANNEL1;
	spektrumData.channelCount = 0;
	break;

    case SPEKTRUM_CHANNEL1:
	spektrumData.rawBuf[0] = c;
	spektrumData.state = SPEKTRUM_CHANNEL2;
	break;

    case SPEKTRUM_CHANNEL2:
	spektrumData.rawBuf[1] = c;
	spektrumDecode();
	spektrumData.channelCount++;
	if (spektrumData.channelCount == (p[RADIO_TYPE] == 1 ? 7 : 6)) {
	    spektrumData.state = SPEKTRUM_WAIT_SYNC1;
	    radioData.frameCount++;
	}
	else {
	    spektrumData.state = SPEKTRUM_CHANNEL1;
	}
	return 1;

	break;
    }

    return 0;
}

void spektrumInit(void) {
    radioData.serialPort = serialOpen(SPEKTRUM_UART, SPEKTRUM_BAUD, USART_HardwareFlowControl_None, SPEKTRUM_RXBUF_SIZE, 0);
}

