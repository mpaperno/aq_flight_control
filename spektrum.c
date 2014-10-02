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
#include "spektrum.h"
#include "aq_timer.h"
#include "imu.h"
#include "aq_mavlink.h"
#include "util.h"
#include "config.h"
#include <string.h>
#include <CoOS.h>

spektrumStruct_t spektrumData __attribute__((section(".ccm")));

uint8_t spektrumDecode(radioInstance_t *r) {
    uint8_t *buf = spektrumData.rawBuf;
    uint8_t ret = 0;
    int addr;
    int val;
    int i;

    // check over frame to make sure everything looks legit
    for (i = 0; i < 7; i++) {
        uint8_t *b = &buf[2 + i*2];

        // empty channel
        if (b[0] == 0xff && b[1] == 0xff)
            continue;

        // channels (other than first) should have MSB reset
        if (i != 0 && (b[0] & 0x80) != 0)
            return 0;
    }

    switch (r->radioType) {
        case RADIO_TYPE_SPEKTRUM11:
            r->errorCount = (buf[0]<<8) | buf[1];

            for (i = 0; i < 7; i++) {
                uint8_t *b = &buf[2 + i*2];

                // empty channel
                if (b[0] == 0xff && b[1] == 0xff)
                    continue;

                addr = (b[0]>>3) & 0x0f;
                val = ((b[0] & 0x07)<<8) | b[1];

                // throttle
                if (addr == (int)p[RADIO_THRO_CH])
                    val -= 338;
                else
                    val -= 1024;

                r->channels[addr] = val;

                ret = 1;
            }
            break;

        case RADIO_TYPE_SPEKTRUM10:
        case RADIO_TYPE_DELTANG:
            if (r->radioType == RADIO_TYPE_DELTANG) {
                uint8_t checksum = 0;

                for (i = 1; i < 16; i++)
                    checksum += buf[i];

                // checksum error?
                if (buf[0] != checksum)
                    return 0;

                r->errorCount = buf[1] & 0x1f;   // actually RSSI (5 bits)

                // valid data?
                if ((buf[1] & 0x80) == 0)
                    return 0;
            }
            else {
                r->errorCount = (buf[0]<<8) | buf[1];
            }

            for (i = 0; i < 7; i++) {
                uint8_t *b = &buf[2 + i*2];

                // empty channel
                if (b[0] == 0xff && b[1] == 0xff)
                    continue;

                addr = (b[0]>>2) & 0x0f;
                val = (((b[0] & 0x03)<<8) | b[1])<<1;

                // throttle
                if (addr == (int)p[RADIO_THRO_CH])
                    val -= 338;
                else
                    val -= 1024;

                r->channels[addr] = val;
            }

            ret = 1;
            break;

        default:
            break;
    }

    return ret;
}

uint8_t spektrumCharIn(radioInstance_t *r, int c) {
    uint32_t receiveTime = timerMicros();

    // top of frame if it's been more than 7.5ms
    if (receiveTime - spektrumData.lastCharReceived > 7500)
	spektrumData.state = 0;

    spektrumData.lastCharReceived = receiveTime;

    spektrumData.rawBuf[spektrumData.state] = c;

    if (++spektrumData.state == 16) {
	spektrumData.state = 0;
        return spektrumDecode(r);
    }
    else {
        return 0;
    }
}

void spektrumInit(radioInstance_t *r, USART_TypeDef *uart) {
    memset((void *)&spektrumData, 0, sizeof(spektrumData));

#ifdef RC1_DELTANG_BAUD
    if (r->radioType == RADIO_TYPE_DELTANG)
        r->serialPort = serialOpen(uart, RC1_DELTANG_BAUD, USART_HardwareFlowControl_None, SPEKTRUM_RXBUF_SIZE, 0);
    else
#endif
    r->serialPort = serialOpen(uart, SPEKTRUM_BAUD, USART_HardwareFlowControl_None, SPEKTRUM_RXBUF_SIZE, 0);
}
