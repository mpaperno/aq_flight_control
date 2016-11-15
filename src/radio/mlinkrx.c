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

// mlinkrx module written by Igor van Airde

#include "aq.h"
#include "radio.h"
#include "mlinkrx.h"
#include "aq_timer.h"
#include "imu.h"
#include "aq_mavlink.h"
#include "util.h"
#include "config.h"
#include <string.h>
#include <CoOS.h>

mlinkrxStruct_t mlinkrxData __attribute__((section(".ccm")));

// MPX CRC algorithm
uint16_t mpxCRC(uint16_t crc, uint8_t value) {
    uint8_t i;

    crc = crc ^ (int16_t)value<<8;

    for (i = 0; i < 8; i++) {
        if (crc & 0x8000)
            crc = crc << 1^0x1021;
        else
            crc = crc << 1;
    }

    return crc;
}

// process received chars
unsigned char mlinkrxCharIn(radioInstance_t *r, uint8_t c) {
    unsigned long receiveTime = timerMicros();
    int i;
    int val;

    // top of frame if it's been more than 7.5ms
    if (receiveTime - mlinkrxData.lastCharReceived > 7500)
        mlinkrxData.count = 0;

    mlinkrxData.lastCharReceived = timerMicros();

    // collect frame data to array
    mlinkrxData.framebuf[mlinkrxData.count] = c;
    mlinkrxData.count++;

    // if frame seems complete
    if (mlinkrxData.count == MLINKRX_FRAMESIZE) {
        // get ready to collect new frame data
        mlinkrxData.count = 0;
        // calculate CRC from frame
        mlinkrxData.crc_calc = 0;

        for (i = 0; i < (MLINKRX_FRAMESIZE - 2); i++)
            mlinkrxData.crc_calc = mpxCRC(mlinkrxData.crc_calc, mlinkrxData.framebuf[i]);

        // put together received CRC
        mlinkrxData.crc_rx = ((uint16_t)mlinkrxData.framebuf[MLINKRX_FRAMESIZE-2] << 8) + (uint16_t)mlinkrxData.framebuf[MLINKRX_FRAMESIZE-1];

        // if CRC matches
        if (mlinkrxData.crc_calc == mlinkrxData.crc_rx) {
            // decode channels
            for (i = 0; i < MLINKRX_CHANNELS; i++) {
                // delete upper 4 bytes from MSB, MLINK is 12bit
                mlinkrxData.framebuf[2*i+1] &= 0x0F;

                // MLINK is in range 0..4095
                // "(MSB+LSB / 2) - 1024" gives range of -1024 to +1023
                val = (((uint16_t)mlinkrxData.framebuf[2*i+1] << 8) + (uint16_t)mlinkrxData.framebuf[2*i+2]) >> 1;
                if (&RADIO_THROT == &r->channels[i])
                    r->channels[i] = val - 338;  // throttle
                else
                    r->channels[i] = val - 1024; // all other
            }
        }
        else {
            return 0; // CRC fail
        }
    }

    return 1;
}

void mlinkrxInit(radioInstance_t *r, USART_TypeDef *uart) {
    memset((void *)&mlinkrxData, 0, sizeof(mlinkrxData));

    mlinkrxData.count = 0;
    r->serialPort = serialOpen(uart, MLINKRX_BAUD, USART_HardwareFlowControl_None, MLINKRX_RXBUF_SIZE, 0);
}

