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

    Author: Frederic Guichard
*/

#include "radio.h"
#include "grhott.h"
#include "util.h"
#include <string.h>

grhottStruct_t grhottData __attribute__((section(".ccm")));

int grhottDecode(radioInstance_t *r) {
    int crc;
    int grhott_crc;
    int nb_channel;
    int num;
    int i;

    // How many channels are transmitted?
    switch ((grhottData.nb_channel - 2) / 2) {
/*
    Now set the CRC based on the detected channels
    CRC checksum for 12 channel output (MX20) - XMODEM CRC A801xx
    where xx is the number of channels in hex, for 12 enter A8010C in the calculation
*/
    case 6: // 6 channels (SUMDOF06 in transmitter)
	crc = 0x47CA;
	break;

    case 8: // 8 channels (SUMDOF08 in transmitter)
	crc = 0xA604;
	break;

    case 10: // 10 channels (SUMDOF10 in transmitter)
	crc = 0x8646;
	break;

    case 12: // 12 channels (SUMDOF12 in transmitter)
	crc = 0xE680;
	break;

    case 14: // 14 channels (SUMDOF14 in transmitter)
	crc = 0xC6C2;
	break;

    default:
	return -1;
    }

    // Step through the received frame to calculate the CRC
    for (num = 0; num < grhottData.nb_channel - 2; num++) {
	// Fetch byte from memory, XOR into CRC top byte
	crc = crc ^ (grhottData.rawBuf[num] << 8);

	// Prepare to rotate 8 bits
	for (i = 0; i < 8; i++) {
	    // b15 is set...
	    if (crc & 0x8000)
		crc = (crc << 1) ^ POLYNOM; // rotate and XOR with XMODEM polynomic
	    // b15 is clear...
	    else
		crc = crc << 1;		    // just rotate
	}
    }

    // received CRC is 2 bytes at the end of the data, data frame is number of channels * 2
    grhott_crc = (grhottData.rawBuf[grhottData.nb_channel - 2] << 8) + grhottData.rawBuf[grhottData.nb_channel - 1];

    if (crc == grhott_crc) {
	nb_channel = (grhottData.nb_channel - 2) / 2;
	r->channels[0] = (int16_t) (((grhottData.rawBuf[0] << 8) + grhottData.rawBuf[1] - GRHOTT_MIN) / 5);

	// Prepare to rotate 8 bits
	for (i = 1; i < nb_channel; i++)
	    r->channels[i] = (int16_t) (((grhottData.rawBuf[i * 2] << 8) + grhottData.rawBuf[i * 2 + 1] - GRHOTT_MID) / 5);

	return 1;
    }
    else {
	return -1;
    }
}

int grhottCharIn(radioInstance_t *r, uint8_t c) {
    switch (grhottData.state) {
    case GRHOTT_WAIT_SYNC1:
	if (c == GRHOTT_START_CHAR1) {
	    grhottData.state = GRHOTT_WAIT_SYNC2;
	    grhottData.dataCount = 0;
	    grhottData.nb_channel = 0;
	}
	break;

    case GRHOTT_WAIT_SYNC2:
	if (c == GRHOTT_START_CHAR2) {
	    grhottData.state = GRHOTT_WAIT_NB_CHANNEL;
	}
	break;

    case GRHOTT_WAIT_NB_CHANNEL:
	grhottData.nb_channel = c * 2 + 2;
	grhottData.state = GRHOTT_WAIT_DATA;
	grhottData.dataCount = 0;
	break;

    case GRHOTT_WAIT_DATA:
	grhottData.rawBuf[grhottData.dataCount++] = c;
	if (grhottData.dataCount == grhottData.nb_channel) {
	    grhottData.state = GRHOTT_WAIT_SYNC1;
	    return grhottDecode(r);
	}
	break;
    }

    return 0;
}

void grhottInit(radioInstance_t *r, USART_TypeDef *uart) {
    memset((void *) &grhottData, 0, sizeof(grhottData));

    r->serialPort = serialOpen(uart, GRHOTT_BAUD, USART_HardwareFlowControl_None, GRHOTT_RXBUF_SIZE, 0);

    grhottData.state = GRHOTT_WAIT_SYNC1;
}
