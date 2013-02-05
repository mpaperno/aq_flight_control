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

#ifndef _spektrum_h
#define _spektrum_h

#define SPEKTRUM_BAUD		115200
#define SPEKTRUM_RXBUF_SIZE	64

#define SPEKTRUM_WAIT_SYNC1	0x00
#define SPEKTRUM_WAIT_SYNC2	0x01
#define SPEKTRUM_ERR_COUNT1	0x02
#define SPEKTRUM_ERR_COUNT2	0x03
#define SPEKTRUM_CHANNEL1	0x04
#define SPEKTRUM_CHANNEL2	0x05

typedef struct {
    unsigned long lastCharReceived;
    unsigned char state;
    unsigned char channelCount;
    unsigned char rawBuf[2];
} spektrumStruct_t;

extern void spektrumInit(void);
extern unsigned char spektrumCharIn(int c);

extern spektrumStruct_t spektrumData;

#endif
