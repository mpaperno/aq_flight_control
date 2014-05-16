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

#ifndef _spektrum_h
#define _spektrum_h

#define SPEKTRUM_UART		RC1_UART
#define SPEKTRUM_BAUD		115200
#define SPEKTRUM_RXBUF_SIZE	64

typedef struct {
    uint32_t lastCharReceived;
    uint8_t rawBuf[16];
    uint8_t state;
} spektrumStruct_t;

extern void spektrumInit(void);
extern uint8_t spektrumCharIn(int c);

extern spektrumStruct_t spektrumData;

#endif
