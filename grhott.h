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

    Copyright © 2013  Bill Nesbitt
*/

#ifndef grhott_h
#define grhott_h

#define GRHOTT_UART		RC1_UART
#define GRHOTT_BAUD             115200
#define GRHOTT_RXBUF_SIZE       40

#define GRHOTT_START_CHAR1      0xa8
#define GRHOTT_START_CHAR2      0x01

#define GRHOTT_WAIT_SYNC1       0x00
#define GRHOTT_WAIT_SYNC2       0x01
#define GRHOTT_WAIT_NB_CHANNEL  0x02
#define GRHOTT_WAIT_DATA        0x03

#define GRHOTT_MIN              8000
#define GRHOTT_MID              12000
#define POLYNOM                 0x11021

typedef struct {
    unsigned char state;
    unsigned char nb_channel;
    unsigned char dataCount;
    unsigned char rawBuf[32];
} grhottStruct_t;

extern grhottStruct_t grhottData;

extern void grhottInit(void);
extern int grhottCharIn(unsigned char c);

#endif
