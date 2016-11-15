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

#ifndef _futaba_h
#define _futaba_h

#include "radio.h"

#define FUTABA_BAUD		100000
#define FUTABA_RXBUF_SIZE	32

#define FUTABA_START_CHAR	0x0f
#define FUTABA_END_CHAR		0b11000011

#define FUTABA_WAIT_SYNC	0x00
#define FUTABA_WAIT_DATA	0x01
#define FUTABA_WAIT_END		0x02

typedef struct {
    unsigned channel1  : 11;
    unsigned channel2  : 11;
    unsigned channel3  : 11;
    unsigned channel4  : 11;
    unsigned channel5  : 11;
    unsigned channel6  : 11;
    unsigned channel7  : 11;
    unsigned channel8  : 11;
    unsigned channel9  : 11;
    unsigned channel10 : 11;
    unsigned channel11 : 11;
    unsigned channel12 : 11;
    unsigned channel13 : 11;
    unsigned channel14 : 11;
    unsigned channel15 : 11;
    unsigned channel16 : 11;
}  __attribute__((packed)) futabaChannelStruct_t;

typedef struct {
    uint8_t state;
    uint8_t dataCount;
    uint8_t validFrame;
    uint32_t lastCharReceived;

    union {
	uint8_t rawBuf[23];
	futabaChannelStruct_t channels;
    } u;
} futabaStruct_t;

extern futabaStruct_t futabaData;

extern void futabaInit(radioInstance_t *r, USART_TypeDef *uart);
extern int futabaCharIn(radioInstance_t *r, uint8_t c);

#endif
