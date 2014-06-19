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

#ifndef _mlinkrx_h
#define _mlinkrx_h

#define MLINKRX_BAUD			115200
#define MLINKRX_RXBUF_SIZE		64

#define MLINKRX_FRAMESIZE		27
#define MLINKRX_CHANNELS		12

typedef struct {
    unsigned long lastCharReceived;
    int count;
    unsigned char framebuf[MLINKRX_FRAMESIZE];
    uint16_t crc_rx;
    uint16_t crc_calc;
} mlinkrxStruct_t;

extern void mlinkrxInit(radioInstance_t *r, USART_TypeDef *uart);
extern unsigned char mlinkrxCharIn(radioInstance_t *r, uint8_t c);

extern mlinkrxStruct_t mlinkrxData;

#endif
