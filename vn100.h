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

#ifndef _vn100_h
#define _vn100_h

#include "serial.h"
#include "digital.h"

#define VN100_BAUD		115200	// baud
#define VN100_SERIAL_TIMEOUT	50000	// us
#define VN100_CONFIG_FNAME	"VN100.TXT"
#define VN100_BUF_SIZE		512	// bytes

typedef struct {
    uint8_t command[4];
    float parameters[10];
    uint8_t checksum;
} __attribute__((packed)) vn100Command_t;

typedef struct {
    serialPort_t *serialPort;
    digitalPin *syncIn;
    digitalPin *nrst;
    digitalPin *reprgm;

    float doubleRates[3];
    float rates[3];
    float accs[3];
    float mags[3];

    char outBuf[192];
    char inBuf[192];

    volatile vn100Command_t sendBuf;
    uint8_t padding1[3];
    volatile vn100Command_t recvBuf;
    uint8_t padding2[3];

    volatile uint32_t solutionTime;
    volatile int32_t checksumErrors;
} vn100Struct_t;

extern vn100Struct_t vn100Data;

extern void vn100Init(void);
extern void vn100Sync(int polarity);
extern void vn100BootLoader(serialPort_t *s);
extern void vn100PassThrough(serialPort_t *s);
extern int8_t vn100ReadConfig(void);
extern int8_t vn100WriteConfig(void);

#endif

