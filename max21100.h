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

#ifndef _max21100_h
#define _max21100_h

#include "spi.h"
#include "util.h"

#define MAX21100_SPI_BAUD           SPI_BaudRatePrescaler_4	// 10.5 MHz

#define MAX21100_READ_BIT           (0x01<<7)
#define MAX21100_WRITE_BIT          (0x00<<7)
#define MAX21100_INCR               (0x00<<6)
#define MAX21100_NO_INCR            (0x01<<6)

#define MAX21100_BYTES              21
#define MAX21100_SLOT_SIZE          ((MAX21100_BYTES+sizeof(int)-1) / sizeof(int) * sizeof(int))

#ifdef USE_QUATOS
    #define MAX21100_SLOTS          80						    // 100Hz bandwidth
    #define MAX21100_DRATE_SLOTS	(MAX21100_SLOTS * 100.0f * DIMU_INNER_DT * 2.0f) // variable
#else
    #define MAX21100_SLOTS          80						    // 100Hz bandwidth
    #define MAX21100_DRATE_SLOTS    40						    // 200Hz
#endif

typedef struct {
    utilFilter_t tempFilter;
    spiClient_t *spi;
    volatile uint32_t spiFlag;
    volatile uint8_t rxBuf[MAX21100_SLOT_SIZE*MAX21100_SLOTS];
    volatile uint8_t slot;
    float rawTemp;
    float rawAcc[3];
    float rawGyo[3];
    float dRateRawGyo[3];
    float gyoOffset[3];
    volatile float acc[3];
    volatile float temp;
    volatile float gyo[3];
    volatile float dRateGyo[3];
    volatile uint32_t lastUpdate;
    float accSign[3];
    float gyoSign[3];
    uint8_t readReg;
    uint8_t enabled;
} max21100Struct_t;

extern max21100Struct_t max21100Data;

extern void max21100PreInit(void);
extern void max21100Init(void);
extern void mpu6600InitialBias(void);
extern void max21100Decode(void);
extern void max21100DrateDecode(void);
extern void max21100Enable(void);
extern void max21100Disable(void);
extern void max21100StartTransfer(void);

#endif
