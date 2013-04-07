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

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#include "ms5611.h"
#include "imu.h"
#include "aq_timer.h"
#include <math.h>

ms5611Struct_t ms5611Data;

void ms5611SendCommand(uint8_t cmd) {
    static uint8_t txBuf[1];
    static uint8_t rxBuf[1];

    txBuf[0] = cmd;
    ms5611Data.spiFlag = 0;
    spiTransaction(ms5611Data.spi, rxBuf, txBuf, 1);

    while (!ms5611Data.spiFlag)
	;
}

uint32_t ms5611Read(uint8_t reg, uint8_t n) {
    static uint8_t rxBuf[4];
    static uint8_t txBuf[4];
    uint32_t val;
    int i;

    txBuf[0] = reg;

    ms5611Data.spiFlag = 0;
    spiTransaction(ms5611Data.spi, rxBuf, txBuf, n+1);

    while (!ms5611Data.spiFlag)
	;

    val = 0;
    for (i = 0; i < n; i++)
	val = (val<<8) | rxBuf[1+i];

    return val;
}

void ms5611InitialBias(void) {
    uint32_t lastUpdate = ms5611Data.lastUpdate;
    float tempSum;
    int i;

    tempSum = 0.0f;

    for (i = 0; i < 10; i++) {
	while (lastUpdate == ms5611Data.lastUpdate)
	    delay(1);
	lastUpdate = ms5611Data.lastUpdate;

	tempSum += ms5611Data.rawTemp;
    }

    ms5611Data.temp = tempSum / 10.0f;
    utilFilterReset(&ms5611Data.tempFilter, ms5611Data.temp);
}

void ms5611Callback(int unused) {
    static uint8_t rxBuf[1];

    if (ms5611Data.enabled) {
	switch (ms5611Data.step) {
	    case 0:
		// temp conversion
		spiTransaction(ms5611Data.spi, &rxBuf[0], &ms5611Data.startTempConv, 1);
//		dIMUSetAlarm1(9040, ms5611Callback, 0);
		dIMUSetAlarm1(600, ms5611Callback, 0);
		break;

	    case 1:
		ms5611Data.slot = (ms5611Data.slot + 1) % MS5611_SLOTS;
		break;

	    case 2:
		// temp result
		spiTransaction(ms5611Data.spi, &ms5611Data.d2[ms5611Data.slot], &ms5611Data.adcRead, 4);
		break;

	    case 3:
		// pres conversion
		spiTransaction(ms5611Data.spi, &rxBuf[0], &ms5611Data.startPresConv, 1);
		dIMUSetAlarm1(9040, ms5611Callback, 0);
//		dIMUSetAlarm1(600, ms5611Callback, 0);
		break;

	    case 4:
		break;

	    case 5:
		// press result
		spiTransaction(ms5611Data.spi, &ms5611Data.d1[ms5611Data.slot], &ms5611Data.adcRead, 4);
		break;
	}

	ms5611Data.step = (ms5611Data.step + 1) % 6;
    }
}

float ms5611PresToAlt(float pressure) {
    return (1.0f -  powf(pressure / 101325.0f, 0.19f)) * (1.0f / 22.558e-6f);
}

void ms5611Decode(void) {
    uint32_t rawTemp, rawPres;
    uint8_t *ptr;
    int32_t dT;
    int32_t temp;
    int64_t off;
    int64_t sens;
//    int32_t p;
    int divisor;
    int i;

    rawTemp = 0;
    rawPres = 0;
    divisor = MS5611_SLOTS;
    for (i = 0; i < MS5611_SLOTS; i++) {
	// check if we are in the middle of a transaction for this slot
	if (i == ms5611Data.slot && ms5611Data.spiFlag == 0) {
	    divisor--;
	}
	else {
	    ptr = (uint8_t *)&ms5611Data.d2[i];
	    rawTemp += (ptr[1]<<16 | ptr[2]<<8 | ptr[3]);

	    ptr = (uint8_t *)&ms5611Data.d1[i];
	    rawPres += (ptr[1]<<16 | ptr[2]<<8 | ptr[3]);
	}
    }

    // temperature
    dT = rawTemp / divisor - (ms5611Data.p[5]<<8);
    temp = (int64_t)dT * ms5611Data.p[6] / 8388608 + 2000;
    ms5611Data.rawTemp = temp / 100.0f;

    ms5611Data.temp = utilFilter(&ms5611Data.tempFilter, ms5611Data.rawTemp);

    // pressure
    off = ((int64_t)ms5611Data.p[2]<<16) + (((int64_t)dT * ms5611Data.p[4])>>7);
    if (off < -8589672450)
	off = -8589672450;
    else if (off > 12884705280)
	off = 12884705280;

    sens = ((int64_t)ms5611Data.p[1]<<15) + (((int64_t)dT * ms5611Data.p[3])>>8);
    if (sens < -4294836225)
	sens = -4294836225;
    else if (sens > 6442352640)
	sens = 6442352640;

//    p = ((int64_t)rawPres * sens / 2097152 - off)>>15;
    ms5611Data.pres = ((int64_t)rawPres * sens / 2097152 / divisor - off) * (1.0f / 32768.0f);

//    ms5611Data.alt = ms5611PresToAlt(ms5611Data.pres);

    ms5611Data.lastUpdate = timerMicros();
}

inline void ms5611Enable(void) {
    if (!ms5611Data.enabled) {
	ms5611Data.enabled = 1;
	ms5611Data.step = 0;
	ms5611Callback(0);
    }
}

inline void ms5611Disable(void) {
    ms5611Data.enabled = 0;
}

void ms5611PreInit(void) {
    ms5611Data.spi = spiClientInit(MS5611_SPI, MS5611_SPI_BAUD, MS5611_CS_PORT, MS5611_CS_PIN, &ms5611Data.spiFlag, 0);
}

// code from MS's AN520
uint8_t ms5611CheckSum(void) {
    uint16_t crc_read;
    uint16_t n_rem;
    uint8_t n_bit;
    int i;

    crc_read = ms5611Data.p[7];
    ms5611Data.p[7] = (0xff00 & crc_read);
    n_rem = 0;

    for (i = 0; i < 16; i++) {
	if ((i % 2) == 1)
	    n_rem ^= (uint16_t)(ms5611Data.p[i>>1] & 0x00ff);
	else
	    n_rem ^= (uint16_t)(ms5611Data.p[i>>1]>>8);

	for (n_bit = 8; n_bit > 0; n_bit--) {
	    if (n_rem & 0x8000)
		n_rem = (n_rem<<1) ^ 0x3000;
	    else
		n_rem = (n_rem<<1);
	}
    }

    n_rem = 0x000f & (n_rem>>12);
    ms5611Data.p[7] = crc_read;

    return (n_rem ^ 0x00);
}

void ms5611Init(void) {
    int i;

    utilFilterInit(&ms5611Data.tempFilter, (1.0f / 13.0f), DIMU_TEMP_TAU, IMU_ROOM_TEMP);

    do {
	// reset
	ms5611SendCommand(0x1e);
	delay(5);

	// read coefficients
	for (i = 0; i < 8; i++) {
	    while (ms5611Data.p[i] == 0x0000 || ms5611Data.p[i] == 0xffff) {
		ms5611Data.p[i] = ms5611Read(0xa0 + i*2, 2);
		delay(1);
	    }
	}
    }
    while (ms5611CheckSum() != (ms5611Data.p[7] & 0x000f));

    ms5611Data.adcRead = 0x00;

//    ms5611Data.startTempConv = 0x58;    // 4096 OSR
    ms5611Data.startTempConv = 0x50;    // 256 OSR
    ms5611Data.startPresConv = 0x48;	// 4096 OSR
//    ms5611Data.startPresConv = 0x40;	// 256 OSR

    spiChangeCallback(ms5611Data.spi, ms5611Callback);
}