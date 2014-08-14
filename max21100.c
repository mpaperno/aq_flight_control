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

#include "config.h"
#if defined(HAS_DIGITAL_IMU) && defined(DIMU_HAVE_MAX21100)
#include "imu.h"
#include "max21100.h"
#include "aq_timer.h"
#include "util.h"
#include "config.h"
#include "ext_irq.h"
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

max21100Struct_t max21100Data;

static void max21100TransferComplete(int unused) {
    max21100Data.slot = (max21100Data.slot + 1) % MAX21100_SLOTS;
}

void max21100InitialBias(void) {
    uint32_t lastUpdate = max21100Data.lastUpdate;
    float tempSum;
    int i;

    tempSum = 0.0f;

    for (i = 0; i < 50; i++) {
        while (lastUpdate == max21100Data.lastUpdate)
        delay(1);
        lastUpdate = max21100Data.lastUpdate;

        tempSum += max21100Data.rawTemp;
    }

    max21100Data.temp = tempSum / 50.0f;
    utilFilterReset(&max21100Data.tempFilter, max21100Data.temp);
}

static void max21100CalibAcc(float *in, volatile float *out) {
    float a, b, c;
    float x, y, z;

    // bias
    a = -(in[0] + p[IMU_ACC_BIAS_X] + p[IMU_ACC_BIAS1_X]*dImuData.dTemp + p[IMU_ACC_BIAS2_X]*dImuData.dTemp2 + p[IMU_ACC_BIAS3_X]*dImuData.dTemp3);
    b = +(in[1] + p[IMU_ACC_BIAS_Y] + p[IMU_ACC_BIAS1_Y]*dImuData.dTemp + p[IMU_ACC_BIAS2_Y]*dImuData.dTemp2 + p[IMU_ACC_BIAS3_X]*dImuData.dTemp3);
    c = -(in[2] + p[IMU_ACC_BIAS_Z] + p[IMU_ACC_BIAS1_Z]*dImuData.dTemp + p[IMU_ACC_BIAS2_Z]*dImuData.dTemp2 + p[IMU_ACC_BIAS3_X]*dImuData.dTemp3);

    // misalignment
    x = a + b*p[IMU_ACC_ALGN_XY] + c*p[IMU_ACC_ALGN_XZ];
    y = a*p[IMU_ACC_ALGN_YX] + b + c*p[IMU_ACC_ALGN_YZ];
    z = a*p[IMU_ACC_ALGN_ZX] + b*p[IMU_ACC_ALGN_ZY] + c;

    // scale
    x /= (p[IMU_ACC_SCAL_X] + p[IMU_ACC_SCAL1_X]*dImuData.dTemp + p[IMU_ACC_SCAL2_X]*dImuData.dTemp2 + p[IMU_ACC_SCAL3_X]*dImuData.dTemp3);
    y /= (p[IMU_ACC_SCAL_Y] + p[IMU_ACC_SCAL1_Y]*dImuData.dTemp + p[IMU_ACC_SCAL2_Y]*dImuData.dTemp2 + p[IMU_ACC_SCAL3_Y]*dImuData.dTemp3);
    z /= (p[IMU_ACC_SCAL_Z] + p[IMU_ACC_SCAL1_Z]*dImuData.dTemp + p[IMU_ACC_SCAL2_Z]*dImuData.dTemp2 + p[IMU_ACC_SCAL3_Z]*dImuData.dTemp3);

    // IMU rotation
    out[0] = x * imuData.cosRot - y * imuData.sinRot;
    out[1] = y * imuData.cosRot + x * imuData.sinRot;
    out[2] = z;
}

static void max21100ScaleGyo(int32_t *in, float *out, float divisor) {
    float scale;

    scale = 1.0f / (60000 / (MAX21100_GYO_SCALE * 2.0f)) * divisor * DEG_TO_RAD;

    out[0] = max21100Data.gyoSign[0] * DIMU_ORIENT_GYO_X * scale;
    out[1] = max21100Data.gyoSign[1] * DIMU_ORIENT_GYO_Y * scale;
    out[2] = max21100Data.gyoSign[2] * DIMU_ORIENT_GYO_Z * scale;
}

static void max21100ScaleAcc(int32_t *in, float *out, float divisor) {
    float scale;

    scale = 1.0f / (60000 / (MAX21100_ACC_SCALE * 2.0f)) * divisor * GRAVITY;

    out[0] = max21100Data.accSign[0] * DIMU_ORIENT_ACC_X * scale;
    out[1] = max21100Data.accSign[1] * DIMU_ORIENT_ACC_Y * scale;
    out[2] = max21100Data.accSign[2] * DIMU_ORIENT_ACC_Z * scale;
}

static void max21100CalibGyo(float *in, volatile float *out) {
    float a, b, c;
    float x, y, z;

    // bias
    a = +(in[0] + max21100Data.gyoOffset[0] + p[IMU_GYO_BIAS_X] + p[IMU_GYO_BIAS1_X]*dImuData.dTemp + p[IMU_GYO_BIAS2_X]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_X]*dImuData.dTemp3);
    b = -(in[1] + max21100Data.gyoOffset[1] + p[IMU_GYO_BIAS_Y] + p[IMU_GYO_BIAS1_Y]*dImuData.dTemp + p[IMU_GYO_BIAS2_Y]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_Y]*dImuData.dTemp3);
    c = -(in[2] + max21100Data.gyoOffset[2] + p[IMU_GYO_BIAS_Z] + p[IMU_GYO_BIAS1_Z]*dImuData.dTemp + p[IMU_GYO_BIAS2_Z]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_Z]*dImuData.dTemp3);

    // misalignment
    x = a + b*p[IMU_GYO_ALGN_XY] + c*p[IMU_GYO_ALGN_XZ];
    y = a*p[IMU_GYO_ALGN_YX] + b + c*p[IMU_GYO_ALGN_YZ];
    z = a*p[IMU_GYO_ALGN_ZX] + b*p[IMU_GYO_ALGN_ZY] + c;

    // scale
    x /= p[IMU_GYO_SCAL_X];
    y /= p[IMU_GYO_SCAL_Y];
    z /= p[IMU_GYO_SCAL_Z];

    // IMU rotation
    out[0] = x * imuData.cosRot - y * imuData.sinRot;
    out[1] = y * imuData.cosRot + x * imuData.sinRot;
    out[2] = z;
}

void max21100DrateDecode(void) {
    volatile uint8_t *d = max21100Data.rxBuf;
    int32_t gyo[3];
    float divisor;
    int s, i;

    if (max21100Data.enabled) {
        for (i = 0; i < 3; i++)
            gyo[i] = 0;

        divisor = (float)MAX21100_DRATE_SLOTS;
        s = max21100Data.slot - 1;
        if (s < 0)
            s = MAX21100_SLOTS - 1;

        for (i = 0; i < MAX21100_DRATE_SLOTS; i++) {
            int j = s*MAX21100_SLOT_SIZE;

            // check if we are in the middle of a transaction for this slot
            if (s == max21100Data.slot && max21100Data.spiFlag == 0) {
                divisor -= 1.0f;
            }
            else {
                gyo[0] += (int16_t)__rev16(*(uint16_t *)&d[j+1]);
                gyo[1] += (int16_t)__rev16(*(uint16_t *)&d[j+3]);
                gyo[2] += (int16_t)__rev16(*(uint16_t *)&d[j+5]);
            }

            if (--s < 0)
                s = MAX21100_SLOTS - 1;
        }

        divisor = 1.0f / divisor;

        max21100ScaleGyo(gyo, max21100Data.dRateRawGyo, divisor);
        max21100CalibGyo(max21100Data.dRateRawGyo, max21100Data.dRateGyo);
    }
}

void max21100Decode(void) {
    volatile uint8_t *d = max21100Data.rxBuf;
    int32_t acc[3], temp, gyo[3];
    float divisor;
    int i;

    for (i = 0; i < 3; i++) {
        acc[i] = 0;
        gyo[i] = 0;
    }
    temp = 0;

    divisor = (float)MAX21100_SLOTS;
    for (i = 0; i < MAX21100_SLOTS; i++) {
        int j = i*MAX21100_SLOT_SIZE;

        // check if we are in the middle of a transaction for this slot
        if (i == max21100Data.slot && max21100Data.spiFlag == 0) {
            divisor -= 1.0f;
        }
        else {
            gyo[0] += (int16_t)__rev16(*(uint16_t *)&d[j+1]);
            gyo[1] += (int16_t)__rev16(*(uint16_t *)&d[j+3]);
            gyo[2] += (int16_t)__rev16(*(uint16_t *)&d[j+5]);

            acc[0] += (int16_t)__rev16(*(uint16_t *)&d[j+7]);
            acc[1] += (int16_t)__rev16(*(uint16_t *)&d[j+9]);
            acc[2] += (int16_t)__rev16(*(uint16_t *)&d[j+11]);

            temp += (int16_t)__rev16(*(uint16_t *)&d[j+19]);
        }
    }

    divisor = 1.0f / divisor;

    max21100Data.rawTemp = temp * divisor * (1.0f / 255.0f);
    max21100Data.temp = utilFilter(&max21100Data.tempFilter, max21100Data.rawTemp);

    max21100ScaleAcc(acc, max21100Data.rawAcc, divisor);
    max21100CalibAcc(max21100Data.rawAcc, max21100Data.acc);

    max21100ScaleGyo(gyo, max21100Data.rawGyo, divisor);
    max21100CalibGyo(max21100Data.rawGyo, max21100Data.gyo);

    max21100Data.lastUpdate = timerMicros();
}

static uint32_t max21100RxBuf;
static uint32_t max21100TxBuf;

static uint8_t max21100GetReg(uint8_t reg) {
    ((uint8_t *)&max21100TxBuf)[0] = MAX21100_READ_BIT | reg;

    max21100Data.spiFlag = 0;
    spiTransaction(max21100Data.spi, &max21100RxBuf, &max21100TxBuf, 2);

    while (!max21100Data.spiFlag)
        ;

    return ((uint8_t *)&max21100RxBuf)[1];
}

static void max21100SetReg(uint8_t reg, uint8_t val) {
    ((uint8_t *)&max21100TxBuf)[0] = MAX21100_WRITE_BIT | reg;
    ((uint8_t *)&max21100TxBuf)[1] = val;

    max21100Data.spiFlag = 0;
    spiTransaction(max21100Data.spi, &max21100RxBuf, &max21100TxBuf, 2);

    while (!max21100Data.spiFlag)
        ;
}

static void max21100ReliablySetReg(uint8_t reg, uint8_t val) {
    do {
        delay(10);
        max21100SetReg(reg, val);
        delay(10);
    } while (max21100GetReg(reg) != val);
}

void max21100IntHandler(void) {
    if (max21100Data.enabled)
        spiTransaction(max21100Data.spi, &max21100Data.rxBuf[max21100Data.slot*MAX21100_SLOT_SIZE], &max21100Data.readReg, MAX21100_BYTES);
}

inline void max21100Enable(void) {
    max21100Data.enabled = 1;

    max21100IntHandler();
}

inline void max21100Disable(void) {
    max21100Data.enabled = 0;
}

void max21100PreInit(void) {
    max21100Data.spi = spiClientInit(DIMU_MAX21100_SPI, MAX21100_SPI_BAUD, 0, DIMU_MAX21100_CS_PORT, DIMU_MAX21100_CS_PIN, &max21100Data.spiFlag, 0);
}

void max21100Init(void) {
    switch ((int)p[IMU_FLIP]) {
        case 1:
            max21100Data.accSign[0] =  1.0f;
            max21100Data.accSign[1] = -1.0f;
            max21100Data.accSign[2] = -1.0f;
            max21100Data.gyoSign[0] =  1.0f;
            max21100Data.gyoSign[1] = -1.0f;
            max21100Data.gyoSign[2] = -1.0f;
            break;

        case 2:
            max21100Data.accSign[0] = -1.0f;
            max21100Data.accSign[1] =  1.0f;
            max21100Data.accSign[2] = -1.0f;
            max21100Data.gyoSign[0] = -1.0f;
            max21100Data.gyoSign[1] =  1.0f;
            max21100Data.gyoSign[2] = -1.0f;
            break;

        case 0:
        default:
            max21100Data.accSign[0] = 1.0f;
            max21100Data.accSign[1] = 1.0f;
            max21100Data.accSign[2] = 1.0f;
            max21100Data.gyoSign[0] = 1.0f;
            max21100Data.gyoSign[1] = 1.0f;
            max21100Data.gyoSign[2] = 1.0f;
            break;
    }

    utilFilterInit(&max21100Data.tempFilter, DIMU_OUTER_DT, DIMU_TEMP_TAU, IMU_ROOM_TEMP);

    max21100ReliablySetReg(0x22, 0x00); // BNK 00

    // turn off I2C
    max21100ReliablySetReg(0x16, 0x01);

    // check device ID
    while (max21100GetReg(0x20) != 0xb2)
        ;

    // power up
    max21100ReliablySetReg(0x00, 0b01111111); // Normal Mode GYO+ACC

    // GYO scale @ 2KHz
#if MAX21100_GYO_SCALE == 250
    max21100ReliablySetReg(0x01, 0b00100011);
#endif
#if MAX21100_GYO_SCALE == 500
    max21100ReliablySetReg(0x01, 0b00100010);
#endif
#if MAX21100_GYO_SCALE == 1000
    max21100ReliablySetReg(0x01, 0b00100001);
#endif
#if MAX21100_GYO_SCALE == 2000
    max21100ReliablySetReg(0x01, 0b00100000);
#endif
    max21100ReliablySetReg(0x02, 0b00100000); // GYO 8KHz ODR

    // ACC scale
#if MAX21100_ACC_SCALE == 2
    max21100ReliablySetReg(0x04, 0b11000111);
#endif
#if MAX21100_ACC_SCALE == 4
    max21100ReliablySetReg(0x04, 0b10000111);
#endif
#if MAX21100_ACC_SCALE == 8
    max21100ReliablySetReg(0x04, 0b01000111);
#endif
#if MAX21100_ACC_SCALE == 16
    max21100ReliablySetReg(0x04, 0b00000111);
#endif
    max21100ReliablySetReg(0x05, 0b00110000); // ACC 2KHz ODR

    max21100SetReg(0x22, 0x01); // BNK 01

    max21100ReliablySetReg(0x0b, 0b00100000);   // IRQ config
    max21100ReliablySetReg(0x0c, 0b00000000);   // IRQ config
    max21100SetReg(0x0d, 0b10000000);   // IRQ config

    max21100Data.readReg = MAX21100_READ_BIT | 0x24;	// start of sensor registers

    // External Interrupt line for data ready
    extRegisterCallback(DIMU_MAX21100_INT_PORT, DIMU_MAX21100_INT_PIN, EXTI_Trigger_Rising, 1, GPIO_PuPd_NOPULL, max21100IntHandler);

    spiChangeCallback(max21100Data.spi, max21100TransferComplete);
}
#endif
