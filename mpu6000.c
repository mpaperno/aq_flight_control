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

#include "imu.h"
#include "mpu6000.h"
#include "aq_timer.h"
#include "util.h"
#include "config.h"

mpu6000Struct_t mpu6000Data;

void mpu6000TransferComplete(int unused) {
    mpu6000Data.slot = (mpu6000Data.slot + 1) % MPU6000_SLOTS;
}

void mpu6600InitialBias(void) {
    uint32_t lastUpdate = mpu6000Data.lastUpdate;
    float gyoSum[3];
    float tempSum;
//    float dTemp, dTemp2, dTemp3;
    int i;

    gyoSum[0] = 0.0f;
    gyoSum[1] = 0.0f;
    gyoSum[2] = 0.0f;
    tempSum = 0.0f;

    for (i = 0; i < 50; i++) {
	while (lastUpdate == mpu6000Data.lastUpdate)
	    delay(1);
	lastUpdate = mpu6000Data.lastUpdate;

	gyoSum[0] += mpu6000Data.rawGyo[0];
	gyoSum[1] += mpu6000Data.rawGyo[1];
	gyoSum[2] += mpu6000Data.rawGyo[2];

	tempSum += mpu6000Data.rawTemp;
    }

    mpu6000Data.temp = tempSum / 50.0f;
    utilFilterReset(&mpu6000Data.tempFilter, mpu6000Data.temp);

//    dTemp = mpu6000Data.temp - IMU_ROOM_TEMP;
//    dTemp2 = dTemp*dTemp;
//    dTemp3 = dTemp*dTemp2;

//    mpu6000Data.gyoOffset[0] = -(gyoSum[0] / 50.0f + p[IMU_GYO_BIAS_X] + p[IMU_GYO_BIAS1_X]*dTemp + p[IMU_GYO_BIAS2_X]*dTemp2 + p[IMU_GYO_BIAS3_X]*dTemp3);
//    mpu6000Data.gyoOffset[1] = -(gyoSum[1] / 50.0f + p[IMU_GYO_BIAS_Y] + p[IMU_GYO_BIAS1_Y]*dTemp + p[IMU_GYO_BIAS2_Y]*dTemp2 + p[IMU_GYO_BIAS3_Y]*dTemp3);
//    mpu6000Data.gyoOffset[2] = -(gyoSum[2] / 50.0f + p[IMU_GYO_BIAS_Z] + p[IMU_GYO_BIAS1_Z]*dTemp + p[IMU_GYO_BIAS2_Z]*dTemp2 + p[IMU_GYO_BIAS3_Z]*dTemp3);
}

void mpu6000Decode(void) {
    volatile uint8_t *d = mpu6000Data.rxBuf;
    int32_t acc[3], temp, gyo[3];
    float divisor;
    float x, y, z;
    float a, b, c;
    int i;

    for (i = 0; i < 3; i++) {
	acc[i] = 0;
	gyo[i] = 0;
    }
    temp = 0;

    divisor = (float)MPU6000_SLOTS;
    for (i = 0; i < MPU6000_SLOTS; i++) {
	int j = i*MPU6000_BYTES;

	// check if we are in the middle of a transaction for this slot
	if (i == mpu6000Data.slot && mpu6000Data.spiFlag == 0)	{
	    divisor -= 1.0f;
	}
	else {
	    acc[0] += (int16_t)((uint16_t)d[j+1]<<8 | d[j+2]);
	    acc[1] += (int16_t)((uint16_t)d[j+3]<<8 | d[j+4]);
	    acc[2] += (int16_t)((uint16_t)d[j+5]<<8 | d[j+6]);

	    temp += (int16_t)((uint16_t)d[j+7]<<8 | d[j+8]);

	    gyo[0] += (int16_t)((uint16_t)d[j+9]<<8 | d[j+10]);
	    gyo[1] += (int16_t)((uint16_t)d[j+11]<<8 | d[j+12]);
	    gyo[2] += (int16_t)((uint16_t)d[j+13]<<8 | d[j+14]);
	}
    }

    divisor = 1.0f / divisor;

    mpu6000Data.rawTemp = temp * divisor * (1.0f / 340.0f) + 36.53f;

    mpu6000Data.temp = utilFilter(&mpu6000Data.tempFilter, mpu6000Data.rawTemp);

    // 4g
#ifdef DIUM_IMUV1
    mpu6000Data.rawAcc[0] = +acc[0] * divisor * (1.0f / 8192.0f) * GRAVITY;
    mpu6000Data.rawAcc[1] = +acc[1] * divisor * (1.0f / 8192.0f) * GRAVITY;
    mpu6000Data.rawAcc[2] = +acc[2] * divisor * (1.0f / 8192.0f) * GRAVITY;
#else
    mpu6000Data.rawAcc[0] = +acc[0] * divisor * (1.0f / 8192.0f) * GRAVITY;
    mpu6000Data.rawAcc[1] = -acc[1] * divisor * (1.0f / 8192.0f) * GRAVITY;
    mpu6000Data.rawAcc[2] = -acc[2] * divisor * (1.0f / 8192.0f) * GRAVITY;
#endif

    // bias
    a = -(mpu6000Data.rawAcc[0] + p[IMU_ACC_BIAS_X] + p[IMU_ACC_BIAS1_X]*dImuData.dTemp + p[IMU_ACC_BIAS2_X]*dImuData.dTemp2 + p[IMU_ACC_BIAS3_X]*dImuData.dTemp3);
    b = +(mpu6000Data.rawAcc[1] + p[IMU_ACC_BIAS_Y] + p[IMU_ACC_BIAS1_Y]*dImuData.dTemp + p[IMU_ACC_BIAS2_Y]*dImuData.dTemp2 + p[IMU_ACC_BIAS3_X]*dImuData.dTemp3);
    c = -(mpu6000Data.rawAcc[2] + p[IMU_ACC_BIAS_Z] + p[IMU_ACC_BIAS1_Z]*dImuData.dTemp + p[IMU_ACC_BIAS2_Z]*dImuData.dTemp2 + p[IMU_ACC_BIAS3_X]*dImuData.dTemp3);

    // misalignment
    x = a + b*p[IMU_ACC_ALGN_XY] + c*p[IMU_ACC_ALGN_XZ];
    y = a*p[IMU_ACC_ALGN_YX] + b + c*p[IMU_ACC_ALGN_YZ];
    z = a*p[IMU_ACC_ALGN_ZX] + b*p[IMU_ACC_ALGN_ZY] + c;

    // scale
    x /= (p[IMU_ACC_SCAL_X] + p[IMU_ACC_SCAL1_X]*dImuData.dTemp + p[IMU_ACC_SCAL2_X]*dImuData.dTemp2 + p[IMU_ACC_SCAL3_X]*dImuData.dTemp3);
    y /= (p[IMU_ACC_SCAL_Y] + p[IMU_ACC_SCAL1_Y]*dImuData.dTemp + p[IMU_ACC_SCAL2_Y]*dImuData.dTemp2 + p[IMU_ACC_SCAL3_Y]*dImuData.dTemp3);
    z /= (p[IMU_ACC_SCAL_Z] + p[IMU_ACC_SCAL1_Z]*dImuData.dTemp + p[IMU_ACC_SCAL2_Z]*dImuData.dTemp2 + p[IMU_ACC_SCAL3_Z]*dImuData.dTemp3);

    // IMU rotation
    mpu6000Data.acc[0] = x * imuData.cosRot - y * imuData.sinRot;
    mpu6000Data.acc[1] = y * imuData.cosRot + x * imuData.sinRot;
    mpu6000Data.acc[2] = z;

    // 500 deg/s
#ifdef DIUM_IMUV1
    mpu6000Data.rawGyo[0] = -gyo[0] * divisor * (1.0f / 65.5f) * DEG_TO_RAD;
    mpu6000Data.rawGyo[1] = -gyo[1] * divisor * (1.0f / 65.5f) * DEG_TO_RAD;
    mpu6000Data.rawGyo[2] = +gyo[2] * divisor * (1.0f / 65.5f) * DEG_TO_RAD;
#else
    mpu6000Data.rawGyo[0] = -gyo[0] * divisor * (1.0f / 65.5f) * DEG_TO_RAD;
    mpu6000Data.rawGyo[1] = +gyo[1] * divisor * (1.0f / 65.5f) * DEG_TO_RAD;
    mpu6000Data.rawGyo[2] = -gyo[2] * divisor * (1.0f / 65.5f) * DEG_TO_RAD;
#endif

    // bias
    a = +(mpu6000Data.rawGyo[0] + mpu6000Data.gyoOffset[0] + p[IMU_GYO_BIAS_X] + p[IMU_GYO_BIAS1_X]*dImuData.dTemp + p[IMU_GYO_BIAS2_X]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_X]*dImuData.dTemp3);
    b = -(mpu6000Data.rawGyo[1] + mpu6000Data.gyoOffset[1] + p[IMU_GYO_BIAS_Y] + p[IMU_GYO_BIAS1_Y]*dImuData.dTemp + p[IMU_GYO_BIAS2_Y]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_Y]*dImuData.dTemp3);
    c = -(mpu6000Data.rawGyo[2] + mpu6000Data.gyoOffset[2] + p[IMU_GYO_BIAS_Z] + p[IMU_GYO_BIAS1_Z]*dImuData.dTemp + p[IMU_GYO_BIAS2_Z]*dImuData.dTemp2 + p[IMU_GYO_BIAS3_Z]*dImuData.dTemp3);

    // misalignment
    x = a + b*p[IMU_GYO_ALGN_XY] + c*p[IMU_GYO_ALGN_XZ];
    y = a*p[IMU_GYO_ALGN_YX] + b + c*p[IMU_GYO_ALGN_YZ];
    z = a*p[IMU_GYO_ALGN_ZX] + b*p[IMU_GYO_ALGN_ZY] + c;

    // scale
    x /= p[IMU_GYO_SCAL_X];
    y /= p[IMU_GYO_SCAL_Y];
    z /= p[IMU_GYO_SCAL_Z];

    // IMU rotation
    mpu6000Data.gyo[0] = x * imuData.cosRot - y * imuData.sinRot;
    mpu6000Data.gyo[1] = y * imuData.cosRot + x * imuData.sinRot;
    mpu6000Data.gyo[2] = z;

    mpu6000Data.lastUpdate = timerMicros();
}

uint8_t mpu6000GetReg(uint8_t reg) {
    static uint8_t rxBuf[2];
    static uint8_t txBuf[2];

    txBuf[0] = MPU6000_READ_BIT | reg;

    mpu6000Data.spiFlag = 0;
    spiTransaction(mpu6000Data.spi, rxBuf, txBuf, 2);

    while (!mpu6000Data.spiFlag)
	;

    return rxBuf[1];
}

void mpu6000SetReg(uint8_t reg, uint8_t val) {
    static uint8_t rxBuf[2];
    static uint8_t txBuf[2];

    txBuf[0] = MPU6000_WRITE_BIT | reg;
    txBuf[1] = val;

    mpu6000Data.spiFlag = 0;
    spiTransaction(mpu6000Data.spi, rxBuf, txBuf, 2);

    while (!mpu6000Data.spiFlag)
	;
}

void mpu6000ReliablySetReg(uint8_t reg, uint8_t val) {
    do {
	delay(10);
	mpu6000SetReg(reg, val);
	delay(10);
    } while (mpu6000GetReg(reg) != val);
}

void mpu6000StartTransfer(void) {
    if (mpu6000Data.enabled)
	spiTransaction(mpu6000Data.spi, &mpu6000Data.rxBuf[mpu6000Data.slot*MPU6000_BYTES], &mpu6000Data.readReg, MPU6000_BYTES);
}

inline void mpu6000Enable(void) {
    mpu6000Data.enabled = 1;
}

inline void mpu6000Disable(void) {
    mpu6000Data.enabled = 0;
}

void mpu6000PreInit(void) {
    mpu6000Data.spi = spiClientInit(DIUM_MPU6000_SPI, MPU6000_SPI_REG_BAUD, DIUM_MPU6000_CS_PORT, DIUM_MPU6000_CS_PIN, &mpu6000Data.spiFlag, 0);
}

void mpu6000Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    utilFilterInit(&mpu6000Data.tempFilter, DIMU_DT, DIMU_TEMP_TAU, IMU_ROOM_TEMP);

    // reset
    mpu6000SetReg(107, 0b10000000);
    delay(100);

    // wake up
    mpu6000ReliablySetReg(107, 0x01);

    // disable I2C interface
    mpu6000ReliablySetReg(106, 0b010000);

    // wait for a valid response
    while (mpu6000GetReg(117) != 0x68)
	delay(10);

    // GYO scale - 500 deg/s
    mpu6000ReliablySetReg(27, 0b01000);

    // ACC scale - 4g
    mpu6000ReliablySetReg(28, 0b01000);

    // Sample rate
    mpu6000ReliablySetReg(25, 0x00);

    // LPF
    mpu6000ReliablySetReg(26, 0x00);

    // Interrupt setup
    mpu6000ReliablySetReg(55, 0x01<<4);
    mpu6000ReliablySetReg(56, 0x01);

    // bump clock rate up to 21MHz
    spiChangeBaud(mpu6000Data.spi, MPU6000_SPI_RUN_BAUD);

    mpu6000Data.readReg = MPU6000_READ_BIT | 0x3b;	// start of sensor registers

    spiChangeCallback(mpu6000Data.spi, mpu6000TransferComplete);

    // External Interrupt line for data ready
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = DIUM_MPU6000_INT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(DIUM_MPU6000_INT_PORT, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(DIUM_MPU6000_INT_EXTI_PORT, DIUM_MPU6000_INT_EXTI_PIN);

    EXTI_InitStructure.EXTI_Line = DIUM_MPU6000_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = DIUM_MPU6000_INT_EXTI_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void MPU6000_INT_ISR(void) {
    EXTI_ClearITPendingBit(DIUM_MPU6000_INT_EXTI_LINE);
    mpu6000StartTransfer();
}

