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

#include "config.h"
#ifdef HAS_DIGITAL_IMU
#include "imu.h"
#include "hmc5983.h"
#include "aq_timer.h"
#include "util.h"
#include "config.h"
#ifndef __CC_ARM
#include <intrinsics.h>
#endif

hmc5983Struct_t hmc5983Data;

static void hmc5983TransferComplete(int unused) {
    hmc5983Data.slot = (hmc5983Data.slot + 1) % HMC5983_SLOTS;
}

static void hmc5983ScaleMag(int32_t *in, float *out, float divisor) {
//    out[0] = DIMU_ORIENT_MAG_X * divisor * (1.0f / 390.0f);
//    out[1] = DIMU_ORIENT_MAG_Y * divisor * (1.0f / 390.0f);
//    out[2] = DIMU_ORIENT_MAG_Z * divisor * (1.0f / 390.0f);
    out[0] = DIMU_ORIENT_MAG_X * divisor * (1.0f / 187.88f);
    out[1] = DIMU_ORIENT_MAG_Y * divisor * (1.0f / 187.88f);
    out[2] = DIMU_ORIENT_MAG_Z * divisor * (1.0f / 187.88f);
}

static void hmc5983CalibMag(float *in, volatile float *out) {
    float a, b, c;
    float x, y, z;

    // bias
    a = +(in[0] + p[IMU_MAG_BIAS_X] + p[IMU_MAG_BIAS1_X]*dImuData.dTemp + p[IMU_MAG_BIAS2_X]*dImuData.dTemp2 + p[IMU_MAG_BIAS3_X]*dImuData.dTemp3);
    b = +(in[1] + p[IMU_MAG_BIAS_Y] + p[IMU_MAG_BIAS1_Y]*dImuData.dTemp + p[IMU_MAG_BIAS2_Y]*dImuData.dTemp2 + p[IMU_MAG_BIAS3_Y]*dImuData.dTemp3);
    c = -(in[2] + p[IMU_MAG_BIAS_Z] + p[IMU_MAG_BIAS1_Z]*dImuData.dTemp + p[IMU_MAG_BIAS2_Z]*dImuData.dTemp2 + p[IMU_MAG_BIAS3_Z]*dImuData.dTemp3);

    // misalignment
    x = a + b*p[IMU_MAG_ALGN_XY] + c*p[IMU_MAG_ALGN_XZ];
    y = a*p[IMU_MAG_ALGN_YX] + b + c*p[IMU_MAG_ALGN_YZ];
    z = a*p[IMU_MAG_ALGN_ZX] + b*p[IMU_MAG_ALGN_ZY] + c;

    // scale
    x /= (p[IMU_MAG_SCAL_X] + p[IMU_MAG_SCAL1_X]*dImuData.dTemp + p[IMU_MAG_SCAL2_X]*dImuData.dTemp2 + p[IMU_MAG_SCAL3_X]*dImuData.dTemp3);
    y /= (p[IMU_MAG_SCAL_Y] + p[IMU_MAG_SCAL1_Y]*dImuData.dTemp + p[IMU_MAG_SCAL2_Y]*dImuData.dTemp2 + p[IMU_MAG_SCAL3_Y]*dImuData.dTemp3);
    z /= (p[IMU_MAG_SCAL_Z] + p[IMU_MAG_SCAL1_Z]*dImuData.dTemp + p[IMU_MAG_SCAL2_Z]*dImuData.dTemp2 + p[IMU_MAG_SCAL3_Z]*dImuData.dTemp3);

    out[0] = x * imuData.cosRot - y * imuData.sinRot;
    out[1] = y * imuData.cosRot + x * imuData.sinRot;
    out[2] = z;
}

void hmc5983Decode(void) {
    volatile uint8_t *d = hmc5983Data.rxBuf;
    int32_t mag[3];
    float divisor;
    int i;

    mag[0] = 0;
    mag[1] = 0;
    mag[2] = 0;

    divisor = (float)HMC5983_SLOTS;
    for (i = 0; i < HMC5983_SLOTS; i++) {
	int j = i*HMC5983_SLOT_SIZE;

	// check if we are in the middle of a transaction for this slot
	if (i == hmc5983Data.slot && hmc5983Data.spiFlag == 0)	{
	    divisor -= 1.0f;
	}
	else {
	    mag[1] += (int16_t)__rev16(*(uint16_t *)&d[j+1]);
	    mag[2] += (int16_t)__rev16(*(uint16_t *)&d[j+3]);
	    mag[0] += (int16_t)__rev16(*(uint16_t *)&d[j+5]);
	}
    }

    divisor = 1.0f / divisor;

    hmc5983ScaleMag(mag, hmc5983Data.rawMag, divisor);
    hmc5983CalibMag(hmc5983Data.rawMag, hmc5983Data.mag);

    hmc5983Data.lastUpdate = timerMicros();
}

static uint8_t hmc5983GetReg(uint8_t reg) {
    static uint8_t rxBuf[2];
    static uint8_t txBuf[2];

    txBuf[0] = HMC5983_READ_BIT | reg;

    hmc5983Data.spiFlag = 0;
    spiTransaction(hmc5983Data.spi, rxBuf, txBuf, 2);

    while (!hmc5983Data.spiFlag)
	;

    return rxBuf[1];
}

static void hmc5983SetReg(uint8_t reg, uint8_t val) {
    static uint8_t rxBuf[2];
    static uint8_t txBuf[2];

    txBuf[0] = HMC5983_WRITE_BIT | reg;
    txBuf[1] = val;

    hmc5983Data.spiFlag = 0;
    spiTransaction(hmc5983Data.spi, rxBuf, txBuf, 2);

    while (!hmc5983Data.spiFlag)
	;
}

static void hmc5983ReliablySetReg(uint8_t reg, uint8_t val) {
    uint8_t ret;

    do {
	delay(10);
	hmc5983SetReg(reg, val);
	delay(10);
	ret = hmc5983GetReg(reg);
    } while (ret != val);
}

static void hmc5983StartTransfer(void) {
    if (hmc5983Data.enabled)
	spiTransaction(hmc5983Data.spi, &hmc5983Data.rxBuf[hmc5983Data.slot*HMC5983_SLOT_SIZE], &hmc5983Data.readCmd, HMC5983_BYTES);
}

inline void hmc5983Enable(void) {
    if (hmc5983Data.initialized)
      hmc5983Data.enabled = 1;
}

inline void hmc5983Disable(void) {
    hmc5983Data.enabled = 0;
}

void hmc5983PreInit(void) {
    hmc5983Data.spi = spiClientInit(DIMU_HMC5983_SPI, HMC5983_SPI_BAUD, DIMU_HMC5983_CS_PORT, DIMU_HMC5983_CS_PIN, &hmc5983Data.spiFlag, 0);
}

uint8_t hmc5983Init(void) {
    int i = HMC5983_RETRIES;

    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // wait for a valid response
    while (--i && hmc5983GetReg(0x0a) != 'H')
	delay(100);

    if (i > 0) {
      // 75Hz, 8x oversample
      hmc5983ReliablySetReg(0x00, 0b11111000);
      delay(10);

  //    // highest gain (+-0.88 Ga)
  //    hmc5983ReliablySetReg(0x01, 0b00000000);
      // gain (+-2.5 Ga)
      hmc5983ReliablySetReg(0x01, 0b01100000);
      delay(10);

      hmc5983ReliablySetReg(0x02, 0b00000000);
      delay(10);

      hmc5983Data.readCmd = HMC5983_READ_MULT_BIT | 0x03;

      spiChangeCallback(hmc5983Data.spi, hmc5983TransferComplete);

      // External Interrupt line for data ready
      GPIO_StructInit(&GPIO_InitStructure);
      GPIO_InitStructure.GPIO_Pin = DIMU_HMC5983_INT_PIN;
      GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
      GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
      GPIO_Init(DIMU_HMC5983_INT_PORT, &GPIO_InitStructure);

      SYSCFG_EXTILineConfig(DIMU_HMC5983_INT_EXTI_PORT, DIMU_HMC5983_INT_EXTI_PIN);

      EXTI_InitStructure.EXTI_Line = DIMU_HMC5983_INT_EXTI_LINE;
      EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
      EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
      EXTI_InitStructure.EXTI_LineCmd = ENABLE;
      EXTI_Init(&EXTI_InitStructure);

      NVIC_InitStructure.NVIC_IRQChannel = DIMU_HMC5983_INT_EXTI_IRQ;
      NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
      NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
      NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
      NVIC_Init(&NVIC_InitStructure);

      hmc5983Data.initialized = 1;
  }
  else {
      hmc5983Data.initialized = 0;
  }

  return hmc5983Data.initialized;
}

void DIMU_HMC5983_INT_ISR(void) {
    EXTI_ClearITPendingBit(DIMU_HMC5983_INT_EXTI_LINE);
    hmc5983StartTransfer();
}
#endif
