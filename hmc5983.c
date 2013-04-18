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
#include "hmc5983.h"
#include "aq_timer.h"
#include "util.h"
#include "config.h"

hmc5983Struct_t hmc5983Data;

void hmc5983TransferComplete(int unused) {
    hmc5983Data.slot = (hmc5983Data.slot + 1) % HMC5983_SLOTS;
}

void hmc5983Decode(void) {
    uint8_t *d = hmc5983Data.rxBuf;
    int32_t mag[3];
    float divisor;
    float x, y, z;
    float a, b, c;
    int i;

    mag[0] = 0;
    mag[1] = 0;
    mag[2] = 0;

    divisor = (float)HMC5983_SLOTS;
    for (i = 0; i < HMC5983_SLOTS; i++) {
	int j = i*HMC5983_BYTES;

	// check if we are in the middle of a transaction for this slot
	if (i == hmc5983Data.slot && hmc5983Data.spiFlag == 0)	{
	    divisor -= 1.0f;
	}
	else {
	    mag[1] += (int16_t)((uint16_t)d[j+1]<<8 | d[j+2]);
	    mag[2] += (int16_t)((uint16_t)d[j+3]<<8 | d[j+4]);
	    mag[0] += (int16_t)((uint16_t)d[j+5]<<8 | d[j+6]);
	}
    }

    divisor = 1.0f / divisor;

#ifdef DIUM_IMUV1
    hmc5983Data.rawMag[0] = -mag[0] * divisor * (1.0f / 390.0f);
    hmc5983Data.rawMag[1] = -mag[1] * divisor * (1.0f / 390.0f);
    hmc5983Data.rawMag[2] = +mag[2] * divisor * (1.0f / 390.0f);
#else
    hmc5983Data.rawMag[0] = -mag[0] * divisor * (1.0f / 390.0f);
    hmc5983Data.rawMag[1] = +mag[1] * divisor * (1.0f / 390.0f);
    hmc5983Data.rawMag[2] = -mag[2] * divisor * (1.0f / 390.0f);
#endif

    // bias
    a = +(hmc5983Data.rawMag[0] + p[IMU_MAG_BIAS_X] + p[IMU_MAG_BIAS1_X]*dImuData.dTemp + p[IMU_MAG_BIAS2_X]*dImuData.dTemp2 + p[IMU_MAG_BIAS3_X]*dImuData.dTemp3);
    b = +(hmc5983Data.rawMag[1] + p[IMU_MAG_BIAS_Y] + p[IMU_MAG_BIAS1_Y]*dImuData.dTemp + p[IMU_MAG_BIAS2_Y]*dImuData.dTemp2 + p[IMU_MAG_BIAS3_Y]*dImuData.dTemp3);
    c = -(hmc5983Data.rawMag[2] + p[IMU_MAG_BIAS_Z] + p[IMU_MAG_BIAS1_Z]*dImuData.dTemp + p[IMU_MAG_BIAS2_Z]*dImuData.dTemp2 + p[IMU_MAG_BIAS3_Z]*dImuData.dTemp3);

    // misalignment
    x = a + b*p[IMU_MAG_ALGN_XY] + c*p[IMU_MAG_ALGN_XZ];
    y = a*p[IMU_MAG_ALGN_YX] + b + c*p[IMU_MAG_ALGN_YZ];
    z = a*p[IMU_MAG_ALGN_ZX] + b*p[IMU_MAG_ALGN_ZY] + c;

    // scale
    x /= (p[IMU_MAG_SCAL_X] + p[IMU_MAG_SCAL1_X]*dImuData.dTemp + p[IMU_MAG_SCAL2_X]*dImuData.dTemp2 + p[IMU_MAG_SCAL3_X]*dImuData.dTemp3);
    y /= (p[IMU_MAG_SCAL_Y] + p[IMU_MAG_SCAL1_Y]*dImuData.dTemp + p[IMU_MAG_SCAL2_Y]*dImuData.dTemp2 + p[IMU_MAG_SCAL3_Y]*dImuData.dTemp3);
    z /= (p[IMU_MAG_SCAL_Z] + p[IMU_MAG_SCAL1_Z]*dImuData.dTemp + p[IMU_MAG_SCAL2_Z]*dImuData.dTemp2 + p[IMU_MAG_SCAL3_Z]*dImuData.dTemp3);

    hmc5983Data.mag[0] = x * imuData.cosRot - y * imuData.sinRot;
    hmc5983Data.mag[1] = y * imuData.cosRot + x * imuData.sinRot;
    hmc5983Data.mag[2] = z;

    hmc5983Data.lastUpdate = timerMicros();
}

uint8_t hmc5983GetReg(uint8_t reg) {
    static uint8_t rxBuf[2];
    static uint8_t txBuf[2];

    txBuf[0] = HMC5983_READ_BIT | reg;

    hmc5983Data.spiFlag = 0;
    spiTransaction(hmc5983Data.spi, rxBuf, txBuf, 2);

    while (!hmc5983Data.spiFlag)
	;

    return rxBuf[1];
}

void hmc5983SetReg(uint8_t reg, uint8_t val) {
    static uint8_t rxBuf[2];
    static uint8_t txBuf[2];

    txBuf[0] = HMC5983_WRITE_BIT | reg;
    txBuf[1] = val;

    hmc5983Data.spiFlag = 0;
    spiTransaction(hmc5983Data.spi, rxBuf, txBuf, 2);

    while (!hmc5983Data.spiFlag)
	;
}

void hmc5983StartTransfer(void) {
    if (hmc5983Data.enabled)
	spiTransaction(hmc5983Data.spi, &hmc5983Data.rxBuf[hmc5983Data.slot*HMC5983_BYTES], &hmc5983Data.readCmd, HMC5983_BYTES);
}

inline void hmc5983Enable(void) {
    hmc5983Data.enabled = 1;
}

inline void hmc5983Disable(void) {
    hmc5983Data.enabled = 0;
}

void hmc5983PreInit(void) {
    hmc5983Data.spi = spiClientInit(HMC5983_SPI, HMC5983_SPI_BAUD, HMC5983_CS_PORT, HMC5983_CS_PIN, &hmc5983Data.spiFlag, 0);
}

void hmc5983Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    // wait for a valid response
    while (hmc5983GetReg(0x0a) != 'H')
	delay(100);

    hmc5983SetReg(0x00, 0b11111100);
    delay(10);
    hmc5983SetReg(0x01, 0b00000000);
    delay(10);
    hmc5983SetReg(0x02, 0b00000000);
    delay(10);

    hmc5983Data.readCmd = HMC5983_READ_MULT_BIT | 0x03;

    spiChangeCallback(hmc5983Data.spi, hmc5983TransferComplete);

    // External Interrupt line for data ready
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = HMC5983_INT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(HMC5983_INT_PORT, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(HMC5983_INT_EXTI_PORT, HMC5983_INT_EXTI_PIN);

    EXTI_InitStructure.EXTI_Line = HMC5983_INT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = HMC5983_INT_EXTI_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

void HMC5983_INT_ISR(void) {
    EXTI_ClearITPendingBit(HMC5983_INT_EXTI_LINE);
    hmc5983StartTransfer();
}