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

#include "aq.h"
#ifdef HAS_AIMU
#include "analog.h"
#include "imu.h"
#include "comm.h"
#include "aq_timer.h"
#include "util.h"
#include "config.h"
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <CoOS.h>

adcStruct_t adcData __attribute__((section(".ccm")));
struct {
    uint16_t adc123Raw1[ADC_CHANNELS*3];
    uint16_t adc123Raw2[ADC_CHANNELS*3];
} adcDMAData;

OS_STK *adcTaskStack;

float adcIDGVoltsToTemp(float volts) {
    return (25.0f + (volts - ADC_IDG_TEMP_OFFSET) * ADC_IDG_TEMP_SLOPE);
}

// TODO: optimize
float adcT1VoltsToTemp(float volts) {
    float r1;
    float ln;
    float kelvin;

    r1 = (ADC_TEMP_R2 * ADC_REF_VOLTAGE) / volts - ADC_TEMP_R2;
    ln = log(r1);
    kelvin = ADC_TEMP_A + ADC_TEMP_B * ln + ADC_TEMP_C * (ln*ln*ln);

    return (kelvin + ADC_KELVIN_TO_CELCIUS);
}

float adcVsenseToVin(float volts) {
    return (volts - ADC_VIN_OFFSET) * ADC_VIN_SLOPE;
}

void adcTaskCode(void *unused) {
    float dRateVoltageX, dRateVoltageY, dRateVoltageZ;
    double sumMagX, sumMagY, sumMagZ;
    unsigned char magSign;
    int countMag, firstAfterFlip;
    float x, y, z;
    float a, b, c;
    float dTemp, dTemp2, dTemp3;
    unsigned long loops;
    register int i;

    AQ_NOTICE("ADC task started\n");

    magSign = ADC_MAG_SIGN;
    sumMagX = sumMagY = sumMagZ = 0.0f;
    dTemp = dTemp2 = dTemp3 = 0.0f;
    countMag = 0;
    firstAfterFlip = 0;

    while (1) {
	// wait for work
	CoWaitForSingleFlag(adcData.adcFlag, 0);

	loops = adcData.loops;

	// sum adc values
	for (i = 0; i < ADC_SENSORS; i++)
	    adcData.channelSums[i] += adcData.adcSums[i];

	// IDG500 & ISZ500 gyro double rate
	dRateVoltageX = adcData.adcSums[ADC_VOLTS_RATEX] * ADC_DIVISOR * (1.0 / 4.0);
	dRateVoltageY = adcData.adcSums[ADC_VOLTS_RATEY] * ADC_DIVISOR * (1.0 / 4.0);
	dRateVoltageZ = adcData.adcSums[ADC_VOLTS_RATEZ] * ADC_DIVISOR * (1.0 / 4.0);

	// rates
	x = +(dRateVoltageX + adcData.rateBiasX + p[IMU_GYO_BIAS1_X]*dTemp + p[IMU_GYO_BIAS2_X]*dTemp2 + p[IMU_GYO_BIAS3_X]*dTemp3);// / p[IMU_GYO_SCAL_X];
	y = -(dRateVoltageY + adcData.rateBiasY + p[IMU_GYO_BIAS1_Y]*dTemp + p[IMU_GYO_BIAS2_Y]*dTemp2 + p[IMU_GYO_BIAS3_Y]*dTemp3);// / p[IMU_GYO_SCAL_Y];
	z = -(dRateVoltageZ + adcData.rateBiasZ + p[IMU_GYO_BIAS1_Z]*dTemp + p[IMU_GYO_BIAS2_Z]*dTemp2 + p[IMU_GYO_BIAS3_Z]*dTemp3);// / p[IMU_GYO_SCAL_Z];

	a = x + y*p[IMU_GYO_ALGN_XY] + z*p[IMU_GYO_ALGN_XZ];
	b = x*p[IMU_GYO_ALGN_YX] + y + z*p[IMU_GYO_ALGN_YZ];
	c = x*p[IMU_GYO_ALGN_ZX] + y*p[IMU_GYO_ALGN_ZY] + z;

	a /= p[IMU_GYO_SCAL_X];
	b /= p[IMU_GYO_SCAL_Y];
	c /= p[IMU_GYO_SCAL_Z];

	adcData.dRateX = (adcData.dRateX + a * imuData.cosRot - b * imuData.sinRot) * 0.5f;
	adcData.dRateY = (adcData.dRateY + b * imuData.cosRot + a * imuData.sinRot) * 0.5f;
	adcData.dRateZ = (adcData.dRateZ + c) * 0.5f;
//	adcData.dRateX = a * imuData.cosRot - b * imuData.sinRot;
//	adcData.dRateY = b * imuData.cosRot + a * imuData.sinRot;
//	adcData.dRateZ = c;

	// notify IMU of double rate GYO readings are ready
	imuAdcDRateReady();

	// mags
	// we need to discard frames after mag bias flips
	if (firstAfterFlip) {
	    adcData.channelSums[ADC_VOLTS_MAGX] -= adcData.adcSums[ADC_VOLTS_MAGX];
	    adcData.channelSums[ADC_VOLTS_MAGY] -= adcData.adcSums[ADC_VOLTS_MAGY];
	    adcData.channelSums[ADC_VOLTS_MAGZ] -= adcData.adcSums[ADC_VOLTS_MAGZ];
	    firstAfterFlip = 0;
	}

	if (!(loops & 0x01)) {
	    // calculate voltages
	    adcData.voltages[ADC_VOLTS_RATEX] = adcData.channelSums[ADC_VOLTS_RATEX] * ADC_DIVISOR * (1.0f / 8.0f);   // ratex
	    adcData.voltages[ADC_VOLTS_RATEY] = adcData.channelSums[ADC_VOLTS_RATEY] * ADC_DIVISOR * (1.0f / 8.0f);   // ratey
	    adcData.voltages[ADC_VOLTS_RATEZ] = adcData.channelSums[ADC_VOLTS_RATEZ] * ADC_DIVISOR * (1.0f / 8.0f);   // ratez

	    adcData.voltages[ADC_VOLTS_MAGX]  = adcData.channelSums[ADC_VOLTS_MAGX] * ADC_DIVISOR * (1.0f / 4.0f);   // magx
	    adcData.voltages[ADC_VOLTS_MAGY]  = adcData.channelSums[ADC_VOLTS_MAGY] * ADC_DIVISOR * (1.0f / 4.0f);   // magy
	    adcData.voltages[ADC_VOLTS_MAGZ]  = adcData.channelSums[ADC_VOLTS_MAGZ] * ADC_DIVISOR * (1.0f / 4.0f);   // magz

	    adcData.voltages[ADC_VOLTS_TEMP1] = adcData.channelSums[ADC_VOLTS_TEMP1] * ADC_DIVISOR * (1.0f / 2.0f);   // temp
	    adcData.voltages[ADC_VOLTS_VIN]   = adcData.channelSums[ADC_VOLTS_VIN]   * ADC_DIVISOR * (1.0f / 2.0f);   // Vin

	    adcData.voltages[ADC_VOLTS_ACCX] = adcData.channelSums[ADC_VOLTS_ACCX] * ADC_DIVISOR * (1.0f / 8.0f);   // accx
	    adcData.voltages[ADC_VOLTS_ACCY] = adcData.channelSums[ADC_VOLTS_ACCY] * ADC_DIVISOR * (1.0f / 8.0f);   // accy
	    adcData.voltages[ADC_VOLTS_ACCZ] = adcData.channelSums[ADC_VOLTS_ACCZ] * ADC_DIVISOR * (1.0f / 8.0f);   // accz

	    adcData.voltages[ADC_VOLTS_PRES1] = adcData.channelSums[ADC_VOLTS_PRES1] * ADC_DIVISOR * (1.0f / 8.0f);   // press1
	    adcData.voltages[ADC_VOLTS_PRES2] = adcData.channelSums[ADC_VOLTS_PRES2] * ADC_DIVISOR * (1.0f / 8.0f);   // press2

	    adcData.voltages[ADC_VOLTS_TEMP2] = adcData.channelSums[ADC_VOLTS_TEMP2] * ADC_DIVISOR * (1.0f / 2.0f);   // temp2
	    adcData.voltages[ADC_VOLTS_TEMP3] = adcData.channelSums[ADC_VOLTS_TEMP3] * ADC_DIVISOR * (1.0f / 2.0f);   // temp3

	    for (i = 0; i < ADC_SENSORS; i++)
		adcData.channelSums[i] = 0;

	    // temperature from IDG500
	    adcData.temp1 = adcData.temp1 * (1.0f - ADC_TEMP_SMOOTH) + (adcIDGVoltsToTemp(adcData.voltages[ADC_VOLTS_TEMP1]) + ADC_TEMP1_OFFSET) * ADC_TEMP_SMOOTH;

	    // temperature from ISZ500
	    adcData.temp2 = adcData.temp2 * (1.0f - ADC_TEMP_SMOOTH) + (adcIDGVoltsToTemp(adcData.voltages[ADC_VOLTS_TEMP2]) + ADC_TEMP2_OFFSET) * ADC_TEMP_SMOOTH;

	    // temperature from T1
	    adcData.temp3 = adcData.temp3 * (1.0f - ADC_TEMP_SMOOTH) + (adcT1VoltsToTemp(adcData.voltages[ADC_VOLTS_TEMP3]) + ADC_TEMP3_OFFSET) * ADC_TEMP_SMOOTH;

	    adcData.temperature = adcData.temp3;

	    // temperature difference
	    dTemp = adcData.temperature - IMU_ROOM_TEMP;
	    dTemp2 = dTemp*dTemp;
	    dTemp3 = dTemp2*dTemp;

	    // rates
	    x = +(adcData.voltages[ADC_VOLTS_RATEX] + adcData.rateBiasX + p[IMU_GYO_BIAS1_X]*dTemp + p[IMU_GYO_BIAS2_X]*dTemp2 + p[IMU_GYO_BIAS3_X]*dTemp3);// / p[IMU_GYO_SCAL_X];
	    y = -(adcData.voltages[ADC_VOLTS_RATEY] + adcData.rateBiasY + p[IMU_GYO_BIAS1_Y]*dTemp + p[IMU_GYO_BIAS2_Y]*dTemp2 + p[IMU_GYO_BIAS3_Y]*dTemp3);// / p[IMU_GYO_SCAL_Y];
	    z = -(adcData.voltages[ADC_VOLTS_RATEZ] + adcData.rateBiasZ + p[IMU_GYO_BIAS1_Z]*dTemp + p[IMU_GYO_BIAS2_Z]*dTemp2 + p[IMU_GYO_BIAS3_Z]*dTemp3);// / p[IMU_GYO_SCAL_Z];

	    a = x + y*p[IMU_GYO_ALGN_XY] + z*p[IMU_GYO_ALGN_XZ];
	    b = x*p[IMU_GYO_ALGN_YX] + y + z*p[IMU_GYO_ALGN_YZ];
	    c = x*p[IMU_GYO_ALGN_ZX] + y*p[IMU_GYO_ALGN_ZY] + z;

	    a /= p[IMU_GYO_SCAL_X];
	    b /= p[IMU_GYO_SCAL_Y];
	    c /= p[IMU_GYO_SCAL_Z];

	    adcData.rateX = a * imuData.cosRot - b * imuData.sinRot;
	    adcData.rateY = b * imuData.cosRot + a * imuData.sinRot;
	    adcData.rateZ = c;

	    // Vin
	    analogData.vIn = analogData.vIn * (1.0f - ADC_TEMP_SMOOTH) + adcVsenseToVin(adcData.voltages[ADC_VOLTS_VIN]) * ADC_TEMP_SMOOTH;

	    // ADXL335: bias
	    x = -(adcData.voltages[ADC_VOLTS_ACCX] + p[IMU_ACC_BIAS_X] + p[IMU_ACC_BIAS1_X]*dTemp + p[IMU_ACC_BIAS2_X]*dTemp2 + p[IMU_ACC_BIAS3_X]*dTemp3);
	    y = +(adcData.voltages[ADC_VOLTS_ACCY] + p[IMU_ACC_BIAS_Y] + p[IMU_ACC_BIAS1_Y]*dTemp + p[IMU_ACC_BIAS2_Y]*dTemp2 + p[IMU_ACC_BIAS3_X]*dTemp3);
	    z = -(adcData.voltages[ADC_VOLTS_ACCZ] + p[IMU_ACC_BIAS_Z] + p[IMU_ACC_BIAS1_Z]*dTemp + p[IMU_ACC_BIAS2_Z]*dTemp2 + p[IMU_ACC_BIAS3_X]*dTemp3);

	    // misalignment
	    a = x + y*p[IMU_ACC_ALGN_XY] + z*p[IMU_ACC_ALGN_XZ];
	    b = x*p[IMU_ACC_ALGN_YX] + y + z*p[IMU_ACC_ALGN_YZ];
	    c = x*p[IMU_ACC_ALGN_ZX] + y*p[IMU_ACC_ALGN_ZY] + z;

	    // scale
	    a /= (p[IMU_ACC_SCAL_X] + p[IMU_ACC_SCAL1_X]*dTemp + p[IMU_ACC_SCAL2_X]*dTemp2 + p[IMU_ACC_SCAL3_X]*dTemp3);
	    b /= (p[IMU_ACC_SCAL_Y] + p[IMU_ACC_SCAL1_Y]*dTemp + p[IMU_ACC_SCAL2_Y]*dTemp2 + p[IMU_ACC_SCAL3_Y]*dTemp3);
	    c /= (p[IMU_ACC_SCAL_Z] + p[IMU_ACC_SCAL1_Z]*dTemp + p[IMU_ACC_SCAL2_Z]*dTemp2 + p[IMU_ACC_SCAL3_Z]*dTemp3);

	    adcData.accX = a * imuData.cosRot - b * imuData.sinRot;
	    adcData.accY = b * imuData.cosRot + a * imuData.sinRot;
	    adcData.accZ = c;

#ifdef ADC_PRESSURE_3V3
	    // MP3H61115A
	    adcData.pressure1 = (adcData.pressure1  + ((adcData.voltages[ADC_VOLTS_PRES1] + (0.095f * ADC_REF_VOLTAGE)) * (1000.0f / (0.009f * ADC_REF_VOLTAGE)))) * 0.5f;
	    adcData.pressure2 = (adcData.pressure2  + ((adcData.voltages[ADC_VOLTS_PRES2] + (0.095f * ADC_REF_VOLTAGE)) * (1000.0f / (0.009f * ADC_REF_VOLTAGE)))) * 0.5f;
#endif

#ifdef ADC_PRESSURE_5V
            // MPXH6101A
            adcData.pressure1 = (adcData.pressure1  + (((ADC_REF_VOLTAGE - adcData.voltages[ADC_VOLTS_PRES1]) * (5.0f / ADC_REF_VOLTAGE)) + 0.54705f) / 0.05295f * 1000.0f) * 0.5f;
#endif
	    if (p[IMU_PRESS_SENSE] == 0.0f)
		adcData.pressure = adcData.pressure1;
	    else if (p[IMU_PRESS_SENSE] == 1.0f)
		adcData.pressure = adcData.pressure2;
	    else if (p[IMU_PRESS_SENSE] == 2.0f)
		adcData.pressure = (adcData.pressure1 + adcData.pressure2) * 0.5f;

	    // MAGS: bias
	    x = +((adcData.voltages[ADC_VOLTS_MAGX] - adcData.magBridgeBiasX) * (magSign ? -1 : 1) + p[IMU_MAG_BIAS_X] + p[IMU_MAG_BIAS1_X]*dTemp + p[IMU_MAG_BIAS2_X]*dTemp2 + p[IMU_MAG_BIAS3_X]*dTemp3);
	    y = +((adcData.voltages[ADC_VOLTS_MAGY] - adcData.magBridgeBiasY) * (magSign ? -1 : 1) + p[IMU_MAG_BIAS_Y] + p[IMU_MAG_BIAS1_Y]*dTemp + p[IMU_MAG_BIAS2_Y]*dTemp2 + p[IMU_MAG_BIAS3_Y]*dTemp3);
	    z = -((adcData.voltages[ADC_VOLTS_MAGZ] - adcData.magBridgeBiasZ) * (magSign ? -1 : 1) + p[IMU_MAG_BIAS_Z] + p[IMU_MAG_BIAS1_Z]*dTemp + p[IMU_MAG_BIAS2_Z]*dTemp2 + p[IMU_MAG_BIAS3_Z]*dTemp3);

	    // store the mag sign used for this iteration
	    adcData.magSign = (magSign ? -1 : 1);

	    // misalignment
	    a = x + y*p[IMU_MAG_ALGN_XY] + z*p[IMU_MAG_ALGN_XZ];
	    b = x*p[IMU_MAG_ALGN_YX] + y + z*p[IMU_MAG_ALGN_YZ];
	    c = x*p[IMU_MAG_ALGN_ZX] + y*p[IMU_MAG_ALGN_ZY] + z;

	    // scale
	    a /= (p[IMU_MAG_SCAL_X] + p[IMU_MAG_SCAL1_X]*dTemp + p[IMU_MAG_SCAL2_X]*dTemp2 + p[IMU_MAG_SCAL3_X]*dTemp3);
	    b /= (p[IMU_MAG_SCAL_Y] + p[IMU_MAG_SCAL1_Y]*dTemp + p[IMU_MAG_SCAL2_Y]*dTemp2 + p[IMU_MAG_SCAL3_Y]*dTemp3);
	    c /= (p[IMU_MAG_SCAL_Z] + p[IMU_MAG_SCAL1_Z]*dTemp + p[IMU_MAG_SCAL2_Z]*dTemp2 + p[IMU_MAG_SCAL3_Z]*dTemp3);

	    adcData.magX = a * imuData.cosRot - b * imuData.sinRot;
	    adcData.magY = b * imuData.cosRot + a * imuData.sinRot;
	    adcData.magZ = c;

	    sumMagX += (double)adcData.voltages[ADC_VOLTS_MAGX];
	    sumMagY += (double)adcData.voltages[ADC_VOLTS_MAGY];
	    sumMagZ += (double)adcData.voltages[ADC_VOLTS_MAGZ];
	    countMag++;

	    adcData.dt = (float)(adcData.sampleTime * 2) * (1.0f / (float)AQ_US_PER_SEC);
	    adcData.lastUpdate = adcData.lastSample;

	    imuAdcSensorReady();
	}

	// flip our mag sign and try to refine bridge bias estimate
	if (magSign != ADC_MAG_SIGN) {
	    magSign = ADC_MAG_SIGN;

	    if (magSign) {
		adcData.magBridgeBiasX = sumMagX / countMag;
		adcData.magBridgeBiasY = sumMagY / countMag;
		adcData.magBridgeBiasZ = sumMagZ / countMag;

		if (countMag > 100) {
		    sumMagX -= (double)(adcData.magBridgeBiasX * 2.0f);
		    sumMagY -= (double)(adcData.magBridgeBiasY * 2.0f);
		    sumMagZ -= (double)(adcData.magBridgeBiasZ * 2.0f);
		    countMag -= 2;
		}
	    }
	    firstAfterFlip = 1;
	}
    }
}

void adcCalibOffsets(void) {
    unsigned long lastUpdate;
    float sumRate[3];
    float dTemp, dTemp2, dTemp3;
    int i;

    delay(100);

#ifndef USE_DIGITAL_IMU
    imuQuasiStatic(ADC_RATE_CALIB_SAMPLES);
#endif

    sumRate[0] = sumRate[1] = sumRate[2] = 0.0;

    lastUpdate = adcData.lastUpdate;
    for (i = 0; i < ADC_RATE_CALIB_SAMPLES; i++) {
	while (lastUpdate == adcData.lastUpdate)
	    ;

	lastUpdate = adcData.lastUpdate;

	sumRate[0] += adcData.voltages[ADC_VOLTS_RATEX];
	sumRate[1] += adcData.voltages[ADC_VOLTS_RATEY];
	sumRate[2] += adcData.voltages[ADC_VOLTS_RATEZ];
    }

    dTemp = adcData.temperature - IMU_ROOM_TEMP;
    dTemp2 = dTemp*dTemp;
    dTemp3 = dTemp*dTemp2;

    adcData.rateBiasX = -(sumRate[0] / ADC_RATE_CALIB_SAMPLES + p[IMU_GYO_BIAS1_X]*dTemp + p[IMU_GYO_BIAS2_X]*dTemp2 + p[IMU_GYO_BIAS3_X]*dTemp3);
    adcData.rateBiasY = -(sumRate[1] / ADC_RATE_CALIB_SAMPLES + p[IMU_GYO_BIAS1_Y]*dTemp + p[IMU_GYO_BIAS2_Y]*dTemp2 + p[IMU_GYO_BIAS3_Y]*dTemp3);
    adcData.rateBiasZ = -(sumRate[2] / ADC_RATE_CALIB_SAMPLES + p[IMU_GYO_BIAS1_Z]*dTemp + p[IMU_GYO_BIAS2_Z]*dTemp2 + p[IMU_GYO_BIAS3_Z]*dTemp3);

    adcData.rateX = 0.0f;
    adcData.rateY = 0.0f;
    adcData.rateZ = 0.0f;
}

void adcInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    AQ_NOTICE("ADC init\n");

    memset((void *)&adcData, 0, sizeof(adcData));

    // energize mag's set/reset circuit
    adcData.magSetReset = digitalInit(GPIOE, GPIO_Pin_10, 1);

    // use auto-zero function of gyros
    adcData.rateAutoZero = digitalInit(GPIOE, GPIO_Pin_8, 0);

    // bring ACC's SELF TEST line low
    adcData.accST = digitalInit(GPIOE, GPIO_Pin_12, 0);

    // bring ACC's SCALE line low (ADXL3X5 requires this line be tied to GND or left floating)
    adcData.accScale = digitalInit(GPIOC, GPIO_Pin_15, 0);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_ADC2 | RCC_APB2Periph_ADC3, ENABLE);

    adcData.sample = ADC_SAMPLES - 1;

    // Use STM32F4's Triple Regular Simultaneous Mode capable of ~ 6M samples per second

    DMA_DeInit(ADC_DMA_STREAM);
    DMA_InitStructure.DMA_Channel = ADC_DMA_CHANNEL;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)adcDMAData.adc123Raw1;
    DMA_InitStructure.DMA_PeripheralBaseAddr = ((uint32_t)0x40012308);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = ADC_CHANNELS * 3 * 2;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(ADC_DMA_STREAM, &DMA_InitStructure);

    DMA_ITConfig(ADC_DMA_STREAM, DMA_IT_HT | DMA_IT_TC, ENABLE);
    DMA_ClearITPendingBit(ADC_DMA_STREAM, ADC_DMA_FLAGS);

    DMA_Cmd(ADC_DMA_STREAM, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = ADC_DMA_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // ADC Common Init
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode = ADC_TripleMode_RegSimult;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_1;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ADC1 configuration
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 16;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGX, 1, ADC_SAMPLE_TIME);	// magX
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGX, 2, ADC_SAMPLE_TIME);	// magX
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGX, 3, ADC_SAMPLE_TIME);	// magX
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGX, 4, ADC_SAMPLE_TIME);	// magX

    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGY, 5, ADC_SAMPLE_TIME);	// magY
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGY, 6, ADC_SAMPLE_TIME);	// magY
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGY, 7, ADC_SAMPLE_TIME);	// magY
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGY, 8, ADC_SAMPLE_TIME);	// magY

    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGZ, 9, ADC_SAMPLE_TIME);	// magZ
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGZ, 10, ADC_SAMPLE_TIME);	// magZ
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGZ, 11, ADC_SAMPLE_TIME);	// magZ
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_MAGZ, 12, ADC_SAMPLE_TIME);	// magZ

    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_RATEX, 13, ADC_SAMPLE_TIME);	// rateX
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_RATEX, 14, ADC_SAMPLE_TIME);	// rateX
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_RATEX, 15, ADC_SAMPLE_TIME);	// rateX
    ADC_RegularChannelConfig(ADC1, ADC_CHANNEL_RATEX, 16, ADC_SAMPLE_TIME);	// rateX

    // Enable ADC1 DMA since ADC1 is the Master
    ADC_DMACmd(ADC1, ENABLE);

    // ADC2 configuration
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 16;
    ADC_Init(ADC2, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_RATEY, 1, ADC_SAMPLE_TIME);	// rateY
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_RATEY, 2, ADC_SAMPLE_TIME);	// rateY
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_RATEY, 3, ADC_SAMPLE_TIME);	// rateY
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_RATEY, 4, ADC_SAMPLE_TIME);	// rateY

    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCX, 5, ADC_SAMPLE_TIME);	// accX
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCX, 6, ADC_SAMPLE_TIME);	// accX
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCX, 7, ADC_SAMPLE_TIME);	// accX
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCX, 8, ADC_SAMPLE_TIME);	// accX

    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCY, 9, ADC_SAMPLE_TIME);	// accY
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCY, 10, ADC_SAMPLE_TIME);	// accY
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCY, 11, ADC_SAMPLE_TIME);	// accY
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCY, 12, ADC_SAMPLE_TIME);	// accY

    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCZ, 13, ADC_SAMPLE_TIME);	// accZ
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCZ, 14, ADC_SAMPLE_TIME);	// accZ
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCZ, 15, ADC_SAMPLE_TIME);	// accZ
    ADC_RegularChannelConfig(ADC2, ADC_CHANNEL_ACCZ, 16, ADC_SAMPLE_TIME);	// accZ

    // ADC3 configuration
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 16;
    ADC_Init(ADC3, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_RATEZ, 1, ADC_SAMPLE_TIME);	// rateZ
    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_RATEZ, 2, ADC_SAMPLE_TIME);	// rateZ
    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_RATEZ, 3, ADC_SAMPLE_TIME);	// rateZ
    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_RATEZ, 4, ADC_SAMPLE_TIME);	// rateZ

    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_TEMP1, 5, ADC_SAMPLE_TIME);	// temp1
    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_TEMP2, 6, ADC_SAMPLE_TIME);	// temp2

    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_PRES1, 7, ADC_SAMPLE_TIME);	// pressure1
    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_PRES1, 8, ADC_SAMPLE_TIME);	// pressure1
    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_PRES1, 9, ADC_SAMPLE_TIME);	// pressure1
    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_PRES1, 10, ADC_SAMPLE_TIME);	// pressure1

    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_VIN, 11, ADC_SAMPLE_TIME);	// Vin
    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_TEMP3, 12, ADC_SAMPLE_TIME);	// temp3

    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_PRES2, 13, ADC_SAMPLE_TIME);	// pressure2
    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_PRES2, 14, ADC_SAMPLE_TIME);	// pressure2
    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_PRES2, 15, ADC_SAMPLE_TIME);	// pressure2
    ADC_RegularChannelConfig(ADC3, ADC_CHANNEL_PRES2, 16, ADC_SAMPLE_TIME);	// pressure2

    // Enable DMA request after last transfer (Multi-ADC mode)
    ADC_MultiModeDMARequestAfterLastTransferCmd(ENABLE);

    // Enable
    ADC_Cmd(ADC1, ENABLE);
    ADC_Cmd(ADC2, ENABLE);
    ADC_Cmd(ADC3, ENABLE);

    adcData.adcFlag = CoCreateFlag(1, 0);
    adcTaskStack = aqStackInit(ADC_STACK_SIZE, "ADC");

    adcData.adcTask = CoCreateTask(adcTaskCode, (void *)0, ADC_PRIORITY, &adcTaskStack[ADC_STACK_SIZE-1], ADC_STACK_SIZE);

    // Start ADC1 Software Conversion
    ADC_SoftwareStartConv(ADC1);

    yield(100);

    // set initial temperatures
    adcData.temp1 = adcIDGVoltsToTemp(adcData.voltages[ADC_VOLTS_TEMP1]);
    adcData.temp2 = adcIDGVoltsToTemp(adcData.voltages[ADC_VOLTS_TEMP2]);
    adcData.temp3 = adcT1VoltsToTemp(adcData.voltages[ADC_VOLTS_TEMP3]);
    analogData.vIn = adcVsenseToVin(adcData.voltages[ADC_VOLTS_VIN]);

    adcCalibOffsets();
}

#pragma GCC optimize ("-O1")
// every ~15.25us
void ADC_DMA_HANDLER(void) {
    register uint32_t flag = ADC_DMA_ISR;
    register unsigned long *s, *a;
    register uint16_t *w;
    int i;

    // clear intr flags
    ADC_DMA_CR = (uint32_t)ADC_DMA_FLAGS;

    // second half?
    w = ((flag & ADC_DMA_TC_FLAG) == RESET) ? adcDMAData.adc123Raw1 : adcDMAData.adc123Raw2;

    // accumulate totals
    s = adcData.interrupt123Sums;
    *s++ += (w[36] + w[39] + w[42] + w[45]);	// rateX
    *s++ += (w[1]  + w[4]  + w[7]  + w[10]);	// rateY
    *s++ += (w[2]  + w[5]  + w[8]  + w[11]);	// rateZ
    *s++ += (w[0]  + w[3]  + w[6]  + w[9]);	// magX
    *s++ += (w[12] + w[15] + w[18] + w[21]);	// magY
    *s++ += (w[24] + w[27] + w[30] + w[33]);	// magZ
    *s++ += (w[14]);				// temp1
    *s++ += (w[32]);				// Vin
    *s++ += (w[13] + w[16] + w[19] + w[22]);	// accX
    *s++ += (w[25] + w[28] + w[31] + w[34]);	// accY
    *s++ += (w[37] + w[40] + w[43] + w[46]);	// accZ
    *s++ += (w[20] + w[23] + w[26] + w[29]);	// press1
    *s++ += (w[38] + w[41] + w[44] + w[47]);	// press2
    *s++ += (w[17]);				// temp2
    *s   += (w[35]);				// temp3

    if (++adcData.sample == ADC_SAMPLES) {
	register unsigned long micros = timerMicros();
	adcData.sample = 0;

	if (++adcData.loops & 0x01) {
	    if (ADC_MAG_SIGN) {
		digitalLo(adcData.magSetReset);
	    }
	    else {
		digitalHi(adcData.magSetReset);
	    }
	}

	s = adcData.interrupt123Sums;
	a = (unsigned long *)adcData.adcSums;
	for (i = 0; i < ADC_SENSORS; i++) {
	    *a++ = *s;
	    *s++ = 0;
	}

	adcData.sampleTime = micros - adcData.lastSample;
	adcData.lastSample = micros;

	CoEnterISR();
	isr_SetFlag(adcData.adcFlag);
	CoExitISR();
    }
}
#endif	// HAS_AIMU
