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

#ifndef _adc_h
#define _adc_h

#include "digital.h"
#include <CoOS.h>

#define ADC_STACK_SIZE		130
#define ADC_PRIORITY		10

// 400 Hz inner loop

// 153.6Khz sample rate
//#define ADC_SAMPLES		96
//#define ADC_SAMPLE_TIME	ADC_SampleTime_56Cycles

// 262.4Khz sample rate
#define ADC_SAMPLES		164
#define ADC_SAMPLE_TIME	ADC_SampleTime_28Cycles

#define ADC_SENSORS		15
#define ADC_CHANNELS		16

#define ADC_REF_VOLTAGE		3.3f
#define ADC_DIVISOR		(ADC_REF_VOLTAGE / 4096.0f / ADC_SAMPLES)

#define ADC_VIN_SLOPE		((ANALOG_VIN_RTOP + ANALOG_VIN_RBOT) / ANALOG_VIN_RBOT)
#define ADC_VIN_OFFSET		0.0f

#define ADC_IDG_TEMP_OFFSET	1.25f			    // volts (IDG500 PTATS)
#define ADC_IDG_TEMP_SLOPE	(1.0f / 0.004f)		    // deg C / volts

#define ADC_TEMP1_OFFSET	+0.0f
#define ADC_TEMP2_OFFSET	+0.0f
#define ADC_TEMP3_OFFSET	+0.0f

#define ADC_TEMP_A		+167.5358f
#define ADC_TEMP_B		+6.6871f
#define ADC_TEMP_C		+0.0347f
#define ADC_TEMP_R2		100000.0f		    // ohms
#define ADC_KELVIN_TO_CELCIUS	-273.0f

#define ADC_TEMP_SMOOTH		0.0113f

#define ADC_RATE_CALIB_SAMPLES	25

#define ADC_MAG_SIGN		digitalGet(adcData.magSetReset)

#define	ADC_CHANNEL_RATEX	ADC_Channel_8
#define	ADC_CHANNEL_RATEY	ADC_Channel_9
#define	ADC_CHANNEL_RATEZ	ADC_Channel_3
#define	ADC_CHANNEL_ACCX	ADC_Channel_4
#define	ADC_CHANNEL_ACCY	ADC_Channel_5
#define	ADC_CHANNEL_ACCZ	ADC_Channel_6
#define	ADC_CHANNEL_MAGX	ADC_Channel_7
#define	ADC_CHANNEL_MAGY	ADC_Channel_14
#define	ADC_CHANNEL_MAGZ	ADC_Channel_15
#define	ADC_CHANNEL_PRES1	ADC_Channel_2
#define	ADC_CHANNEL_PRES2	ADC_Channel_0
#define	ADC_CHANNEL_TEMP1	ADC_Channel_10
#define	ADC_CHANNEL_TEMP2	ADC_Channel_11
#define	ADC_CHANNEL_TEMP3	ADC_Channel_12
#define	ADC_CHANNEL_VIN		ADC_Channel_13

#define ADC_VOLTS_RATEX		0
#define ADC_VOLTS_RATEY		1
#define ADC_VOLTS_RATEZ		2
#define ADC_VOLTS_MAGX		3
#define ADC_VOLTS_MAGY		4
#define ADC_VOLTS_MAGZ		5
#define ADC_VOLTS_TEMP1		6
#define ADC_VOLTS_VIN		7
#define ADC_VOLTS_ACCX		8
#define ADC_VOLTS_ACCY		9
#define ADC_VOLTS_ACCZ		10
#define ADC_VOLTS_PRES1		11
#define ADC_VOLTS_PRES2		12
#define ADC_VOLTS_TEMP2		13
#define ADC_VOLTS_TEMP3		14

typedef struct {
    OS_TID adcTask;
    OS_FlagID adcFlag;

    unsigned long interrupt123Sums[ADC_SENSORS];

    unsigned long volatile adcSums[ADC_SENSORS];
    unsigned long channelSums[ADC_SENSORS];
    float volatile voltages[ADC_SENSORS];

    float temp1, temp2, temp3;
    float temperature;
    float pressure1;
    float pressure2;
    float pressure;

    float volatile dRateX, dRateY, dRateZ;
    float volatile rateX, rateY, rateZ;
    float volatile accX, accY, accZ;
    float volatile magX, magY, magZ;

    float magBridgeBiasX, magBridgeBiasY, magBridgeBiasZ;
    float rateBiasX, rateBiasY, rateBiasZ;

    digitalPin *magSetReset;
    digitalPin *rateAutoZero;
    digitalPin *accST;
    digitalPin *accScale;

    volatile unsigned long sample;
    volatile unsigned long loops;
    volatile unsigned long lastUpdate;
    volatile unsigned long lastSample;
    volatile unsigned long sampleTime;
    float dt;
    int8_t magSign;
} adcStruct_t;

extern adcStruct_t adcData;

extern void adcInit(void);
extern void adcClearCalibration(void);
extern float adcPressureAdjust(float altitude);
extern void adcCalcRot(void);
extern float adcPresToAlt(float pressure);
extern float adcVoltsToTemp(float volts);

#endif
