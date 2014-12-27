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

#include "analog.h"
#include "comm.h"
#include "util.h"
#include "config.h"
#include <stdio.h>

analogStruct_t analogData;

#ifdef ANALOG_DMA_STREAM
void analogDecode(void) {
    int i, j;

    for (i = 0; i < ANALOG_CHANNELS; i++)
	analogData.rawChannels[i] = 0;

    for (i = 0; i < ANALOG_SAMPLES; i++)
	for (j = 0; j < ANALOG_CHANNELS; j++)
	    analogData.rawChannels[j] += analogData.rawSamples[i * ANALOG_CHANNELS + j];

    for (i = 0; i < ANALOG_CHANNELS; i++)
	analogData.voltages[i] = analogData.rawChannels[i] * ANALOG_DIVISOR;

    analogData.vIn = analogData.voltages[analogData.vInSourceIndex] * analogData.vInSlope;
#ifdef ANALOG_EXT_VOLT_SLOPE
    analogData.extVolt = analogData.voltages[ANALOG_VOLTS_EXT_VOLT] * ANALOG_EXT_VOLT_SLOPE;
#endif
#ifdef ANALOG_EXT_AMP_SLOPE
    analogData.extAmp = analogData.voltages[ANALOG_VOLTS_EXT_AMP] * ANALOG_EXT_AMP_SLOPE;
#endif
}
#endif

void analogInit(void) {

#ifdef ANALOG_DMA_STREAM

    analogData.vInSourceIndex = ANALOG_VOLTS_VIN;
    analogData.vInSlope = ANALOG_VIN_SLOPE;
#ifdef ANALOG_EXT_VOLT_SLOPE
    if ((int)p[SPVR_VIN_SOURCE] == 1) {
	analogData.vInSourceIndex = ANALOG_VOLTS_EXT_VOLT;
	analogData.vInSlope = ANALOG_EXT_VOLT_SLOPE;
    }
#endif

    DMA_InitTypeDef DMA_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    DMA_DeInit(ANALOG_DMA_STREAM);
    DMA_InitStructure.DMA_Channel = ANALOG_DMA_CHANNEL;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&analogData.rawSamples;
    DMA_InitStructure.DMA_PeripheralBaseAddr = ((uint32_t)ADC1+0x4c);
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = ANALOG_SAMPLES*ANALOG_CHANNELS;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(ANALOG_DMA_STREAM, &DMA_InitStructure);

    DMA_Cmd(ANALOG_DMA_STREAM, ENABLE);

    // ADC Common Init
    ADC_CommonStructInit(&ADC_CommonInitStructure);
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div2;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ADC1 configuration
    ADC_StructInit(&ADC_InitStructure);
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = ENABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = ANALOG_CHANNELS;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_RegularChannelConfig(ADC1, ANALOG_CHANNEL_VIN, 1, ANALOG_SAMPLE_TIME);
    ADC_RegularChannelConfig(ADC1, ANALOG_CHANNEL_EXT_VOLT, 2, ANALOG_SAMPLE_TIME);
    ADC_RegularChannelConfig(ADC1, ANALOG_CHANNEL_EXT_AMP, 3, ANALOG_SAMPLE_TIME);
    ADC_RegularChannelConfig(ADC1, ANALOG_CHANNEL_SPARE4, 4, ANALOG_SAMPLE_TIME);

    ADC_RegularChannelConfig(ADC1, ANALOG_CHANNEL_SPARE5, 5, ANALOG_SAMPLE_TIME);
    ADC_RegularChannelConfig(ADC1, ANALOG_CHANNEL_SPARE6, 6, ANALOG_SAMPLE_TIME);
    ADC_RegularChannelConfig(ADC1, ANALOG_CHANNEL_SPARE7, 7, ANALOG_SAMPLE_TIME);

    ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE);

    ADC_DMACmd(ADC1, ENABLE);
    ADC_Cmd(ADC1, ENABLE);
    ADC_SoftwareStartConv(ADC1);

    yield(10);
    analogDecode();
#endif // ANALOG_DMA_STREAM

    // determine LiPo battery cell count
    if (analogData.vIn < 5.0f)
	analogData.batCellCount = 1;
    else if (analogData.vIn < 8.5f)
	analogData.batCellCount = 2;
    else if (analogData.vIn < 12.8f)
	analogData.batCellCount = 3;
    else if (analogData.vIn < 17.0f)
	analogData.batCellCount = 4;
    else if (analogData.vIn < 21.3f)
	analogData.batCellCount = 5;
    else if (analogData.vIn < 25.8f)
	analogData.batCellCount = 6;

    AQ_PRINTF("Battery cells: %d\n", analogData.batCellCount);
}
