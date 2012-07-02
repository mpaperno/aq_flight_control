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

#include "aq.h"
#include "pwm.h"
#include "1wire.h"
#include "notice.h"
#include "util.h"
#include <stdio.h>

pwmStruct_t pwmData[PWM_NUM_PORTS] __attribute__((section(".ccm")));

PWM_TIMERS;
PWM_AFS;
PWM_PORTS;
PWM_PINS;
PWM_PINSOURCES;
PWM_TIMERCHANNELS;
PWM_BDTRS;
PWM_CLOCKS;

void pwmTimeBase(const TIM_TypeDef *tim, uint32_t period, uint16_t prescaler) {
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

    // Time base configuration
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = period - 1;
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = 0;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit((TIM_TypeDef *)tim, &TIM_TimeBaseStructure);
}

void pwmOCInit(const TIM_TypeDef *tim, uint8_t channel, uint32_t inititalValue) {
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = inititalValue;

    switch (channel) {
	case 1:
	    TIM_OC1Init((TIM_TypeDef *)tim, &TIM_OCInitStructure);
	    break;
	case 2:
	    TIM_OC2Init((TIM_TypeDef *)tim, &TIM_OCInitStructure);
	    break;
	case 3:
	    TIM_OC3Init((TIM_TypeDef *)tim, &TIM_OCInitStructure);
	    break;
	case 4:
	    TIM_OC4Init((TIM_TypeDef *)tim, &TIM_OCInitStructure);
	    break;
    }
}

// set the output enable - this is only required for TIM1 | TIM8
void pwmBDTRInit(const TIM_TypeDef *tim) {
    TIM_BDTRInitTypeDef TIM_BDTRInitStruct;

    TIM_BDTRStructInit(&TIM_BDTRInitStruct);
    TIM_BDTRInitStruct.TIM_LOCKLevel = TIM_LOCKLevel_OFF;
    TIM_BDTRInitStruct.TIM_AutomaticOutput = TIM_AutomaticOutput_Enable;
    TIM_BDTRInitStruct.TIM_OSSIState = TIM_OSSIState_Enable;
    TIM_BDTRConfig((TIM_TypeDef *)tim, &TIM_BDTRInitStruct);
}

void pwmGPIOInit(const GPIO_TypeDef *gpio, uint32_t pin) {
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init((GPIO_TypeDef *)gpio, &GPIO_InitStructure);
}

float pwmESC32ReadParam(uint8_t paramId) {
    float value;
    uint8_t *p;

    owData.buf[0] = OW_PARAM_READ;
    owData.buf[1] = paramId;
    owTransaction(2, 6);

    p = (uint8_t *)&value;
    p[0] = owData.buf[2];
    p[1] = owData.buf[3];
    p[2] = owData.buf[4];
    p[3] = owData.buf[5];

    return value;
}

float pwmESC32WriteParam(uint8_t paramId, float value) {
    uint8_t *p;

    owData.buf[0] = OW_PARAM_WRITE;
    owData.buf[1] = paramId;

    p = (uint8_t *)&value;
    owData.buf[2] = p[0];
    owData.buf[3] = p[1];
    owData.buf[4] = p[2];
    owData.buf[5] = p[3];
    owTransaction(6, 6);

    return pwmESC32ReadParam(paramId);
}

// set ESC32 mode
uint8_t pwmSetESC32Mode(uint8_t ESC32Mode) {
    owData.buf[0] = OW_SET_MODE;
    owData.buf[1] = ESC32Mode;
    owTransaction(2, 2);

    owData.buf[0] = OW_GET_MODE;
    owData.buf[1] = 0;
    owTransaction(1, 2);

    if (owData.buf[0] != OW_GET_MODE)
	return -1;
    else
	return owData.buf[1];
}

void pwmESC32Setup(const GPIO_TypeDef *port, const uint16_t pin, uint8_t ESC32Mode) {
    char s[32];

    owInit((GPIO_TypeDef *)port, pin);

    // get ESC32 version
    owData.buf[0] = OW_VERSION;
    owTransaction(1, 16);

    if (owData.status != OW_STATUS_NO_PRESENSE) {
	sprintf(s, "ESC32 ver: %s\n", owData.buf);
	AQ_NOTICE(s);

	// set parameters
	if (pwmESC32ReadParam(ESC32_STARTUP_MODE) != (float)ESC32Mode) {
	    if (pwmESC32WriteParam(ESC32_STARTUP_MODE, (float)ESC32Mode) != (float)ESC32Mode)
		AQ_NOTICE("ESC32: failed to write ESC32_STARTUP_MODE parameter\n");

	    // write to flash
	    owData.buf[0] = OW_CONFIG_WRITE;
	    owTransaction(1, 0);

	    AQ_NOTICE("ESC32: stored config to flash\n");

	    // wait for flash to finish
	    yield(250);

	    if (pwmSetESC32Mode(ESC32Mode) != ESC32Mode)
		    AQ_NOTICE("ESC32: failed to set mode\n");
	}
    }
}

// note - assumes all timer clocks have been enable during system startup
pwmStruct_t *pwmInit(uint8_t pwmPort, uint32_t period, uint8_t direction, uint32_t inititalValue, int8_t ESC32Mode) {
    pwmStruct_t *p = 0;

    if (pwmPort < PWM_NUM_PORTS) {
	p = &pwmData[pwmPort];

	pwmTimeBase(pwmTimers[pwmPort], period, pwmClocks[pwmPort] / 1000000);

	// set ESC32 mode via 1-wire protocol if necessary before activating PWM output
	if (ESC32Mode >= 0)
	    pwmESC32Setup(pwmPorts[pwmPort], pwmPins[pwmPort], ESC32Mode);

	pwmGPIOInit(pwmPorts[pwmPort], pwmPins[pwmPort]);
	GPIO_PinAFConfig((GPIO_TypeDef *)pwmPorts[pwmPort], pwmPinSources[pwmPort], pwmAFs[pwmPort]);

	if (direction == PWM_OUTPUT) {
	    pwmOCInit(pwmTimers[pwmPort], pwmTimerChannels[pwmPort], inititalValue);
	    if (pwmBDTRs[pwmPort])
		pwmBDTRInit(pwmTimers[pwmPort]);
	}

	TIM_Cmd((TIM_TypeDef *)pwmTimers[pwmPort], ENABLE);

	switch (pwmTimerChannels[pwmPort]) {
	    case 1:
		p->ccr = (volatile uint32_t *)&pwmTimers[pwmPort]->CCR1;
		break;
	    case 2:
		p->ccr = (volatile uint32_t *)&pwmTimers[pwmPort]->CCR2;
		break;
	    case 3:
		p->ccr = (volatile uint32_t *)&pwmTimers[pwmPort]->CCR3;
		break;
	    case 4:
		p->ccr = (volatile uint32_t *)&pwmTimers[pwmPort]->CCR4;
		break;
	}
	p->cnt = (volatile uint32_t *)&pwmTimers[pwmPort]->CNT;
    }

    return p;
}
