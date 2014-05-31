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
#include "pwm.h"
#include "esc32.h"
#include "comm.h"
#include "util.h"
#include <stdio.h>

pwmPortStruct_t pwmData[PWM_NUM_PORTS] __attribute__((section(".ccm")));

// these are defined in the board header file
PWM_TIMERS;
PWM_AFS;
PWM_PORTS;
PWM_PINS;
PWM_PINSOURCES;
PWM_TIMERCHANNELS;
PWM_BDTRS;
PWM_CLOCKS;
PWM_IC_IRQS;

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

void pwmOCInit(const TIM_TypeDef *tim, uint16_t channel, uint32_t inititalValue) {
    TIM_OCInitTypeDef  TIM_OCInitStructure;

    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
    TIM_OCInitStructure.TIM_Pulse = inititalValue;

    switch (channel) {
	case TIM_Channel_1:
	    TIM_OC1Init((TIM_TypeDef *)tim, &TIM_OCInitStructure);
	    break;
	case TIM_Channel_2:
	    TIM_OC2Init((TIM_TypeDef *)tim, &TIM_OCInitStructure);
	    break;
	case TIM_Channel_3:
	    TIM_OC3Init((TIM_TypeDef *)tim, &TIM_OCInitStructure);
	    break;
	case TIM_Channel_4:
	    TIM_OC4Init((TIM_TypeDef *)tim, &TIM_OCInitStructure);
	    break;
    }
}

void pwmICInit(const TIM_TypeDef *tim, uint16_t channel, uint16_t polarity) {
    TIM_ICInitTypeDef  TIM_ICInitStructure;

    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_Channel = channel;
    TIM_ICInitStructure.TIM_ICPolarity = polarity;
    TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStructure.TIM_ICFilter = 0x08;

    TIM_ICInit((TIM_TypeDef *)tim, &TIM_ICInitStructure);
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

void pwmGPIOInit(const GPIO_TypeDef *gpio, uint32_t pin, GPIOMode_TypeDef mode) {
    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = mode;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init((GPIO_TypeDef *)gpio, &GPIO_InitStructure);
}

int pwmValidatePort(uint8_t pwmPort, uint32_t period) {
    int ret = 0;

    if (pwmPort >= PWM_NUM_PORTS) {
	AQ_NOTICE("pwm: faliure: invalid pwm port number!\n");
    }
    else if (pwmData[pwmPort].direction != 0) {
	AQ_NOTICE("pwm: faliure: cannot re-allocate port!\n");
    }
    else if (period && pwmData[pwmPort].period && pwmData[pwmPort].period != period) {
	AQ_NOTICE("pwm: faliure: pwm frequency mis-match!\n");
    }
    else {
	 ret = 1;
    }

    return ret;
}

// Sets the TIMx prescaler values to zero to facilitate detection of used timers.
void pwmZeroTimers(void) {
    int i;

    for (i = 0; i < PWM_NUM_PORTS; i++)
	TIM_PrescalerConfig((TIM_TypeDef *) pwmTimers[i], 0, TIM_PSCReloadMode_Immediate);
}

// returns configured timer prescaler value for given port
// should be zero if timer is not used
uint16_t pwmCheckTimer(uint8_t pwmPort) {
    return TIM_GetPrescaler((TIM_TypeDef *) pwmTimers[pwmPort]);
}

// note - assumes all timer clocks have been enable during system startup
pwmPortStruct_t *pwmInitOut(uint8_t pwmPort, uint32_t resolution, uint32_t freq, uint32_t inititalValue, int8_t ESC32Mode) {
    uint32_t period = resolution / freq;
    pwmPortStruct_t *p = 0;

    if (pwmValidatePort(pwmPort, period)) {
	p = &pwmData[pwmPort];
	p->direction = PWM_OUTPUT;

	pwmTimeBase(pwmTimers[pwmPort], period, pwmClocks[pwmPort] / resolution);

	// set ESC32 mode via 1-wire protocol if necessary before activating PWM output
	if (ESC32Mode > 0)
	    esc32SetupOw(pwmPorts[pwmPort], pwmPins[pwmPort], ESC32Mode);

	pwmOCInit(pwmTimers[pwmPort], pwmTimerChannels[pwmPort], inititalValue);
	if (pwmBDTRs[pwmPort])
	    pwmBDTRInit(pwmTimers[pwmPort]);

	TIM_Cmd((TIM_TypeDef *)pwmTimers[pwmPort], ENABLE);

	switch (pwmTimerChannels[pwmPort]) {
	    case TIM_Channel_1:
		p->ccr = (volatile uint32_t *)&pwmTimers[pwmPort]->CCR1;
		TIM_OC1PreloadConfig((TIM_TypeDef *)pwmTimers[pwmPort], TIM_OCPreload_Enable);
		break;
	    case TIM_Channel_2:
		p->ccr = (volatile uint32_t *)&pwmTimers[pwmPort]->CCR2;
		TIM_OC2PreloadConfig((TIM_TypeDef *)pwmTimers[pwmPort], TIM_OCPreload_Enable);
		break;
	    case TIM_Channel_3:
		p->ccr = (volatile uint32_t *)&pwmTimers[pwmPort]->CCR3;
		TIM_OC3PreloadConfig((TIM_TypeDef *)pwmTimers[pwmPort], TIM_OCPreload_Enable);
		break;
	    case TIM_Channel_4:
		p->ccr = (volatile uint32_t *)&pwmTimers[pwmPort]->CCR4;
		TIM_OC4PreloadConfig((TIM_TypeDef *)pwmTimers[pwmPort], TIM_OCPreload_Enable);
		break;
	}

	p->cnt = (volatile uint32_t *)&pwmTimers[pwmPort]->CNT;

	// finally allow the timer access to the port
	GPIO_PinAFConfig((GPIO_TypeDef *)pwmPorts[pwmPort], pwmPinSources[pwmPort], pwmAFs[pwmPort]);
	pwmGPIOInit(pwmPorts[pwmPort], pwmPins[pwmPort], GPIO_Mode_AF);
    }

    return p;
}


pwmPortStruct_t *pwmInitDigitalOut(uint8_t pwmPort) {
    pwmPortStruct_t *p = 0;

    if (pwmValidatePort(pwmPort, 0)) {
	p = &pwmData[pwmPort];
	p->direction = PWM_OUTPUT;
	p->pin = pwmPins[pwmPort];
	p->port = (GPIO_TypeDef *)pwmPorts[pwmPort];

	pwmDigitalLo(p);

	pwmGPIOInit(pwmPorts[pwmPort], pwmPins[pwmPort], GPIO_Mode_OUT);
    }

    return p;
}

void pwmDigitalToggle(pwmPortStruct_t *p) {
    if (pwmDigitalGet(p)) {
	pwmDigitalLo(p);
    } else {
	pwmDigitalHi(p);
    }
}

void pwmNVICInit(uint8_t irqChannel) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_InitStructure.NVIC_IRQChannel = irqChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
}

pwmPortStruct_t *pwmInitIn(uint8_t pwmPort, int16_t polarity, uint32_t period, pwmCallback_t callback) {
    pwmPortStruct_t *p = 0;

    if (pwmValidatePort(pwmPort, 0) && callback) {
	p = &pwmData[pwmPort];
    	p->direction = PWM_INPUT;
	p->callback = callback;

	pwmTimeBase(pwmTimers[pwmPort], period, pwmClocks[pwmPort] / 1000000);

	pwmGPIOInit(pwmPorts[pwmPort], pwmPins[pwmPort], GPIO_Mode_AF);
	GPIO_PinAFConfig((GPIO_TypeDef *)pwmPorts[pwmPort], pwmPinSources[pwmPort], pwmAFs[pwmPort]);

	polarity = (polarity > 0) ? TIM_ICPolarity_Rising : ((polarity < 0) ? TIM_ICPolarity_Falling : TIM_ICPolarity_BothEdge);

	pwmICInit(pwmTimers[pwmPort], pwmTimerChannels[pwmPort], (uint16_t)polarity);

	TIM_Cmd((TIM_TypeDef *)pwmTimers[pwmPort], ENABLE);

	pwmNVICInit(pwmIcIrqChannels[pwmPort]);

	switch (pwmTimerChannels[pwmPort]) {
	    case TIM_Channel_1:
		TIM_ITConfig((TIM_TypeDef *)pwmTimers[pwmPort], TIM_IT_CC1, ENABLE);
		break;
	    case TIM_Channel_2:
		TIM_ITConfig((TIM_TypeDef *)pwmTimers[pwmPort], TIM_IT_CC2, ENABLE);
		break;
	    case TIM_Channel_3:
		TIM_ITConfig((TIM_TypeDef *)pwmTimers[pwmPort], TIM_IT_CC3, ENABLE);
		break;
	    case TIM_Channel_4:
		TIM_ITConfig((TIM_TypeDef *)pwmTimers[pwmPort], TIM_IT_CC4, ENABLE);
		break;
	}
    }

    return p;
}

void TIM1_CC_IRQHandler(void) {
#ifdef PWM_IRQ_TIM1_CH1
    if (TIM_GetITStatus(TIM1, TIM_IT_CC1) == SET) {
	int8_t pwmPort = PWM_IRQ_TIM1_CH1;
	TIM_ClearITPendingBit(TIM1, TIM_IT_CC1);
	pwmData[pwmPort].callback(TIM_GetCapture1(TIM1), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
#endif
#ifdef PWM_IRQ_TIM1_CH2
    if (TIM_GetITStatus(TIM1, TIM_IT_CC2) == SET) {
	int8_t pwmPort = PWM_IRQ_TIM1_CH2;
	TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);
	pwmData[pwmPort].callback(TIM_GetCapture2(TIM1), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
#endif
#ifdef PWM_IRQ_TIM1_CH3
    if (TIM_GetITStatus(TIM1, TIM_IT_CC3) == SET) {
	int8_t pwmPort = PWM_IRQ_TIM1_CH3;
	TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);
	pwmData[pwmPort].callback(TIM_GetCapture3(TIM1), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
#endif
#ifdef PWM_IRQ_TIM1_CH4
    if (TIM_GetITStatus(TIM1, TIM_IT_CC4) == SET) {
	int8_t pwmPort = PWM_IRQ_TIM1_CH4;
	TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);
	pwmData[pwmPort].callback(TIM_GetCapture4(TIM1), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
#endif
}

void TIM4_IRQHandler(void) {
#ifdef PWM_IRQ_TIM4_CH1
    if (TIM_GetITStatus(TIM4, TIM_IT_CC1) == SET) {
	int8_t pwmPort = PWM_IRQ_TIM4_CH1;
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC1);
	pwmData[pwmPort].callback(TIM_GetCapture1(TIM4), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
#endif
#ifdef PWM_IRQ_TIM4_CH2
    if (TIM_GetITStatus(TIM4, TIM_IT_CC2) == SET) {
	int8_t pwmPort = PWM_IRQ_TIM4_CH2;
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC2);
	pwmData[pwmPort].callback(TIM_GetCapture2(TIM4), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
    else
#endif
#ifdef PWM_IRQ_TIM4_CH3
    if (TIM_GetITStatus(TIM4, TIM_IT_CC3) == SET) {
	int8_t pwmPort = PWM_IRQ_TIM4_CH3;
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC3);
	pwmData[pwmPort].callback(TIM_GetCapture3(TIM4), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
#endif
#ifdef PWM_IRQ_TIM4_CH4
    if (TIM_GetITStatus(TIM4, TIM_IT_CC4) == SET) {
	int8_t pwmPort = PWM_IRQ_TIM4_CH4;
	TIM_ClearITPendingBit(TIM4, TIM_IT_CC4);
	pwmData[pwmPort].callback(TIM_GetCapture4(TIM4), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
#endif
}

void TIM1_BRK_TIM9_IRQHandler(void) {
#ifdef PWM_IRQ_TIM9_CH1
    if (TIM_GetITStatus(TIM9, TIM_IT_CC1) == SET) {
	int8_t pwmPort = PWM_IRQ_TIM9_CH1;
	TIM_ClearITPendingBit(TIM9, TIM_IT_CC1);
	pwmData[pwmPort].callback(TIM_GetCapture1(TIM9), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
#endif
#ifdef PWM_IRQ_TIM9_CH2
    if (TIM_GetITStatus(TIM9, TIM_IT_CC2) == SET) {
	int8_t pwmPort = PWM_IRQ_TIM9_CH2;
	TIM_ClearITPendingBit(TIM9, TIM_IT_CC2);
	pwmData[pwmPort].callback(TIM_GetCapture2(TIM9), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
#endif
}

#if defined(PWM_IRQ_TIM2_CH3) || defined(PWM_IRQ_TIM2_CH4)
void TIM2_IRQHandler(void) {
    int8_t pwmPort;

    if (TIM_GetITStatus(TIM2, TIM_IT_CC3) == SET) {
	pwmPort = PWM_IRQ_TIM2_CH3;
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC3);
	pwmData[pwmPort].callback(TIM_GetCapture3(TIM2), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
    else if (TIM_GetITStatus(TIM2, TIM_IT_CC4) == SET) {
	pwmPort = PWM_IRQ_TIM2_CH4;
	TIM_ClearITPendingBit(TIM2, TIM_IT_CC4);
	pwmData[pwmPort].callback(TIM_GetCapture4(TIM2), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
}
#endif

#ifdef PWM_IRQ_TIM10_CH1
void TIM1_UP_TIM10_IRQHandler(void) {
    int8_t pwmPort;

    if (TIM_GetITStatus(TIM10, TIM_IT_CC1) == SET) {
	pwmPort = PWM_IRQ_TIM10_CH1;
	TIM_ClearITPendingBit(TIM10, TIM_IT_CC1);
	pwmData[pwmPort].callback(TIM_GetCapture1(TIM10), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
}
#endif

#ifdef PWM_IRQ_TIM11_CH1
void TIM1_TRG_COM_TIM11_IRQHandler(void) {
    int8_t pwmPort;

    if (TIM_GetITStatus(TIM11, TIM_IT_CC1) == SET) {
	pwmPort = PWM_IRQ_TIM11_CH1;
	TIM_ClearITPendingBit(TIM11, TIM_IT_CC1);
	pwmData[pwmPort].callback(TIM_GetCapture1(TIM11), GPIO_ReadInputDataBit((GPIO_TypeDef *)pwmPorts[pwmPort], (uint16_t)pwmPins[pwmPort]));
    }
}
#endif
