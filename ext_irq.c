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

#include "stm32f4xx.h"
#include "ext_irq.h"
#include "comm.h"

extStruct_t extData;

int extRegisterCallback(GPIO_TypeDef *port, uint16_t pin, EXTITrigger_TypeDef polarity, uint8_t priority, GPIOPuPd_TypeDef pull, extCallback_t *callback) {
    GPIO_InitTypeDef GPIO_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint32_t extiLine;
    uint8_t exitPortSource;
    uint8_t exitPinSource;
    IRQn_Type nvicChannel;
    int ret;

    if (port == GPIOA) {
        exitPortSource = EXTI_PortSourceGPIOA;
    }
    else if (port == GPIOB) {
        exitPortSource = EXTI_PortSourceGPIOB;
    }
    else if (port == GPIOC) {
        exitPortSource = EXTI_PortSourceGPIOC;
    }
    else if (port == GPIOD) {
        exitPortSource = EXTI_PortSourceGPIOD;
    }
    else if (port == GPIOE) {
        exitPortSource = EXTI_PortSourceGPIOE;
    }
    else {
        AQ_NOTICE("EXT: invalid port\n");
        return -1;
    }

    if (pin == GPIO_Pin_0) {
        exitPinSource = EXTI_PinSource0;
        extiLine = EXTI_Line0;
        nvicChannel = EXTI0_IRQn;
        ret = 0;
    }
    else if (pin == GPIO_Pin_1) {
        exitPinSource = EXTI_PinSource1;
        extiLine = EXTI_Line1;
        nvicChannel = EXTI1_IRQn;
        ret = 1;
    }
    else if (pin == GPIO_Pin_2) {
        exitPinSource = EXTI_PinSource2;
        extiLine = EXTI_Line2;
        nvicChannel = EXTI2_IRQn;
        ret = 2;
    }
    else if (pin == GPIO_Pin_3) {
        exitPinSource = EXTI_PinSource3;
        extiLine = EXTI_Line3;
        nvicChannel = EXTI3_IRQn;
        ret = 3;
    }
    else if (pin == GPIO_Pin_4) {
        exitPinSource = EXTI_PinSource4;
        extiLine = EXTI_Line4;
        nvicChannel = EXTI4_IRQn;
        ret = 4;
    }
    else if (pin == GPIO_Pin_5) {
        exitPinSource = EXTI_PinSource5;
        extiLine = EXTI_Line5;
        nvicChannel = EXTI9_5_IRQn;
        ret = 5;
    }
    else if (pin == GPIO_Pin_6) {
        exitPinSource = EXTI_PinSource6;
        extiLine = EXTI_Line6;
        nvicChannel = EXTI9_5_IRQn;
        ret = 6;
    }
    else if (pin == GPIO_Pin_7) {
        exitPinSource = EXTI_PinSource7;
        extiLine = EXTI_Line7;
        nvicChannel = EXTI9_5_IRQn;
        ret = 7;
    }
    else if (pin == GPIO_Pin_8) {
        exitPinSource = EXTI_PinSource8;
        extiLine = EXTI_Line8;
        nvicChannel = EXTI9_5_IRQn;
        ret = 8;
    }
    else if (pin == GPIO_Pin_9) {
        exitPinSource = EXTI_PinSource9;
        extiLine = EXTI_Line9;
        nvicChannel = EXTI9_5_IRQn;
        ret = 9;
    }
    else if (pin == GPIO_Pin_10) {
        exitPinSource = EXTI_PinSource10;
        extiLine = EXTI_Line10;
        nvicChannel = EXTI15_10_IRQn;
        ret = 10;
    }
    else if (pin == GPIO_Pin_11) {
        exitPinSource = EXTI_PinSource11;
        extiLine = EXTI_Line11;
        nvicChannel = EXTI15_10_IRQn;
        ret = 11;
    }
    else if (pin == GPIO_Pin_12) {
        exitPinSource = EXTI_PinSource12;
        extiLine = EXTI_Line12;
        nvicChannel = EXTI15_10_IRQn;
        ret = 12;
    }
    else if (pin == GPIO_Pin_13) {
        exitPinSource = EXTI_PinSource13;
        extiLine = EXTI_Line13;
        nvicChannel = EXTI15_10_IRQn;
        ret = 13;
    }
    else if (pin == GPIO_Pin_14) {
        exitPinSource = EXTI_PinSource14;
        extiLine = EXTI_Line14;
        nvicChannel = EXTI15_10_IRQn;
        ret = 14;
    }
    else if (pin == GPIO_Pin_15) {
        exitPinSource = EXTI_PinSource15;
        extiLine = EXTI_Line15;
        nvicChannel = EXTI15_10_IRQn;
        ret = 15;
    }
    else {
        AQ_NOTICE("EXT: invalid pin\n");
        return -1;
    }

    if (extData.callbacks[ret]) {
        AQ_NOTICE("EXT: external irq conflict\n");
        return -1;
    }
    else {
        extData.callbacks[ret] = callback;
    }

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = pull;
    GPIO_Init(port, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(exitPortSource, exitPinSource);

    EXTI_InitStructure.EXTI_Line = extiLine;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = polarity;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = nvicChannel;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = priority;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    return ret;
}

void EXTI0_IRQHandler(void) {
    EXTI->PR = EXTI_Line0;
    if (extData.callbacks[0])
        extData.callbacks[0]();
}

void EXTI1_IRQHandler(void) {
    EXTI->PR = EXTI_Line1;
    if (extData.callbacks[1])
        extData.callbacks[1]();
}

void EXTI2_IRQHandler(void) {
    EXTI->PR = EXTI_Line2;
    if (extData.callbacks[2])
        extData.callbacks[2]();
}

void EXTI3_IRQHandler(void) {
    EXTI->PR = EXTI_Line3;
    if (extData.callbacks[3])
        extData.callbacks[3]();
}

void EXTI4_IRQHandler(void) {
    EXTI->PR = EXTI_Line4;
    if (extData.callbacks[4])
        extData.callbacks[4]();
}

void EXTI9_5_IRQHandler(void) {
    if (EXTI->PR & EXTI_Line5) {
        EXTI->PR = EXTI_Line5;
        if (extData.callbacks[5])
            extData.callbacks[5]();
    }
    else if (EXTI->PR & EXTI_Line6) {
        EXTI->PR = EXTI_Line6;
        if (extData.callbacks[6])
            extData.callbacks[6]();
    }
    else if (EXTI->PR & EXTI_Line7) {
        EXTI->PR = EXTI_Line7;
        if (extData.callbacks[7])
            extData.callbacks[7]();
    }
    else if (EXTI->PR & EXTI_Line8) {
        EXTI->PR = EXTI_Line8;
        if (extData.callbacks[8])
            extData.callbacks[8]();
    }
    else if (EXTI->PR & EXTI_Line9) {
        EXTI->PR = EXTI_Line9;
        if (extData.callbacks[9])
            extData.callbacks[9]();
    }
}

void EXTI15_10_IRQHandler(void) {
    if (EXTI->PR & EXTI_Line10) {
        EXTI->PR = EXTI_Line10;
        if (extData.callbacks[10])
            extData.callbacks[10]();
    }
    else if (EXTI->PR & EXTI_Line11) {
        EXTI->PR = EXTI_Line11;
        if (extData.callbacks[11])
            extData.callbacks[11]();
    }
    else if (EXTI->PR & EXTI_Line12) {
        EXTI->PR = EXTI_Line12;
        if (extData.callbacks[12])
            extData.callbacks[12]();
    }
    else if (EXTI->PR & EXTI_Line13) {
        EXTI->PR = EXTI_Line13;
        if (extData.callbacks[13])
            extData.callbacks[13]();
    }
    else if (EXTI->PR & EXTI_Line14) {
        EXTI->PR = EXTI_Line14;
        if (extData.callbacks[14])
            extData.callbacks[14]();
    }
    else if (EXTI->PR & EXTI_Line15) {
        EXTI->PR = EXTI_Line15;
        if (extData.callbacks[15])
            extData.callbacks[15]();
    }
}
