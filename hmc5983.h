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

#ifndef _hmc5983_h
#define _hmc5983_h

#include "spi.h"

#define HMC5983_INT_PORT	    GPIOE
#define HMC5983_INT_PIN		    GPIO_Pin_2
#define HMC5983_INT_EXTI_PORT	    EXTI_PortSourceGPIOE
#define HMC5983_INT_EXTI_PIN	    EXTI_PinSource2
#define HMC5983_INT_EXTI_LINE	    EXTI_Line2
#define HMC5983_INT_EXTI_IRQ	    EXTI2_IRQn
#define HMC5983_INT_ISR		    EXTI2_IRQHandler

#define HMC5983_SPI		    SPI2
#define HMC5983_SPI_BAUD	    SPI_BaudRatePrescaler_8	// 5.25 Mhz
#define HMC5983_CS_PORT		    GPIOD
#define HMC5983_CS_PIN		    GPIO_Pin_10

#define HMC5983_BYTES		    (1+6)
#define HMC5983_SLOTS		    4				// 55Hz bandwidth

#define HMC5983_READ_BIT	    (0b10000000)
#define HMC5983_READ_MULT_BIT	    (0b11000000)
#define HMC5983_WRITE_BIT	    (0b00000000)

typedef struct {
    spiClient_t *spi;
    volatile uint32_t spiFlag;
    uint8_t rxBuf[HMC5983_BYTES*HMC5983_SLOTS];
    uint8_t slot;
    uint8_t readCmd;
    uint8_t enabled;
    float rawMag[3];
    float mag[3];
    volatile uint32_t lastUpdate;
} hmc5983Struct_t;

extern hmc5983Struct_t hmc5983Data;

extern void hmc5983PreInit(void);
extern void hmc5983Init(void);
extern void hmc5983Decode(void);
extern void hmc5983Enable(void);
extern void hmc5983Disable(void);

#endif