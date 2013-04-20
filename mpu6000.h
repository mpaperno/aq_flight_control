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

#ifndef _mpu6000_h
#define _mpu6000_h

#include "spi.h"
#include "util.h"

#define MPU6000_INT_PORT	    GPIOD
#define MPU6000_INT_PIN		    GPIO_Pin_6
#define MPU6000_INT_EXTI_PORT	    EXTI_PortSourceGPIOD
#define MPU6000_INT_EXTI_PIN	    EXTI_PinSource6
#define MPU6000_INT_EXTI_LINE	    EXTI_Line6
#define MPU6000_INT_EXTI_IRQ	    EXTI9_5_IRQn
#define MPU6000_INT_ISR		    EXTI9_5_IRQHandler

#define MPU6000_SPI		    SPI2
#define MPU6000_SPI_BAUD	    SPI_BaudRatePrescaler_64
#define MPU6000_CS_PORT		    GPIOD
#define MPU6000_CS_PIN		    GPIO_Pin_4

#define MPU6000_READ_BIT	    (0x01<<7)
#define MPU6000_WRITE_BIT	    (0x00<<7)

#define MPU6000_BYTES		    15
//#define MPU6000_SLOTS		    40	    // 200Hz bandwidth
#define MPU6000_SLOTS		    80	    // 100Hz bandwidth
//#define MPU6000_SLOTS		    160	    // 50Hz bandwidth

typedef struct {
    utilFilter_t tempFilter;
    spiClient_t *spi;
    volatile uint32_t spiFlag;
    volatile uint8_t rxBuf[MPU6000_BYTES*MPU6000_SLOTS];
    volatile uint8_t slot;
    uint8_t readReg;
    uint8_t enabled;
    float rawTemp;
    float rawAcc[3];
    float rawGyo[3];
    float gyoOffset[3];
    float acc[3];
    float temp;
    float gyo[3];
    volatile uint32_t lastUpdate;
} mpu6000Struct_t;

extern mpu6000Struct_t mpu6000Data;

extern void mpu6000PreInit(void);
extern void mpu6000Init(void);
extern void mpu6600InitialBias(void);
extern void mpu6000Decode(void);
extern void mpu6000Enable(void);
extern void mpu6000Disable(void);

#endif