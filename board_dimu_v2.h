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

#ifndef _board_dimu_v2_h
#define _board_dimu_v2_h

#define HAS_DIGITAL_IMU
#define USE_DIGITAL_IMU

#define DIMU_HAVE_EEPROM

#define DIMU_HAVE_MPU6000
#define DIMU_HAVE_HMC5983
#define DIMU_HAVE_MS5611

#define MPU6000_ACC_SCALE           8     // g      (2, 4, 8, 16)
#define MPU6000_GYO_SCALE           1000  // deg/s  (250, 500, 1000, 2000)

#define DIMU_ORIENT_ACC_X	    (+in[0])
#define DIMU_ORIENT_ACC_Y	    (+in[1])
#define DIMU_ORIENT_ACC_Z	    (+in[2])

#define DIMU_ORIENT_GYO_X	    (-in[0])
#define DIMU_ORIENT_GYO_Y	    (-in[1])
#define DIMU_ORIENT_GYO_Z	    (+in[2])

#define DIMU_ORIENT_MAG_X	    (-in[0])
#define DIMU_ORIENT_MAG_Y	    (-in[1])
#define DIMU_ORIENT_MAG_Z	    (+in[2])


#define DIMU_EEPROM_SPI		    SPI2
#define DIMU_EEPROM_CS_PORT	    GPIOB
#define DIMU_EEPROM_CS_PIN	    GPIO_Pin_12

#define DIMU_HMC5983_SPI	    SPI2
#define DIMU_HMC5983_CS_PORT	    GPIOD
#define DIMU_HMC5983_CS_PIN	    GPIO_Pin_10
#define DIMU_HMC5983_INT_PORT	    GPIOE
#define DIMU_HMC5983_INT_PIN	    GPIO_Pin_2
#define DIMU_HMC5983_INT_EXTI_PORT  EXTI_PortSourceGPIOE
#define DIMU_HMC5983_INT_EXTI_PIN   EXTI_PinSource2
#define DIMU_HMC5983_INT_EXTI_LINE  EXTI_Line2
#define DIMU_HMC5983_INT_EXTI_IRQ   EXTI2_IRQn
#define DIMU_HMC5983_INT_ISR	    EXTI2_IRQHandler

#define DIMU_MS5611_CS_PORT	    GPIOE
#define DIMU_MS5611_CS_PIN	    GPIO_Pin_7
#define DIMU_MS5611_SPI		    SPI2

#define DIMU_MPU6000_SPI	    SPI2
#define DIMU_MPU6000_CS_PORT	    GPIOD
#define DIMU_MPU6000_CS_PIN	    GPIO_Pin_4

#define DIMU_MPU6000_INT_PORT	    GPIOD
#define DIMU_MPU6000_INT_PIN	    GPIO_Pin_6
#define DIMU_MPU6000_INT_EXTI_PORT  EXTI_PortSourceGPIOD
#define DIMU_MPU6000_INT_EXTI_PIN   EXTI_PinSource6
#define DIMU_MPU6000_INT_EXTI_LINE  EXTI_Line6
#define DIMU_MPU6000_INT_EXTI_IRQ   EXTI9_5_IRQn
#define DIMU_MPU6000_INT_ISR	    EXTI9_5_IRQHandler

#endif