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

#ifndef _1wire_h
#define _1wire_h

#include "digital.h"
#include "stm32f4xx.h"

#define owHi()		    { owData.port->BSRRL = owData.pin; }
#define owLo()		    { owData.port->BSRRH = owData.pin; }
#define owRead()	    ((owData.port->IDR & owData.pin) != (uint32_t)Bit_RESET)

#define OW_ROM_READ	    0x33
#define OW_ROM_MATCH	    0x55
#define OW_ROM_SKIP	    0xCC
#define OW_READ_SCRATCH	    0xBE
#define OW_TCONV	    0x44
#define OW_VERSION	    0x03
#define OW_PARAM_READ	    0x04
#define OW_PARAM_WRITE	    0x05
#define OW_CONFIG_READ	    0x06
#define OW_CONFIG_WRITE	    0x07
#define OW_CONFIG_DEFAULT   0x08
#define OW_SET_MODE	    0x09
#define OW_GET_MODE	    0x0A
#define OW_GET_PARAM_ID     0x0B

enum {
    OW_STATE_0,
    OW_STATE_1,
    OW_STATE_2,
    OW_STATE_3,
    OW_STATE_4
};

enum {
    OW_STATUS_UNINITIALIZED,
    OW_STATUS_NO_PRESENSE,
    OW_STATUS_BUSY,
    OW_STATUS_COMPLETE
};

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
    digitalPin *dp;
    uint8_t buf[256];
    uint8_t *ptr;
    GPIO_InitTypeDef owGPIO;
    volatile uint8_t status;
    volatile uint8_t present;
    volatile uint8_t bitValue;
    volatile uint8_t byteValue;
    int8_t writeBytes;
    int8_t readBytes;
} owStruct_t;

extern owStruct_t owData;

extern void owInit(GPIO_TypeDef* port, const uint16_t pin);
extern unsigned char owReset(void);
extern void owWriteByte(uint8_t byteval);
extern uint8_t owReadByte(void);
extern void owReadROM(char *rom);
extern void owResetISR(int state);
extern void owReadBitISR(int state);
extern void owWriteBit1ISR(int state);
extern void owWriteBit0ISR(int state);
extern void owReadByteISR(int bit);
extern void owWriteByteISR(int bit);
extern void owTransactionISR(int state);
extern uint8_t owTransaction(int8_t write, int8_t read);

#endif
