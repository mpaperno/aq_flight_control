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

#ifndef _eeprom_h
#define _eeprom_h

#include "spi.h"

#define DIMU_EEPROM_SPI_BAUD	    SPI_BaudRatePrescaler_4	// 10.5 MHz

#define DIMU_EEPROM_SIZE	    0x8000
#define DIMU_EEPROM_BLOCK_SIZE	    0x40
#define DIMU_EEPROM_MASK	    0x7fff
//#define DIUM_EEPROM_SIZE		    0x10000
//#define DIUM_EEPROM_BLOCK_SIZE	    0x80
//#define DIUM_EEPROM_MASK		    0xffff

#define EEPROM_WREN		    0b0110
#define EEPROM_WRDI		    0b0100
#define EEPROM_RDSR		    0b0101
#define EEPROM_WRSR		    0b0001
#define EEPROM_READ		    0b0011
#define EEPROM_WRITE		    0b0010

#define EEPROM_VERSION		    0x00010001
#define EEPROM_SIGNATURE	    0xdeafbeef

typedef struct {
    uint8_t cmd;
    uint8_t addr[2];
    uint8_t data[DIMU_EEPROM_BLOCK_SIZE];
}  __attribute__((packed)) eepromBuf_t;

typedef struct {
    uint32_t signature;
    uint32_t version;
    uint32_t seq;
    uint16_t size;
    uint16_t start;
    uint8_t fileCk[2];
    uint8_t headerCk[2];
} __attribute__((packed)) eepromHeader_t;

typedef struct {
    spiClient_t *spi;
    volatile uint32_t spiFlag;
    eepromBuf_t buf;
    eepromHeader_t header;
    uint16_t readPointer;
    uint8_t ck[2];
    uint8_t status;
} eepromStruct_t;

extern void eepromPreInit(void);
extern void eepromInit(void);
extern uint8_t *eepromOpenWrite(void);
extern void eepromWrite(void);
extern void eepromClose(void);
extern uint8_t *eepromOpenRead(void);
extern uint8_t eepromRead(int size);

#endif
