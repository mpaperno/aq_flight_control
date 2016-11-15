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

#ifndef _command_h
#define _command_h

#include <CoOS.h>

#define COMMAND_SYNC1		    'A'
#define COMMAND_SYNC2		    'q'
#define COMMAND_COMMAND		    'C'
#define COMMAND_FLOAT_DUMP	    'F'

#define COMMAND_WAIT_SYNC1	    0x00
#define COMMAND_WAIT_SYNC2	    0x01
#define COMMAND_WAIT_SYNC3	    0x02
#define COMMAND_WAIT_RQID1	    0x03
#define COMMAND_WAIT_RQID2	    0x04
#define COMMAND_WAIT_RQID3	    0x05
#define COMMAND_WAIT_RQID4	    0x06
#define COMMAND_WAIT_LEN	    0x07
#define COMMAND_PAYLOAD		    0x08
#define COMMAND_WAIT_ROWS	    0x09
#define COMMAND_WAIT_FLOATS	    0x0a
#define COMMAND_CHECK1		    0xfe
#define COMMAND_CHECK2		    0xff

#define COMMAND_ID_GPS_PASSTHROUGH  0x01
#define COMMAND_CONFIG_PARAM_READ   0x07
#define COMMAND_CONFIG_PARAM_WRITE  0x08
#define COMMAND_CONFIG_FLASH_READ   0x09
#define COMMAND_CONFIG_FLASH_WRITE  0x0a
#define COMMAND_CONFIG_FACTORY_RESET	0x0b
#define COMMAND_TELEMETRY_ENABLE    0x0c
#define COMMAND_TELEMETRY_DISABLE   0x0d
#define COMMAND_PID_START_DUMP	    0x0e
#define COMMAND_PID_STOP_DUMP	    0x0f
#define COMMAND_GPS_PACKET	    0x10
#define COMMAND_ACC_IN		    0x11
#define COMMAND_ID_ACK		    0xfe
#define COMMAND_ID_NACK		    0xff

typedef struct {
    unsigned char commandId;
    char data[256];
} __attribute__((packed)) commandBufStruct_t;

typedef struct {
    commandBufStruct_t buf;
    float floatDump[10];

    unsigned char requestId[sizeof(int)];
    unsigned char bufPoint;
    unsigned char state;
    unsigned char len;
    unsigned char checkA, checkB;
    unsigned long checksumErrors;
} commandStruct_t;

extern commandStruct_t commandData;

extern void commandInit(void);

#endif
