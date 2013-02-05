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
#include "digital.h"
#include "downlink.h"
#include "config.h"
#include "supervisor.h"

downlinkStruct_t downlinkData;

unsigned char downlinkCkA, downlinkCkB;

void downlinkInit(void) {
    downlinkData.serialPort = serialOpen(DOWNLINK_USART, p[DOWNLINK_BAUD], USART_HardwareFlowControl_RTS_CTS, DOWNLINK_RX_BUF_SIZE, DOWNLINK_TX_BUF_SIZE);

    // setup mutex for serial port access
    downlinkData.serialPortMutex = CoCreateMutex();
}

void downlinkChecksum(unsigned char c) {
    downlinkCkA += c;
    downlinkCkB += downlinkCkA;
}

void downlinkResetChecksum(void) {
    downlinkCkA = downlinkCkB = 0;
    supervisorSendDataStart();
}

void downlinkSendChecksum(void) {
    serialWrite(downlinkData.serialPort, downlinkCkA);
    serialWrite(downlinkData.serialPort, downlinkCkB);
    supervisorSendDataStop();
}

void downlinkSendChar(unsigned char c) {
    serialWrite(downlinkData.serialPort, c);
    downlinkChecksum(c);
}

void downlinkSendInt(unsigned int i) {
    unsigned char *c = (unsigned char *)&i;
    unsigned char j;

    for (j = 0; j < sizeof(int); j++)
	downlinkSendChar(*c++);
}

void downlinkSendFloat(float f) {
    unsigned char *c = (unsigned char *)&f;
    unsigned char j;

    for (j = 0; j < sizeof(float); j++)
	downlinkSendChar(*c++);
}

void downlinkSendString(const char *s) {
    serialPrint(downlinkData.serialPort, s);
}

unsigned char downlinkReadChar(void) {
    return serialRead(downlinkData.serialPort);
}

void downlinkGetLong(unsigned long *v) {
    unsigned int i;
    unsigned char *c = (unsigned char *)v;

    for (i = 0; i < sizeof(unsigned long); i++)
	*c++ = downlinkReadChar();
}
