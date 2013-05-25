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

#include "aq.h"
#include "config.h"
#include "vn100.h"
#include "imu.h"
#include "aq_timer.h"
#include "serial.h"
#include "digital.h"
#include "util.h"
#include "aq_mavlink.h"
#include "filer.h"
#include "util.h"
#include "fpu.h"
#include "coocox.h"
#include <stdio.h>
#include <string.h>

#ifdef IMU_HAS_VN100

vn100Struct_t vn100Data;
char vn100Buf[VN100_BUF_SIZE];

const unsigned char vn100Hex[] = "0123456789ABCDEF";

void vn100Sync(int polarity) {
    if (vn100Data.syncIn) {
	if (polarity) {
	    digitalHi(vn100Data.syncIn);
	}
	else {
	    digitalLo(vn100Data.syncIn);
	}
    }
}

void vn100SerialSendHex(unsigned char n) {
    serialWrite(vn100Data.serialPort, vn100Hex[n >> 4]);
    serialWrite(vn100Data.serialPort, vn100Hex[n & 0x0F]);
}


unsigned char vn100CalculateChecksum(const char *command) {
    unsigned char cksum = 0;
    int i = 0;

    while (command[i] && command[i] != '*') {
	if (command[i] != '$')
	    cksum ^= (unsigned char)command[i];
	i++;
    }

    return cksum;
}

unsigned char vn100CalculateChecksumLen(char *command, int len) {
    unsigned char cksum = 0;
    int i;

    for (i = 0; i < len; i++)
	cksum ^= (unsigned char)command[i];

    return cksum;
}

void vn100SerialSend(const char *command) {
    serialPort_t *s = vn100Data.serialPort;
    int i = 0;

    serialWrite(s, '$');
    while (command[i])
	serialWrite(s, command[i++]);
    serialWrite(s, '*');
    vn100SerialSendHex(vn100CalculateChecksum(command));
    serialWrite(s, '\n');
}

int8_t vn100ValidateChecksum(char *response) {
    uint8_t cksum;
    int i = 0;

    cksum = vn100CalculateChecksum(response);

    while (response[i] && response[i] != '*')
	i++;

    if (response[++i] != vn100Hex[cksum >> 4] || response[++i] != vn100Hex[cksum & 0x0F])
	return -1;
    else
	return 0;
}

int8_t vn100SendReceive(const char *command) {
    uint32_t micros = timerMicros() + VN100_SERIAL_TIMEOUT;
    uint8_t i;
    uint8_t c;

    vn100SerialSend(command);

    i = 0;
    c = 0;
    do {
	yield(1);

	while (serialAvailable(vn100Data.serialPort)) {
	    c = serialRead(vn100Data.serialPort);
	    if (c == '$')
		i = 0;
	    if (i < sizeof(vn100Data.inBuf))
		vn100Data.inBuf[i++] = c;
	}
    } while (c != '\n' && timerMicros() <= micros && i < sizeof(vn100Data.inBuf));

    if (timerMicros() > micros) {
	AQ_NOTICE("VN100: serial timeout\n");
	return -1;
    }
    else {
	vn100Data.inBuf[i] = 0;
	AQ_NOTICE(vn100Data.inBuf);
	if (vn100ValidateChecksum(vn100Data.inBuf)) {
	    AQ_NOTICE("VN100: checksum error\n");
	    return -1;
	}
	else if (memcmp(&vn100Data.inBuf[1], command, 8)) {
	    AQ_NOTICE("VN100: compare failed\n");
	    return -1;
	}
	else
	    return i;
    }
}

int8_t vn100ReliablySend(const char *command) {
    int8_t retries;
    int8_t n;

    retries = 3;
    do {
	n = vn100SendReceive(command);
	yield(50);
    } while (n < 0 && --retries);

    return n;
}

int8_t vn100WriteConfig(void) {
    char buf[128];
    digitalPin *syncIn;
    int32_t n;
    int8_t c;
    int8_t fh;
    int8_t ret;
    int i, p1, p2;

    // turn off SPI communications
    syncIn = vn100Data.syncIn;
    vn100Data.syncIn = 0;

    ret = 0;
    if ((fh = filerGetHandle(VN100_CONFIG_FNAME)) < 0) {
	AQ_NOTICE("vn100: cannot get write file handle, abort write config\n");
	ret = -1;
    }
    else {
	p1 = 0;
	do {
	    n = filerRead(fh, vn100Buf, -1, VN100_BUF_SIZE);
	    if (n < 0) {
		ret = -1;
	    }
	    else {
		p2 = 0;
		for (i = 0; i < n; i++) {
		    c = vn100Buf[p2++];
		    if (c == '\n' || p1 == (sizeof(buf)-1)) {
			buf[p1] = 0;

			if (vn100ReliablySend(buf) < 0)
			    ret = -1;
			p1 = 0;
		    }
		    else {
			buf[p1++] = c;
		    }
		}
	    }
	} while (n > 0);

	filerClose(fh);
    }

    if (ret >= 0) {
	AQ_NOTICE("vn100: write config succesful\n");
	// reset
	digitalLo(vn100Data.nrst);
	delay(1);
	digitalHi(vn100Data.nrst);
	delay(200);
    }
    else {
	AQ_NOTICE("vn100: write config failed\n");
    }


    // turn back on SPI communications
    vn100Data.syncIn = syncIn;

    return ret;
}

int8_t vn100ReadConfig(void) {
    const char unlock[] = ",1234\n";
    int8_t fh;
    int8_t ret;
    int8_t n;
    int i;

    ret = 0;
    if ((fh = filerGetHandle(VN100_CONFIG_FNAME)) < 0) {
	AQ_NOTICE("vn100: cannot get write file handle, abort read config\n");
	ret = -1;
    }
    else {
	n = vn100ReliablySend("VNRRG,01");
	vn100Data.inBuf[3] = 'W';
	filerWrite(fh, vn100Data.inBuf+1, -1, n-6);
	filerWrite(fh, (char *)unlock, -1, sizeof(unlock)-1);

	n = vn100ReliablySend("VNRRG,02");
	vn100Data.inBuf[3] = 'W';
	filerWrite(fh, vn100Data.inBuf+1, -1, n-6);
	filerWrite(fh, (char *)unlock, -1, sizeof(unlock)-1);

	for (i = 1; i <= 230; i++) {
	    snprintf(vn100Data.outBuf, sizeof(vn100Data.outBuf), "VNRRG,255,%d,1234", i);
	    n = vn100ReliablySend(vn100Data.outBuf);
	    vn100Data.inBuf[3] = 'W';

	    if (n < 0 || filerWrite(fh, vn100Data.inBuf+1, -1, n-6) < 0) {
		AQ_NOTICE("vn100: file write error\n");
		ret = -1;
		break;
	    }
	    else {
		filerWrite(fh, (char *)unlock, -1, sizeof(unlock)-1);
	    }
	    yield(10);
	}
    }

    if (fh > 0) {
	filerClose(fh);
	if (ret >= 0)
	    AQ_NOTICE("vn100: read config succesful\n");
	else
	    AQ_NOTICE("vn100: read config failed\n");
    }

    return ret;
}

void vn100Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    EXTI_InitTypeDef EXTI_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;
    int i;

    AQ_NOTICE("VN100 init\n");

    memset((void *)&vn100Data, 0, sizeof(vn100Data));

    // reprgm pin
    vn100Data.reprgm = digitalInit(VN100_REPRGM_PORT, VN100_REPRGM_PIN);
    digitalLo(vn100Data.reprgm);

    // setup reset pin
    vn100Data.nrst = digitalInit(VN100_RESET_PORT, VN100_RESET_PIN);
    digitalLo(vn100Data.nrst);
    delay(1);
    digitalHi(vn100Data.nrst);
    delay(200);

    // setup serial
     vn100Data.serialPort = serialOpen(VN100_UART, VN100_BAUD, USART_HardwareFlowControl_None, 128, 256);

//   vn100BootLoader(downlinkData.serialPort);
//    vn100WriteConfig();
//   vn100PassThrough(downlinkData.serialPort);
//    vn100SerialSend("VNWRG,06,0");			// turn off Async output
    vn100ReliablySend("VNRRG,30");
    vn100ReliablySend("VNWRG,30,0,0,1,1,1");		// turn on SPI checksum

    // send mag calibration
    i = sprintf(vn100Data.outBuf, "VNWRG,23");
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], 1.0f/p[VN100_MAG_SCAL_X], 5);
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], p[VN100_MAG_ALGN_XY], 5);
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], p[VN100_MAG_ALGN_XZ], 5);
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], p[VN100_MAG_ALGN_YX], 5);
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], 1.0f/p[VN100_MAG_SCAL_Y], 5);
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], p[VN100_MAG_ALGN_YZ], 5);
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], p[VN100_MAG_ALGN_ZX], 5);
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], p[VN100_MAG_ALGN_ZY], 5);
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], 1.0f/p[VN100_MAG_SCAL_Z], 5);
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], -p[VN100_MAG_BIAS_X], 5);
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], -p[VN100_MAG_BIAS_Y], 5);
    vn100Data.outBuf[i++] = ',';
    i += ftoa(&vn100Data.outBuf[i], -p[VN100_MAG_BIAS_Z], 5);

    vn100ReliablySend(vn100Data.outBuf);

//vn100ReadConfig();
    // setup SPI
    RCC_APB1PeriphClockCmd(VN100_SPI_CLOCK, ENABLE);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

    // SPI SCK / MOSI / MISO pin configuration
    GPIO_InitStructure.GPIO_Pin = VN100_SPI_NSS_PIN;
    GPIO_Init(VN100_SPI_NSS_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = VN100_SPI_SCK_PIN;
    GPIO_Init(VN100_SPI_SCK_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = VN100_SPI_MISO_PIN;
    GPIO_Init(VN100_SPI_MISO_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin = VN100_SPI_MOSI_PIN;
    GPIO_Init(VN100_SPI_MOSI_PORT, &GPIO_InitStructure);

    // Connect SPI pins to AF6
    GPIO_PinAFConfig(VN100_SPI_NSS_PORT, VN100_SPI_NSS_SOURCE, VN100_SPI_AF);
    GPIO_PinAFConfig(VN100_SPI_SCK_PORT, VN100_SPI_SCK_SOURCE, VN100_SPI_AF);
    GPIO_PinAFConfig(VN100_SPI_MISO_PORT, VN100_SPI_MISO_SOURCE, VN100_SPI_AF);
    GPIO_PinAFConfig(VN100_SPI_MOSI_PORT, VN100_SPI_MOSI_SOURCE, VN100_SPI_AF);

    // SPI configuration
    SPI_I2S_DeInit(VN100_SPI);
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Hard;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_CRCPolynomial = 7;
    SPI_Init(VN100_SPI, &SPI_InitStructure);

    SPI_SSOutputCmd(VN100_SPI, ENABLE);

    SPI_I2S_DMACmd(VN100_SPI, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

    // RX DMA - continuous, circular
    DMA_DeInit(VN100_DMA_RX);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = VN100_DMA_RX_CHANNEL;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&vn100Data.recvBuf.command;
    DMA_InitStructure.DMA_PeripheralBaseAddr = VN100_SPI_DR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    DMA_InitStructure.DMA_BufferSize = sizeof(vn100Command_t);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(VN100_DMA_RX, &DMA_InitStructure);

    DMA_ITConfig(VN100_DMA_RX, DMA_IT_TC, ENABLE);
    DMA_ClearITPendingBit(VN100_DMA_RX, VN100_DMA_RX_FLAGS);
    DMA_Cmd(VN100_DMA_RX, ENABLE);

    // Enable RX DMA global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = VN100_DMA_RX_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // TX DMA - one shot
    DMA_DeInit(VN100_DMA_TX);
    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_Channel = VN100_DMA_TX_CHANNEL;
    DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)&vn100Data.sendBuf.command;
    DMA_InitStructure.DMA_PeripheralBaseAddr = VN100_SPI_DR;
    DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    DMA_InitStructure.DMA_BufferSize = sizeof(vn100Command_t);
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure.DMA_Priority = DMA_Priority_High;
    DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(VN100_DMA_TX, &DMA_InitStructure);

    DMA_ClearITPendingBit(VN100_DMA_TX, VN100_DMA_TX_FLAGS);

    // calibrated, unfiltered sensor readings (Reg 252)
    memset((char *)vn100Data.sendBuf.command, 0, sizeof(vn100Command_t));
    vn100Data.sendBuf.command[0] = 0x01;
    vn100Data.sendBuf.command[1] = 0xFC;
    vn100Data.sendBuf.checksum = vn100CalculateChecksumLen((char*)(&vn100Data.sendBuf.command), 44);

    // External Interrupt line for VN100 data ready
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = VN100_SYNCOUT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(VN100_SYNCOUT_PORT, &GPIO_InitStructure);

    SYSCFG_EXTILineConfig(VN100_SYNCOUT_EXTI_PORT, VN100_SYNCOUT_EXTI_PIN);

    EXTI_InitStructure.EXTI_Line = VN100_SYNCOUT_EXTI_LINE;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = VN100_SYNCOUT_EXTI_IRQ;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    delay(50);

    // drain the serial port
    while (serialAvailable(vn100Data.serialPort))
	serialRead(vn100Data.serialPort);

    // setup sync_in pin
    vn100Data.syncIn = digitalInit(VN100_SYNCIN_PORT, VN100_SYNCIN_PIN);
    digitalLo(vn100Data.syncIn);

    delay(50);

    vn100Data.doubleRates[0] = 0.0f;
    vn100Data.doubleRates[1] = 0.0f;
    vn100Data.doubleRates[2] = 0.0f;

    vn100Data.rates[0] = 0.0f;
    vn100Data.rates[1] = 0.0f;
    vn100Data.rates[2] = 0.0f;

    vn100Data.accs[0] = vn100Data.recvBuf.parameters[3];
    vn100Data.accs[1] = vn100Data.recvBuf.parameters[4];
    vn100Data.accs[2] = vn100Data.recvBuf.parameters[5];

    vn100Data.mags[0] = vn100Data.recvBuf.parameters[0];
    vn100Data.mags[1] = vn100Data.recvBuf.parameters[1];
    vn100Data.mags[2] = vn100Data.recvBuf.parameters[2];
}

// dead end
void vn100BootLoader(serialPort_t *s) {
    USART_InitTypeDef USART_InitStructure;
    unsigned long micros = timerMicros();

    // don't allow any preemption
    CoSetPriority(CoGetCurTaskID(), 1);

    // drain serial ports
    while (serialAvailable(vn100Data.serialPort))
	serialRead(vn100Data.serialPort);

    // wait until there is at least 3 second of inactivity
    while ((timerMicros() - micros) < 3000000) {
	while (serialAvailable(s)) {
	    serialRead(s);
	    micros = timerMicros();
	}
    }

    // force UART to 9E1
    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = VN100_BAUD;
    USART_InitStructure.USART_StopBits = USART_StopBits_1;
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_WordLength = USART_WordLength_9b;
    USART_InitStructure.USART_Parity = USART_Parity_Even;
    USART_Init(VN100_UART, &USART_InitStructure);
    USART_Cmd(VN100_UART, ENABLE);

    // restart VN100 in bootloader mode
    delay(100);
    digitalLo(vn100Data.nrst);
    digitalHi(vn100Data.reprgm);
    delay(10);
    digitalHi(vn100Data.nrst);
    delay(10);

    while (1) {
	delay(1);
	if (serialAvailable(vn100Data.serialPort))
	    serialWrite(s, serialRead(vn100Data.serialPort));
	delay(1);
	if (serialAvailable(s))
	    serialWrite(vn100Data.serialPort, serialRead(s));
    }
}

// dead end
void vn100PassThrough(serialPort_t *s) {
    // don't allow any preemption
    CoSetPriority(CoGetCurTaskID(), 1);

    while (1) {
	delay(1);
	if (serialAvailable(vn100Data.serialPort))
	    serialWrite(s, serialRead(vn100Data.serialPort));
	delay(1);
	if (serialAvailable(s))
	    serialWrite(vn100Data.serialPort, serialRead(s));
    }
}

void VN100_SYNCOUT_ISR(void) {
    EXTI_ClearITPendingBit(VN100_SYNCOUT_EXTI_LINE);

    if (SPI_I2S_GetFlagStatus(VN100_SPI, SPI_I2S_FLAG_BSY) != SET) {
	// only on certain edge combinations
	if ((VN100_SYNCOUT_PORT->IDR & VN100_SYNCOUT_PIN) || (VN100_SYNCIN_PORT->ODR & VN100_SYNCIN_PIN)) {
	    vn100Data.solutionTime = timerMicros();

	    // shutdown SPI
	    SPI_Cmd(VN100_SPI, DISABLE);

	    VN100_DMA_TX->NDTR = sizeof(vn100Command_t);

	    DMA_ClearITPendingBit(VN100_DMA_TX, VN100_DMA_TX_FLAGS);
	    DMA_Cmd(VN100_DMA_TX, ENABLE);

	    // let 'er rip
	    SPI_Cmd(VN100_SPI, ENABLE);
	}
    }
}

// TODO: remove FPU instructions from IRQ
uint32_t vn100Cache[33];

void VN100_DMX_RX_HANDLER(void) {
    float x, y, z;
    int32_t checksumBad;

    // shutdown TX DMA
    DMA_Cmd(VN100_DMA_TX, DISABLE);
    // shutdown SPI
    SPI_Cmd(VN100_SPI, DISABLE);

    checksumBad = (vn100CalculateChecksumLen((char*)(&vn100Data.recvBuf.command), 44) != vn100Data.recvBuf.checksum);
    vn100Data.checksumErrors += checksumBad;

    CoEnterISR();
    UTIL_ISR_DISABLE;
    __vfp_store(vn100Cache);
    // SyncOut: hi == double rate   (400 Hz)
    if (((VN100_SYNCOUT_PORT->IDR & VN100_SYNCOUT_PIN) != 0)) {
	if (!checksumBad) {
	    x = vn100Data.recvBuf.parameters[6];
	    y = vn100Data.recvBuf.parameters[7];
	    z = vn100Data.recvBuf.parameters[8];

	    vn100Data.doubleRates[0] = (vn100Data.doubleRates[0] + x * imuData.cosRot - y * imuData.sinRot) * 0.5f;
	    vn100Data.doubleRates[1] = (vn100Data.doubleRates[1] + y * imuData.cosRot + x * imuData.sinRot) * 0.5f;
	    vn100Data.doubleRates[2] = (vn100Data.doubleRates[2] + z) * 0.5f;
	}
	imuVN100DRateReady();
    }
    // SyncOut: lo == remaining sensors
    else {
	//  (if syncIn == hi)	(200 Hz)
	if (digitalGet(vn100Data.syncIn)) {
	    if (!checksumBad) {
		x = vn100Data.recvBuf.parameters[6];
		y = vn100Data.recvBuf.parameters[7];
		z = vn100Data.recvBuf.parameters[8];

		vn100Data.rates[0] = x * imuData.cosRot - y * imuData.sinRot;
		vn100Data.rates[1] = y * imuData.cosRot + x * imuData.sinRot;
		vn100Data.rates[2] = z;

		x = vn100Data.recvBuf.parameters[3];
		y = vn100Data.recvBuf.parameters[4];
		z = vn100Data.recvBuf.parameters[5];

		vn100Data.accs[0] = x * imuData.cosRot - y * imuData.sinRot;
		vn100Data.accs[1] = y * imuData.cosRot + x * imuData.sinRot;
		vn100Data.accs[2] = z;

		x = vn100Data.recvBuf.parameters[0];
		y = vn100Data.recvBuf.parameters[1];
		z = vn100Data.recvBuf.parameters[2];

		vn100Data.mags[0] = x * imuData.cosRot - y * imuData.sinRot;
		vn100Data.mags[1] = y * imuData.cosRot + x * imuData.sinRot;
		vn100Data.mags[2] = z;
	    }
	    imuVN100SensorReady();
	}
    }
    __vfp_restore(vn100Cache);
    UTIL_ISR_ENABLE;
    CoExitISR();

    // clear all RX DMA flags
    DMA_ClearITPendingBit(VN100_DMA_RX, VN100_DMA_RX_FLAGS);
}
#endif	// IMU_HAS_VN100
