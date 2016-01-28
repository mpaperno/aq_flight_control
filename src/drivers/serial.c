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

#include "serial.h"
#include "util.h"
#include <stdio.h>
#include <stdlib.h>

#ifdef SERIAL_UART1_PORT
serialPort_t *serialPort1;
#endif
#ifdef SERIAL_UART2_PORT
serialPort_t *serialPort2;
#endif
#ifdef SERIAL_UART3_PORT
serialPort_t *serialPort3;
#endif
#ifdef SERIAL_UART4_PORT
serialPort_t *serialPort4;
#endif
#ifdef SERIAL_UART5_PORT
serialPort_t *serialPort5;
#endif
#ifdef SERIAL_UART6_PORT
serialPort_t *serialPort6;
#endif

serialPort_t *serialSTDIO;

int _serialStartTxDMA(serialPort_t *s, void *buf, int size, serialTxDMACallback_t *txDMACallback, void *txDMACallbackParam) {
    if (!s->txDmaRunning) {
	s->txDmaRunning = 1;

	s->txDMACallback = txDMACallback;
	s->txDMACallbackParam = txDMACallbackParam;

	s->txDMAStream->M0AR = (uint32_t)buf;
	s->txDMAStream->NDTR = size;

	DMA_Cmd(s->txDMAStream, ENABLE);

	return 1;
    }
    else {
	return 0;
    }
}

void serialStartTxDMA(void *param) {
    serialPort_t *s = (serialPort_t *)param;
    uint32_t tail = s->txTail;
    int size;

    if (!s->txDmaRunning && s->txHead != tail) {
	if (s->txHead > tail) {
	    size = s->txHead - tail;
	    s->txTail = s->txHead;
	}
	else {
	    size = s->txBufSize - tail;
	    s->txTail = 0;
	}

	_serialStartTxDMA(s, (void *)&s->txBuf[tail], size, serialStartTxDMA, s);
    }
}

void serialOpenUART(serialPort_t *s) {
    USART_InitTypeDef USART_InitStructure;

    // reduce oversampling to allow for higher baud rates
    USART_OverSampling8Cmd(s->USARTx, ENABLE);

    USART_StructInit(&USART_InitStructure);
    USART_InitStructure.USART_BaudRate = s->baudRate;
    USART_InitStructure.USART_WordLength = USART_WordLength_8b;
    USART_InitStructure.USART_StopBits = s->stopBits;
    USART_InitStructure.USART_Parity = s->parity;
    USART_InitStructure.USART_HardwareFlowControl = s->flowControl;
    USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
    USART_Init(s->USARTx, &USART_InitStructure);

    USART_Cmd(s->USARTx, ENABLE);
}

#ifdef SERIAL_UART1_PORT
serialPort_t *serialUSART1(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    serialPort_t *s;

    s = serialPort1 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // Enable USART1 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

#ifdef SERIAL_UART1_RX_PIN
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    GPIO_PinAFConfig(SERIAL_UART1_PORT, SERIAL_UART1_RX_SOURCE, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART1_RX_PIN;
    GPIO_Init(SERIAL_UART1_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART1_RX_DMA_ST
    s->rxDMAStream = SERIAL_UART1_RX_DMA_ST;
    s->rxDMAChannel = SERIAL_UART1_RX_DMA_CH;
    s->rxDmaFlags = SERIAL_UART1_RX_TC_FLAG | SERIAL_UART1_RX_HT_FLAG | SERIAL_UART1_RX_TE_FLAG | SERIAL_UART1_RX_DM_FLAG | SERIAL_UART1_RX_FE_FLAG;
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART1_RX_DMA_ST
#endif	// SERIAL_UART1_RX_PIN

#ifdef SERIAL_UART1_TX_PIN
    s->txBufSize = txBufSize;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);

    GPIO_PinAFConfig(SERIAL_UART1_PORT, SERIAL_UART1_TX_SOURCE, GPIO_AF_USART1);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART1_TX_PIN;
    GPIO_Init(SERIAL_UART1_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART1_TX_DMA_ST
    s->txDMAStream = SERIAL_UART1_TX_DMA_ST;
    s->txDMAChannel = SERIAL_UART1_TX_DMA_CH;
    s->txDmaFlags = SERIAL_UART1_TX_TC_FLAG | SERIAL_UART1_TX_HT_FLAG | SERIAL_UART1_TX_TE_FLAG | SERIAL_UART1_TX_DM_FLAG | SERIAL_UART1_TX_FE_FLAG;

    // Enable the DMA global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = SERIAL_UART1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART1_TX_DMA_ST
#endif	// SERIAL_UART1_TX_PIN

#ifdef SERIAL_UART1_CTS_PIN
    if (flowControl == USART_HardwareFlowControl_RTS_CTS) {
	GPIO_PinAFConfig(SERIAL_UART1_PORT, SERIAL_UART1_RTS_SOURCE, GPIO_AF_USART1);
	GPIO_PinAFConfig(SERIAL_UART1_PORT, SERIAL_UART1_CTS_SOURCE, GPIO_AF_USART1);

	GPIO_InitStructure.GPIO_Pin = SERIAL_UART1_CTS_PIN | SERIAL_UART1_RTS_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(SERIAL_UART1_PORT, &GPIO_InitStructure);
    }
#endif	// SERIAL_UART1_CTS_PIN

    return s;
}
#endif

#ifdef SERIAL_UART2_PORT
serialPort_t *serialUSART2(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    serialPort_t *s;

    s = serialPort2 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // Enable USART2 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);

#ifdef SERIAL_UART2_RX_PIN
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    GPIO_PinAFConfig(SERIAL_UART2_PORT, SERIAL_UART2_RX_SOURCE, GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART2_RX_PIN;
    GPIO_Init(SERIAL_UART2_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART2_RX_DMA_ST
    s->rxDMAStream = SERIAL_UART2_RX_DMA_ST;
    s->rxDMAChannel = SERIAL_UART2_RX_DMA_CH;
    s->rxDmaFlags = SERIAL_UART2_RX_TC_FLAG | SERIAL_UART2_RX_HT_FLAG | SERIAL_UART2_RX_TE_FLAG | SERIAL_UART2_RX_DM_FLAG | SERIAL_UART2_RX_FE_FLAG;
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART2_RX_DMA_ST
#endif	// SERIAL_UART2_RX_PIN

#ifdef SERIAL_UART2_TX_PIN
    s->txBufSize = txBufSize;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);

    GPIO_PinAFConfig(SERIAL_UART2_PORT, SERIAL_UART2_TX_SOURCE, GPIO_AF_USART2);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART2_TX_PIN;
    GPIO_Init(SERIAL_UART2_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART2_TX_DMA_ST
    s->txDMAStream = SERIAL_UART2_TX_DMA_ST;
    s->txDMAChannel = SERIAL_UART2_TX_DMA_CH;
    s->txDmaFlags = SERIAL_UART2_TX_TC_FLAG | SERIAL_UART2_TX_HT_FLAG | SERIAL_UART2_TX_TE_FLAG | SERIAL_UART2_TX_DM_FLAG | SERIAL_UART2_TX_FE_FLAG;

    // Enable the DMA global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = SERIAL_UART2_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART2_TX_DMA_ST
#endif	// SERIAL_UART2_TX_PIN

#ifdef SERIAL_UART2_CTS_PIN
    if (flowControl == USART_HardwareFlowControl_RTS_CTS) {
	GPIO_PinAFConfig(SERIAL_UART2_PORT, SERIAL_UART2_RTS_SOURCE, GPIO_AF_USART2);
	GPIO_PinAFConfig(SERIAL_UART2_PORT, SERIAL_UART2_CTS_SOURCE, GPIO_AF_USART2);

	GPIO_InitStructure.GPIO_Pin = SERIAL_UART2_CTS_PIN | SERIAL_UART2_RTS_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(SERIAL_UART2_PORT, &GPIO_InitStructure);
    }
#endif	// SERIAL_UART2_CTS_PIN

    return s;
}
#endif

#ifdef SERIAL_UART3_PORT
serialPort_t *serialUSART3(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    serialPort_t *s;

    s = serialPort3 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // Enable USART3 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

#ifdef SERIAL_UART3_RX_PIN
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    GPIO_PinAFConfig(SERIAL_UART3_PORT, SERIAL_UART3_RX_SOURCE, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART3_RX_PIN;
    GPIO_Init(SERIAL_UART3_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART3_RX_DMA_ST
    s->rxDMAStream = SERIAL_UART3_RX_DMA_ST;
    s->rxDMAChannel = SERIAL_UART3_RX_DMA_CH;
    s->rxDmaFlags = SERIAL_UART3_RX_TC_FLAG | SERIAL_UART3_RX_HT_FLAG | SERIAL_UART3_RX_TE_FLAG | SERIAL_UART3_RX_DM_FLAG | SERIAL_UART3_RX_FE_FLAG;
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART3_RX_DMA_ST
#endif	// SERIAL_UART3_RX_PIN

#ifdef SERIAL_UART3_TX_PIN
    s->txBufSize = txBufSize;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);

    GPIO_PinAFConfig(SERIAL_UART3_PORT, SERIAL_UART3_TX_SOURCE, GPIO_AF_USART3);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART3_TX_PIN;
    GPIO_Init(SERIAL_UART3_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART3_TX_DMA_ST
    s->txDMAStream = SERIAL_UART3_TX_DMA_ST;
    s->txDMAChannel = SERIAL_UART3_TX_DMA_CH;
    s->txDmaFlags = SERIAL_UART3_TX_TC_FLAG | SERIAL_UART3_TX_HT_FLAG | SERIAL_UART3_TX_TE_FLAG | SERIAL_UART3_TX_DM_FLAG | SERIAL_UART3_TX_FE_FLAG;

    // Enable the DMA global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = SERIAL_UART3_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART3_TX_DMA_ST
#endif	// SERIAL_UART3_TX_PIN

#ifdef SERIAL_UART3_CTS_PIN
    if (flowControl == USART_HardwareFlowControl_RTS_CTS) {
	GPIO_PinAFConfig(SERIAL_UART3_PORT, SERIAL_UART3_RTS_SOURCE, GPIO_AF_USART3);
	GPIO_PinAFConfig(SERIAL_UART3_PORT, SERIAL_UART3_CTS_SOURCE, GPIO_AF_USART3);

	GPIO_InitStructure.GPIO_Pin = SERIAL_UART3_CTS_PIN | SERIAL_UART3_RTS_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(SERIAL_UART3_PORT, &GPIO_InitStructure);
    }
#endif	// SERIAL_UART3_CTS_PIN

    return s;
}
#endif

#ifdef SERIAL_UART4_PORT
serialPort_t *serialUSART4(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    serialPort_t *s;

    s = serialPort4 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // Enable USART4 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);

#ifdef SERIAL_UART4_RX_PIN
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    GPIO_PinAFConfig(SERIAL_UART4_PORT, SERIAL_UART4_RX_SOURCE, GPIO_AF_UART4);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART4_RX_PIN;
    GPIO_Init(SERIAL_UART4_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART4_RX_DMA_ST
    s->rxDMAStream = SERIAL_UART4_RX_DMA_ST;
    s->rxDMAChannel = SERIAL_UART4_RX_DMA_CH;
    s->rxDmaFlags = SERIAL_UART4_RX_TC_FLAG | SERIAL_UART4_RX_HT_FLAG | SERIAL_UART4_RX_TE_FLAG | SERIAL_UART4_RX_DM_FLAG | SERIAL_UART4_RX_FE_FLAG;
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART4_RX_DMA_ST
#endif	// SERIAL_UART4_RX_PIN

#ifdef SERIAL_UART4_TX_PIN
    s->txBufSize = txBufSize;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);

    GPIO_PinAFConfig(SERIAL_UART4_PORT, SERIAL_UART4_TX_SOURCE, GPIO_AF_UART4);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART4_TX_PIN;
    GPIO_Init(SERIAL_UART4_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART4_TX_DMA_ST
    s->txDMAStream = SERIAL_UART4_TX_DMA_ST;
    s->txDMAChannel = SERIAL_UART4_TX_DMA_CH;
    s->txDmaFlags = SERIAL_UART4_TX_TC_FLAG | SERIAL_UART4_TX_HT_FLAG | SERIAL_UART4_TX_TE_FLAG | SERIAL_UART4_TX_DM_FLAG | SERIAL_UART4_TX_FE_FLAG;

    // Enable the DMA global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = SERIAL_UART4_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART4_TX_DMA_ST
#endif	// SERIAL_UART4_TX_PIN

    return s;
}
#endif

#ifdef SERIAL_UART5_PORT
serialPort_t *serialUSART5(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    serialPort_t *s;

    s = serialPort5 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // Enable USART5 clock
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);

#ifdef SERIAL_UART5_RX_PIN
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    GPIO_PinAFConfig(SERIAL_UART5_PORT, SERIAL_UART5_RX_SOURCE, GPIO_AF_UART5);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART5_RX_PIN;
    GPIO_Init(SERIAL_UART5_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART5_RX_DMA_ST
    s->rxDMAStream = SERIAL_UART5_RX_DMA_ST;
    s->rxDMAChannel = SERIAL_UART5_RX_DMA_CH;
    s->rxDmaFlags = SERIAL_UART5_RX_TC_FLAG | SERIAL_UART5_RX_HT_FLAG | SERIAL_UART5_RX_TE_FLAG | SERIAL_UART5_RX_DM_FLAG | SERIAL_UART5_RX_FE_FLAG;
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = UART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART5_RX_DMA_ST
#endif	// SERIAL_UART5_RX_PIN

#ifdef SERIAL_UART5_TX_PIN
    s->txBufSize = (txBufSize) ? txBufSize : SERIAL_DEFAULT_BUFSIZE;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);

    GPIO_PinAFConfig(SERIAL_UART5_PORT, SERIAL_UART5_TX_SOURCE, GPIO_AF_UART5);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART5_TX_PIN;
    GPIO_Init(SERIAL_UART5_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART5_TX_DMA_ST
    s->txDMAStream = SERIAL_UART5_TX_DMA_ST;
    s->txDMAChannel = SERIAL_UART5_TX_DMA_CH;
    s->txDmaFlags = SERIAL_UART5_TX_TC_FLAG | SERIAL_UART5_TX_HT_FLAG | SERIAL_UART5_TX_TE_FLAG | SERIAL_UART5_TX_DM_FLAG | SERIAL_UART5_TX_FE_FLAG;

    // Enable the DMA global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = SERIAL_UART5_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART5_TX_DMA_ST
#endif	// SERIAL_UART5_TX_PIN

    return s;
}
#endif

#ifdef SERIAL_UART6_PORT
serialPort_t *serialUSART6(unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    serialPort_t *s;

    s = serialPort6 = (serialPort_t *)aqCalloc(1, sizeof(serialPort_t));

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_25MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;

    // Enable USART6 clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

#ifdef SERIAL_UART6_RX_PIN
    s->rxBufSize = (rxBufSize) ? rxBufSize : SERIAL_DEFAULT_RX_BUFSIZE;
    s->rxBuf = (volatile unsigned char*)aqCalloc(1, s->rxBufSize);

    GPIO_PinAFConfig(SERIAL_UART6_PORT, SERIAL_UART6_RX_SOURCE, GPIO_AF_USART6);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART6_RX_PIN;
    GPIO_Init(SERIAL_UART6_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART6_RX_DMA_ST
    s->rxDMAStream = SERIAL_UART6_RX_DMA_ST;
    s->rxDMAChannel = SERIAL_UART6_RX_DMA_CH;
    s->rxDmaFlags = SERIAL_UART6_RX_TC_FLAG | SERIAL_UART6_RX_HT_FLAG | SERIAL_UART6_RX_TE_FLAG | SERIAL_UART6_RX_DM_FLAG | SERIAL_UART6_RX_FE_FLAG;
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART6_RX_DMA_ST
#endif	// SERIAL_UART6_RX_PIN

#ifdef SERIAL_UART6_TX_PIN
    s->txBufSize = txBufSize;
    s->txBuf = (volatile unsigned char*)aqCalloc(1, s->txBufSize);

    GPIO_PinAFConfig(SERIAL_UART6_PORT, SERIAL_UART6_TX_SOURCE, GPIO_AF_USART6);

    GPIO_InitStructure.GPIO_Pin = SERIAL_UART6_TX_PIN;
    GPIO_Init(SERIAL_UART6_PORT, &GPIO_InitStructure);

#ifdef SERIAL_UART6_TX_DMA_ST
    s->txDMAStream = SERIAL_UART6_TX_DMA_ST;
    s->txDMAChannel = SERIAL_UART6_TX_DMA_CH;
    s->txDmaFlags = SERIAL_UART6_TX_TC_FLAG | SERIAL_UART6_TX_HT_FLAG | SERIAL_UART6_TX_TE_FLAG | SERIAL_UART6_TX_DM_FLAG | SERIAL_UART6_TX_FE_FLAG;

    // Enable the DMA global Interrupt
    NVIC_InitStructure.NVIC_IRQChannel = SERIAL_UART6_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#else
    // otherwise use interrupts
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	// SERIAL_UART6_TX_DMA_ST
#endif	// SERIAL_UART6_TX_PIN

#ifdef SERIAL_UART6_CTS_PIN
    if (flowControl == USART_HardwareFlowControl_RTS_CTS) {
	GPIO_PinAFConfig(SERIAL_UART6_PORT, SERIAL_UART6_RTS_SOURCE, GPIO_AF_USART6);
	GPIO_PinAFConfig(SERIAL_UART6_PORT, SERIAL_UART6_CTS_SOURCE, GPIO_AF_USART6);

	GPIO_InitStructure.GPIO_Pin = SERIAL_UART6_CTS_PIN | SERIAL_UART6_RTS_PIN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_DOWN;
	GPIO_Init(SERIAL_UART6_PORT, &GPIO_InitStructure);
    }
#endif	// SERIAL_UART6_CTS_PIN

    return s;
}
#endif

serialPort_t *serialOpen(USART_TypeDef *USARTx, unsigned int baud, uint16_t flowControl, unsigned int rxBufSize, unsigned int txBufSize) {
    DMA_InitTypeDef DMA_InitStructure;
    serialPort_t *s = 0;

    // Enable USART clocks/ports
#ifdef SERIAL_UART1_PORT
    if (USARTx == USART1) {
	s = serialUSART1(flowControl, rxBufSize, txBufSize);
    }
#endif

#ifdef SERIAL_UART2_PORT
    if (USARTx == USART2) {
	s = serialUSART2(flowControl, rxBufSize, txBufSize);
    }
#endif

#ifdef SERIAL_UART3_PORT
    if (USARTx == USART3) {
	s = serialUSART3(flowControl, rxBufSize, txBufSize);
    }
#endif

#ifdef SERIAL_UART4_PORT
    if (USARTx == UART4) {
	s = serialUSART4(flowControl, rxBufSize, txBufSize);
    }
#endif

#ifdef SERIAL_UART5_PORT
    if (USARTx == UART5) {
	s = serialUSART5(flowControl, rxBufSize, txBufSize);
    }
#endif

#ifdef SERIAL_UART6_PORT
    if (USARTx == USART6) {
	s = serialUSART6(flowControl, rxBufSize, txBufSize);
    }
#endif

    s->waitFlag = CoCreateFlag(0, 0);	// manual reset
    s->USARTx = USARTx;
    s->rxHead = s->rxTail = 0;
    s->txHead = s->txTail = 0;
    s->baudRate = baud;
    s->flowControl = flowControl;
    s->parity = USART_Parity_No;
    s->stopBits = USART_StopBits_1;

    serialOpenUART(s);

    DMA_StructInit(&DMA_InitStructure);
    DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)USARTx + 0x04;
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
    DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;

    // Configure DMA for rx
    if (s->rxDMAStream) {
	DMA_DeInit(s->rxDMAStream);
	DMA_InitStructure.DMA_Channel = s->rxDMAChannel;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)s->rxBuf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = s->rxBufSize;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(s->rxDMAStream, &DMA_InitStructure);

	DMA_ClearFlag(s->rxDMAStream, s->rxDmaFlags);

	DMA_Cmd(s->rxDMAStream, ENABLE);

	USART_DMACmd(USARTx, USART_DMAReq_Rx, ENABLE);
	s->rxPos = DMA_GetCurrDataCounter(s->rxDMAStream);
    }
    // otherwise use ISR
    else {
	USART_ITConfig(USARTx, USART_IT_RXNE, ENABLE);
    }

    // Configure DMA for tx
    if (s->txDMAStream) {
	DMA_DeInit(s->txDMAStream);
	DMA_InitStructure.DMA_Channel = s->txDMAChannel;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize = (s->txBufSize != 0) ? s->txBufSize : 16;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(s->txDMAStream, &DMA_InitStructure);

	DMA_SetCurrDataCounter(s->txDMAStream, 0);
	DMA_ITConfig(s->txDMAStream, DMA_IT_TC, ENABLE);

	USART_DMACmd(USARTx, USART_DMAReq_Tx, ENABLE);
    }
    // otherwise use ISR
    else {
	USART_ITConfig(USARTx, USART_IT_TXE, ENABLE);
    }

    // use this port for STDIO if not already defined
    if (serialSTDIO == 0)
	serialSTDIO = s;

    return s;
}

void serialWrite(serialPort_t *s, unsigned char ch) {
    s->txBuf[s->txHead] = ch;
    s->txHead = (s->txHead + 1) % s->txBufSize;

    if (s->txDMAStream)
	serialStartTxDMA(s);
    else
	USART_ITConfig(s->USARTx, USART_IT_TXE, ENABLE);
}

unsigned char serialAvailable(serialPort_t *s) {
    if (s->rxDMAStream)
	return (s->rxDMAStream->NDTR != s->rxPos);
    else
	return (s->rxTail != s->rxHead);
}

int serialRead(serialPort_t *s) {
    int ch;

    if (s->rxDMAStream) {
	ch = s->rxBuf[s->rxBufSize - s->rxPos];
	if (--s->rxPos == 0)
	    s->rxPos = s->rxBufSize;
    }
    else {
	ch = s->rxBuf[s->rxTail];
	s->rxTail = (s->rxTail + 1) % s->rxBufSize;
    }

    return ch;
}

int serialReadBlock(serialPort_t *s) {
    while (!serialAvailable(s))
	yield(1);

    return serialRead(s);
}

void serialPrint(serialPort_t *s, const char *str) {
    while (*str)
	serialWrite(s, *(str++));
}

void serialChangeBaud(serialPort_t *s, unsigned int baud) {
    s->baudRate = baud;
    serialOpenUART(s);
}

void serialChangeParity(serialPort_t *s, uint16_t parity) {
    s->parity = parity;
    serialOpenUART(s);
}

void serialChangeStopBits(serialPort_t *s, uint16_t stopBits) {
    s->stopBits = stopBits;
    serialOpenUART(s);
}

void serialWatch(void) {
#ifdef SERIAL_UART1_PORT
    if (serialPort1 && serialAvailable(serialPort1))
	CoSetFlag(serialPort1->waitFlag);
#endif
#ifdef SERIAL_UART2_PORT
    if (serialPort2 && serialAvailable(serialPort2))
	CoSetFlag(serialPort2->waitFlag);
#endif
#ifdef SERIAL_UART3_PORT
    if (serialPort3 && serialAvailable(serialPort3))
	CoSetFlag(serialPort3->waitFlag);
#endif
#ifdef SERIAL_UART4_PORT
    if (serialPort4 && serialAvailable(serialPort4))
	CoSetFlag(serialPort4->waitFlag);
#endif
#ifdef SERIAL_UART5_PORT
    if (serialPort5 && serialAvailable(serialPort5))
	CoSetFlag(serialPort5->waitFlag);
#endif
#ifdef SERIAL_UART6_PORT
    if (serialPort6 && serialAvailable(serialPort6))
	CoSetFlag(serialPort6->waitFlag);
#endif
}

void serialSetSTDIO(serialPort_t *s) {
    serialSTDIO = s;
}

int __putchar(int ch) {
    if (serialSTDIO)
	serialWrite(serialSTDIO, ch);

    return ch;
}

int __getchar(void) {
    int ch = 0;

    if (serialSTDIO)
	ch = serialRead(serialSTDIO);

    return ch;
}

//
// Interrupt handlers
//

// UART1 TX DMA
#ifdef SERIAL_UART1_PORT
#ifdef SERIAL_UART1_TX_DMA_ST
void SERIAL_UART1_TX_DMA_IT(void) {
    serialPort_t *s = serialPort1;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART1 global IRQ handler (might not be used)
void USART1_IRQHandler(void) {
    serialPort_t *s = serialPort1;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART1_PORT

// UART2 TX DMA
#ifdef SERIAL_UART2_PORT
#ifdef SERIAL_UART2_TX_DMA_ST
void SERIAL_UART2_TX_DMA_IT(void) {
    serialPort_t *s = serialPort2;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART2 global IRQ handler (might not be used)
void USART2_IRQHandler(void) {
    serialPort_t *s = serialPort2;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART2_PORT

// UART3 TX DMA
#ifdef SERIAL_UART3_PORT
#ifdef SERIAL_UART3_TX_DMA_ST
void SERIAL_UART3_TX_DMA_IT(void) {
    serialPort_t *s = serialPort3;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART3 global IRQ handler (might not be used)
void USART3_IRQHandler(void) {
    serialPort_t *s = serialPort3;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART3_PORT

// UART4 TX DMA
#ifdef SERIAL_UART4_PORT
#ifdef SERIAL_UART4_TX_DMA_ST
void SERIAL_UART4_TX_DMA_IT(void) {
    serialPort_t *s = serialPort4;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART4 global IRQ handler (might not be used)
void UART4_IRQHandler(void) {
    serialPort_t *s = serialPort4;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART4_PORT

// UART5 TX DMA
#ifdef SERIAL_UART5_PORT
#ifdef SERIAL_UART5_TX_DMA_ST
void SERIAL_UART5_TX_DMA_IT(void) {
    serialPort_t *s = serialPort5;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART5 global IRQ handler (might not be used)
void UART5_IRQHandler(void) {
    serialPort_t *s = serialPort5;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART5_PORT

// UART6 TX DMA
#ifdef SERIAL_UART6_PORT
#ifdef SERIAL_UART6_TX_DMA_ST
void SERIAL_UART6_TX_DMA_IT(void) {
    serialPort_t *s = serialPort6;

    DMA_ClearFlag(s->txDMAStream, s->txDmaFlags);

    s->txDmaRunning = 0;

    s->txDMACallback(s->txDMACallbackParam);
}
#endif

// USART6 global IRQ handler (might not be used)
void USART6_IRQHandler(void) {
    serialPort_t *s = serialPort6;
    uint16_t SR = s->USARTx->SR;

    if (SR & USART_FLAG_RXNE) {
	s->rxBuf[s->rxHead] = s->USARTx->DR;
	s->rxHead = (s->rxHead + 1) % s->rxBufSize;
    }

    if (SR & USART_FLAG_TXE) {
	if (s->txTail != s->txHead) {
	    s->USARTx->DR = s->txBuf[s->txTail];
	    s->txTail = (s->txTail + 1) % s->txBufSize;
	}
	// EOT
	else {
	    USART_ITConfig(s->USARTx, USART_IT_TXE, DISABLE);
	}
    }
}
#endif	// SERIAL_UART6_PORT
