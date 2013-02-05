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

#ifndef _serial_h
#define _serial_h

#include "stm32f4xx.h"
#include <CoOS.h>

#define SERIAL_UART1_PORT	GPIOA
#define SERIAL_UART1_RX_PIN	GPIO_Pin_10
#define SERIAL_UART1_TX_PIN	GPIO_Pin_9
#define SERIAL_UART1_CTS_PIN	GPIO_Pin_11
#define SERIAL_UART1_RTS_PIN	GPIO_Pin_12
#define SERIAL_UART1_RX_SOURCE	GPIO_PinSource10
#define SERIAL_UART1_TX_SOURCE	GPIO_PinSource9
#define SERIAL_UART1_CTS_SOURCE	GPIO_PinSource11
#define SERIAL_UART1_RTS_SOURCE	GPIO_PinSource12
#define SERIAL_UART1_RX_DMA_ST	DMA2_Stream5
#define SERIAL_UART1_TX_DMA_ST	DMA2_Stream7
#define SERIAL_UART1_RX_DMA_CH	DMA_Channel_4
#define SERIAL_UART1_TX_DMA_CH	DMA_Channel_4
#define SERIAL_UART1_TX_DMA_IT	DMA2_Stream7_IRQHandler
#define SERIAL_UART1_TX_IRQn	DMA2_Stream7_IRQn
#define SERIAL_UART1_RX_TC_FLAG	DMA_FLAG_TCIF5
#define SERIAL_UART1_RX_HT_FLAG	DMA_FLAG_HTIF5
#define SERIAL_UART1_RX_TE_FLAG	DMA_FLAG_TEIF5
#define SERIAL_UART1_RX_DM_FLAG	DMA_FLAG_DMEIF5
#define SERIAL_UART1_RX_FE_FLAG	DMA_FLAG_FEIF5
#define SERIAL_UART1_TX_TC_FLAG	DMA_FLAG_TCIF7
#define SERIAL_UART1_TX_HT_FLAG	DMA_FLAG_HTIF7
#define SERIAL_UART1_TX_TE_FLAG	DMA_FLAG_TEIF7
#define SERIAL_UART1_TX_DM_FLAG	DMA_FLAG_DMEIF7
#define SERIAL_UART1_TX_FE_FLAG	DMA_FLAG_FEIF7

#define SERIAL_UART2_PORT	GPIOD
#define SERIAL_UART2_RX_PIN	GPIO_Pin_6
#define SERIAL_UART2_TX_PIN	GPIO_Pin_5
#define SERIAL_UART2_CTS_PIN	GPIO_Pin_3
#define SERIAL_UART2_RTS_PIN	GPIO_Pin_4
#define SERIAL_UART2_RX_SOURCE	GPIO_PinSource6
#define SERIAL_UART2_TX_SOURCE	GPIO_PinSource5
#define SERIAL_UART2_CTS_SOURCE	GPIO_PinSource3
#define SERIAL_UART2_RTS_SOURCE	GPIO_PinSource4
#define SERIAL_UART2_RX_DMA_ST	DMA1_Stream5
#define SERIAL_UART2_TX_DMA_ST	DMA1_Stream6
#define SERIAL_UART2_RX_DMA_CH	DMA_Channel_4
#define SERIAL_UART2_TX_DMA_CH	DMA_Channel_4
#define SERIAL_UART2_TX_DMA_IT	DMA1_Stream6_IRQHandler
#define SERIAL_UART2_TX_IRQn	DMA1_Stream6_IRQn
#define SERIAL_UART2_RX_TC_FLAG	DMA_FLAG_TCIF5
#define SERIAL_UART2_RX_HT_FLAG	DMA_FLAG_HTIF5
#define SERIAL_UART2_RX_TE_FLAG	DMA_FLAG_TEIF5
#define SERIAL_UART2_RX_DM_FLAG	DMA_FLAG_DMEIF5
#define SERIAL_UART2_RX_FE_FLAG	DMA_FLAG_FEIF5
#define SERIAL_UART2_TX_TC_FLAG	DMA_FLAG_TCIF6
#define SERIAL_UART2_TX_HT_FLAG	DMA_FLAG_HTIF6
#define SERIAL_UART2_TX_TE_FLAG	DMA_FLAG_TEIF6
#define SERIAL_UART2_TX_DM_FLAG	DMA_FLAG_DMEIF6
#define SERIAL_UART2_TX_FE_FLAG	DMA_FLAG_FEIF6

#define SERIAL_UART3_PORT	GPIOD
#define SERIAL_UART3_RX_PIN	GPIO_Pin_9
#define SERIAL_UART3_TX_PIN	GPIO_Pin_8
//#define SERIAL_UART3_CTS_PIN	GPIO_Pin_11
//#define SERIAL_UART3_RTS_PIN	GPIO_Pin_12
#define SERIAL_UART3_RX_SOURCE	GPIO_PinSource9
#define SERIAL_UART3_TX_SOURCE	GPIO_PinSource8
//#define SERIAL_UART3_CTS_SOURCE	GPIO_PinSource11
//#define SERIAL_UART3_RTS_SOURCE	GPIO_PinSource12
#define SERIAL_UART3_RX_DMA_ST	DMA1_Stream1
//#define SERIAL_UART3_TX_DMA_ST	DMA1_Stream3
#define SERIAL_UART3_RX_DMA_CH	DMA_Channel_4
//#define SERIAL_UART3_TX_DMA_CH	DMA_Channel_4
//#define SERIAL_UART3_TX_DMA_IT	DMA1_Stream3_IRQHandler
//#define SERIAL_UART3_TX_IRQn	DMA1_Stream3_IRQn
#define SERIAL_UART3_RX_TC_FLAG	DMA_FLAG_TCIF1
#define SERIAL_UART3_RX_HT_FLAG	DMA_FLAG_HTIF1
#define SERIAL_UART3_RX_TE_FLAG	DMA_FLAG_TEIF1
#define SERIAL_UART3_RX_DM_FLAG	DMA_FLAG_DMEIF1
#define SERIAL_UART3_RX_FE_FLAG	DMA_FLAG_FEIF1
//#define SERIAL_UART3_TX_TC_FLAG	DMA_FLAG_TCIF3
//#define SERIAL_UART3_TX_HT_FLAG	DMA_FLAG_HTIF3
//#define SERIAL_UART3_TX_TE_FLAG	DMA_FLAG_TEIF3
//#define SERIAL_UART3_TX_DM_FLAG	DMA_FLAG_DMEIF3
//#define SERIAL_UART3_TX_FE_FLAG	DMA_FLAG_FEIF3

#define SERIAL_UART4_PORT	GPIOA
#define SERIAL_UART4_RX_PIN	GPIO_Pin_1
//#define SERIAL_UART4_TX_PIN	GPIO_Pin_0
#define SERIAL_UART4_RX_SOURCE	GPIO_PinSource1
//#define SERIAL_UART4_TX_SOURCE	GPIO_PinSource0
#define SERIAL_UART4_RX_DMA_ST	DMA1_Stream2
//#define SERIAL_UART4_TX_DMA_ST
#define SERIAL_UART4_RX_DMA_CH	DMA_Channel_4
//#define SERIAL_UART4_TX_DMA_CH
//#define SERIAL_UART4_TX_DMA_IT
//#define SERIAL_UART4_TX_IRQn
#define SERIAL_UART4_RX_TC_FLAG	DMA_FLAG_TCIF2
#define SERIAL_UART4_RX_HT_FLAG	DMA_FLAG_HTIF2
#define SERIAL_UART4_RX_TE_FLAG	DMA_FLAG_TEIF2
#define SERIAL_UART4_RX_DM_FLAG	DMA_FLAG_DMEIF2
#define SERIAL_UART4_RX_FE_FLAG	DMA_FLAG_FEIF2
//#define SERIAL_UART4_TX_TC_FLAG
//#define SERIAL_UART4_TX_HT_FLAG
//#define SERIAL_UART4_TX_TE_FLAG
//#define SERIAL_UART4_TX_DM_FLAG
//#define SERIAL_UART4_TX_FE_FLAG

#define SERIAL_UART5_PORT	GPIOD
//#define SERIAL_UART5_TX_PIN	GPIO_Pin_10
#define SERIAL_UART5_RX_PIN	GPIO_Pin_2
//#define SERIAL_UART5_TX_SOURCE	GPIO_PinSource10
#define SERIAL_UART5_RX_SOURCE	GPIO_PinSource2
#define SERIAL_UART5_RX_DMA_ST	DMA1_Stream0
//#define SERIAL_UART5_TX_DMA_ST	DMA1_Stream7
#define SERIAL_UART5_RX_DMA_CH	DMA_Channel_4
//#define SERIAL_UART5_TX_DMA_CH	DMA_Channel_4
//#define SERIAL_UART5_TX_DMA_IT	DMA1_Stream7_IRQHandler
//#define SERIAL_UART5_TX_IRQn		DMA1_Stream7_IRQn
#define SERIAL_UART5_RX_TC_FLAG	DMA_FLAG_TCIF0
#define SERIAL_UART5_RX_HT_FLAG	DMA_FLAG_HTIF0
#define SERIAL_UART5_RX_TE_FLAG	DMA_FLAG_TEIF0
#define SERIAL_UART5_RX_DM_FLAG	DMA_FLAG_DMEIF0
#define SERIAL_UART5_RX_FE_FLAG	DMA_FLAG_FEIF0
//#define SERIAL_UART5_TX_TC_FLAG	DMA_FLAG_TCIF7
//#define SERIAL_UART5_TX_HT_FLAG	DMA_FLAG_HTIF7
//#define SERIAL_UART5_TX_TE_FLAG	DMA_FLAG_TEIF7
//#define SERIAL_UART5_TX_DM_FLAG	DMA_FLAG_DMEIF7
//#define SERIAL_UART5_TX_FE_FLAG	DMA_FLAG_FEIF7

#define SERIAL_UART6_PORT	GPIOC
#define SERIAL_UART6_RX_PIN	GPIO_Pin_7
#define SERIAL_UART6_TX_PIN	GPIO_Pin_6
//#define SERIAL_UART6_CTS_PIN	GPIO_Pin_11
//#define SERIAL_UART6_RTS_PIN	GPIO_Pin_12
#define SERIAL_UART6_RX_SOURCE	GPIO_PinSource7
#define SERIAL_UART6_TX_SOURCE	GPIO_PinSource6
//#define SERIAL_UART6_CTS_SOURCE	GPIO_PinSource11
//#define SERIAL_UART6_RTS_SOURCE	GPIO_PinSource12
#define SERIAL_UART6_RX_DMA_ST	DMA2_Stream1
#define SERIAL_UART6_TX_DMA_ST	DMA2_Stream6
#define SERIAL_UART6_RX_DMA_CH	DMA_Channel_5
#define SERIAL_UART6_TX_DMA_CH	DMA_Channel_5
#define SERIAL_UART6_TX_DMA_IT	DMA2_Stream6_IRQHandler
#define SERIAL_UART6_TX_IRQn	DMA2_Stream6_IRQn
#define SERIAL_UART6_RX_TC_FLAG	DMA_FLAG_TCIF1
#define SERIAL_UART6_RX_HT_FLAG	DMA_FLAG_HTIF1
#define SERIAL_UART6_RX_TE_FLAG	DMA_FLAG_TEIF1
#define SERIAL_UART6_RX_DM_FLAG	DMA_FLAG_DMEIF1
#define SERIAL_UART6_RX_FE_FLAG	DMA_FLAG_FEIF1
#define SERIAL_UART6_TX_TC_FLAG	DMA_FLAG_TCIF6
#define SERIAL_UART6_TX_HT_FLAG	DMA_FLAG_HTIF6
#define SERIAL_UART6_TX_TE_FLAG	DMA_FLAG_TEIF6
#define SERIAL_UART6_TX_DM_FLAG	DMA_FLAG_DMEIF6
#define SERIAL_UART6_TX_FE_FLAG	DMA_FLAG_FEIF6

#define SERIAL_DEFAULT_BUFSIZE	128

typedef struct {
    unsigned int baudRate;
    unsigned int flowControl;

    unsigned int txBufSize;
    volatile unsigned char *txBuf;
    volatile unsigned int txHead, txTail;

    unsigned int rxBufSize;
    volatile unsigned char *rxBuf;
    volatile unsigned int rxHead, rxTail;
    volatile unsigned int rxPos;

    USART_TypeDef *USARTx;
    DMA_Stream_TypeDef *rxDMAStream;
    DMA_Stream_TypeDef *txDMAStream;
    uint32_t rxDMAChannel;
    uint32_t txDMAChannel;
    uint32_t rxDmaFlags, txDmaFlags;
    volatile unsigned char txDmaRunning;

    OS_FlagID waitFlag;
} serialPort_t;

extern serialPort_t *serialOpen(USART_TypeDef *USARTx, unsigned int baud, unsigned int flowControl, unsigned int rxBufSize, unsigned int txBufSize);
extern void serialChangeBaud(serialPort_t *s, unsigned int baud);
extern void serialSetSTDIO(serialPort_t *s);
extern void serialWrite(serialPort_t *s, unsigned char ch);
extern void serialWatch(void);
extern unsigned char serialAvailable(serialPort_t *s);
extern int serialRead(serialPort_t *s);
extern void serialPrint(serialPort_t *s, const char *str);
extern int __putchar(int ch);
extern int __getchar(void);

extern serialPort_t *serialSTDIO;

#endif
