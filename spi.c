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

/*
    This module handles the coordination of multiple SPI clients which may be
    sharing an SPI bus.  Once a transfer request has been queued, the caller
    will be notified via a flag or callback at their option when complete.
    Requests are handled on a FIFO basis.  It is up to the client(s) to make sure
    that they do not call spiTransaction() concurrently as it is not thread safe.
    This can easily be handled if clients all submit transaction requests via
    data-ready ISR's of the same priority and preemption level.

    The Ethernet periphreal's interrupt lines are stolen for the SPI scheduler.
    If Ethernet is one day needed, this must be changed.
*/

#include "spi.h"
#include "aq_timer.h"
#include "util.h"
#include "digital.h"

spiStruct_t spiData[2];

inline void spiTriggerSchedule(uint8_t interface) {
    if (interface)
	NVIC->STIR = ETH_WKUP_IRQn;
    else
	NVIC->STIR = ETH_IRQn;
}

#ifdef SPI_SPI1_CLOCK
void spi1Init(uint16_t baud) {
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    if (!spiData[0].initialized) {
	// SPI interface
	RCC_APB2PeriphClockCmd(SPI_SPI1_CLOCK, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	// SPI SCK / MOSI / MISO pin configuration
	GPIO_InitStructure.GPIO_Pin = SPI_SPI1_SCK_PIN;
	GPIO_Init(SPI_SPI1_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_SPI1_MISO_PIN;
	GPIO_Init(SPI_SPI1_MISO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_SPI1_MOSI_PIN;
	GPIO_Init(SPI_SPI1_MOSI_PORT, &GPIO_InitStructure);

	// Connect SPI pins to Alternate Function
	GPIO_PinAFConfig(SPI_SPI1_SCK_PORT,SPI_SPI1_SCK_SOURCE, SPI_SPI1_AF);
	GPIO_PinAFConfig(SPI_SPI1_MISO_PORT, SPI_SPI1_MISO_SOURCE, SPI_SPI1_AF);
	GPIO_PinAFConfig(SPI_SPI1_MOSI_PORT, SPI_SPI1_MOSI_SOURCE, SPI_SPI1_AF);

	// SPI configuration
	SPI_I2S_DeInit(SPI1);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_BaudRatePrescaler = baud;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI1, &SPI_InitStructure);

	SPI_I2S_DMACmd(SPI1, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	// RX DMA
	DMA_DeInit(SPI_SPI1_DMA_RX);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = SPI_SPI1_DMA_RX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)spiData;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(SPI_SPI1_DMA_RX, &DMA_InitStructure);

	DMA_ClearITPendingBit(SPI_SPI1_DMA_RX, SPI_SPI1_DMA_RX_FLAGS);
	DMA_ITConfig(SPI_SPI1_DMA_RX, DMA_IT_TC, ENABLE);

	// Enable RX DMA global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = SPI_SPI1_DMA_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// TX DMA - one shot
	DMA_DeInit(SPI_SPI1_DMA_TX);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = SPI_SPI1_DMA_TX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI1->DR;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(SPI_SPI1_DMA_TX, &DMA_InitStructure);

	DMA_ClearITPendingBit(SPI_SPI1_DMA_TX, SPI_SPI1_DMA_TX_FLAGS);

	spiData[0].spi = SPI1;
	spiData[0].rxDMAStream = SPI_SPI1_DMA_RX;
	spiData[0].txDMAStream = SPI_SPI1_DMA_TX;
	spiData[0].initialized = 1;

	// Enable Ethernet Interrupt (for our stack management)
	NVIC_InitStructure.NVIC_IRQChannel = ETH_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    }
}
#endif

#ifdef SPI_SPI2_CLOCK
void spi2Init(uint16_t baud) {
    GPIO_InitTypeDef GPIO_InitStructure;
    SPI_InitTypeDef  SPI_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    DMA_InitTypeDef DMA_InitStructure;

    if (!spiData[1].initialized) {
	// SPI interface
	RCC_APB1PeriphClockCmd(SPI_SPI2_CLOCK, ENABLE);

	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_NOPULL;

	// SPI SCK / MOSI / MISO pin configuration
	GPIO_InitStructure.GPIO_Pin = SPI_SPI2_SCK_PIN;
	GPIO_Init(SPI_SPI2_SCK_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_SPI2_MISO_PIN;
	GPIO_Init(SPI_SPI2_MISO_PORT, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = SPI_SPI2_MOSI_PIN;
	GPIO_Init(SPI_SPI2_MOSI_PORT, &GPIO_InitStructure);

	// Connect SPI pins to AF6
	GPIO_PinAFConfig(SPI_SPI2_SCK_PORT,SPI_SPI2_SCK_SOURCE, SPI_SPI2_AF);
	GPIO_PinAFConfig(SPI_SPI2_MISO_PORT, SPI_SPI2_MISO_SOURCE, SPI_SPI2_AF);
	GPIO_PinAFConfig(SPI_SPI2_MOSI_PORT, SPI_SPI2_MOSI_SOURCE, SPI_SPI2_AF);

	// SPI configuration
	SPI_I2S_DeInit(SPI2);
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStructure.SPI_BaudRatePrescaler = baud;
	SPI_InitStructure.SPI_CRCPolynomial = 7;
	SPI_Init(SPI2, &SPI_InitStructure);

	SPI_I2S_DMACmd(SPI2, SPI_I2S_DMAReq_Rx | SPI_I2S_DMAReq_Tx, ENABLE);

	// RX DMA
	DMA_DeInit(SPI_SPI2_DMA_RX);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = SPI_SPI2_DMA_RX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)spiData;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(SPI_SPI2_DMA_RX, &DMA_InitStructure);

	DMA_ClearITPendingBit(SPI_SPI2_DMA_RX, SPI_SPI2_DMA_RX_FLAGS);
	DMA_ITConfig(SPI_SPI2_DMA_RX, DMA_IT_TC, ENABLE);

	// Enable RX DMA global Interrupt
	NVIC_InitStructure.NVIC_IRQChannel = SPI_SPI2_DMA_RX_IRQ;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	// TX DMA - one shot
	DMA_DeInit(SPI_SPI2_DMA_TX);
	DMA_StructInit(&DMA_InitStructure);
	DMA_InitStructure.DMA_Channel = SPI_SPI2_DMA_TX_CHANNEL;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&SPI2->DR;
	DMA_InitStructure.DMA_BufferSize = 1;
	DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(SPI_SPI2_DMA_TX, &DMA_InitStructure);

	DMA_ClearITPendingBit(SPI_SPI2_DMA_TX, SPI_SPI2_DMA_TX_FLAGS);

	spiData[1].spi = SPI2;
	spiData[1].rxDMAStream = SPI_SPI2_DMA_RX;
	spiData[1].txDMAStream = SPI_SPI2_DMA_TX;
	spiData[1].initialized = 1;

	// Enable Ethernet Interrupt (for our stack management)
	NVIC_InitStructure.NVIC_IRQChannel = ETH_WKUP_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
    }
}
#endif

void spiDeselect(spiClient_t *client) {
    digitalHi(client->cs);
}

void spiSelect(spiClient_t *client) {
    digitalLo(client->cs);
}

void spiNotify(spiClient_t *client) {
    if (client->flag)
	*client->flag = timerMicros();

    if (client->callback)
	client->callback(0);
}

void spiStartTxn(spiStruct_t *interface) {
    uint32_t tmp;

    if (interface->txRunning == 0 && interface->tail != interface->head) {
	spiSlot_t *slot = &interface->slots[interface->tail];
	spiClient_t *client = slot->client;

	// set baud rate
	tmp = interface->spi->CR1 & SPI_BAUD_MASK;
	tmp |= client->baud;
	interface->spi->CR1 = tmp;

	spiSelect(client);
	SPI_Cmd(interface->spi, ENABLE);

	// specify "in transaction"
	if (client->flag)
	    *client->flag = 0;

	interface->rxDMAStream->M0AR = slot->rxBuf;
	interface->rxDMAStream->NDTR = slot->size;
	DMA_Cmd(interface->rxDMAStream, ENABLE);

	interface->txDMAStream->M0AR = slot->txBuf;
	interface->txDMAStream->NDTR = slot->size;
	DMA_Cmd(interface->txDMAStream, ENABLE);

	interface->txRunning = 1;
    }
}

void spiEndTxn(spiStruct_t *interface) {
    uint8_t tail = interface->tail;
    spiClient_t *client = interface->slots[tail].client;

    // shutdown DMA
    DMA_Cmd(interface->txDMAStream, DISABLE);
    DMA_Cmd(interface->rxDMAStream, DISABLE);

    spiDeselect(client);
    SPI_Cmd(interface->spi, DISABLE);

    spiNotify(client);

    tail = (tail + 1) % SPI_SLOTS;
    interface->tail = tail;

    interface->txRunning = 0;

    spiTriggerSchedule(client->interface);
}

void spiChangeBaud(spiClient_t *client, uint16_t baud) {
    client->baud = baud;
}

void spiChangeCallback(spiClient_t *client, spiCallback_t *callback) {
    client->callback = callback;
}

// TODO: not yet thread safe
void spiTransaction(spiClient_t *client, volatile void *rxBuf, void *txBuf, uint16_t size) {
    spiStruct_t *interface = &spiData[client->interface];
    uint8_t head = interface->head;
    spiSlot_t *slot = &interface->slots[head];

    slot->size = size;
    slot->rxBuf = (uint32_t)rxBuf;
    slot->txBuf = (uint32_t)txBuf;
    slot->client = client;

    interface->head = (head + 1) % SPI_SLOTS;

    spiTriggerSchedule(client->interface);
}

spiClient_t *spiClientInit(SPI_TypeDef *spi, uint16_t baud, GPIO_TypeDef *csPort, uint16_t csPin, volatile uint32_t *flag, spiCallback_t *callback) {
    spiClient_t *client;

    client = (spiClient_t *)aqCalloc(1, sizeof(spiClient_t));

#ifdef SPI_SPI1_CLOCK
    if (spi == SPI1) {
	client->interface = 0;
	spi1Init(baud);
    }
#endif
#ifdef SPI_SPI2_CLOCK
    if (spi == SPI2) {
	client->interface = 1;
	spi2Init(baud);
    }
#endif

    client->cs = digitalInit(csPort, csPin);
    spiDeselect(client);

    client->baud = baud;
    client->flag = flag;
    client->callback = callback;

    return client;
}

#ifdef SPI_SPI2_CLOCK
void SPI_SPI2_DMA_RX_HANDLER(void) {
    DMA_ClearITPendingBit(SPI_SPI2_DMA_RX, SPI_SPI2_DMA_RX_FLAGS);
    DMA_ClearITPendingBit(SPI_SPI2_DMA_TX, SPI_SPI2_DMA_TX_FLAGS);

    // finish transaction
    spiEndTxn(&spiData[1]);
}
#endif

#ifdef SPI_SPI1_CLOCK
void SPI_SPI1_DMA_RX_HANDLER(void) {
    DMA_ClearITPendingBit(SPI_SPI1_DMA_RX, SPI_SPI1_DMA_RX_FLAGS);
    DMA_ClearITPendingBit(SPI_SPI1_DMA_TX, SPI_SPI1_DMA_TX_FLAGS);

    // finish transaction
    spiEndTxn(&spiData[0]);
}
#endif

// co-op the Ethernet IRQ for our purposes
void ETH_IRQHandler(void) {
    spiStartTxn(&spiData[0]);
}

void ETH_WKUP_IRQHandler(void) {
    spiStartTxn(&spiData[1]);
}
