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

    Copyright © 2014  Bill Nesbitt

    Portions borrowed from the superbitrf project
    Copyright (C) 2013 Freek van Tienen <freek.v.tienen@gmail.com>
*/

#include "dsm.h"
#include "cyrf6936.h"
#include "cyrf6936_regs.h"
#include "comm.h"
#include "aq_timer.h"
#include "util.h"
#include "config.h"
#include <string.h>

#ifdef CYRF_SPI

// The PN codes
const uint8_t pn_codes[5][9][8] = {
{ /* Row 0 */
  /* Col 0 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8},
  /* Col 1 */ {0x88, 0x17, 0x13, 0x3B, 0x2D, 0xBF, 0x06, 0xD6},
  /* Col 2 */ {0xF1, 0x94, 0x30, 0x21, 0xA1, 0x1C, 0x88, 0xA9},
  /* Col 3 */ {0xD0, 0xD2, 0x8E, 0xBC, 0x82, 0x2F, 0xE3, 0xB4},
  /* Col 4 */ {0x8C, 0xFA, 0x47, 0x9B, 0x83, 0xA5, 0x66, 0xD0},
  /* Col 5 */ {0x07, 0xBD, 0x9F, 0x26, 0xC8, 0x31, 0x0F, 0xB8},
  /* Col 6 */ {0xEF, 0x03, 0x95, 0x89, 0xB4, 0x71, 0x61, 0x9D},
  /* Col 7 */ {0x40, 0xBA, 0x97, 0xD5, 0x86, 0x4F, 0xCC, 0xD1},
  /* Col 8 */ {0xD7, 0xA1, 0x54, 0xB1, 0x5E, 0x89, 0xAE, 0x86}
},
{ /* Row 1 */
  /* Col 0 */ {0x83, 0xF7, 0xA8, 0x2D, 0x7A, 0x44, 0x64, 0xD3},
  /* Col 1 */ {0x3F, 0x2C, 0x4E, 0xAA, 0x71, 0x48, 0x7A, 0xC9},
  /* Col 2 */ {0x17, 0xFF, 0x9E, 0x21, 0x36, 0x90, 0xC7, 0x82},
  /* Col 3 */ {0xBC, 0x5D, 0x9A, 0x5B, 0xEE, 0x7F, 0x42, 0xEB},
  /* Col 4 */ {0x24, 0xF5, 0xDD, 0xF8, 0x7A, 0x77, 0x74, 0xE7},
  /* Col 5 */ {0x3D, 0x70, 0x7C, 0x94, 0xDC, 0x84, 0xAD, 0x95},
  /* Col 6 */ {0x1E, 0x6A, 0xF0, 0x37, 0x52, 0x7B, 0x11, 0xD4},
  /* Col 7 */ {0x62, 0xF5, 0x2B, 0xAA, 0xFC, 0x33, 0xBF, 0xAF},
  /* Col 8 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97}
},
{ /* Row 2 */
  /* Col 0 */ {0x40, 0x56, 0x32, 0xD9, 0x0F, 0xD9, 0x5D, 0x97},
  /* Col 1 */ {0x8E, 0x4A, 0xD0, 0xA9, 0xA7, 0xFF, 0x20, 0xCA},
  /* Col 2 */ {0x4C, 0x97, 0x9D, 0xBF, 0xB8, 0x3D, 0xB5, 0xBE},
  /* Col 3 */ {0x0C, 0x5D, 0x24, 0x30, 0x9F, 0xCA, 0x6D, 0xBD},
  /* Col 4 */ {0x50, 0x14, 0x33, 0xDE, 0xF1, 0x78, 0x95, 0xAD},
  /* Col 5 */ {0x0C, 0x3C, 0xFA, 0xF9, 0xF0, 0xF2, 0x10, 0xC9},
  /* Col 6 */ {0xF4, 0xDA, 0x06, 0xDB, 0xBF, 0x4E, 0x6F, 0xB3},
  /* Col 7 */ {0x9E, 0x08, 0xD1, 0xAE, 0x59, 0x5E, 0xE8, 0xF0},
  /* Col 8 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E}
},
{ /* Row 3 */
  /* Col 0 */ {0xC0, 0x90, 0x8F, 0xBB, 0x7C, 0x8E, 0x2B, 0x8E},
  /* Col 1 */ {0x80, 0x69, 0x26, 0x80, 0x08, 0xF8, 0x49, 0xE7},
  /* Col 2 */ {0x7D, 0x2D, 0x49, 0x54, 0xD0, 0x80, 0x40, 0xC1},
  /* Col 3 */ {0xB6, 0xF2, 0xE6, 0x1B, 0x80, 0x5A, 0x36, 0xB4},
  /* Col 4 */ {0x42, 0xAE, 0x9C, 0x1C, 0xDA, 0x67, 0x05, 0xF6},
  /* Col 5 */ {0x9B, 0x75, 0xF7, 0xE0, 0x14, 0x8D, 0xB5, 0x80},
  /* Col 6 */ {0xBF, 0x54, 0x98, 0xB9, 0xB7, 0x30, 0x5A, 0x88},
  /* Col 7 */ {0x35, 0xD1, 0xFC, 0x97, 0x23, 0xD4, 0xC9, 0x88},
  /* Col 8 */ {0x88, 0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40}
},
{ /* Row 4 */
  /* Col 0 */ {0xE1, 0xD6, 0x31, 0x26, 0x5F, 0xBD, 0x40, 0x93},
  /* Col 1 */ {0xDC, 0x68, 0x08, 0x99, 0x97, 0xAE, 0xAF, 0x8C},
  /* Col 2 */ {0xC3, 0x0E, 0x01, 0x16, 0x0E, 0x32, 0x06, 0xBA},
  /* Col 3 */ {0xE0, 0x83, 0x01, 0xFA, 0xAB, 0x3E, 0x8F, 0xAC},
  /* Col 4 */ {0x5C, 0xD5, 0x9C, 0xB8, 0x46, 0x9C, 0x7D, 0x84},
  /* Col 5 */ {0xF1, 0xC6, 0xFE, 0x5C, 0x9D, 0xA5, 0x4F, 0xB7},
  /* Col 6 */ {0x58, 0xB5, 0xB3, 0xDD, 0x0E, 0x28, 0xF1, 0xB0},
  /* Col 7 */ {0x5F, 0x30, 0x3B, 0x56, 0x96, 0x45, 0xF4, 0xA1},
  /* Col 8 */ {0x03, 0xBC, 0x6E, 0x8A, 0xEF, 0xBD, 0xFE, 0xF8}
},
};

const uint8_t pn_bind[] = {0x98, 0x88, 0x1B, 0xE4, 0x30, 0x79, 0x03, 0x84};

// The CYRF initial config, binding config and transfer config
const uint8_t cyrf_config[][2] = {
    {CYRF_MODE_OVERRIDE | CYRF_WRITE_BIT, CYRF_RST},                                         // Reset the device
    {CYRF_CLK_EN | CYRF_WRITE_BIT, CYRF_RXF},                                                // Enable the clock
    {CYRF_AUTO_CAL_TIME | CYRF_WRITE_BIT, 0x3C},                                             // From manual, needed for initialization
    {CYRF_AUTO_CAL_OFFSET | CYRF_WRITE_BIT, 0x14},                                           // From manual, needed for initialization
    {CYRF_RX_CFG | CYRF_WRITE_BIT, CYRF_LNA | CYRF_FAST_TURN_EN},                            // Enable low noise amplifier and fast turning
    {CYRF_TX_OFFSET_LSB | CYRF_WRITE_BIT, 0x55},                                             // From manual, typical configuration
    {CYRF_TX_OFFSET_MSB | CYRF_WRITE_BIT, 0x05},                                             // From manual, typical configuration
    {CYRF_XACT_CFG | CYRF_WRITE_BIT, CYRF_MODE_SYNTH_RX | CYRF_FRC_END},                     // Force in Synth RX mode
    {CYRF_TX_CFG | CYRF_WRITE_BIT, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},  // Enable 64 chip codes, SDR mode and amplifier +4dBm
    {CYRF_DATA64_THOLD | CYRF_WRITE_BIT, 0x0E},                                              // From manual, typical configuration
    {CYRF_XACT_CFG | CYRF_WRITE_BIT, CYRF_MODE_SYNTH_RX},                                    // Set in Synth RX mode (again, really needed?)
    {0xff, 0xff}
};

const uint8_t cyrf_bind_config[][2] = {
    {CYRF_TX_CFG | CYRF_WRITE_BIT, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_SDR | CYRF_PA_4},  // Enable 64 chip codes, SDR mode and amplifier +4dBm
    {CYRF_FRAMING_CFG | CYRF_WRITE_BIT, CYRF_SOP_LEN | 0xE},                                 // Set SOP CODE to 64 chips and SOP Correlator Threshold to 0xE
    {CYRF_RX_OVERRIDE | CYRF_WRITE_BIT, CYRF_FRC_RXDR | CYRF_DIS_RXCRC},                     // Force receive data rate and disable receive CRC checker
    {CYRF_EOP_CTRL | CYRF_WRITE_BIT, 0x02},                                                  // Only enable EOP symbol count of 2
    {CYRF_TX_OVERRIDE | CYRF_WRITE_BIT, CYRF_DIS_TXCRC},                                     // Disable transmit CRC generate
    {0xff, 0xff}
};

const uint8_t cyrf_transfer_config[][2] = {
    {CYRF_TX_CFG | CYRF_WRITE_BIT, CYRF_DATA_CODE_LENGTH | CYRF_DATA_MODE_8DR | CYRF_PA_4},  // Enable 64 chip codes, 8DR mode and amplifier +4dBm
    {CYRF_FRAMING_CFG | CYRF_WRITE_BIT, CYRF_SOP_EN | CYRF_SOP_LEN | CYRF_LEN_EN | 0xE},     // Set SOP CODE enable, SOP CODE to 64 chips, Packet length enable, and SOP Correlator Threshold to 0xE
    {CYRF_TX_OVERRIDE | CYRF_WRITE_BIT, 0x00},                                               // Reset TX overrides
    {CYRF_RX_OVERRIDE | CYRF_WRITE_BIT, 0x00},                                               // Reset RX overrides
    {0xff, 0xff}
};

// Start the receive of the cyrf6936
const uint8_t cyrf_start_receive[][2] = {
    {CYRF_RX_CTRL | CYRF_WRITE_BIT, CYRF_RX_GO | CYRF_RXC_IRQEN | CYRF_RXE_IRQEN},           // Start receiving and set the IRQ
    {CYRF_RSSI, 0x00},                                                                       // clear SOP bit in RSSI reg
    {0xff, 0xff}
};

void dsmTransferTimeoutCallback(int unused);
void dsmSyncTimeoutCallback(int unused);
void dsmBindTimeoutCallback(int unused);
void dsmTransferReceive(int state);
void dsmSyncAReceive(int state);
void dsmSyncBReceive(int state);
void dsmBindReceive(int state);
void dsmStartTransfer(void);
void _dsmAbortReceive(int state);

dsmStruct_t dsmData;

void dsmSetRfChannel(uint8_t chan) {
    dsmData.rfChannel = chan;
    cyrfQueueWriteReg(CYRF_CHANNEL, chan);
}

void dsmSetBindChannel(void) {
    uint8_t chan = dsmData.rfChannel + 2;

    if (chan > DSM_MAX_CHANNEL)
        chan = 1;

    dsmSetRfChannel(chan);
}

uint16_t dsmChannelTime(void) {
	if (dsmData.crcSeed == dsmData.crc)
		return DSM_CHB_TIME;
	else
		return DSM_CHA_TIME;
}

void dsmSetChannel(uint8_t channel, uint8_t isDsm2, uint8_t sopCol, uint8_t dataCol, uint16_t crcSeed) {
    uint8_t pnRow;

    pnRow = isDsm2 ? channel % 5 : (channel-2) % 5;

    // Update the CRC, SOP and Data code
    cyrfQueueWriteReg(CYRF_CRC_SEED_LSB, crcSeed & 0xff);
    cyrfQueueWriteReg(CYRF_CRC_SEED_MSB, crcSeed >> 8);
    cyrfQueueWriteBlock(CYRF_SOP_CODE, 8, (uint8_t *)pn_codes[pnRow][sopCol]);
    cyrfQueueWriteBlock(CYRF_DATA_CODE, 16, (uint8_t *)pn_codes[pnRow][dataCol]);

    // Change channel
    dsmSetRfChannel(channel);
}

void dsmReceiverSetChannel(uint8_t chan) {
    dsmSetChannel(chan, dsmData.isDsm2, dsmData.sopCol, dsmData.dataCol, dsmData.crcSeed);
}

// Calculate the DSMX channels
void dsmGenerateChannelsDsmx(uint8_t mfgId[], uint8_t *channels) {
    int idx = 0;
    uint32_t id = ~((mfgId[0] << 24) | (mfgId[1] << 16) |
        (mfgId[2] << 8) | (mfgId[3] << 0));
    uint32_t id_tmp = id;

    // While not all channels are set
    while (idx < 23) {
        int i;
        int count_3_27 = 0, count_28_51 = 0, count_52_76 = 0;

        id_tmp = id_tmp * 0x0019660D + 0x3C6EF35F;                  // Randomization
        uint8_t next_ch = ((id_tmp >> 8) % 0x49) + 3;               // Use least-significant byte and must be larger than 3
        if (((next_ch ^ id) & 0x01 ) == 0)
            continue;

        // Go trough all already set channels
        for (i = 0; i < idx; i++) {
            // Channel is already used
            if(channels[i] == next_ch)
            break;

            // Count the channel groups
            if(channels[i] <= 27)
                count_3_27++;
            else if (channels[i] <= 51)
                count_28_51++;
            else
                count_52_76++;
        }

        // When channel is already used continue
        if (i != idx)
            continue;

        // Set the channel when channel groups aren't full
        if ((next_ch < 28 && count_3_27 < 8)                        // Channels 3-27: max 8
            || (next_ch >= 28 && next_ch < 52 && count_28_51 < 7)   // Channels 28-52: max 7
            || (next_ch >= 52 && count_52_76 < 8)) {                // Channels 52-76: max 8
                channels[idx++] = next_ch;
        }
    }
}

void dsmSetNextChannel(void) {
	if (dsmData.isDsm2) {
		dsmData.rfChannelIdx = (dsmData.rfChannelIdx+1) % 2;
		if (dsmData.rfChannelIdx == 0)
			dsmData.crcSeed = ~dsmData.crc;
		else
			dsmData.crcSeed = dsmData.crc;
	}
	else {
		dsmData.rfChannelIdx = (dsmData.rfChannelIdx+1) % 23;
		dsmData.crcSeed = ~dsmData.crcSeed;
	}

    dsmData.rfChannel = dsmData.rfChannels[dsmData.rfChannelIdx];
    dsmSetChannel(dsmData.rfChannel, dsmData.isDsm2, dsmData.sopCol, dsmData.dataCol, dsmData.crcSeed);
}

void dsmReadPacket(cyrfCallback_t *callback, uint8_t state) {
    memset(dsmData.packet, 0, sizeof(dsmData.packet));

    // read the packet
    cyrfQueueReadBlock(CYRF_RX_BUFFER, 16, dsmData.packet);

    // get RSSI
    cyrfQueueReadReg(CYRF_RSSI, &dsmData.rssi);

    // Get RX Status
    cyrfQueueReadReg(CYRF_RX_STATUS, &dsmData.rxStatus);

    // force state
    cyrfQueueWriteReg(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
    cyrfQueueWriteReg(CYRF_RX_ABORT, 0x00);

    cyrfQueueExec(callback, state);
}

cyrfCallback_t *abortCallback;
int abortCallbackParam;

void _dsmAbortReceive(int unused) {
    // have we averted a receive?
    if (!dsmData.inReceive && !(dsmData.rssi & CYRF_SOP)) {
        // force end of receive mode
        cyrfQueueWriteReg(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);

        // clear abort mode
        cyrfQueueWriteReg(CYRF_RX_ABORT, 0x00);

        // return to caller
        abortCallback(abortCallbackParam);
    }
    // else let the receive handle things
}

void dsmAbortReceive(cyrfCallback_t *callback, uint8_t param) {
    dsmData.inAbort = 1;
    abortCallback = callback;
    abortCallbackParam = param;

    if (!dsmData.inReceive) {
        // abort the receive
        cyrfQueueWriteReg(CYRF_RX_ABORT, CYRF_ABORT_EN);
        cyrfQueueReadReg(CYRF_RSSI, &dsmData.rssi);
        cyrfQueueExec(_dsmAbortReceive, 0);
        cyrfProcessStack();
    }
    // else let the receive handle things
}

void dsmSyncTimeoutCallback(int state) {
    dsmData.inAbort = 0;

    switch (state) {
    case 0:
        // abort the receive
        dsmAbortReceive(dsmSyncTimeoutCallback, 1);
        break;

    // no receive
    case 1:
    // receive failed
    case 2:
        // When we are in DSM2 mode we need to scan all channels
        if (dsmData.isDsm2) {
            // Set the next channel
            dsmReceiverSetChannel((dsmData.rfChannel + 2) % DSM_MAX_CHANNEL);
        }
        else {
            // Just set the next channel we know
            dsmSetNextChannel();
        }

        cyrfQueueBatch(cyrf_start_receive);

        // Set the new timeout
        cyrfTimeoutSetRelative(DSM_SYNC_RECV_TIME, dsmSyncTimeoutCallback, 0);
        break;

    // receive succeeded
    case 3:
        break;
    }
}

uint32_t doubleAbort;
void dsmTransferTimeoutCallback(int state) {
    if (dsmData.inAbort & !state) doubleAbort++;
    dsmData.inAbort = 0;

    switch (state) {
    case 0:
        // abort the receive
        dsmAbortReceive(dsmTransferTimeoutCallback, 1);
        break;

    // no receive
    case 1:
    // receive failed
    case 2:
        dsmData.missedPacketsTotal++;

        // too many packets missed?
        if (++dsmData.missedPackets < DSM_MAX_MISSED_PACKETS) {
            // no, switch channels and keep going
            dsmSetNextChannel();

            // Set the new timeout
            cyrfTimeoutSet(dsmChannelTime(), dsmTransferTimeoutCallback, 0);
        }
        else {
            // yes, drop back to sync mode
            dsmData.lostSyncs++;
            dsmStartTransfer();
            break;
        }

        cyrfQueueBatch(cyrf_start_receive);
        break;

    // receive succeeded
    case 3:
        break;
    }
}

void dsmTransferReceive(int state) {
    static uint8_t myIrqStatus;
    uint8_t error;

    // OR up the IRQ status
    myIrqStatus = ((!state) ? 0 : myIrqStatus) | cyrfData.rxIrqStatus;
    error = myIrqStatus & CYRF_RXE_IRQ;

    switch (state) {
    case 0:
        dsmData.inReceive = 1;

		// de-bounce
		if ((myIrqStatus & CYRF_RXC_IRQ) && !error) {
			// get the RX IRQ status
			cyrfQueueReadReg(CYRF_RX_IRQ_STATUS, &cyrfData.rxIrqStatus);
		}

		// get the number of bytes available for read
		cyrfQueueReadReg(CYRF_RX_COUNT, &dsmData.packetLen);

		cyrfQueueExec(dsmTransferReceive, 1);
		cyrfProcessStack();
		break;

	case 1:
        dsmReadPacket(dsmTransferReceive, 2);
        break;

    case 2:
        dsmData.inReceive = 0;

        if (error || !CHECK_MFG_ID(dsmData.dsmProtocol, dsmData.packet, dsmData.mfgId)) {
            if (dsmData.rxStatus & CYRF_BAD_CRC)
                dsmData.crcErrors++;

            // tell abort that we failed
            if (dsmData.inAbort)
                abortCallback(abortCallbackParam + 1);

            break;
        }

        // sync the timer
        cyrfTimeoutReset();

        // received a new frame
        memcpy(dsmData.frame, dsmData.packet, 16);
        dsmData.packetTime = cyrfData.irqTime;
        dsmData.missedPackets = 0;
        dsmData.packetsReceived++;

        // Go to the next channel
        dsmSetNextChannel();

        // Start the timer
        cyrfTimeoutSet(dsmChannelTime()+DSM_TIMEOUT, dsmTransferTimeoutCallback, 0);

        cyrfQueueBatch(cyrf_start_receive);

        // tell abort that we succeeded
        if (dsmData.inAbort)
            abortCallback(abortCallbackParam + 2);

        break;
    }
}

void dsmSyncBReceive(int state) {
    static uint8_t myIrqStatus;
    uint8_t error;

    // OR up the IRQ statuses
    myIrqStatus = ((!state) ? 0 : myIrqStatus) | cyrfData.rxIrqStatus;
    error = myIrqStatus & CYRF_RXE_IRQ;

    switch (state) {
    case 0:
        dsmData.inReceive = 1;

		// de-bounce
		if ((myIrqStatus & CYRF_RXC_IRQ) && !error) {
			// get the RX IRQ status
			cyrfQueueReadReg(CYRF_RX_IRQ_STATUS, &cyrfData.rxIrqStatus);
		}

		// get the number of bytes available for read
		cyrfQueueReadReg(CYRF_RX_COUNT, &dsmData.packetLen);

		cyrfQueueExec(dsmSyncBReceive, 1);
		cyrfProcessStack();
		break;

	case 1:
        dsmReadPacket(dsmSyncBReceive, 2);
        break;

    case 2:
        dsmData.inReceive = 0;

        if (error || !CHECK_MFG_ID(dsmData.dsmProtocol, dsmData.packet, dsmData.mfgId) || dsmData.rfChannel == dsmData.rfChannels[0]) {
            // tell abort that we failed
            if (dsmData.inAbort)
                abortCallback(abortCallbackParam + 1);
            break;
        }

        // sync the timer
        cyrfTimeoutReset();

        dsmData.rfChannels[1] = dsmData.rfChannel;

        // Set the next channel and start receiving
        dsmData.bound = 1;
        dsmData.state = DSM_RECEIVER_RECV;
        dsmData.missedPackets = 0;
		dsmData.missedPacketsTotal = 0;
		dsmData.rfChannelIdx = 1;
        dsmSetNextChannel();

        cyrfQueueBatch(cyrf_start_receive);

        cyrfSetIsrCallback(dsmTransferReceive, 0);

        // Start the timer
        cyrfTimeoutSet(dsmChannelTime()+DSM_TIMEOUT, dsmTransferTimeoutCallback, 0);

        // tell abort that we succeeded
        if (dsmData.inAbort)
            abortCallback(abortCallbackParam + 2);

        break;
    }
}

void dsmSyncAReceive(int state) {
    static uint8_t myIrqStatus;
    uint8_t error;

    // OR up the IRQ statuses
    myIrqStatus = ((!state) ? 0 : myIrqStatus) | cyrfData.rxIrqStatus;
    error = myIrqStatus & CYRF_RXE_IRQ;

    switch (state) {
    case 0:
        dsmData.inReceive = 1;

		// de-bounce
		if ((myIrqStatus & CYRF_RXC_IRQ) && !error) {
			// get the RX IRQ status
			cyrfQueueReadReg(CYRF_RX_IRQ_STATUS, &cyrfData.rxIrqStatus);
		}

		// get the number of bytes available for read
		cyrfQueueReadReg(CYRF_RX_COUNT, &dsmData.packetLen);

		cyrfQueueExec(dsmSyncAReceive, 1);
		cyrfProcessStack();
		break;

	case 1:
        dsmReadPacket(dsmSyncAReceive, 2);
        break;

    case 2:
        dsmData.inReceive = 0;

        if (error || !CHECK_MFG_ID(dsmData.dsmProtocol, dsmData.packet, dsmData.mfgId)) {
            // tell abort that we failed
            if (dsmData.inAbort)
                abortCallback(abortCallbackParam + 1);
            break;
        }

        // sync the timer
        cyrfTimeoutReset();

		// Check whether it is DSM2 or DSMX
        if (dsmData.isDsm2) {
            dsmData.rfChannels[0] = dsmData.rfChannel;

            dsmData.state = DSM_RECEIVER_SYNC_B;
            cyrfSetIsrCallback(dsmSyncBReceive, 0);
			dsmData.crcSeed = dsmData.crc;

            // scan next channel
            dsmReceiverSetChannel((dsmData.rfChannel + 2) % DSM_MAX_CHANNEL);

			// Start the timer
            cyrfTimeoutSetRelative(DSM_SYNC_RECV_TIME, dsmSyncTimeoutCallback, 0);
		}
        else {
            // When it is DSMX we can stop because we know all the channels
            dsmData.state = DSM_RECEIVER_RECV;
			dsmData.bound = 1;
            dsmData.missedPackets = 0;
			dsmData.missedPacketsTotal = 0;
            cyrfSetIsrCallback(dsmTransferReceive, 0);

            // Set the next channel and start receiving
            dsmSetNextChannel();

            cyrfTimeoutSet(dsmChannelTime()+DSM_TIMEOUT, dsmTransferTimeoutCallback, 0);
		}

        cyrfQueueBatch(cyrf_start_receive);

        // tell abort that we succeeded
        if (dsmData.inAbort)
            abortCallback(abortCallbackParam + 2);

        break;
    }
}

void dsmStartTransfer(void) {
    dsmData.state = DSM_RECEIVER_SYNC_A;
    cyrfData.running = 1;

    cyrfSetIsrCallback(0, 0);

    cyrfData.isTx = 0;
    dsmData.inAbort = 0;
    dsmData.inReceive = 0;

#ifdef DSM_RESET_BEFORE_SCAN
    // initial configuration
    cyrfQueueBatch(cyrf_config);
#else
    // abort the receive
    cyrfQueueWriteReg(CYRF_RX_ABORT, CYRF_ABORT_EN);
    cyrfQueueWriteReg(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
    cyrfQueueWriteReg(CYRF_RX_ABORT, 0x00);
#endif

    // Set the CYRF configuration
    cyrfQueueBatch(cyrf_transfer_config);

    // Calculate the CRC seed, SOP column and Data column
    dsmData.crc = (dsmData.mfgId[0] << 8) + dsmData.mfgId[1];
	dsmData.crcSeed = ~dsmData.crc;
    dsmData.sopCol = (dsmData.mfgId[0] + dsmData.mfgId[1] + dsmData.mfgId[2] + 2) & 0x07;
    dsmData.dataCol = 7 - dsmData.sopCol;

    // When DSMX, generate channels and set channel
    if (!dsmData.isDsm2) {
        dsmGenerateChannelsDsmx(dsmData.mfgId, dsmData.rfChannels);
        dsmData.rfChannelIdx = 22;
        dsmSetNextChannel();
    }
    else {
        dsmReceiverSetChannel(0);
    }

    // Start receiving
    cyrfQueueBatch(cyrf_start_receive);

    cyrfProcessStack();

    // Enable the timer
    if (dsmData.isDsm2)
        cyrfTimeoutSetRelative(DSM_SYNC_RECV_TIME, dsmSyncTimeoutCallback, 0);
    else
        cyrfTimeoutSetRelative(DSM_SYNC_FRECV_TIME, dsmSyncTimeoutCallback, 0); // Because we know for sure where DSMX starts we can wait the full bind

    cyrfSetIsrCallback(dsmSyncAReceive, 0);
}

void dsmBindAck(int state) {
    uint8_t *packet = dsmData.packet;
    int i;

    if (state == 0) {
        uint16_t ckSum = 0x170;

        // build ack packet
        packet[4] = packet[10];
        packet[5] = dsmData.dsmNumChannels;
        packet[6] = dsmData.dsmProtocol;
        packet[7] = packet[13];

        for (i = 0; i < 8; i++)
            ckSum += packet[i];

        packet[8] = ckSum >> 8;
        packet[9] = ckSum & 0xff;

        cyrfData.isTx = 1;

        cyrfQueueWriteBlock(CYRF_DATA_CODE, 8, (uint8_t *)pn_bind);
        cyrfQueueWriteReg(CYRF_TX_CTRL, CYRF_TX_CLR);
        cyrfQueueWriteReg(CYRF_TX_LENGTH, 10);
        cyrfQueueWriteBlock(CYRF_TX_BUFFER, 10, packet);
    }

    // keep sending ACK packet
    if (state < DSM_BIND_ACK_NUM) {
        cyrfQueueWriteReg(CYRF_TX_CTRL, CYRF_TX_GO);
        cyrfProcessStack();

        cyrfTimeoutSetRelative(10000, dsmBindAck, ++state);
    }
    // finished
    else {
        dsmStartTransfer();
    }
}

void dsmBindTimeoutCallback(int state) {
    dsmData.inAbort = 0;

    switch (state) {
    case 0:
        // abort the receive
        dsmAbortReceive(dsmBindTimeoutCallback, 1);
        break;

    // no receive
    case 1:
    // receive failed
    case 2:
        if (dsmData.bound) {
            cyrfSetIsrCallback(0, 0);
            dsmBindAck(0);
        }
        else {
            // Set the next bind channel
            dsmSetBindChannel();

            // Start receiving
            cyrfQueueBatch(cyrf_start_receive);

            // Set the new timeout
            cyrfTimeoutSetRelative(DSM_BIND_RECV_TIME, dsmBindTimeoutCallback, 0);

            cyrfSetIsrCallback(dsmBindReceive, 0);
        }
        break;

    // receive succeeded
    case 3:
        break;
    }
}

void dsmBindReceive(int state) {
    static uint8_t myIrqStatus;
    uint8_t *packet = dsmData.packet;
    uint8_t error;
    uint16_t bind_sum;
    int i;

    // or up the irq status
    myIrqStatus = ((!state) ? 0 : myIrqStatus) | cyrfData.rxIrqStatus;
    error = myIrqStatus & CYRF_RXE_IRQ;

    switch (state) {
    case 0:
        dsmData.inReceive = 1;

		// de-bounce
		if ((myIrqStatus & CYRF_RXC_IRQ) && !error) {
			// get the RX IRQ status
			cyrfQueueReadReg(CYRF_RX_IRQ_STATUS, &cyrfData.rxIrqStatus);
		}

		// get the number of bytes available for read
		cyrfQueueReadReg(CYRF_RX_COUNT, &dsmData.packetLen);

		cyrfQueueExec(dsmBindReceive, 1);
		cyrfProcessStack();
		break;

    case 1:
        dsmReadPacket(dsmBindReceive, 2);
        break;

    case 2:
        dsmData.inReceive = 0;

        // Check if the MFG id is exactly the same twice
        if (dsmData.packetLen < 16 || packet[0] != packet[4] || packet[1] != packet[5]
                || packet[2] != packet[6] || packet[3] != packet[7]) {

            goto bindRcvEnd;
        }

        // Calculate the first sum
        bind_sum = 384 - 0x10;
        for (i = 0; i < 8; i++)
            bind_sum += packet[i];

        // Check the first sum
        if (packet[8] != bind_sum >> 8 || packet[9] != (bind_sum & 0xFF))
            goto bindRcvEnd;

        // Calculate second sum
        for (i = 8; i < 14; i++)
            bind_sum += packet[i];

        // Check the second sum
        if (packet[14] != bind_sum >> 8 || packet[15] != (bind_sum & 0xFF))
            goto bindRcvEnd;

        // stop the timer
        cyrfTimeoutReset();

        // Update the mfg id, number of channels and protocol
        dsmData.mfgId[0] = ~packet[0];
        dsmData.mfgId[1] = ~packet[1];
        dsmData.mfgId[2] = ~packet[2];
        dsmData.mfgId[3] = ~packet[3];
        dsmData.dsmNumChannels = packet[11];
        dsmData.dsmProtocol = packet[12];
		dsmData.isDsm2 = IS_DSM2(dsmData.dsmProtocol);
        dsmData.bound++;// = 1;
        dsmData.newBind = 1;

        cyrfQueueBatch(cyrf_start_receive);

        // wait until the incoming bind packets have stopped
        cyrfTimeoutSetRelative(60000, dsmBindTimeoutCallback, 0);

        // tell abort that we succeeded
        if (dsmData.inAbort)
            abortCallback(abortCallbackParam + 2);
        break;

        bindRcvEnd:

        // Start receiving
        cyrfQueueBatch(cyrf_start_receive);

        // if the RSSI is high enough, temporarily consider this the binding channel
        if ((dsmData.rssi & 0b11111) >= DSM_BIND_RSSI)
            // listen here for a while
            cyrfTimeoutSetRelative(40000, dsmBindTimeoutCallback, 0);
        else
            // normal timeout
            cyrfTimeoutSetRelative(DSM_BIND_RECV_TIME, dsmBindTimeoutCallback, 0);

        // tell abort that we failed
        if (dsmData.inAbort)
            abortCallback(abortCallbackParam + 1);
    }
}

void dsmStartBind(void) {
    dsmData.state = DSM_RECEIVER_BIND;
    cyrfData.running = 1;

    // Stop the timer
    cyrfTimeoutCancel();

    cyrfSetIsrCallback(0, 0);

    cyrfData.isTx = 0;
    dsmData.bound = 0;
    dsmData.inAbort = 0;
    dsmData.inReceive = 0;

    // abort the receive
    cyrfQueueWriteReg(CYRF_RX_ABORT, CYRF_ABORT_EN);
    cyrfQueueWriteReg(CYRF_XACT_CFG, CYRF_MODE_SYNTH_RX | CYRF_FRC_END);
    cyrfQueueWriteReg(CYRF_RX_ABORT, 0x00);

    // Set the CYRF configuration
    cyrfQueueBatch(cyrf_bind_config);

    // Set the CYRF data code
	cyrfQueueWriteBlock(CYRF_DATA_CODE, 8, (uint8_t *)pn_codes[0][8]);
	cyrfQueueWriteBlock(CYRF_DATA_CODE, 8, (uint8_t *)pn_bind);

    // Set the initial bind channel
    dsmSetRfChannel(1);

    // Start receiving
    cyrfQueueBatch(cyrf_start_receive);

    cyrfProcessStack();

    // Enable the timer
    cyrfTimeoutSetRelative(DSM_BIND_RECV_TIME, dsmBindTimeoutCallback, 0);
}

void dsmStop(void) {
    dsmData.state = DSM_RECEIVER_STOP;
    cyrfData.running = 0;

    // Stop the timer
    cyrfTimeoutCancel();
}

void dsmDecode(radioInstance_t *r) {
    uint8_t resolution = (dsmData.dsmProtocol & 0x10)>>4;
    uint8_t *chan;
    int addr;
    int val;
    int i;

    for (i = 0; i < 7; i++) {
        chan = &dsmData.frame[i*2+2];

        if (chan[0] != 0xff && chan[1] != 0xff) {
            if (resolution) {
                // 11bit
                addr = (chan[0]>>3) & 0x0f;
                val = ((chan[0] & 0x07)<<8) | chan[1];
            }
            else {
                // 10bit
                addr = (chan[0]>>2) & 0x0f;
                val = (((chan[0] & 0x03)<<8) | chan[1])<<1;
            }

            // throttle
            if (addr == (int)p[RADIO_THRO_CH])
                val -= 338;
            else
                val -= 1024;

            r->channels[addr] = val;
        }
    }

    r->errorCount = dsmData.rssi & 0x1f;   // actually RSSI (5 bits)
}
#endif

uint8_t dsmReceive(radioInstance_t *r) {
#ifdef CYRF_SPI
    // announce state changes
    if (dsmData.lastState != dsmData.state) {
        dsmData.lastState = dsmData.state;

        switch (dsmData.state) {
            case DSM_RECEIVER_STOP:
                AQ_NOTICE("CYRF: stopped\n");
                break;

            case DSM_RECEIVER_BIND:
                AQ_NOTICE("CYRF: binding\n");
                break;

            case DSM_RECEIVER_SYNC_A:
                AQ_NOTICE("CYRF: sync A\n");
                break;

            case DSM_RECEIVER_SYNC_B:
                AQ_NOTICE("CYRF: sync B\n");
                break;

            case DSM_RECEIVER_RECV:
                AQ_NOTICE("CYRF: receiving\n");
                break;
        }
    }

    r->binding = (dsmData.state == DSM_RECEIVER_BIND && !dsmData.bound);

    // check for new binding
    if (dsmData.newBind) {
        configToken_t t;

        // store it to flash
        t.key = *(uint32_t *)"CYRF";
        t.data[0] = dsmData.mfgId[0];
        t.data[1] = dsmData.mfgId[1];
        t.data[2] = dsmData.mfgId[2];
        t.data[3] = dsmData.mfgId[3];

        t.data[4] = dsmData.dsmProtocol;
        t.data[5] = dsmData.dsmNumChannels;

        configTokenStore(&t);

        dsmData.newBind = 0;
    }
    // no TX signal, go into bind mode after 15s
    else if (cyrfData.initialized && !dsmData.bound && dsmData.state != DSM_RECEIVER_BIND && timerMicros() > 15000000) {
        dsmStartBind();
    }

    // valid packet received since last check?
    if (dsmData.state == DSM_RECEIVER_RECV && dsmData.packetTime && dsmData.lastPacketTime != dsmData.packetTime) {
        dsmData.lastPacketTime = dsmData.packetTime;

        dsmDecode(r);

        return 1;
    }
    else {
        return 0;
    }
#else
    return 0;
#endif
}

uint8_t dsmInit(void) {
    configToken_t *t;
    uint8_t ret = 0;

#ifdef CYRF_SPI
    ret = cyrfInit();

    if (ret) {
        // initial configuration
        cyrfQueueBatch(cyrf_config);

        t = configTokenGet(*(uint32_t *)"CYRF");

        if (1 && t) {
			// load binding data from flash
            dsmData.mfgId[0] = t->data[0];
            dsmData.mfgId[1] = t->data[1];
            dsmData.mfgId[2] = t->data[2];
            dsmData.mfgId[3] = t->data[3];

            dsmData.dsmProtocol = t->data[4];
            dsmData.dsmNumChannels = t->data[5];
			dsmData.isDsm2 = IS_DSM2(dsmData.dsmProtocol);

            dsmStartTransfer();
        }
        else {
            dsmStartBind();
        }

        AQ_NOTICE("DSM: Found CYRF6963 chip\n");
    }
    else {
        AQ_NOTICE("DSM: Could not find CYRF6963 chip\n");
    }
#else
        AQ_NOTICE("DSM: CYRF6936 not installed\n");
#endif


    return ret;
}

