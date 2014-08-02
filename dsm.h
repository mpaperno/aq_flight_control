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

#include <stdint.h>
#include "radio.h"

#define DSM_BIND_RECV_TIME              10000       // Time before timeout when receiving bind packets
#define DSM_SYNC_RECV_TIME              20000       // Time before timeout when trying to sync
#define DSM_SYNC_FRECV_TIME             65000       // Time before timeout when trying to sync first packet of DSMX (bigger then bind sending)
#define DSM_TIMEOUT						500			// grace period before timeout

#define DSM_CHA_TIME					7000
#define DSM_CHB_TIME					4000
#define DSM_BIND_SEND_TIME              10000       // Time between sending bind packets
#define DSM_SEND_TIME                   22000       // Time between sending both Channel A and Channel B

// The maximum channel number for DSM2 and DSMX
#define DSM_MAX_CHANNEL                 0x4F        // Maximum channel number used for DSM2 and DSMX

#define DSM_BIND_RSSI                   5           // minimum RSSI to consider a bind channel
#define DSM_MAX_MISSED_PACKETS          100         // after this, the sync is considered lost
#define DSM_BIND_ACK_NUM                200         // number of bin ACK packets to send

#define IS_DSM2(x)			(x == DSM_DSM2_1 || x == DSM_DSM2_2 || x == DSM_DSM2_JR)
#define IS_DSMX(x)			(!IS_DSM2(x))
#define CHECK_MFG_ID(protocol, packet, id) ((IS_DSM2(protocol) && packet[0] == (~id[2]&0xFF) && packet[1] == (~id[3]&0xFF)) || \
            (IS_DSMX(protocol) && packet[0] == id[2] && packet[1] == id[3]))

//#define DSM_RESET_BEFORE_SCAN

// The different kind of protocol definitions DSM2 and DSMX with 1 and 2 packets of data
enum dsm_protocol {
    DSM_DSM2_1                          = 0x01,     // The original DSM2 protocol with 1 packet of data
    DSM_DSM2_2                          = 0x02,     // The original DSM2 protocol with 2 packets of data
    DSM_DSM2_JR                         = 0x12,     // JR 12X2.4 DSM2 with 2 packets of data
    DSM_DSMX_1                          = 0xA2,     // The original DSMX protocol with 1 packet of data
    DSM_DSMX_2                          = 0xB2,     // The original DSMX protocol with 2 packets of data
};

enum dsm_receiver_state {
    DSM_RECEIVER_STOP					= 0x0,      // The receiver is stopped
    DSM_RECEIVER_BIND					= 0x1,      // The receiver is binding
    DSM_RECEIVER_SYNC_A					= 0x2,      // The receiver is syncing channel A
    DSM_RECEIVER_SYNC_B					= 0x3,      // The receiver is syncing channel B
    DSM_RECEIVER_RECV					= 0x4,      // The receiver is receiving
};

typedef struct {
    uint8_t packet[16];
    uint8_t frame[16];
    uint8_t mfgId[4];
    uint8_t state;
    uint8_t lastState;
    uint8_t rssi;
    uint8_t packetLen;
    uint8_t rxStatus;
    uint8_t rfChannel;
    uint8_t dsmNumChannels;
    uint8_t dsmProtocol;
	uint8_t isDsm2;
    uint8_t bound;
    uint8_t newBind;
    uint8_t rfChannels[23];
    uint8_t rfChannelIdx;
    uint8_t inAbort;
    uint8_t inReceive;
    uint16_t crc;
    uint16_t crcSeed;
    uint8_t sopCol;
    uint8_t dataCol;
    uint8_t missedPackets;
    uint32_t lostSyncs;
    uint32_t missedPacketsTotal;
    uint32_t packetsReceived;
    uint32_t crcErrors;
    uint32_t packetTime;
    uint32_t lastPacketTime;
} dsmStruct_t;

extern uint8_t dsmInit(void);
extern uint8_t dsmReceive(radioInstance_t *r);
