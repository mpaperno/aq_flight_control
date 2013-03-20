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
#include "ublox.h"
#include "gps.h"
#include "aq_timer.h"
#include "downlink.h"
#include "imu.h"
#include "config.h"
#include "util.h"
#include "rtc.h"
#include "filer.h"
#include "supervisor.h"
#include <CoOS.h>
#include <string.h>

ubloxStruct_t ubloxData __attribute__((section(".ccm")));

void ubloxWriteU1(unsigned char c) {
    serialWrite(gpsData.gpsPort, c);
    ubloxData.ubloxTxCK_A += c;
    ubloxData.ubloxTxCK_B += ubloxData.ubloxTxCK_A;
}

void ubloxWriteI1(signed char c) {
    serialWrite(gpsData.gpsPort, (unsigned char)c);
    ubloxData.ubloxTxCK_A += c;
    ubloxData.ubloxTxCK_B += ubloxData.ubloxTxCK_A;
}

void ubloxWriteU2(unsigned short int x) {
    ubloxWriteU1(x);
    ubloxWriteU1(x>>8);
}

void ubloxWriteI2(signed short int x) {
    ubloxWriteU1(x);
    ubloxWriteU1(x>>8);
}

void ubloxWriteU4(unsigned long int x) {
    ubloxWriteU1(x);
    ubloxWriteU1(x>>8);
    ubloxWriteU1(x>>16);
    ubloxWriteU1(x>>24);
}

void ubloxWriteI4(signed long int x) {
    ubloxWriteU1(x);
    ubloxWriteU1(x>>8);
    ubloxWriteU1(x>>16);
    ubloxWriteU1(x>>24);
}

void ubloxSendPreamble(void) {
    ubloxWriteU1(0xb5);	// u
    ubloxWriteU1(0x62);	// b

    ubloxData.ubloxTxCK_A = 0;
    ubloxData.ubloxTxCK_B = 0;
}

void ubloxEnableMessage(unsigned char c, unsigned char i, unsigned char rate) {
    ubloxSendPreamble();

    ubloxWriteU1(0x06);	// CFG
    ubloxWriteU1(0x01);	// MSG
    ubloxWriteU1(0x03);	// length lsb
    ubloxWriteU1(0x00);	// length msb
    ubloxWriteU1(c);	// class
    ubloxWriteU1(i);	// id
    ubloxWriteU1(rate);	// rate
    serialWrite(gpsData.gpsPort, ubloxData.ubloxTxCK_A);
    serialWrite(gpsData.gpsPort, ubloxData.ubloxTxCK_B);
    yield(50);
}

void ubloxSetRate(unsigned short int ms) {
    ubloxSendPreamble();

    ubloxWriteU1(0x06);	// CFG
    ubloxWriteU1(0x08);	// RTATE
    ubloxWriteU1(0x06);	// length lsb
    ubloxWriteU1(0x00);	// length msb
    ubloxWriteU2(ms);	// rate
    ubloxWriteU2(0x01);	// cycles
    ubloxWriteU2(0x01);	// timeRef	0 == UTC, 1 == GPS time
    serialWrite(gpsData.gpsPort, ubloxData.ubloxTxCK_A);
    serialWrite(gpsData.gpsPort, ubloxData.ubloxTxCK_B);
    yield(50);
}

void ubloxSetMode(void) {
    int i;

    ubloxSendPreamble();

    ubloxWriteU1(0x06);		// CFG
    ubloxWriteU1(0x24);		// NAV5
    ubloxWriteU1(0x24);		// length lsb
    ubloxWriteU1(0x00);		// length msb
    ubloxWriteU1(0b0000101);	// mask LSB (fixMode, dyn)
    ubloxWriteU1(0x00);		// mask MSB (reserved)
    ubloxWriteU1(0x06);		// dynModel (6 == airborne < 1g, 8 == airborne < 4g)
    ubloxWriteU1(0x02);		// fixMode (2 == 3D only)

    // the rest of the packet is ignored due to the above mask
    for (i = 0; i < 32; i++)
	ubloxWriteU1(0x00);

    serialWrite(gpsData.gpsPort, ubloxData.ubloxTxCK_A);
    serialWrite(gpsData.gpsPort, ubloxData.ubloxTxCK_B);
    yield(50);
}

void ubloxSetTimepulse(void) {
    ubloxSendPreamble();

    ubloxWriteU1(0x06);		// CFG
    ubloxWriteU1(0x07);		// TP

    ubloxWriteU1(0x14);		// length lsb
    ubloxWriteU1(0x00);		// length msb

    ubloxWriteU4(1000000);	// interval (us)
    ubloxWriteU4(100000);	// length (us)
#ifdef GPS_LATENCY
    ubloxWriteI1(0x00);		// config setting (0 == off)
#else
    ubloxWriteI1(0x01);		// config setting (1 == +polarity)
#endif
    ubloxWriteU1(0x01);		// alignment reference time (GPS)
    ubloxWriteU1(0x00);		// bitmask (syncmode 0)
    ubloxWriteU1(0x00);		// reserved
    ubloxWriteI2(0x00);		// antenna delay
    ubloxWriteI2(0x00);		// rf group delay
    ubloxWriteI4(0x00);		// user delay

    serialWrite(gpsData.gpsPort, ubloxData.ubloxTxCK_A);
    serialWrite(gpsData.gpsPort, ubloxData.ubloxTxCK_B);
    yield(50);
}

void ubloxSendSetup(void) {
    yield(50);
    ubloxSetTimepulse();
    ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_VALNED, 1);	// NAV VALNED
    ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_POSLLH, 1);	// NAV POSLLH
    ubloxEnableMessage(UBLOX_TIM_CLASS, UBLOX_TP, 1);		// TIM TP
    ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_DOP, 5);		// NAV DOP
    ubloxEnableMessage(UBLOX_AID_CLASS, UBLOX_AID_REQ, 1);	// AID REQ
    ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_TIMEUTC, 5);    // TIME UTC
#ifdef GPS_DO_RTK
    ubloxEnableMessage(UBLOX_RXM_CLASS, UBLOX_RAW, 1);		// RXM RAW
    ubloxEnableMessage(UBLOX_RXM_CLASS, UBLOX_SFRB, 1);		// RXM SFRB
#endif
    ubloxSetMode();						// 3D, airborne
    ubloxSetRate((unsigned short int)p[GPS_RATE]);	// ms
}

void ubloxInit(void) {
    memset((void *)&ubloxData, 0, sizeof(ubloxData));

    ubloxSendSetup();
    ubloxData.state = UBLOX_WAIT_SYNC1;
}

void ubloxInitGps(void) {
    serialPrint(gpsData.gpsPort, "$PUBX,41,1,0007,0001,230400,0*18\n");
    yield(200);
}

unsigned char ubloxPublish(void) {
    unsigned char ret = 0;

    // don't allow preemption
    CoSetPriority(gpsData.gpsTask, 1);

    if (ubloxData.class == UBLOX_NAV_CLASS && ubloxData.id == UBLOX_POSLLH) {
	// work around uBlox's inability to give new data on each report sometimes
	if (ubloxData.lastLat != ubloxData.payload.posllh.lat || ubloxData.lastLon != ubloxData.payload.posllh.lon) {
	    ubloxData.lastLat = ubloxData.payload.posllh.lat;
	    ubloxData.lastLon = ubloxData.payload.posllh.lon;

	    gpsData.iTOW = ubloxData.payload.posllh.iTOW;
	    gpsData.lat = (double)ubloxData.payload.posllh.lat * (double)1e-7;
	    gpsData.lon = (double)ubloxData.payload.posllh.lon * (double)1e-7;
	    gpsData.height = ubloxData.payload.posllh.hMSL * 0.001f;    // mm => m
	    gpsData.hAcc = ubloxData.payload.posllh.hAcc * 0.001f;	    // mm => m
	    gpsData.vAcc = ubloxData.payload.posllh.vAcc * 0.001f;	    // mm => m

#ifdef GPS_LATENCY
	    gpsData.lastPosUpdate = timerMicros() - GPS_LATENCY;
#else
	    gpsData.lastPosUpdate = gpsData.lastTimepulse + (ubloxData.payload.posllh.iTOW - gpsData.TPtowMS) * 1000;
#endif
	    // position update
	    ret = 1;
	}
    }
    else if (ubloxData.class == UBLOX_NAV_CLASS && ubloxData.id == UBLOX_VALNED) {
	gpsData.iTOW = ubloxData.payload.valned.iTOW;
	gpsData.velN = ubloxData.payload.valned.velN * 0.01f;	    // cm => m
	gpsData.velE = ubloxData.payload.valned.velE * 0.01f;	    // cm => m
	gpsData.velD = ubloxData.payload.valned.velD * 0.01f;	    // cm => m
	gpsData.speed = ubloxData.payload.valned.gSpeed * 0.01f;    // cm/s => m/s
	gpsData.heading = ubloxData.payload.valned.heading * 1e-5f;
	gpsData.sAcc = ubloxData.payload.valned.sAcc * 0.01f;	    // cm/s => m/s
	gpsData.cAcc = ubloxData.payload.valned.cAcc * 1e-5f;

#ifdef GPS_LATENCY
	gpsData.lastVelUpdate = timerMicros() - GPS_LATENCY;
#else
	gpsData.lastVelUpdate = gpsData.lastTimepulse + (ubloxData.payload.valned.iTOW - gpsData.TPtowMS) * 1000;
#endif

	// velocity update
	ret = 2;
    }
    else if (ubloxData.class == UBLOX_TIM_CLASS && ubloxData.id == UBLOX_TP) {
	gpsData.lastReceivedTPtowMS = ubloxData.payload.tp.towMS;
    }
    else if (ubloxData.class == UBLOX_NAV_CLASS && ubloxData.id == UBLOX_DOP) {
	gpsData.pDOP = ubloxData.payload.dop.pDOP * 0.01f;
	gpsData.hDOP = ubloxData.payload.dop.hDOP * 0.01f;
	gpsData.vDOP = ubloxData.payload.dop.vDOP * 0.01f;
	gpsData.tDOP = ubloxData.payload.dop.tDOP * 0.01f;
	gpsData.nDOP = ubloxData.payload.dop.nDOP * 0.01f;
	gpsData.eDOP = ubloxData.payload.dop.eDOP * 0.01f;
	gpsData.gDOP = ubloxData.payload.dop.gDOP * 0.01f;
    }
    else if ( ubloxData.class == UBLOX_NAV_CLASS && ubloxData.id == UBLOX_TIMEUTC && (ubloxData.payload.timeutc.valid & 0b100) ) {

	// if setting the RTC succeeds, disable the TIMEUTC message
	if (rtcSetDataTime(ubloxData.payload.timeutc.year, ubloxData.payload.timeutc.month, ubloxData.payload.timeutc.day,
		ubloxData.payload.timeutc.hour, ubloxData.payload.timeutc.min, ubloxData.payload.timeutc.sec))
	    ubloxEnableMessage(UBLOX_NAV_CLASS, UBLOX_TIMEUTC, 0);
    }

    gpsData.lastMessage = IMU_LASTUPD;

    CoSetPriority(gpsData.gpsTask, GPS_PRIORITY);

#ifndef USE_MAVLINK
    if (0) {
	int i;

	// grab port lock
	CoEnterMutexSection(downlinkData.serialPortMutex);

	downlinkSendString("AqG"); // gps header

	downlinkResetChecksum();

	i = ubloxData.length + 8;
	downlinkSendChar(i & 0xff);
	downlinkSendChar((i & 0xff00) >> 8);

	downlinkSendChar(UBLOX_SYNC1);
	downlinkSendChar(UBLOX_SYNC2);
	downlinkSendChar(ubloxData.class);
	downlinkSendChar(ubloxData.id);
	downlinkSendChar(ubloxData.length & 0xff);
	downlinkSendChar((ubloxData.length & 0xff00) >> 8);

	for (i = 0; i < ubloxData.length; i++)
	    downlinkSendChar(*((char *)&ubloxData.payload + i));

	downlinkSendChar(ubloxData.ubloxRxCK_A);
	downlinkSendChar(ubloxData.ubloxRxCK_B);

	downlinkSendChecksum();

	// release serial port
	CoLeaveMutexSection(downlinkData.serialPortMutex);
    }
#endif

    return ret;
}

void ubloxChecksum(unsigned char c) {
    ubloxData.ubloxRxCK_A += c;
    ubloxData.ubloxRxCK_B += ubloxData.ubloxRxCK_A;
}

unsigned char ubloxCharIn(unsigned char c) {
    switch (ubloxData.state) {
    case UBLOX_WAIT_SYNC1:
	if (c == UBLOX_SYNC1)
	    ubloxData.state = UBLOX_WAIT_SYNC2;
	return 3;	// lost sync
	break;

    case UBLOX_WAIT_SYNC2:
	if (c == UBLOX_SYNC2)
	    ubloxData.state = UBLOX_WAIT_CLASS;
	else
	    ubloxData.state = UBLOX_WAIT_SYNC1;
	return 3;	// lost sync
	break;

    case UBLOX_WAIT_CLASS:
	ubloxData.class = c;
	ubloxData.ubloxRxCK_A = ubloxData.ubloxRxCK_B = 0;
	ubloxChecksum(c);
	ubloxData.state = UBLOX_WAIT_ID;
	break;

    case UBLOX_WAIT_ID:
	ubloxData.id = c;
	ubloxChecksum(c);
	ubloxData.state = UBLOX_WAIT_LEN1;
	break;

    case UBLOX_WAIT_LEN1:
	ubloxData.length = c;
	ubloxChecksum(c);
	ubloxData.state	= UBLOX_WAIT_LEN2;
	break;

    case UBLOX_WAIT_LEN2:
	ubloxData.length += (c << 8);
	ubloxChecksum(c);
        if (ubloxData.length >= (UBLOX_MAX_PAYLOAD-1)) { // avoid length to exceed payload size (just in case of syn loss)
            ubloxData.length = 0;
            ubloxData.state = UBLOX_WAIT_SYNC1;
        } else if (ubloxData.length > 0) {
	    ubloxData.count = 0;
	    ubloxData.state = UBLOX_PAYLOAD;
	}
	else
	    ubloxData.state = UBLOX_CHECK1;
	break;

    case UBLOX_PAYLOAD:
	*((char *)(&ubloxData.payload) + ubloxData.count) = c;
	if (++ubloxData.count == ubloxData.length)
	    ubloxData.state = UBLOX_CHECK1;
	ubloxChecksum(c);
	break;

    case UBLOX_CHECK1:
	if (c == ubloxData.ubloxRxCK_A)
	    ubloxData.state = UBLOX_CHECK2;
	else
	    ubloxData.state = UBLOX_WAIT_SYNC1;
	break;

    case UBLOX_CHECK2:
	ubloxData.state = UBLOX_WAIT_SYNC1;
	if (c == ubloxData.ubloxRxCK_B)
	    return ubloxPublish();
	break;

    default:
	return 3;	// lost sync
	break;
    }

    return 0;
}
