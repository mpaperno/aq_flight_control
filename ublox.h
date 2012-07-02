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

#ifndef ublox_h
#define ublox_h

#define UBLOX_SYNC1	    0xB5
#define UBLOX_SYNC2	    0x62

#define UBLOX_NAV_CLASS	    0x01
#define UBLOX_RXM_CLASS	    0x02
#define UBLOX_AID_CLASS	    0x0b
#define UBLOX_TIM_CLASS	    0x0d

#define UBLOX_WAIT_SYNC1    0x00
#define UBLOX_WAIT_SYNC2    0x01
#define UBLOX_WAIT_CLASS    0x02
#define UBLOX_WAIT_ID	    0x03
#define UBLOX_WAIT_LEN1	    0x04
#define UBLOX_WAIT_LEN2	    0x05
#define UBLOX_PAYLOAD	    0x06
#define UBLOX_CHECK1	    0x07
#define UBLOX_CHECK2	    0x08

#define UBLOX_POSLLH	    0x02
#define UBLOX_DOP	    0x04
#define UBLOX_VALNED	    0x12
#define UBLOX_TP	    0x01
#define UBLOX_TIMEUTC	    0x21
#define UBLOX_AID_REQ	    0x00
#define UBLOX_RAW	    0x10
#define UBLOX_SFRB	    0x11

#define UBLOX_MAX_PAYLOAD   384

// Geodetic Position Solution
typedef struct {
    unsigned long iTOW;	    // GPS Millisecond Time of Week (ms)
    signed long lon;	    // Longitude (deg * 1e-7)
    signed long lat;	    // Latitude (deg * 1e-7)
    signed long height;	    // Height above Ellipsoid (mm)
    signed long hMSL;	    // Height above mean sea level (mm)
    unsigned long hAcc;	    // Horizontal Accuracy Estimate (mm)
    unsigned long vAcc;	    // Vertical Accuracy Estimate (mm)
} __attribute__((packed)) ubloxStructPOSLLH_t;

// Dilution of precision
typedef struct {
    unsigned long iTOW;	    // ms GPS Millisecond Time of Week
    unsigned short gDOP;    // Geometric DOP
    unsigned short pDOP;    // Position DOP
    unsigned short tDOP;    // Time DOP
    unsigned short vDOP;    // Vertical DOP
    unsigned short hDOP;    // Horizontal DOP
    unsigned short nDOP;    // Northing DOP
    unsigned short eDOP;    // Easting DOP
} __attribute__((packed)) ubloxStructDOP_t;

// Velocity Solution in NED
typedef struct {
    unsigned long iTOW;	    // GPS Millisecond Time of Week (ms)
    signed long velN;	    // NED north velocity (cm/s)
    signed long velE;	    // NED east velocity (cm/s)
    signed long velD;	    // NED down velocity (cm/s)
    unsigned long speed;    // Speed (3-D) (cm/s)
    unsigned long gSpeed;   // Ground Speed (2-D) (cm/s)
    signed long heading;    // Heading 2-D (deg * 1e-5)
    unsigned long sAcc;	    // Speed Accuracy Estimate (cm/s)
    unsigned long cAcc;	    // Course / Heading Accuracy Estimate (deg * 1e-5)
} __attribute__((packed)) ubloxStructVALNED_t;

// Timepulse Timedata
typedef struct {
    unsigned long towMS;
    unsigned long towSubMS;
    signed long qErr;
    unsigned short week;
    unsigned char flags;
    unsigned char res;
} __attribute__((packed)) ubloxStructTP_t;

// UTC Time Solution
typedef struct {
    unsigned long iTOW;	    // GPS Millisecond Time of Week (ms)
    unsigned long tAcc;	    // Time Accuracy Estimate
    long nano;		    // Nanosecond of second (UTC)
    unsigned short year;    // Year, range 1999..2099 (UTC)
    unsigned char month;    // Month, range 1..12 (UTC)
    unsigned char day;	    // Day of Month, range 1..31 (UTC)
    unsigned char hour;	    // Hour of Day, range 0..23 (UTC)
    unsigned char min;	    // Minute of Hour, range 0..59 (UTC)
    unsigned char sec;	    // Second of Minute, range 0..59 (UTC)
    unsigned char valid;    // Validity Flags
} __attribute__((packed)) ubloxStructTIMEUTC_t;

extern unsigned char ubloxCharIn(unsigned char c);
extern void ubloxInit();
extern void ubloxSendSetup(void);
extern void ubloxInitGps(void);

#endif
