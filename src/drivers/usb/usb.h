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

#ifndef _usb_h
#define _usb_h

#include <stdint.h>

#define USB_RX_BUFSIZE		512

#define USB_STREAM_TELEMETRY	300
#define USB_STREAM_MAVLINK	600
#define USB_STREAM_GPS		1200

typedef struct {
    uint32_t bitrate;
    uint8_t  format;
    uint8_t  parityType;
    uint8_t  dataType;
} lineCoding_t;

typedef struct {
    uint8_t rxBuf[USB_RX_BUFSIZE];
    uint16_t rxBufHead;
    uint16_t rxBufTail;
    lineCoding_t lineCoding;
} usbStruct_t;

extern usbStruct_t usbData;

extern void usbInit(void);
extern void usbTx(uint8_t* buf, uint32_t len);
extern uint8_t usbRx();
extern uint8_t usbAvailable(void);
extern uint8_t usbIsSuspend(void);

void USBD_USR_Init(void);
void USBD_USR_DeviceReset (uint8_t speed);
void USBD_USR_DeviceConfigured (void);
void USBD_USR_DeviceSuspended(void);
void USBD_USR_DeviceResumed(void);
void USBD_USR_DeviceConnected(void);
void USBD_USR_DeviceDisconnected(void);

#endif
