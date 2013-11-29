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