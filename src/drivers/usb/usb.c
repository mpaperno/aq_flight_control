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

#include "usb.h"
#include "usbd_cdc_msc_core.h"
#include "usb_conf.h"
#include "usbd_desc.h"
#include "usbd_ioreq.h"
#include "comm.h"

#ifdef HAS_USB
static uint16_t usbVcpInit     (void);
static uint16_t usbVcpDeInit   (void);
static uint16_t usbVcpCtrl     (uint32_t cmd, uint8_t* buf, uint32_t len);
static uint16_t usbVcpDataTx   (uint8_t* buf, uint32_t len);
static uint16_t usbVcpDataRx   (uint8_t* buf, uint32_t len);

CDC_IF_Prop_TypeDef VCP_fops = {
    usbVcpInit,
    usbVcpDeInit,
    usbVcpCtrl,
    usbVcpDataTx,
    usbVcpDataRx
};

USBD_Usr_cb_TypeDef USR_cb = {
  USBD_USR_Init,
  USBD_USR_DeviceReset,
  USBD_USR_DeviceConfigured,
  USBD_USR_DeviceSuspended,
  USBD_USR_DeviceResumed,
  USBD_USR_DeviceConnected,
  USBD_USR_DeviceDisconnected,
};

// These are external variables imported from CDC core to be used for IN
// transfer management.
extern uint8_t APP_Rx_Buffer[];	    // Write CDC received data in this buffer.
				    //   These data will be sent over USB IN endpoint
				    //   in the CDC core functions.
extern uint32_t APP_Rx_ptr_in;	    // Increment this pointer or roll it back to
				    //   start address when writing received data
				    //   in the buffer APP_Rx_Buffer.

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */

__ALIGN_BEGIN USB_OTG_CORE_HANDLE    USB_OTG_dev __ALIGN_END ;

usbStruct_t usbData __attribute__((section(".ccm")));

void usbInit(void) {
    USBD_Init(&USB_OTG_dev,
#ifdef USE_USB_OTG_HS
			    USB_OTG_HS_CORE_ID,
#else
			    USB_OTG_FS_CORE_ID,
#endif
			    &USR_desc,
			    &USBD_CDC_MSC_cb,
			    &USR_cb);
}

static uint16_t usbVcpInit(void) {
    return USBD_OK;
}

static uint16_t usbVcpDeInit(void) {
    return USBD_OK;
}

static uint16_t usbVcpCtrl(uint32_t Cmd, uint8_t* Buf, uint32_t Len) {
    switch (Cmd) {
	case SET_LINE_CODING:
	    usbData.lineCoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));
	    usbData.lineCoding.format = Buf[4];
	    usbData.lineCoding.parityType = Buf[5];
	    usbData.lineCoding.dataType = Buf[6];

	    switch (usbData.lineCoding.bitrate) {
		case USB_STREAM_TELEMETRY:
		    commSetStreamType(COMM_USB_PORT, COMM_STREAM_TYPE_TELEMETRY);
		    break;

		case USB_STREAM_MAVLINK:
		    commSetStreamType(COMM_USB_PORT, COMM_STREAM_TYPE_MAVLINK);
		    break;

		case USB_STREAM_GPS:
		    commSetStreamType(COMM_USB_PORT, COMM_STREAM_TYPE_GPS);
		    break;

		default:
		    commSetStreamType(COMM_USB_PORT, COMM_STREAM_TYPE_MAVLINK);
		    break;
	    }
	    break;

	case GET_LINE_CODING:
	    Buf[0] = (uint8_t)(usbData.lineCoding.bitrate);
	    Buf[1] = (uint8_t)(usbData.lineCoding.bitrate >> 8);
	    Buf[2] = (uint8_t)(usbData.lineCoding.bitrate >> 16);
	    Buf[3] = (uint8_t)(usbData.lineCoding.bitrate >> 24);
	    Buf[4] = usbData.lineCoding.format;
	    Buf[5] = usbData.lineCoding.parityType;
	    Buf[6] = usbData.lineCoding.dataType;
	    break;

	default:
	break;
    }

    return USBD_OK;
}

static uint16_t usbVcpDataTx(uint8_t* buf, uint32_t len) {
    uint16_t ptr = APP_Rx_ptr_in;
    int i;

    for (i = 0; i < len; i++)
	APP_Rx_Buffer[ptr++ & (APP_RX_DATA_SIZE-1)] = buf[i];

    APP_Rx_ptr_in = ptr % APP_RX_DATA_SIZE;

    return USBD_OK;
}

static uint16_t usbVcpDataRx(uint8_t* buf, uint32_t len) {
    uint16_t ptr = usbData.rxBufHead;
    int i;

    for (i = 0; i < len; i++)
	usbData.rxBuf[ptr++ & (USB_RX_BUFSIZE-1)] = buf[i];

    usbData.rxBufHead = ptr % USB_RX_BUFSIZE;

    return USBD_OK;
}

extern uint8_t  USB_Tx_State;
extern uint8_t  USB_DTE_Present;

#include <__cross_studio_io.h>
void USBD_USR_Init(void) {
}

void USBD_USR_DeviceReset(uint8_t speed ) {
}

void USBD_USR_DeviceConfigured(void) {
}

void USBD_USR_DeviceSuspended(void) {
    USB_Tx_State = 0;
    USB_DTE_Present = 0;
}

void USBD_USR_DeviceResumed(void) {
}

void USBD_USR_DeviceConnected(void) {
}

void USBD_USR_DeviceDisconnected(void) {
}

void usbTx(uint8_t* buf, uint32_t len) {
    usbVcpDataTx(buf, len);
}

uint8_t usbAvailable(void) {
    return (usbData.rxBufHead != usbData.rxBufTail);
}

uint8_t usbRx(void) {
    uint8_t ch = 0;

    if (usbAvailable()) {
	ch = usbData.rxBuf[usbData.rxBufTail];
	usbData.rxBufTail = (usbData.rxBufTail + 1) % USB_RX_BUFSIZE;
    }

    return ch;
}

uint8_t usbIsSuspend(void) {
    return (USB_OTG_dev.regs.DREGS->DSTS & 1);
}

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
extern uint32_t USBD_OTG_EP1IN_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
extern uint32_t USBD_OTG_EP1OUT_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);
#endif

#ifdef USE_USB_OTG_FS
void OTG_FS_WKUP_IRQHandler(void) {
    if(USB_OTG_dev.cfg.low_power) {
	*(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
	SystemInit();
	USB_OTG_UngateClock(&USB_OTG_dev);
    }
    EXTI_ClearITPendingBit(EXTI_Line18);
}
#endif

#ifdef USE_USB_OTG_HS
void OTG_HS_WKUP_IRQHandler(void) {
    if(USB_OTG_dev.cfg.low_power) {
	*(uint32_t *)(0xE000ED10) &= 0xFFFFFFF9 ;
	SystemInit();
	USB_OTG_UngateClock(&USB_OTG_dev);
    }

    EXTI_ClearITPendingBit(EXTI_Line20);
}
#endif

#ifdef USE_USB_OTG_HS
void OTG_HS_IRQHandler(void)
#else
void OTG_FS_IRQHandler(void)
#endif
{
    USBD_OTG_ISR_Handler (&USB_OTG_dev);
}

#ifdef USB_OTG_HS_DEDICATED_EP1_ENABLED
void OTG_HS_EP1_IN_IRQHandler(void) {
    USBD_OTG_EP1IN_ISR_Handler (&USB_OTG_dev);
}

void OTG_HS_EP1_OUT_IRQHandler(void) {
    USBD_OTG_EP1OUT_ISR_Handler (&USB_OTG_dev);
}
#endif

#endif	// HAS_USB
