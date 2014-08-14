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

#include "aq.h"
#include "sdio.h"
#include "util.h"
#include "aq_timer.h"
#include "comm.h"
#include "rtc.h"
#include "ext_irq.h"
#include <CoOS.h>
#include <stdio.h>
#include <string.h>

sdioStruct_t sdioData __attribute__((section(".ccm")));

// Detect if SD card is correctly plugged in the memory slot.
uint8_t SD_DetectLowLevel(void) {
    __IO uint8_t status = SD_PRESENT;

    // Check GPIO to detect SD
    if (GPIO_ReadInputDataBit(SDIO_DETECT_GPIO_PORT, SDIO_DETECT_PIN) != Bit_RESET) {
	status = SD_NOT_PRESENT;
	sdioData.initialized = 0;
    }

    return status;
}

uint8_t SD_Detect(void) {
    __IO uint8_t status = SD_PRESENT;

    if (sdioData.cardRemovalMicros && (timerMicros() - sdioData.cardRemovalMicros) > 100000) {
#ifdef SDIO_POWER_PORT
	// power off LDO
	digitalLo(sdioData.sdEnable);
#endif
	status = SD_NOT_PRESENT;
	sdioData.initialized = 0;
    }

    return status;
}

// Checks for error conditions for CMD0.
static SD_Error CmdError(void) {
    SD_Error errorstatus = SD_OK;
    uint32_t timeout;

    timeout = SDIO_CMD0TIMEOUT;	// 10000

    while ((timeout > 0) && (SDIO_GetFlagStatus(SDIO_FLAG_CMDSENT) == RESET)) {
	timeout--;
    }

    if (timeout == 0) {
	errorstatus = SD_CMD_RSP_TIMEOUT;
	return(errorstatus);
    }

    // Clear all the static flags
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}

// Checks for error conditions for R7 response.
static SD_Error CmdResp7Error(void) {
    SD_Error errorstatus = SD_OK;
    uint32_t status;
    uint32_t timeout = SDIO_CMD0TIMEOUT;

    status = SDIO->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)) && (timeout > 0)) {
	timeout--;
	status = SDIO->STA;
    }

    if ((timeout == 0) || (status & SDIO_FLAG_CTIMEOUT)) {
	// Card is not V2.0 complient or card does not support the set voltage range
	errorstatus = SD_CMD_RSP_TIMEOUT;
	SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
	return(errorstatus);
    }

    if (status & SDIO_FLAG_CMDREND) {
	// Card is SD V2.0 compliant
	errorstatus = SD_OK;
	SDIO_ClearFlag(SDIO_FLAG_CMDREND);
	return(errorstatus);
    }

    return(errorstatus);
}

// Checks for error conditions for R1 response.
static SD_Error CmdResp1Error(uint8_t cmd) {
    SD_Error errorstatus = SD_OK;
    uint32_t status;
    uint32_t response_r1;

    status = SDIO->STA;
    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT)))
	status = SDIO->STA;

    if (status & SDIO_FLAG_CTIMEOUT) {
	errorstatus = SD_CMD_RSP_TIMEOUT;
	SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);

	return(errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCFAIL) {
	errorstatus = SD_CMD_CRC_FAIL;
	SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);

	return(errorstatus);
    }

    // Check response received is of desired command
    if (SDIO_GetCommandResponse() != cmd) {
	errorstatus = SD_ILLEGAL_CMD;

	return(errorstatus);
    }

    // Clear all the static flags
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    // We have received response, retrieve it for analysis
    response_r1 = SDIO_GetResponse(SDIO_RESP1);

    if ((response_r1 & SD_OCR_ERRORBITS) == SD_ALLZERO)
	return(errorstatus);

    if (response_r1 & SD_OCR_ADDR_OUT_OF_RANGE)
	return(SD_ADDR_OUT_OF_RANGE);

    if (response_r1 & SD_OCR_ADDR_MISALIGNED)
	return(SD_ADDR_MISALIGNED);

    if (response_r1 & SD_OCR_BLOCK_LEN_ERR)
	return(SD_BLOCK_LEN_ERR);

    if (response_r1 & SD_OCR_ERASE_SEQ_ERR)
	return(SD_ERASE_SEQ_ERR);

    if (response_r1 & SD_OCR_BAD_ERASE_PARAM)
	return(SD_BAD_ERASE_PARAM);

    if (response_r1 & SD_OCR_WRITE_PROT_VIOLATION)
	return(SD_WRITE_PROT_VIOLATION);

    if (response_r1 & SD_OCR_LOCK_UNLOCK_FAILED)
	return(SD_LOCK_UNLOCK_FAILED);

    if (response_r1 & SD_OCR_COM_CRC_FAILED)
	return(SD_COM_CRC_FAILED);

    if (response_r1 & SD_OCR_ILLEGAL_CMD)
	return(SD_ILLEGAL_CMD);

    if (response_r1 & SD_OCR_CARD_ECC_FAILED)
	return(SD_CARD_ECC_FAILED);

    if (response_r1 & SD_OCR_CC_ERROR)
	return(SD_CC_ERROR);

    if (response_r1 & SD_OCR_GENERAL_UNKNOWN_ERROR)
	return(SD_GENERAL_UNKNOWN_ERROR);

    if (response_r1 & SD_OCR_STREAM_READ_UNDERRUN)
	return(SD_STREAM_READ_UNDERRUN);

    if (response_r1 & SD_OCR_STREAM_WRITE_OVERRUN)
	return(SD_STREAM_WRITE_OVERRUN);

    if (response_r1 & SD_OCR_CID_CSD_OVERWRIETE)
	return(SD_CID_CSD_OVERWRITE);

    if (response_r1 & SD_OCR_WP_ERASE_SKIP)
	return(SD_WP_ERASE_SKIP);

    if (response_r1 & SD_OCR_CARD_ECC_DISABLED)
	return(SD_CARD_ECC_DISABLED);

    if (response_r1 & SD_OCR_ERASE_RESET)
	return(SD_ERASE_RESET);

    if (response_r1 & SD_OCR_AKE_SEQ_ERROR)
	return(SD_AKE_SEQ_ERROR);

    return(errorstatus);
}

// Checks for error conditions for R3 (OCR) response.
static SD_Error CmdResp3Error(void) {
    SD_Error errorstatus = SD_OK;
    uint32_t status;

    status = SDIO->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CMDREND | SDIO_FLAG_CTIMEOUT))) {
	status = SDIO->STA;
    }

    if (status & SDIO_FLAG_CTIMEOUT) {
	errorstatus = SD_CMD_RSP_TIMEOUT;
	SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
	return(errorstatus);
    }

    // Clear all the static flags
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);
    return(errorstatus);
}

// Checks for error conditions for R2 (CID or CSD) response.
static SD_Error CmdResp2Error(void) {
    SD_Error errorstatus = SD_OK;
    uint32_t status;

    status = SDIO->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND))) {
	status = SDIO->STA;
    }

    if (status & SDIO_FLAG_CTIMEOUT) {
	errorstatus = SD_CMD_RSP_TIMEOUT;
	SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
	return(errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCFAIL) {
	errorstatus = SD_CMD_CRC_FAIL;
	SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
	return(errorstatus);
    }

    // Clear all the static flags
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}

// Checks for error conditions for R6 (RCA) response.
static SD_Error CmdResp6Error(uint8_t cmd, uint16_t *prca) {
    SD_Error errorstatus = SD_OK;
    uint32_t status;
    uint32_t response_r1;

    status = SDIO->STA;

    while (!(status & (SDIO_FLAG_CCRCFAIL | SDIO_FLAG_CTIMEOUT | SDIO_FLAG_CMDREND))) {
	status = SDIO->STA;
    }

    if (status & SDIO_FLAG_CTIMEOUT) {
	errorstatus = SD_CMD_RSP_TIMEOUT;
	SDIO_ClearFlag(SDIO_FLAG_CTIMEOUT);
	return(errorstatus);
    }
    else if (status & SDIO_FLAG_CCRCFAIL) {
	errorstatus = SD_CMD_CRC_FAIL;
	SDIO_ClearFlag(SDIO_FLAG_CCRCFAIL);
	return(errorstatus);
    }

    // Check response received is of desired command
    if (SDIO_GetCommandResponse() != cmd) {
	errorstatus = SD_ILLEGAL_CMD;
	return(errorstatus);
    }

    // Clear all the static flags
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    // We have received response, retrieve it.
    response_r1 = SDIO_GetResponse(SDIO_RESP1);

    if (SD_ALLZERO == (response_r1 & (SD_R6_GENERAL_UNKNOWN_ERROR | SD_R6_ILLEGAL_CMD | SD_R6_COM_CRC_FAILED))) {
	*prca = (uint16_t) (response_r1 >> 16);
	return(errorstatus);
    }

    if (response_r1 & SD_R6_GENERAL_UNKNOWN_ERROR) {
	return(SD_GENERAL_UNKNOWN_ERROR);
    }

    if (response_r1 & SD_R6_ILLEGAL_CMD) {
	return(SD_ILLEGAL_CMD);
    }

    if (response_r1 & SD_R6_COM_CRC_FAILED) {
	return(SD_COM_CRC_FAILED);
    }

    return(errorstatus);
}

// Enquires cards about their operating voltage and configures clock controls.
SD_Error SD_PowerON(void) {
    SDIO_InitTypeDef SDIO_InitStructure;
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    __IO SD_Error errorstatus = SD_OK;
    uint32_t response = 0, count = 0, validvoltage = 0;
    uint32_t SDType = SD_STD_CAPACITY;

    // Power ON Sequence
    // Configure the SDIO peripheral
    // SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_INIT_CLK_DIV)
    // on STM32F2xx devices, SDIOCLK is fixed to 48MHz
    // SDIO_CK for initialization should not exceed 400 KHz
    SDIO_InitStructure.SDIO_ClockDiv = SDIO_INIT_CLK_DIV;
    SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
    SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
    SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
    SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
    SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
    SDIO_Init(&SDIO_InitStructure);

    // Set Power State to ON
    SDIO_SetPowerState(SDIO_PowerState_ON);

    // Enable SDIO Clock
    SDIO_ClockCmd(ENABLE);

    // CMD0: GO_IDLE_STATE
    // No CMD response required
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_GO_IDLE_STATE;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_No;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdError();

    // CMD Response TimeOut (wait for CMDSENT flag)
    if (errorstatus != SD_OK)
	return (errorstatus);

    // CMD8: SEND_IF_COND
    // Send CMD8 to verify SD card interface operating condition
    // Argument: - [31:12]: Reserved (shall be set to '0')
    //           - [11:8]: Supply Voltage (VHS) 0x1 (Range: 2.7-3.6 V)
    //           - [7:0]: Check Pattern (recommended 0xAA)
    // CMD Response: R7
    SDIO_CmdInitStructure.SDIO_Argument = SD_CHECK_PATTERN;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SDIO_SEND_IF_COND;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp7Error();

    if (errorstatus == SD_OK) {
	sdioData.CardType = SDIO_STD_CAPACITY_SD_CARD_V2_0; // SD Card 2.0
	SDType = SD_HIGH_CAPACITY;
    }
    else {
	// CMD55
	SDIO_CmdInitStructure.SDIO_Argument = 0x00;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);
	errorstatus = CmdResp1Error(SD_CMD_APP_CMD);
    }
    // CMD55
    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    // If errorstatus is Command TimeOut, it is a MMC card
    // If errorstatus is SD_OK it is a SD card: SD card 2.0 (voltage range mismatch)
    //   or SD card 1.x
    if (errorstatus == SD_OK) {
	// SD CARD
	// Send ACMD41 SD_APP_OP_COND with Argument 0x80100000
	while ((!validvoltage) && (count < SD_MAX_VOLT_TRIAL)) {
	    // SEND CMD55 APP_CMD with RCA as 0
	    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
	    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
	    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	    SDIO_SendCommand(&SDIO_CmdInitStructure);

	    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

	    if (errorstatus != SD_OK)
		return(errorstatus);

	    SDIO_CmdInitStructure.SDIO_Argument = SD_VOLTAGE_WINDOW_SD | SDType;
	    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_OP_COND;
	    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	    SDIO_SendCommand(&SDIO_CmdInitStructure);

	    errorstatus = CmdResp3Error();
	    if (errorstatus != SD_OK)
		return(errorstatus);

	    response = SDIO_GetResponse(SDIO_RESP1);
	    validvoltage = (((response >> 31) == 1) ? 1 : 0);
	    count++;
	}

	if (count >= SD_MAX_VOLT_TRIAL) {
	    errorstatus = SD_INVALID_VOLTRANGE;
	    return(errorstatus);
	}

	if (response &= SD_HIGH_CAPACITY) {
	    sdioData.CardType = SDIO_HIGH_CAPACITY_SD_CARD;
	}
    } // else MMC Card

    return(errorstatus);
}

// Turns the SDIO output signals off.
SD_Error SD_PowerOFF(void) {
    SD_Error errorstatus = SD_OK;

    // Set Power State to OFF
    SDIO_SetPowerState(SDIO_PowerState_OFF);

    return(errorstatus);
}

SD_Error SD_StopTransfer(void) {
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;

    SD_Error errorstatus = SD_OK;

    // Send CMD12 STOP_TRANSMISSION
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_STOP_TRANSMISSION;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_STOP_TRANSMISSION);

    return(errorstatus);
}

// Intialises all cards or single card as the case may be Card(s) come into standby state.
SD_Error SD_InitializeCards(void) {
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    SD_Error errorstatus = SD_OK;
    uint16_t rca = 0x01;

    if (SDIO_GetPowerState() == SDIO_PowerState_OFF) {
	errorstatus = SD_REQUEST_NOT_APPLICABLE;
	return(errorstatus);
    }

    if (SDIO_SECURE_DIGITAL_IO_CARD != sdioData.CardType) {
	// Send CMD2 ALL_SEND_CID
	SDIO_CmdInitStructure.SDIO_Argument = 0x0;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_ALL_SEND_CID;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp2Error();

	if (SD_OK != errorstatus)
	    return(errorstatus);

	sdioData.CID_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
	sdioData.CID_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
	sdioData.CID_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
	sdioData.CID_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
    }

    if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == sdioData.CardType) ||  (SDIO_STD_CAPACITY_SD_CARD_V2_0 == sdioData.CardType) ||  (SDIO_SECURE_DIGITAL_IO_COMBO_CARD == sdioData.CardType)
	||  (SDIO_HIGH_CAPACITY_SD_CARD == sdioData.CardType)) {
	// Send CMD3 SET_REL_ADDR with argument 0
	// SD Card publishes its RCA.
	SDIO_CmdInitStructure.SDIO_Argument = 0x00;
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_REL_ADDR;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp6Error(SD_CMD_SET_REL_ADDR, &rca);

	if (SD_OK != errorstatus)
	    return(errorstatus);
    }

    if (SDIO_SECURE_DIGITAL_IO_CARD != sdioData.CardType) {
	sdioData.RCA = rca;

	// Send CMD9 SEND_CSD with argument as card's RCA
	SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)(rca << 16);
	SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_CSD;
	SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Long;
	SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	SDIO_SendCommand(&SDIO_CmdInitStructure);

	errorstatus = CmdResp2Error();

	if (SD_OK != errorstatus)
	    return(errorstatus);

	sdioData.CSD_Tab[0] = SDIO_GetResponse(SDIO_RESP1);
	sdioData.CSD_Tab[1] = SDIO_GetResponse(SDIO_RESP2);
	sdioData.CSD_Tab[2] = SDIO_GetResponse(SDIO_RESP3);
	sdioData.CSD_Tab[3] = SDIO_GetResponse(SDIO_RESP4);
    }

    errorstatus = SD_OK;    // All cards get intialized

    return(errorstatus);
}

// Returns information about specific card.
SD_Error SD_GetCardInfo(SD_CardInfo *cardinfo) {
    SD_Error errorstatus = SD_OK;
    uint8_t tmp = 0;

    cardinfo->CardType = (uint8_t)sdioData.CardType;
    cardinfo->RCA = (uint16_t)sdioData.RCA;

    // Byte 0
    tmp = (uint8_t)((sdioData.CSD_Tab[0] & 0xFF000000) >> 24);
    cardinfo->SD_csd.CSDStruct = (tmp & 0xC0) >> 6;
    cardinfo->SD_csd.SysSpecVersion = (tmp & 0x3C) >> 2;
    cardinfo->SD_csd.Reserved1 = tmp & 0x03;

    // Byte 1
    tmp = (uint8_t)((sdioData.CSD_Tab[0] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.TAAC = tmp;

    // Byte 2
    tmp = (uint8_t)((sdioData.CSD_Tab[0] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.NSAC = tmp;

    // Byte 3
    tmp = (uint8_t)(sdioData.CSD_Tab[0] & 0x000000FF);
    cardinfo->SD_csd.MaxBusClkFrec = tmp;

    // Byte 4
    tmp = (uint8_t)((sdioData.CSD_Tab[1] & 0xFF000000) >> 24);
    cardinfo->SD_csd.CardComdClasses = tmp << 4;

    // Byte 5
    tmp = (uint8_t)((sdioData.CSD_Tab[1] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.CardComdClasses |= (tmp & 0xF0) >> 4;
    cardinfo->SD_csd.RdBlockLen = tmp & 0x0F;

    // Byte 6
    tmp = (uint8_t)((sdioData.CSD_Tab[1] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.PartBlockRead = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.WrBlockMisalign = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.RdBlockMisalign = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.DSRImpl = (tmp & 0x10) >> 4;
    cardinfo->SD_csd.Reserved2 = 0; // Reserved

    if ((sdioData.CardType == SDIO_STD_CAPACITY_SD_CARD_V1_1) || (sdioData.CardType == SDIO_STD_CAPACITY_SD_CARD_V2_0)) {
	cardinfo->SD_csd.DeviceSize = (tmp & 0x03) << 10;

	// Byte 7
	tmp = (uint8_t)(sdioData.CSD_Tab[1] & 0x000000FF);
	cardinfo->SD_csd.DeviceSize |= (tmp) << 2;

	// Byte 8
	tmp = (uint8_t)((sdioData.CSD_Tab[2] & 0xFF000000) >> 24);
	cardinfo->SD_csd.DeviceSize |= (tmp & 0xC0) >> 6;

	cardinfo->SD_csd.MaxRdCurrentVDDMin = (tmp & 0x38) >> 3;
	cardinfo->SD_csd.MaxRdCurrentVDDMax = (tmp & 0x07);

	// Byte 9
	tmp = (uint8_t)((sdioData.CSD_Tab[2] & 0x00FF0000) >> 16);
	cardinfo->SD_csd.MaxWrCurrentVDDMin = (tmp & 0xE0) >> 5;
	cardinfo->SD_csd.MaxWrCurrentVDDMax = (tmp & 0x1C) >> 2;
	cardinfo->SD_csd.DeviceSizeMul = (tmp & 0x03) << 1;

	// Byte 10
	tmp = (uint8_t)((sdioData.CSD_Tab[2] & 0x0000FF00) >> 8);
	cardinfo->SD_csd.DeviceSizeMul |= (tmp & 0x80) >> 7;

	cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) ;
	cardinfo->CardCapacity *= (1 << (cardinfo->SD_csd.DeviceSizeMul + 2));
	cardinfo->CardBlockSize = 1 << (cardinfo->SD_csd.RdBlockLen);
	cardinfo->CardCapacity *= cardinfo->CardBlockSize;
    }
    else if (sdioData.CardType == SDIO_HIGH_CAPACITY_SD_CARD) {
	// Byte 7
	tmp = (uint8_t)(sdioData.CSD_Tab[1] & 0x000000FF);
	cardinfo->SD_csd.DeviceSize = (tmp & 0x3F) << 16;

	// Byte 8
	tmp = (uint8_t)((sdioData.CSD_Tab[2] & 0xFF000000) >> 24);

	cardinfo->SD_csd.DeviceSize |= (tmp << 8);

	// Byte 9
	tmp = (uint8_t)((sdioData.CSD_Tab[2] & 0x00FF0000) >> 16);

	cardinfo->SD_csd.DeviceSize |= (tmp);

	// Byte 10
	tmp = (uint8_t)((sdioData.CSD_Tab[2] & 0x0000FF00) >> 8);

	cardinfo->CardCapacity = (cardinfo->SD_csd.DeviceSize + 1) * 512 * 1024;
	cardinfo->CardBlockSize = 512;
    }

    cardinfo->SD_csd.EraseGrSize = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.EraseGrMul = (tmp & 0x3F) << 1;

    // Byte 11
    tmp = (uint8_t)(sdioData.CSD_Tab[2] & 0x000000FF);
    cardinfo->SD_csd.EraseGrMul |= (tmp & 0x80) >> 7;
    cardinfo->SD_csd.WrProtectGrSize = (tmp & 0x7F);

    // Byte 12
    tmp = (uint8_t)((sdioData.CSD_Tab[3] & 0xFF000000) >> 24);
    cardinfo->SD_csd.WrProtectGrEnable = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.ManDeflECC = (tmp & 0x60) >> 5;
    cardinfo->SD_csd.WrSpeedFact = (tmp & 0x1C) >> 2;
    cardinfo->SD_csd.MaxWrBlockLen = (tmp & 0x03) << 2;

    // Byte 13
    tmp = (uint8_t)((sdioData.CSD_Tab[3] & 0x00FF0000) >> 16);
    cardinfo->SD_csd.MaxWrBlockLen |= (tmp & 0xC0) >> 6;
    cardinfo->SD_csd.WriteBlockPaPartial = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.Reserved3 = 0;
    cardinfo->SD_csd.ContentProtectAppli = (tmp & 0x01);

    // Byte 14
    tmp = (uint8_t)((sdioData.CSD_Tab[3] & 0x0000FF00) >> 8);
    cardinfo->SD_csd.FileFormatGrouop = (tmp & 0x80) >> 7;
    cardinfo->SD_csd.CopyFlag = (tmp & 0x40) >> 6;
    cardinfo->SD_csd.PermWrProtect = (tmp & 0x20) >> 5;
    cardinfo->SD_csd.TempWrProtect = (tmp & 0x10) >> 4;
    cardinfo->SD_csd.FileFormat = (tmp & 0x0C) >> 2;
    cardinfo->SD_csd.ECC = (tmp & 0x03);

    // Byte 15
    tmp = (uint8_t)(sdioData.CSD_Tab[3] & 0x000000FF);
    cardinfo->SD_csd.CSD_CRC = (tmp & 0xFE) >> 1;
    cardinfo->SD_csd.Reserved4 = 1;

    // Byte 0
    tmp = (uint8_t)((sdioData.CID_Tab[0] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ManufacturerID = tmp;

    // Byte 1
    tmp = (uint8_t)((sdioData.CID_Tab[0] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.OEM_AppliID = tmp << 8;

    // Byte 2
    tmp = (uint8_t)((sdioData.CID_Tab[0] & 0x000000FF00) >> 8);
    cardinfo->SD_cid.OEM_AppliID |= tmp;

    // Byte 3
    tmp = (uint8_t)(sdioData.CID_Tab[0] & 0x000000FF);
    cardinfo->SD_cid.ProdName1 = tmp << 24;

    // Byte 4
    tmp = (uint8_t)((sdioData.CID_Tab[1] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdName1 |= tmp << 16;

    // Byte 5
    tmp = (uint8_t)((sdioData.CID_Tab[1] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.ProdName1 |= tmp << 8;

    // Byte 6
    tmp = (uint8_t)((sdioData.CID_Tab[1] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ProdName1 |= tmp;

    // Byte 7
    tmp = (uint8_t)(sdioData.CID_Tab[1] & 0x000000FF);
    cardinfo->SD_cid.ProdName2 = tmp;

    // Byte 8
    tmp = (uint8_t)((sdioData.CID_Tab[2] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdRev = tmp;

    // Byte 9
    tmp = (uint8_t)((sdioData.CID_Tab[2] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.ProdSN = tmp << 24;

    // Byte 10
    tmp = (uint8_t)((sdioData.CID_Tab[2] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ProdSN |= tmp << 16;

    // Byte 11
    tmp = (uint8_t)(sdioData.CID_Tab[2] & 0x000000FF);
    cardinfo->SD_cid.ProdSN |= tmp << 8;

    // Byte 12
    tmp = (uint8_t)((sdioData.CID_Tab[3] & 0xFF000000) >> 24);
    cardinfo->SD_cid.ProdSN |= tmp;

    // Byte 13
    tmp = (uint8_t)((sdioData.CID_Tab[3] & 0x00FF0000) >> 16);
    cardinfo->SD_cid.Reserved1 |= (tmp & 0xF0) >> 4;
    cardinfo->SD_cid.ManufactDate = (tmp & 0x0F) << 8;

    // Byte 14
    tmp = (uint8_t)((sdioData.CID_Tab[3] & 0x0000FF00) >> 8);
    cardinfo->SD_cid.ManufactDate |= tmp;

    // Byte 15
    tmp = (uint8_t)(sdioData.CID_Tab[3] & 0x000000FF);
    cardinfo->SD_cid.CID_CRC = (tmp & 0xFE) >> 1;
    cardinfo->SD_cid.Reserved2 = 1;

    return(errorstatus);
}

// Selects od Deselects the corresponding card.
SD_Error SD_SelectDeselect(uint32_t addr) {
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    SD_Error errorstatus = SD_OK;

    // Send CMD7 SDIO_SEL_DESEL_CARD
    SDIO_CmdInitStructure.SDIO_Argument =  addr;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEL_DESEL_CARD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SEL_DESEL_CARD);

    return(errorstatus);
}

// Find the SD card SCR register value.
static SD_Error FindSCR(uint16_t rca, uint32_t *pscr) {
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;
    uint32_t index = 0;
    SD_Error errorstatus = SD_OK;
    uint32_t tempscr[2] = {0, 0};

    // Set Block Size To 8 Bytes
    // Send CMD55 APP_CMD with argument as card's RCA
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)8;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (errorstatus != SD_OK)
	return(errorstatus);

    // Send CMD55 APP_CMD with argument as card's RCA
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) sdioData.RCA << 16;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    if (errorstatus != SD_OK)
	return(errorstatus);

    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
    SDIO_DataInitStructure.SDIO_DataLength = 8;
    SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_8b;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig(&SDIO_DataInitStructure);

    // Send ACMD51 SD_APP_SEND_SCR with argument as 0
    SDIO_CmdInitStructure.SDIO_Argument = 0x0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_SEND_SCR;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SD_APP_SEND_SCR);

    if (errorstatus != SD_OK)
	return(errorstatus);

    while (!(SDIO->STA & (SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR))) {
	if (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET) {
	    *(tempscr + index) = SDIO_ReadData();
	    index++;
	}
	else if (SD_Detect() != SD_PRESENT) {
	    return SD_DATA_TIMEOUT;
	}
    }

    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
	errorstatus = SD_DATA_TIMEOUT;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
	errorstatus = SD_DATA_CRC_FAIL;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
	errorstatus = SD_RX_OVERRUN;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_STBITERR);
	errorstatus = SD_START_BIT_ERR;
	return(errorstatus);
    }

    // Clear all the static flags
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    *(pscr + 1) = ((tempscr[0] & SD_0TO7BITS) << 24) | ((tempscr[0] & SD_8TO15BITS) << 8) | ((tempscr[0] & SD_16TO23BITS) >> 8) | ((tempscr[0] & SD_24TO31BITS) >> 24);

    *(pscr) = ((tempscr[1] & SD_0TO7BITS) << 24) | ((tempscr[1] & SD_8TO15BITS) << 8) | ((tempscr[1] & SD_16TO23BITS) >> 8) | ((tempscr[1] & SD_24TO31BITS) >> 24);

    return(errorstatus);
}

// Enables or disables the SDIO wide bus mode.
static SD_Error SDEnWideBus(FunctionalState NewState) {
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    SD_Error errorstatus = SD_OK;

    uint32_t scr[2] = {0, 0};

    if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED) {
	errorstatus = SD_LOCK_UNLOCK_FAILED;
	return(errorstatus);
    }

    // Get SCR Register
    errorstatus = FindSCR(sdioData.RCA, scr);

    if (errorstatus != SD_OK)
	return(errorstatus);

    // If wide bus operation to be enabled
    if (NewState == ENABLE) {
	// If requested card supports wide bus operation
	if ((scr[1] & SD_WIDE_BUS_SUPPORT) != SD_ALLZERO) {
	    // Send CMD55 APP_CMD with argument as card's RCA.
	    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) sdioData.RCA << 16;
	    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
	    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	    SDIO_SendCommand(&SDIO_CmdInitStructure);

	    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

	    if (errorstatus != SD_OK)
		return(errorstatus);

	    // Send ACMD6 APP_CMD with argument as 2 for wide bus mode
	    SDIO_CmdInitStructure.SDIO_Argument = 0x2;
	    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
	    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	    SDIO_SendCommand(&SDIO_CmdInitStructure);

	    errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

	    if (errorstatus != SD_OK)
		return(errorstatus);

	    return(errorstatus);
	}
	else {
	    errorstatus = SD_REQUEST_NOT_APPLICABLE;
	    return(errorstatus);
	}
    }
    // If wide bus operation to be disabled
    else {
	// If requested card supports 1 bit mode operation
	if ((scr[1] & SD_SINGLE_BUS_SUPPORT) != SD_ALLZERO) {
	    // Send CMD55 APP_CMD with argument as card's RCA.
	    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) sdioData.RCA << 16;
	    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
	    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	    SDIO_SendCommand(&SDIO_CmdInitStructure);

	    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

	    if (errorstatus != SD_OK)
		return(errorstatus);

	    // Send ACMD6 APP_CMD with argument as 2 for wide bus mode
	    SDIO_CmdInitStructure.SDIO_Argument = 0x00;
	    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_SD_SET_BUSWIDTH;
	    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
	    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
	    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
	    SDIO_SendCommand(&SDIO_CmdInitStructure);

	    errorstatus = CmdResp1Error(SD_CMD_APP_SD_SET_BUSWIDTH);

	    if (errorstatus != SD_OK)
		return(errorstatus);

	    return(errorstatus);
	}
	else {
	    errorstatus = SD_REQUEST_NOT_APPLICABLE;
	    return(errorstatus);
	}
    }
}

// Enables wide bus opeartion for the requeseted card if supported by card.
//  WideMode: Specifies the SD card wide bus mode.
//   This parameter can be one of the following values:
//     SDIO_BusWide_8b: 8-bit data transfer (Only for MMC)
//     SDIO_BusWide_4b: 4-bit data transfer
//     SDIO_BusWide_1b: 1-bit data transfer
SD_Error SD_EnableWideBusOperation(uint32_t WideMode) {
    SDIO_InitTypeDef SDIO_InitStructure;
    SD_Error errorstatus = SD_OK;

    // MMC Card doesn't support this feature
    if (SDIO_MULTIMEDIA_CARD == sdioData.CardType) {
	errorstatus = SD_UNSUPPORTED_FEATURE;
	return(errorstatus);
    }
    else if ((SDIO_STD_CAPACITY_SD_CARD_V1_1 == sdioData.CardType) || (SDIO_STD_CAPACITY_SD_CARD_V2_0 == sdioData.CardType) || (SDIO_HIGH_CAPACITY_SD_CARD == sdioData.CardType)) {
	if (SDIO_BusWide_8b == WideMode) {
	    errorstatus = SD_UNSUPPORTED_FEATURE;
	    return(errorstatus);
	}
	else if (SDIO_BusWide_4b == WideMode) {
	    errorstatus = SDEnWideBus(ENABLE);

	    if (SD_OK == errorstatus) {
		// Configure the SDIO peripheral
		SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV;
		SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
		SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
		SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
		SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_4b;
		SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
		SDIO_Init(&SDIO_InitStructure);
	    }
	}
	else {
	    errorstatus = SDEnWideBus(DISABLE);

	    if (SD_OK == errorstatus) {
		// Configure the SDIO peripheral
		SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV;
		SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
		SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
		SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
		SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
		SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
		SDIO_Init(&SDIO_InitStructure);
	    }
	}
    }

    return(errorstatus);
}

// card detect interrupt
void sdioIntHandler(void) {
    if (SD_DetectLowLevel() == SD_NOT_PRESENT)
        sdioData.cardRemovalMicros = timerMicros();
    else
        sdioData.cardRemovalMicros = 0;
}

void sdioLowLevelInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    NVIC_InitTypeDef NVIC_InitStructure;

    memset((void *)&sdioData, 0, sizeof(sdioData));

    GPIO_PinAFConfig(GPIOC, GPIO_PinSource8, GPIO_AF_SDIO);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource9, GPIO_AF_SDIO);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource10, GPIO_AF_SDIO);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_SDIO);
    GPIO_PinAFConfig(GPIOC, GPIO_PinSource12, GPIO_AF_SDIO);
    GPIO_PinAFConfig(GPIOD, GPIO_PinSource2, GPIO_AF_SDIO);

    // Configure PC.08, PC.09, PC.10, PC.11 pins: D0, D1, D2, D3 pins
    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Configure PD.02 CMD line
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_Init(GPIOD, &GPIO_InitStructure);

    // Configure PC.12 pin: CLK pin
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // Enable the SDIO APB2 Clock
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SDIO, ENABLE);

#ifdef SDIO_POWER_PORT
    // turn off SDIO power supply
    sdioData.sdEnable = digitalInit(SDIO_POWER_PORT, SDIO_POWER_PIN, 0);
#endif

    // SDIO Interrupt ENABLE
    NVIC_InitStructure.NVIC_IRQChannel = SDIO_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // DMA2 STREAMx Interrupt ENABLE
    NVIC_InitStructure.NVIC_IRQChannel = SDIO_DMA_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Configure SD Card detect pin
    extRegisterCallback(SDIO_DETECT_GPIO_PORT, SDIO_DETECT_PIN, EXTI_Trigger_Rising_Falling, 2, GPIO_PuPd_UP, sdioIntHandler);

    sdioData.initialized = 0;
    sdioData.cardRemovalMicros = 0;

    if (SD_DetectLowLevel() == SD_NOT_PRESENT)
	sdioData.cardRemovalMicros = timerMicros();
#ifdef SDIO_POWER_PORT
    else
	digitalHi(sdioData.sdEnable);
#endif
}

void SD_LowLevel_DMA_TxConfig(uint32_t *BufferSRC, uint32_t BufferSize) {
    DMA_InitTypeDef SDDMA_InitStructure;

    DMA_ClearFlag(SDIO_DMA_STREAM, SDIO_DMA_FLAG_FEIF | SDIO_DMA_FLAG_DMEIF | SDIO_DMA_FLAG_TEIF | SDIO_DMA_FLAG_HTIF | SDIO_DMA_FLAG_TCIF);

    // DMA2 Stream3 disable
    DMA_Cmd(SDIO_DMA_STREAM, DISABLE);

    // DMA2 Stream3 Config
    DMA_DeInit(SDIO_DMA_STREAM);

    SDDMA_InitStructure.DMA_Channel = SDIO_DMA_CHANNEL;
    SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
    SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferSRC;
    SDDMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
    SDDMA_InitStructure.DMA_BufferSize = BufferSize / 4;
    SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    SDDMA_InitStructure.DMA_Priority = DMA_Priority_High;
    SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
    SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
    DMA_Init(SDIO_DMA_STREAM, &SDDMA_InitStructure);

    DMA_FlowControllerConfig(SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

//    DMA_ITConfig(SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);

    // DMA2 Stream3 enable
    DMA_Cmd(SDIO_DMA_STREAM, ENABLE);
}

void SD_LowLevel_DMA_RxConfig(uint32_t *BufferDST, uint32_t BufferSize) {
    DMA_InitTypeDef SDDMA_InitStructure;

    // DMA2 Stream3 disable
    DMA_Cmd(SDIO_DMA_STREAM, DISABLE);

    // DMA2 Stream3 Config
    DMA_DeInit(SDIO_DMA_STREAM);

    SDDMA_InitStructure.DMA_Channel = SDIO_DMA_CHANNEL;
    SDDMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)SDIO_FIFO_ADDRESS;
    SDDMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)BufferDST;
    SDDMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
    SDDMA_InitStructure.DMA_BufferSize = BufferSize / 4;
    SDDMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    SDDMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    SDDMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Word;
    SDDMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Word;
    SDDMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    SDDMA_InitStructure.DMA_Priority = DMA_Priority_High;
    SDDMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Enable;
    SDDMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
    SDDMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_INC4;
    SDDMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_INC4;
    DMA_Init(SDIO_DMA_STREAM, &SDDMA_InitStructure);

    DMA_FlowControllerConfig(SDIO_DMA_STREAM, DMA_FlowCtrl_Peripheral);

    DMA_ClearFlag(SDIO_DMA_STREAM, SDIO_DMA_FLAG_FEIF | SDIO_DMA_FLAG_DMEIF | SDIO_DMA_FLAG_TEIF | SDIO_DMA_FLAG_HTIF | SDIO_DMA_FLAG_TCIF);

//    DMA_ITConfig(SDIO_DMA_STREAM, DMA_IT_TC, ENABLE);

    // DMA2 Stream3 enable
    DMA_Cmd(SDIO_DMA_STREAM, ENABLE);
}

SD_Error sdioInit(void) {
    SDIO_InitTypeDef SDIO_InitStructure;
    __IO SD_Error errorstatus = SD_OK;

    sdioData.CardType = SDIO_STD_CAPACITY_SD_CARD_V1_1;
    sdioData.RCA = 0;
    sdioData.StopCondition = 0;
    sdioData.TransferError = SD_OK;
    sdioData.TransferEnd = 1;

#ifdef SDIO_POWER_PORT
    // power on LDO
    digitalHi(sdioData.sdEnable);
    yield(10);
#endif

    SDIO_DeInit();

    errorstatus = SD_PowerON();

    // CMD Response TimeOut (wait for CMDSENT flag)
    if (errorstatus != SD_OK)
	return(errorstatus);

    errorstatus = SD_InitializeCards();

    // CMD Response TimeOut (wait for CMDSENT flag)
    if (errorstatus != SD_OK)
	return(errorstatus);

    // Configure the SDIO peripheral
    // SDIOCLK = HCLK, SDIO_CK = HCLK/(2 + SDIO_TRANSFER_CLK_DIV)
    // on STM32F2xx devices, SDIOCLK is fixed to 48MHz
    SDIO_InitStructure.SDIO_ClockDiv = SDIO_TRANSFER_CLK_DIV;
    SDIO_InitStructure.SDIO_ClockEdge = SDIO_ClockEdge_Rising;
    SDIO_InitStructure.SDIO_ClockBypass = SDIO_ClockBypass_Disable;
    SDIO_InitStructure.SDIO_ClockPowerSave = SDIO_ClockPowerSave_Disable;
    SDIO_InitStructure.SDIO_BusWide = SDIO_BusWide_1b;
    SDIO_InitStructure.SDIO_HardwareFlowControl = SDIO_HardwareFlowControl_Disable;
    SDIO_Init(&SDIO_InitStructure);

    // Read CSD/CID MSD registers
    errorstatus = SD_GetCardInfo(&sdioData.SDCardInfo);

    // Select Card
    if (errorstatus == SD_OK)
	errorstatus = SD_SelectDeselect((uint32_t) (sdioData.SDCardInfo.RCA << 16));

    if (errorstatus == SD_OK)
	errorstatus = SD_EnableWideBusOperation(SDIO_BusWide_4b);

    return(errorstatus);
}

// Allows to read one block from a specified address in a card. The Data
// transfer can be managed by DMA mode or Polling mode.
SD_Error SD_ReadBlock(uint8_t *readbuff, uint32_t ReadSector, uint16_t BlockSize) {
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;
    SD_Error errorstatus = SD_OK;
#if defined (SD_POLLING_MODE)
    uint32_t count = 0, *tempbuff = (uint32_t *)readbuff;
#endif

    sdioData.TransferError = SD_OK;
    sdioData.TransferEnd = 0;
    sdioData.StopCondition = 0;

    SDIO->DCTRL = 0x0;

#if defined (SD_DMA_MODE)
  SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_STBITERR, ENABLE);
  SDIO_DMACmd(ENABLE);
  SD_LowLevel_DMA_RxConfig((uint32_t *)readbuff, BlockSize);
#endif

    if (sdioData.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
	BlockSize = 512;

    // Set Block Size for Card
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
	return (errorstatus);

    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
    SDIO_DataInitStructure.SDIO_DataLength = BlockSize;
    SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) 9 << 4;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig(&SDIO_DataInitStructure);

    // Send CMD17 READ_SINGLE_BLOCK
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)ReadSector;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_SINGLE_BLOCK;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_READ_SINGLE_BLOCK);

    if (errorstatus != SD_OK)
	return (errorstatus);

#if defined (SD_POLLING_MODE)
    // In case of single block transfer, no need of stop transfer at all.
    // Polling mode
    while (!(SDIO->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR))) {
	if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET) {
	    for (count = 0; count < 8; count++) {
		*(tempbuff + count) = SDIO_ReadData();
	    }
	    tempbuff += 8;
	}
    }

    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
	errorstatus = SD_DATA_TIMEOUT;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
	errorstatus = SD_DATA_CRC_FAIL;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
	errorstatus = SD_RX_OVERRUN;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_STBITERR);
	errorstatus = SD_START_BIT_ERR;
	return(errorstatus);
    }
    while (SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET) {
	*tempbuff = SDIO_ReadData();
	tempbuff++;
    }

    // Clear all the static flags
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);
#endif

    return(errorstatus);
}


// Allows to read blocks from a specified address  in a card.  The Data
// transfer can be managed by DMA mode or Polling mode.
// This operation should be followed by two functions to check if the
// DMA Controller and SD Card status.
//  - SD_ReadWaitOperation(): this function insure that the DMA
//	controller has finished all data transfer.
//  - SD_GetStatus(): to check that the SD Card has finished the
//	data transfer and it is ready for data.
//  readbuff: pointer to the buffer that will contain the received data.
//  ReadAddr: Address from where data are to be read.
//  BlockSize: the SD card Data block size. The Block size should be 512.
//  NumberOfBlocks: number of blocks to be read.
SD_Error SD_ReadMultiBlocks(uint8_t *readbuff, uint32_t ReadSector, uint16_t BlockSize, uint32_t NumberOfBlocks) {
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;
    SD_Error errorstatus = SD_OK;
    sdioData.TransferError = SD_OK;
    sdioData.TransferEnd = 0;
    sdioData.StopCondition = 1;

    SDIO->DCTRL = 0x0;

    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_STBITERR, ENABLE);
    SD_LowLevel_DMA_RxConfig((uint32_t *)readbuff, (NumberOfBlocks * BlockSize));
    SDIO_DMACmd(ENABLE);

    if (sdioData.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
	BlockSize = 512;

    // Set Block Size for Card
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
	return(errorstatus);

    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
    SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
    SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) 9 << 4;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig(&SDIO_DataInitStructure);

    // Send CMD18 READ_MULT_BLOCK with argument data address
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)ReadSector;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_READ_MULT_BLOCK;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_READ_MULT_BLOCK);

    if (errorstatus != SD_OK)
	return(errorstatus);

    return(errorstatus);
}

// Allows to write one block starting from a specified address in a card.
// The Data transfer can be managed by DMA mode or Polling mode.
SD_Error SD_WriteBlock(uint8_t *writebuff, uint32_t WriteSector, uint16_t BlockSize) {
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;
    SD_Error errorstatus = SD_OK;

#if defined (SD_POLLING_MODE)
    uint32_t bytestransferred = 0, count = 0, restwords = 0;
    uint32_t *tempbuff = (uint32_t *)writebuff;
#endif

    sdioData.TransferError = SD_OK;
    sdioData.TransferEnd = 0;
    sdioData.StopCondition = 0;

    SDIO->DCTRL = 0x0;

#if defined (SD_DMA_MODE)
    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_STBITERR, ENABLE);
    SD_LowLevel_DMA_TxConfig((uint32_t *)writebuff, BlockSize);
    SDIO_DMACmd(ENABLE);
#endif

    if (sdioData.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
	BlockSize = 512;

    // Set Block Size for Card
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
	return(errorstatus);

    // Send CMD24 WRITE_SINGLE_BLOCK
    SDIO_CmdInitStructure.SDIO_Argument = WriteSector;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_SINGLE_BLOCK;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_WRITE_SINGLE_BLOCK);

    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
    SDIO_DataInitStructure.SDIO_DataLength = BlockSize;
    SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) 9 << 4;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig(&SDIO_DataInitStructure);

    // In case of single data block transfer no need of stop command at all
#if defined (SD_POLLING_MODE)
    while (!(SDIO->STA & (SDIO_FLAG_DBCKEND | SDIO_FLAG_TXUNDERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_STBITERR))) {
	if (SDIO_GetFlagStatus(SDIO_FLAG_TXFIFOHE) != RESET) {
	    if ((512 - bytestransferred) < 32) {
		restwords = ((512 - bytestransferred) % 4 == 0) ? ((512 - bytestransferred) / 4) : (( 512 -  bytestransferred) / 4 + 1);
		for (count = 0; count < restwords; count++, tempbuff++, bytestransferred += 4) {
		    SDIO_WriteData(*tempbuff);
		}
	    }
	    else {
		for (count = 0; count < 8; count++) {
		    SDIO_WriteData(*(tempbuff + count));
		}
		tempbuff += 8;
		bytestransferred += 32;
	    }
	}
    }
    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
	errorstatus = SD_DATA_TIMEOUT;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
	errorstatus = SD_DATA_CRC_FAIL;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_TXUNDERR) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_TXUNDERR);
	errorstatus = SD_TX_UNDERRUN;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_STBITERR);
	errorstatus = SD_START_BIT_ERR;
	return(errorstatus);
    }
#endif

    return(errorstatus);
}

// Allows to write blocks starting from a specified address in a card.
// The Data transfer can be managed by DMA mode only.
// This operation should be followed by two functions to check if the
// DMA Controller and SD Card status.
//  - SD_ReadWaitOperation(): this function insure that the DMA
//	controller has finished all data transfer.
//  - SD_GetStatus(): to check that the SD Card has finished the
//	data transfer and it is ready for data.
// WriteAddr: Address from where data are to be read.
// writebuff: pointer to the buffer that contain the data to be transferred.
//  BlockSize: the SD card Data block size. The Block size should be 512.
// NumberOfBlocks: number of blocks to be written.
SD_Error SD_WriteMultiBlocks(uint8_t *writebuff, uint32_t WriteSector, uint16_t BlockSize, uint32_t NumberOfBlocks) {
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;
    SD_Error errorstatus = SD_OK;

    sdioData.TransferError = SD_OK;
    sdioData.TransferEnd = 0;
    sdioData.StopCondition = 1;

    SDIO->DCTRL = 0x0;

    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND | SDIO_IT_RXOVERR | SDIO_IT_STBITERR, ENABLE);
    SD_LowLevel_DMA_TxConfig((uint32_t *)writebuff, (NumberOfBlocks * BlockSize));
    SDIO_DMACmd(ENABLE);

    if (sdioData.CardType == SDIO_HIGH_CAPACITY_SD_CARD)
	BlockSize = 512;

    // Set Block Size for Card
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) BlockSize;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (SD_OK != errorstatus)
	return(errorstatus);

    // To improve performance
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) (sdioData.RCA << 16);
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    if (errorstatus != SD_OK)
	return(errorstatus);

    // To improve performance
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)NumberOfBlocks;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCK_COUNT;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCK_COUNT);

    if (errorstatus != SD_OK)
	return(errorstatus);

    // Send CMD25 WRITE_MULT_BLOCK with argument data address
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t)WriteSector;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_WRITE_MULT_BLOCK;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_WRITE_MULT_BLOCK);

    if (SD_OK != errorstatus)
	return(errorstatus);

    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
    SDIO_DataInitStructure.SDIO_DataLength = NumberOfBlocks * BlockSize;
    SDIO_DataInitStructure.SDIO_DataBlockSize = (uint32_t) 9 << 4;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToCard;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig(&SDIO_DataInitStructure);

    return(errorstatus);
}

// Returns the DMA End Of Transfer Status.
uint32_t SD_DMAEndOfTransferStatus(void) {
    return (uint32_t)DMA_GetFlagStatus(SDIO_DMA_STREAM, SDIO_DMA_FLAG_TCIF);
}

// This function waits until the SDIO DMA data transfer is finished.
// This function should be called after SDIO_WriteBlock() and
// SDIO_WriteMultiBlocks() function to insure that all data sent by the
// card are already transferred by the DMA controller.
SD_Error SD_WaitWriteOperation(void) {
    SD_Error errorstatus = SD_OK;

    while (sdioData.TransferEnd == 0 && sdioData.TransferError == SD_OK)
	yield(1);

    if (sdioData.TransferError != SD_OK)
	return(sdioData.TransferError);
    else
	return(errorstatus);
}


// This function waits until the SDIO DMA data transfer is finished.
// This function should be called after SDIO_ReadMultiBlocks() function
// to insure that all data sent by the card are already transferred by
// the DMA controller.
SD_Error SD_WaitReadOperation(void) {
    SD_Error errorstatus = SD_OK;

    while (sdioData.TransferEnd == 0 && sdioData.TransferError == SD_OK)
	yield(1);

    if (sdioData.TransferError != SD_OK)
	return(sdioData.TransferError);
    else
	return(errorstatus);
}

// Returns the current SD card's status.
SD_Error SD_SendSDStatus(uint32_t *psdstatus) {
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    SDIO_DataInitTypeDef SDIO_DataInitStructure;
    SD_Error errorstatus = SD_OK;
    uint32_t count = 0;

    if (SDIO_GetResponse(SDIO_RESP1) & SD_CARD_LOCKED) {
	errorstatus = SD_LOCK_UNLOCK_FAILED;
	return(errorstatus);
    }

    // Set block size for card if it is not equal to current block size for card.
    SDIO_CmdInitStructure.SDIO_Argument = 64;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SET_BLOCKLEN;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SET_BLOCKLEN);

    if (errorstatus != SD_OK)
	return(errorstatus);

    // CMD55
    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) sdioData.RCA << 16;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_APP_CMD;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SD_CMD_APP_CMD);

    if (errorstatus != SD_OK)
	return(errorstatus);

    SDIO_DataInitStructure.SDIO_DataTimeOut = SD_DATATIMEOUT;
    SDIO_DataInitStructure.SDIO_DataLength = 64;
    SDIO_DataInitStructure.SDIO_DataBlockSize = SDIO_DataBlockSize_64b;
    SDIO_DataInitStructure.SDIO_TransferDir = SDIO_TransferDir_ToSDIO;
    SDIO_DataInitStructure.SDIO_TransferMode = SDIO_TransferMode_Block;
    SDIO_DataInitStructure.SDIO_DPSM = SDIO_DPSM_Enable;
    SDIO_DataConfig(&SDIO_DataInitStructure);

    // Send ACMD13 SD_APP_STAUS  with argument as card's RCA.
    SDIO_CmdInitStructure.SDIO_Argument = 0;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SD_APP_STAUS;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);
    errorstatus = CmdResp1Error(SD_CMD_SD_APP_STAUS);

    if (errorstatus != SD_OK)
	return(errorstatus);

    while (!(SDIO->STA &(SDIO_FLAG_RXOVERR | SDIO_FLAG_DCRCFAIL | SDIO_FLAG_DTIMEOUT | SDIO_FLAG_DBCKEND | SDIO_FLAG_STBITERR))) {
	if (SDIO_GetFlagStatus(SDIO_FLAG_RXFIFOHF) != RESET) {
	    for (count = 0; count < 8; count++) {
		*(psdstatus + count) = SDIO_ReadData();
	    }
	    psdstatus += 8;
	}
	else if (SD_Detect() != SD_PRESENT) {
	    errorstatus = SD_NOT_PRESENT;
	    break;
	}
    }

    if (errorstatus != SD_OK)
	return(errorstatus);

    if (SDIO_GetFlagStatus(SDIO_FLAG_DTIMEOUT) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_DTIMEOUT);
	errorstatus = SD_DATA_TIMEOUT;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_DCRCFAIL) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_DCRCFAIL);
	errorstatus = SD_DATA_CRC_FAIL;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_RXOVERR) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_RXOVERR);
	errorstatus = SD_RX_OVERRUN;
	return(errorstatus);
    }
    else if (SDIO_GetFlagStatus(SDIO_FLAG_STBITERR) != RESET) {
	SDIO_ClearFlag(SDIO_FLAG_STBITERR);
	errorstatus = SD_START_BIT_ERR;
	return(errorstatus);
    }

    count = SD_DATATIMEOUT;
    while ((SDIO_GetFlagStatus(SDIO_FLAG_RXDAVL) != RESET) && (count > 0)) {
	*psdstatus = SDIO_ReadData();
	psdstatus++;
	count--;
    }

    // Clear all the static status flags
    SDIO_ClearFlag(SDIO_STATIC_FLAGS);

    return(errorstatus);
}

SD_Error SD_GetCardStatus(SD_CardStatus *cardstatus) {
    SD_Error errorstatus = SD_OK;
    uint8_t tmp = 0;

    errorstatus = SD_SendSDStatus((uint32_t *)sdioData.SDSTATUS_Tab);

    if (errorstatus  != SD_OK)
	return(errorstatus);

    /*!< Byte 0 */
    tmp = (uint8_t)((sdioData.SDSTATUS_Tab[0] & 0xC0) >> 6);
    cardstatus->DAT_BUS_WIDTH = tmp;

    /*!< Byte 0 */
    tmp = (uint8_t)((sdioData.SDSTATUS_Tab[0] & 0x20) >> 5);
    cardstatus->SECURED_MODE = tmp;

    /*!< Byte 2 */
    tmp = (uint8_t)((sdioData.SDSTATUS_Tab[2] & 0xFF));
    cardstatus->SD_CARD_TYPE = tmp << 8;

    /*!< Byte 3 */
    tmp = (uint8_t)((sdioData.SDSTATUS_Tab[3] & 0xFF));
    cardstatus->SD_CARD_TYPE |= tmp;

    /*!< Byte 4 */
    tmp = (uint8_t)(sdioData.SDSTATUS_Tab[4] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA = tmp << 24;

    /*!< Byte 5 */
    tmp = (uint8_t)(sdioData.SDSTATUS_Tab[5] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 16;

    /*!< Byte 6 */
    tmp = (uint8_t)(sdioData.SDSTATUS_Tab[6] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA |= tmp << 8;

    /*!< Byte 7 */
    tmp = (uint8_t)(sdioData.SDSTATUS_Tab[7] & 0xFF);
    cardstatus->SIZE_OF_PROTECTED_AREA |= tmp;

    /*!< Byte 8 */
    tmp = (uint8_t)((sdioData.SDSTATUS_Tab[8] & 0xFF));
    cardstatus->SPEED_CLASS = tmp;

    /*!< Byte 9 */
    tmp = (uint8_t)((sdioData.SDSTATUS_Tab[9] & 0xFF));
    cardstatus->PERFORMANCE_MOVE = tmp;

    /*!< Byte 10 */
    tmp = (uint8_t)((sdioData.SDSTATUS_Tab[10] & 0xF0) >> 4);
    cardstatus->AU_SIZE = tmp;

    /*!< Byte 11 */
    tmp = (uint8_t)(sdioData.SDSTATUS_Tab[11] & 0xFF);
    cardstatus->ERASE_SIZE = tmp << 8;

    /*!< Byte 12 */
    tmp = (uint8_t)(sdioData.SDSTATUS_Tab[12] & 0xFF);
    cardstatus->ERASE_SIZE |= tmp;

    /*!< Byte 13 */
    tmp = (uint8_t)((sdioData.SDSTATUS_Tab[13] & 0xFC) >> 2);
    cardstatus->ERASE_TIMEOUT = tmp;

    /*!< Byte 13 */
    tmp = (uint8_t)((sdioData.SDSTATUS_Tab[13] & 0x3));
    cardstatus->ERASE_OFFSET = tmp;

    return(errorstatus);
}

// Returns the current card's status.
SD_Error SD_SendStatus(uint32_t *pcardstatus) {
    SDIO_CmdInitTypeDef SDIO_CmdInitStructure;
    SD_Error errorstatus = SD_OK;

    if (pcardstatus == NULL) {
	errorstatus = SD_INVALID_PARAMETER;

	return(errorstatus);
    }

    SDIO_CmdInitStructure.SDIO_Argument = (uint32_t) sdioData.RCA << 16;
    SDIO_CmdInitStructure.SDIO_CmdIndex = SD_CMD_SEND_STATUS;
    SDIO_CmdInitStructure.SDIO_Response = SDIO_Response_Short;
    SDIO_CmdInitStructure.SDIO_Wait = SDIO_Wait_No;
    SDIO_CmdInitStructure.SDIO_CPSM = SDIO_CPSM_Enable;
    SDIO_SendCommand(&SDIO_CmdInitStructure);

    errorstatus = CmdResp1Error(SD_CMD_SEND_STATUS);

    if (errorstatus != SD_OK)
	return(errorstatus);

    *pcardstatus = SDIO_GetResponse(SDIO_RESP1);

    return(errorstatus);
}

// Returns the current card's state.
SDCardState SD_GetState(void) {
    uint32_t resp1 = 0;

    if (SD_Detect() == SD_PRESENT) {
	if (SD_SendStatus(&resp1) != SD_OK) {
	    return SD_CARD_ERROR;
	}
	else {
	    return (SDCardState)((resp1 >> 9) & 0x0F);
	}
    }
    else {
	return SD_CARD_ERROR;
    }
}

// Gets the cuurent sd card data transfer status.
// SDTransferState: Data Transfer state.
//  This value can be:
//	- SD_TRANSFER_OK: No data transfer is acting
//	- SD_TRANSFER_BUSY: Data transfer is acting
SDTransferState SD_GetStatus(void) {
    SDCardState cardstate =  SD_CARD_TRANSFER;

    cardstate = SD_GetState();

    if (cardstate == SD_CARD_TRANSFER) {
	return(SD_TRANSFER_OK);
    }
    else if(cardstate == SD_CARD_ERROR) {
	return (SD_TRANSFER_ERROR);
    }
    else {
	return(SD_TRANSFER_BUSY);
    }
}

DWORD get_fattime(void) {
    return rtcGetDateTime();
}

DSTATUS disk_initialize(BYTE drv) {
    if (drv)
	return STA_NOINIT;	// Supports only single drive

    if (SD_DetectLowLevel() == SD_NOT_PRESENT)
	return STA_NOINIT;	// No card in the socket

    if (sdioInit() != SD_OK)
	return STA_NOINIT;

    sdioData.initialized = 1;

    return SD_OK;
}

DSTATUS disk_status(BYTE drv) {
    if (drv)
	return STA_NOINIT;	// Supports only single drive

    if (SD_Detect() == SD_NOT_PRESENT)
	return STA_NODISK;	// No card in the socket

    if (!sdioData.initialized)
	return STA_NOINIT;

    return SD_OK;
}

DRESULT disk_read(
	BYTE drv,		// Physical drive number (0)
	BYTE *buff,		// Pointer to the data buffer to store read data
	DWORD sector,		// Start sector number (LBA)
	BYTE count		// Sector count (1..255)
	) {
    SD_Error error = SD_OK;
    int tries = 0;

    if (drv || !count)
	return RES_PARERR;

    if (!sdioData.initialized)
	return RES_NOTRDY;	// No card in the socket

    if (SD_Detect() == SD_NOT_PRESENT)
	return RES_NOTRDY;	// No card in the socket

    do {
	if (error != SD_OK) {
	    AQ_NOTICE("SDIO WRITE error != SD_OK\n");
	    sdioData.errCount++;
	}

	if (count > 1)
	    error = SD_ReadMultiBlocks(buff, sector, 512, count);
	else
	    error = SD_ReadBlock(buff, sector, 512);

	if (error == SD_OK)
	    error = SD_WaitReadOperation();

	while (SD_GetStatus() == SD_TRANSFER_BUSY)
	    yield(1);

	tries++;
    } while (error != SD_OK  && tries < SDIO_RETRIES);

    if (error != SD_OK)
	return RES_ERROR;
    else
	return RES_OK;
}

DRESULT disk_write (
	BYTE drv,			// Physical drive number (0)
	const BYTE *buff,		// Pointer to the data to be written
	DWORD sector,			// Start sector number (LBA)
	BYTE count			// Sector count (1..255)
	) {
    SD_Error error = SD_OK;
    int tries = 0;

    if (drv || !count)
	return RES_PARERR;

    if (!sdioData.initialized)
	return RES_NOTRDY;	// No card in the socket

    if (SD_Detect() == SD_NOT_PRESENT)
	return RES_NOTRDY;	// No card in the socket

    do {
	if (error != SD_OK) {
	    AQ_NOTICE("SDIO WRITE error != SD_OK\n");
	    sdioData.errCount++;
	}

	if (count > 1)
	    error = SD_WriteMultiBlocks((uint8_t *)buff, sector, 512, count);
	else
	    error = SD_WriteBlock((uint8_t *)buff, sector, 512);

	if (error == SD_OK)
	    error = SD_WaitWriteOperation();

	// wait for any previous writes to finish
	while (SD_GetStatus() == SD_TRANSFER_BUSY)
	    yield(1);

	tries++;
    } while (error != SD_OK && tries < SDIO_RETRIES);

    if (error != SD_OK)
	return RES_ERROR;
    else
	return RES_OK;
}

DRESULT disk_ioctl (
	BYTE drv,		// Physical drive number (0)
	BYTE ctrl,		// Control code
	void *buff		// Buffer to send/receive control data
	) {
    DRESULT res;

    if (drv)
	return RES_PARERR;

    if (!sdioData.initialized)
	return RES_NOTRDY;

    if (SD_Detect() == SD_NOT_PRESENT)
	return RES_NOTRDY;	// No card in the socket

    res = RES_ERROR;

    switch (ctrl) {
    case CTRL_SYNC :		// Make sure that no pending write process
	while (SD_GetStatus() == SD_TRANSFER_BUSY)
	    ;
	res = RES_OK;
	break;

    case GET_SECTOR_COUNT :	// Get number of sectors on the disk (unsigned long)
	*(unsigned long*)buff = (sdioData.SDCardInfo.SD_csd.DeviceSize + 1) * 1024;
	res = RES_OK;
	break;

    case GET_SECTOR_SIZE :	// Get R/W sector size (unsigned short)
	*(unsigned short*)buff = sdioData.SDCardInfo.CardBlockSize;
	res = RES_OK;
	break;

    // TODO
    case GET_BLOCK_SIZE :	// Get erase block size in unit of sector (unsigned long)
	if (SD_GetCardStatus(&sdioData.SDCardStatus) == SD_OK) {
	    *(unsigned long*)buff = (unsigned long)(0x4000) * sdioData.SDCardStatus.AU_SIZE;
	    res = RES_OK;
	}
	break;

    default:
	res = RES_PARERR;
	break;
    }

    return res;
}

void sdioSetCallback(sdioCallback_t *func, uint32_t param) {
    sdioData.callbackFunc = func;
    sdioData.callbackParam = param;
}

void SD_ProcessDMAIRQ(void) {
    DMA_ClearFlag(SDIO_DMA_STREAM, SDIO_DMA_FLAG_FEIF | SDIO_DMA_FLAG_DMEIF | SDIO_DMA_FLAG_TEIF | SDIO_DMA_FLAG_HTIF | SDIO_DMA_FLAG_TCIF);

    sdioData.DMAEndOfTransfer = 0x01;
}

// Allows to process all the interrupts that are high.
SD_Error SD_ProcessIRQSrc(void) {
    if (SDIO_GetITStatus(SDIO_IT_DATAEND) != RESET) {
	sdioData.TransferError = SD_OK;
	SDIO_ClearITPendingBit(SDIO_IT_DATAEND);
    }
    else if (SDIO_GetITStatus(SDIO_IT_DCRCFAIL) != RESET) {
	SDIO_ClearITPendingBit(SDIO_IT_DCRCFAIL);
	sdioData.TransferError = SD_DATA_CRC_FAIL;
    }
    else if (SDIO_GetITStatus(SDIO_IT_DTIMEOUT) != RESET) {
	SDIO_ClearITPendingBit(SDIO_IT_DTIMEOUT);
	sdioData.TransferError = SD_DATA_TIMEOUT;
    }
    else if (SDIO_GetITStatus(SDIO_IT_RXOVERR) != RESET) {
	SDIO_ClearITPendingBit(SDIO_IT_RXOVERR);
	sdioData.TransferError = SD_RX_OVERRUN;
    }
    else if (SDIO_GetITStatus(SDIO_IT_TXUNDERR) != RESET) {
	SDIO_ClearITPendingBit(SDIO_IT_TXUNDERR);
	sdioData.TransferError = SD_TX_UNDERRUN;
    }
    else if (SDIO_GetITStatus(SDIO_IT_STBITERR) != RESET) {
	SDIO_ClearITPendingBit(SDIO_IT_STBITERR);
	sdioData.TransferError = SD_START_BIT_ERR;
    }

    SDIO_ITConfig(SDIO_IT_DCRCFAIL | SDIO_IT_DTIMEOUT | SDIO_IT_DATAEND |
		    SDIO_IT_TXFIFOHE | SDIO_IT_RXFIFOHF | SDIO_IT_TXUNDERR |
		    SDIO_IT_RXOVERR | SDIO_IT_STBITERR, DISABLE);

    if (sdioData.StopCondition == 1) {
	SD_StopTransfer();
	sdioData.StopCondition = 0;
    }

    sdioData.TransferEnd = 1;

    if (sdioData.callbackFunc) {
	if (sdioData.TransferError == SD_OK)
	    sdioData.callbackFunc(sdioData.callbackParam);
	else
	    sdioData.callbackFunc(0);

	sdioData.callbackFunc = 0;
    }

    return (sdioData.TransferError);
}

int SD_Initialized(void) {
    return sdioData.initialized;
}

int SD_TransferComplete(void) {
    return sdioData.TransferEnd;
}

// Process All SDIO Interrupt Sources
void SDIO_IRQHandler(void) {
    SD_ProcessIRQSrc();
}

void SDIO_DMA_IRQHANDLER(void) {
    SD_ProcessDMAIRQ();
}
