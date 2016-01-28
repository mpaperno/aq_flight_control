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
#include "canCalib.h"
#include "can.h"
#include "imu.h"

canCalibStruct_t canCalibData;

void canCalibInit(void) {
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    canCalibData.lastTxMailbox = -1;

    canLowLevelInit();

    CAN_FilterInitStructure.CAN_FilterNumber = 0;
    CAN_FilterInitStructure.CAN_FilterMode = CAN_FilterMode_IdMask;
    CAN_FilterInitStructure.CAN_FilterScale = CAN_FilterScale_32bit;
    CAN_FilterInitStructure.CAN_FilterIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdHigh = 0x0000;
    CAN_FilterInitStructure.CAN_FilterMaskIdLow = 0x0000;
    CAN_FilterInitStructure.CAN_FilterFIFOAssignment = 0;
    CAN_FilterInitStructure.CAN_FilterActivation = ENABLE;
    CAN_FilterInit(&CAN_FilterInitStructure);

    canCalibData.initialized = 1;
}

void canTxIMUData(uint32_t loop) {
    uint8_t *ptr;
    float value;

    if (canCalibData.initialized) {
	canCalibData.txMessage.ExtId = 0x01;
	canCalibData.txMessage.RTR = CAN_RTR_DATA;
	canCalibData.txMessage.IDE = CAN_ID_STD;
	canCalibData.txMessage.DLC = 8;		// two floats

	// odd - can only stuff 3 mailboxes at a time
	if (loop & 0x1) {
	    // ACCX & ACCY
	    canCalibData.txMessage.StdId = CAN_IMU_VALUES_1 | CAN_ID;

	    ptr = (uint8_t *)&IMU_RAW_ACCX;
	    canCalibData.txMessage.Data[0] = *ptr++;
	    canCalibData.txMessage.Data[1] = *ptr++;
	    canCalibData.txMessage.Data[2] = *ptr++;
	    canCalibData.txMessage.Data[3] = *ptr++;

	    ptr = (uint8_t *)&IMU_RAW_ACCY;
	    canCalibData.txMessage.Data[4] = *ptr++;
	    canCalibData.txMessage.Data[5] = *ptr++;
	    canCalibData.txMessage.Data[6] = *ptr++;
	    canCalibData.txMessage.Data[7] = *ptr++;

	    canCalibData.lastTxMailbox = CAN_Transmit(CANx, &canCalibData.txMessage);

	    // ACCZ & MAGX
	    canCalibData.txMessage.StdId = CAN_IMU_VALUES_2 | CAN_ID;

	    ptr = (uint8_t *)&IMU_RAW_ACCZ;
	    canCalibData.txMessage.Data[0] = *ptr++;
	    canCalibData.txMessage.Data[1] = *ptr++;
	    canCalibData.txMessage.Data[2] = *ptr++;
	    canCalibData.txMessage.Data[3] = *ptr++;

#ifdef HAS_DIGITAL_IMU
	    value = IMU_RAW_MAGX;
#else
	    value = (IMU_RAW_MAGX - adcData.magBridgeBiasX) * (float)adcData.magSign;
#endif

	    ptr = (uint8_t *)&value;
	    canCalibData.txMessage.Data[4] = *ptr++;
	    canCalibData.txMessage.Data[5] = *ptr++;
	    canCalibData.txMessage.Data[6] = *ptr++;
	    canCalibData.txMessage.Data[7] = *ptr++;

	    canCalibData.lastTxMailbox = CAN_Transmit(CANx, &canCalibData.txMessage);

	    // MAGY & MAGZ
	    canCalibData.txMessage.StdId = CAN_IMU_VALUES_3 | CAN_ID;

#ifdef HAS_DIGITAL_IMU
	    value = IMU_RAW_MAGY;
#else
	    value = (IMU_RAW_MAGY - adcData.magBridgeBiasY) * (float)adcData.magSign;
#endif

	    ptr = (uint8_t *)&value;
	    canCalibData.txMessage.Data[0] = *ptr++;
	    canCalibData.txMessage.Data[1] = *ptr++;
	    canCalibData.txMessage.Data[2] = *ptr++;
	    canCalibData.txMessage.Data[3] = *ptr++;

#ifdef HAS_DIGITAL_IMU
	    value = IMU_RAW_MAGZ;
#else
	    value = (IMU_RAW_MAGZ - adcData.magBridgeBiasZ) * (float)adcData.magSign;
#endif

	    ptr = (uint8_t *)&value;
	    canCalibData.txMessage.Data[4] = *ptr++;
	    canCalibData.txMessage.Data[5] = *ptr++;
	    canCalibData.txMessage.Data[6] = *ptr++;
	    canCalibData.txMessage.Data[7] = *ptr++;

	    canCalibData.lastTxMailbox = CAN_Transmit(CANx, &canCalibData.txMessage);
	}
	// even
	else {
	    // GYOX & GYOY
	    canCalibData.txMessage.StdId = CAN_IMU_VALUES_4 | CAN_ID;

	    ptr = (uint8_t *)&IMU_RAW_RATEX;
	    canCalibData.txMessage.Data[0] = *ptr++;
	    canCalibData.txMessage.Data[1] = *ptr++;
	    canCalibData.txMessage.Data[2] = *ptr++;
	    canCalibData.txMessage.Data[3] = *ptr++;

	    ptr = (uint8_t *)&IMU_RAW_RATEY;
	    canCalibData.txMessage.Data[4] = *ptr++;
	    canCalibData.txMessage.Data[5] = *ptr++;
	    canCalibData.txMessage.Data[6] = *ptr++;
	    canCalibData.txMessage.Data[7] = *ptr++;

	    canCalibData.lastTxMailbox = CAN_Transmit(CANx, &canCalibData.txMessage);

	    // GYOZ & TEMP
	    canCalibData.txMessage.StdId = CAN_IMU_VALUES_5 | CAN_ID;

	    ptr = (uint8_t *)&IMU_RAW_RATEZ;
	    canCalibData.txMessage.Data[0] = *ptr++;
	    canCalibData.txMessage.Data[1] = *ptr++;
	    canCalibData.txMessage.Data[2] = *ptr++;
	    canCalibData.txMessage.Data[3] = *ptr++;

	    ptr = (uint8_t *)&IMU_TEMP;
	    canCalibData.txMessage.Data[4] = *ptr++;
	    canCalibData.txMessage.Data[5] = *ptr++;
	    canCalibData.txMessage.Data[6] = *ptr++;
	    canCalibData.txMessage.Data[7] = *ptr++;

	    canCalibData.lastTxMailbox = CAN_Transmit(CANx, &canCalibData.txMessage);
	}
    }
}
