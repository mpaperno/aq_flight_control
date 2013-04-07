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
#include "config.h"
#include "can.h"
#include "supervisor.h"
#include "imu.h"

canStruct_t canData;

void canInit(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    CAN_InitTypeDef CAN_InitStructure;
    CAN_FilterInitTypeDef CAN_FilterInitStructure;
    NVIC_InitTypeDef  NVIC_InitStructure;

    canData.lastTxMailbox = -1;

    // Connect CAN pins to AF9
    GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_RX_SOURCE, CAN_AF_PORT);
    GPIO_PinAFConfig(CAN_GPIO_PORT, CAN_TX_SOURCE, CAN_AF_PORT);

    // Configure CAN RX and TX pins
    GPIO_InitStructure.GPIO_Pin = CAN_RX_PIN | CAN_TX_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd  = GPIO_PuPd_UP;
    GPIO_Init(CAN_GPIO_PORT, &GPIO_InitStructure);

    RCC_APB1PeriphClockCmd(CAN_CLK, ENABLE);

    // CAN register init
    CAN_DeInit(CANx);

    // CAN cell init
    CAN_InitStructure.CAN_TTCM = DISABLE;
    CAN_InitStructure.CAN_ABOM = ENABLE;
    CAN_InitStructure.CAN_AWUM = DISABLE;
    CAN_InitStructure.CAN_NART = DISABLE;
    CAN_InitStructure.CAN_RFLM = DISABLE;
    CAN_InitStructure.CAN_TXFP = DISABLE;
    CAN_InitStructure.CAN_Mode = CAN_Mode_Normal;
    CAN_InitStructure.CAN_SJW = CAN_SJW_2tq;

    // CAN Baudrate = ~1 Mbps (CAN clocked at 42 MHz)
    CAN_InitStructure.CAN_BS1 = CAN_BS1_5tq; //CAN_BS1_6tq;
    CAN_InitStructure.CAN_BS2 = CAN_BS2_8tq;
    CAN_InitStructure.CAN_Prescaler = 3;
    CAN_Init(CANx, &CAN_InitStructure);

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

    NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0x0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Enable FIFO 0 message pending Interrupt
//    CAN_ITConfig(CANx, CAN_IT_FMP0, ENABLE);

    canData.initialized = 1;
}

void canTxIMUData(uint32_t loop) {
    uint8_t *ptr;
    float value;

    if (canData.initialized) {
	canData.txMessage.ExtId = 0x01;
	canData.txMessage.RTR = CAN_RTR_DATA;
	canData.txMessage.IDE = CAN_ID_STD;
	canData.txMessage.DLC = 8;		// two floats

	// odd - can only stuff 3 mailboxes at a time
	if (loop & 0x1) {
	    // ACCX & ACCY
	    canData.txMessage.StdId = CAN_IMU_VALUES_1 | CAN_ID;

#ifdef HAS_DIGITAL_IMU
	    ptr = (uint8_t *)&mpu6000Data.rawAcc[0];
#else
	    ptr = (uint8_t *)&adcData.voltages[ADC_VOLTS_ACCX];
#endif
	    canData.txMessage.Data[0] = *ptr++;
	    canData.txMessage.Data[1] = *ptr++;
	    canData.txMessage.Data[2] = *ptr++;
	    canData.txMessage.Data[3] = *ptr++;

#ifdef HAS_DIGITAL_IMU
	    ptr = (uint8_t *)&mpu6000Data.rawAcc[1];
#else
	    ptr = (uint8_t *)&adcData.voltages[ADC_VOLTS_ACCY];
#endif
	    canData.txMessage.Data[4] = *ptr++;
	    canData.txMessage.Data[5] = *ptr++;
	    canData.txMessage.Data[6] = *ptr++;
	    canData.txMessage.Data[7] = *ptr++;

	    canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);

	    // ACCZ & MAGX
	    canData.txMessage.StdId = CAN_IMU_VALUES_2 | CAN_ID;

#ifdef HAS_DIGITAL_IMU
	    ptr = (uint8_t *)&mpu6000Data.rawAcc[2];
#else
	    ptr = (uint8_t *)&adcData.voltages[ADC_VOLTS_ACCZ];
#endif
	    canData.txMessage.Data[0] = *ptr++;
	    canData.txMessage.Data[1] = *ptr++;
	    canData.txMessage.Data[2] = *ptr++;
	    canData.txMessage.Data[3] = *ptr++;

#ifdef HAS_DIGITAL_IMU
	    value = hmc5983Data.rawMag[0];
#else
	    value = (adcData.voltages[ADC_VOLTS_MAGX] - adcData.magBridgeBiasX) * (float)adcData.magSign;
#endif

	    ptr = (uint8_t *)&value;
	    canData.txMessage.Data[4] = *ptr++;
	    canData.txMessage.Data[5] = *ptr++;
	    canData.txMessage.Data[6] = *ptr++;
	    canData.txMessage.Data[7] = *ptr++;

	    canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);

	    // MAGY & MAGZ
	    canData.txMessage.StdId = CAN_IMU_VALUES_3 | CAN_ID;

#ifdef HAS_DIGITAL_IMU
	    value = hmc5983Data.rawMag[1];
#else
	    value = (adcData.voltages[ADC_VOLTS_MAGY] - adcData.magBridgeBiasY) * (float)adcData.magSign;
#endif

	    ptr = (uint8_t *)&value;
	    canData.txMessage.Data[0] = *ptr++;
	    canData.txMessage.Data[1] = *ptr++;
	    canData.txMessage.Data[2] = *ptr++;
	    canData.txMessage.Data[3] = *ptr++;

#ifdef HAS_DIGITAL_IMU
	    value = hmc5983Data.rawMag[2];
#else
	    value = (adcData.voltages[ADC_VOLTS_MAGZ] - adcData.magBridgeBiasZ) * (float)adcData.magSign;
#endif

	    ptr = (uint8_t *)&value;
	    canData.txMessage.Data[4] = *ptr++;
	    canData.txMessage.Data[5] = *ptr++;
	    canData.txMessage.Data[6] = *ptr++;
	    canData.txMessage.Data[7] = *ptr++;

	    canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);
	}
	// even
	else {
	    // GYOX & GYOY
	    canData.txMessage.StdId = CAN_IMU_VALUES_4 | CAN_ID;

#ifdef HAS_DIGITAL_IMU
	    ptr = (uint8_t *)&mpu6000Data.rawGyo[0];
#else
	    ptr = (uint8_t *)&adcData.voltages[ADC_VOLTS_RATEX];
#endif
	    canData.txMessage.Data[0] = *ptr++;
	    canData.txMessage.Data[1] = *ptr++;
	    canData.txMessage.Data[2] = *ptr++;
	    canData.txMessage.Data[3] = *ptr++;

#ifdef HAS_DIGITAL_IMU
	    ptr = (uint8_t *)&mpu6000Data.rawGyo[1];
#else
	    ptr = (uint8_t *)&adcData.voltages[ADC_VOLTS_RATEY];
#endif
	    canData.txMessage.Data[4] = *ptr++;
	    canData.txMessage.Data[5] = *ptr++;
	    canData.txMessage.Data[6] = *ptr++;
	    canData.txMessage.Data[7] = *ptr++;

	    canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);

	    // GYOZ & TEMP
	    canData.txMessage.StdId = CAN_IMU_VALUES_5 | CAN_ID;

#ifdef HAS_DIGITAL_IMU
	    ptr = (uint8_t *)&mpu6000Data.rawGyo[2];
#else
	    ptr = (uint8_t *)&adcData.voltages[ADC_VOLTS_RATEZ];
#endif
	    canData.txMessage.Data[0] = *ptr++;
	    canData.txMessage.Data[1] = *ptr++;
	    canData.txMessage.Data[2] = *ptr++;
	    canData.txMessage.Data[3] = *ptr++;

	    ptr = (uint8_t *)&IMU_TEMP;
	    canData.txMessage.Data[4] = *ptr++;
	    canData.txMessage.Data[5] = *ptr++;
	    canData.txMessage.Data[6] = *ptr++;
	    canData.txMessage.Data[7] = *ptr++;

	    canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);
	}
    }
}

void canTxAxisTargetPos(uint8_t axisId, float pos) {
    uint8_t *ptr = (uint8_t *)&pos;

    if (canData.initialized) {
	canData.txMessage.StdId = CAN_AXIS_TARGET_POS | axisId;
	canData.txMessage.ExtId = 0x01;
	canData.txMessage.RTR = CAN_RTR_DATA;
	canData.txMessage.IDE = CAN_ID_STD;

	canData.txMessage.DLC = 4;		// one float
	canData.txMessage.Data[0] = *ptr++;
	canData.txMessage.Data[1] = *ptr++;
	canData.txMessage.Data[2] = *ptr++;
	canData.txMessage.Data[3] = *ptr++;
	canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);
    }
}

void canTxAxisTargetVel(uint8_t axisId, float vel) {
    uint8_t *ptr = (uint8_t *)&vel;

    if (canData.initialized) {
	canData.txMessage.StdId = CAN_AXIS_TARGET_VEL | axisId;
	canData.txMessage.ExtId = 0x01;
	canData.txMessage.RTR = CAN_RTR_DATA;
	canData.txMessage.IDE = CAN_ID_STD;

	canData.txMessage.DLC = 4;		// one float
	canData.txMessage.Data[0] = *ptr++;
	canData.txMessage.Data[1] = *ptr++;
	canData.txMessage.Data[2] = *ptr++;
	canData.txMessage.Data[3] = *ptr++;

	canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);
    }
}

void canTxArm(uint8_t axisId) {
    if (canData.initialized) {
	canData.txMessage.StdId = CAN_ARM | axisId;
	canData.txMessage.ExtId = 0x01;
	canData.txMessage.RTR = CAN_RTR_DATA;
	canData.txMessage.IDE = CAN_ID_STD;

	canData.txMessage.DLC = 0;

	canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);
    }
}

void canTxDisarm(uint8_t axisId) {
    if (canData.initialized) {
	canData.txMessage.StdId = CAN_DISARM | axisId;
	canData.txMessage.ExtId = 0x01;
	canData.txMessage.RTR = CAN_RTR_DATA;
	canData.txMessage.IDE = CAN_ID_STD;

	canData.txMessage.DLC = 0;

	canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);
    }
}

void canTxReqParam(uint8_t axisId, int paramId, canParamCallback_t *callback) {
    if (canData.initialized) {
	canData.txMessage.StdId = CAN_GET_PARAM | axisId;
	canData.txMessage.ExtId = 0x01;
	canData.txMessage.RTR = CAN_RTR_DATA;
	canData.txMessage.IDE = CAN_ID_STD;

	canData.txMessage.DLC = 1;		// param id
	canData.txMessage.Data[0] = paramId;

	canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);

	canData.paramCallback = callback;
    }
}

void canTxSetParam(uint8_t axisId, int paramId, float value) {
    uint8_t *ptr = (uint8_t *)&value;

    if (canData.initialized) {
	canData.txMessage.StdId = CAN_SET_PARAM | axisId;
	canData.txMessage.ExtId = 0x01;
	canData.txMessage.RTR = CAN_RTR_DATA;
	canData.txMessage.IDE = CAN_ID_STD;

	canData.txMessage.DLC = 4+1;		// one float + param id
	canData.txMessage.Data[0] = *ptr++;
	canData.txMessage.Data[1] = *ptr++;
	canData.txMessage.Data[2] = *ptr++;
	canData.txMessage.Data[3] = *ptr++;
	canData.txMessage.Data[4] = paramId;

	canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);
    }
}

void canSendParamId(uint8_t id) {
    uint8_t *ptr = (uint8_t *)&p[id];

    if (canData.initialized && id < CONFIG_NUM_PARAMS) {
	canData.txMessage.StdId = CAN_PARAM_DATA | CAN_ID;
	canData.txMessage.ExtId = 0x01;
	canData.txMessage.RTR = CAN_RTR_DATA;
	canData.txMessage.IDE = CAN_ID_STD;

	canData.txMessage.DLC = 4+1;		// one float + param id
	canData.txMessage.Data[0] = *ptr++;
	canData.txMessage.Data[1] = *ptr++;
	canData.txMessage.Data[2] = *ptr++;
	canData.txMessage.Data[3] = *ptr++;
	canData.txMessage.Data[4] = id;

	canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);
    }
}

void canSendAck(uint32_t messId, uint8_t ack) {
    if (canData.initialized) {
	canData.txMessage.StdId = messId | CAN_ID;
	canData.txMessage.ExtId = 0x01;
	canData.txMessage.RTR = CAN_RTR_DATA;
	canData.txMessage.IDE = CAN_ID_STD;

	canData.txMessage.DLC = 1;		// ack / nack
	canData.txMessage.Data[0] = ack;

	canData.lastTxMailbox = CAN_Transmit(CANx, &canData.txMessage);
    }
}

void canCheckMessage(void) {
    uint32_t messId;
    uint8_t node;

    if (canData.initialized) {
	if (CAN_MessagePending(CANx, CAN_FIFO0) > 0) {
	    CAN_Receive(CANx, CAN_FIFO0, &canData.rxMessage);

	    messId = canData.rxMessage.StdId & 0xff0;
	    node = canData.rxMessage.StdId & 0xf;

	    // pos / vel report
	    if (messId == CAN_AXIS_STATUS) {
		// Ignore for now
	    }
	    // arm request
	    else if (messId == CAN_ARM) {
		if (node == CAN_ID)
		    supervisorArm();
	    }
	    // disarm request
	    else if (messId == CAN_DISARM) {
		if (node == CAN_ID)
		    supervisorDisarm();
	    }
	    // set param
	    else if (messId == CAN_SET_PARAM) {
		if (node == CAN_ID) {
		    uint8_t id;

		    id = canData.rxMessage.Data[4];

		    if (!(supervisorData.state & STATE_FLYING)) {
			float value;
			uint8_t *ptr = (uint8_t *)&value;

			if (id < CONFIG_NUM_PARAMS) {
			    *ptr++ = canData.rxMessage.Data[0];
			    *ptr++ = canData.rxMessage.Data[1];
			    *ptr++ = canData.rxMessage.Data[2];
			    *ptr++ = canData.rxMessage.Data[3];

			    configSetParamByID(id, value);
			}
		    }
		    canSendParamId(id);
		}
	    }
	    // get param
	    else if (messId == CAN_GET_PARAM) {
		if (node == CAN_ID)
		    canSendParamId(canData.rxMessage.Data[0]);
	    }
	    else if (messId == CAN_CONFIG_READ) {
		if (node == CAN_ID) {
		    uint8_t ack = 0;

		    if (!(supervisorData.state & STATE_FLYING)) {
			configFlashRead();
			ack = 1;
		    }
		    canSendAck(messId, ack);
		}
	    }
	    else if (messId == CAN_CONFIG_WRITE) {
		if (node == CAN_ID) {
		    uint8_t ack = 0;

		    if (!(supervisorData.state & STATE_FLYING)) {
			ack = configFlashWrite();
		    }
		    canSendAck(messId, ack);
		}
	    }
	    else if (messId == CAN_CONFIG_DEFAULT) {
		if (node == CAN_ID) {
		    uint8_t ack = 0;

		    if (!(supervisorData.state & STATE_FLYING)) {
			configLoadDefault();
			ack = 1;
		    }
		    canSendAck(messId, ack);
		}
	    }
	    else if (messId == CAN_PARAM_DATA) {
		uint8_t id;
		float value;
		uint8_t *ptr = (uint8_t *)&value;

		id = canData.rxMessage.Data[4];

		*ptr++ = canData.rxMessage.Data[0];
		*ptr++ = canData.rxMessage.Data[1];
		*ptr++ = canData.rxMessage.Data[2];
		*ptr++ = canData.rxMessage.Data[3];

		if (canData.paramCallback)
		    canData.paramCallback(id, value);
	    }
	}
    }
}

void canSetConstants(void) {
}

void CAN1_RX0_IRQHandler(void)  {
}
