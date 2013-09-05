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
#include "esc32.h"
#include "1wire.h"
#include "comm.h"
#include "filer.h"
#include "util.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

#ifndef NAN
#define NAN	__float32_nan
#endif

const char *esc32ParameterStrings[] = {
    "CONFIG_VERSION",
    "STARTUP_MODE",
    "BAUD_RATE",
    "PTERM",
    "ITERM",
    "FF1TERM",
    "FF2TERM",
    "CL1TERM",
    "CL2TERM",
    "CL3TERM",
    "CL4TERM",
    "CL5TERM",
    "SHUNT_RESISTANCE",
    "MIN_PERIOD",
    "MAX_PERIOD",
    "BLANKING_MICROS",
    "ADVANCE",
    "START_VOLTAGE",
    "GOOD_DETECTS_START",
    "BAD_DETECTS_DISARM",
    "MAX_CURRENT",
    "SWITCH_FREQ",
    "MOTOR_POLES",
    "PWM_MIN_PERIOD",
    "PWM_MAX_PERIOD",
    "PWM_MIN_VALUE",
    "PWM_LO_VALUE",
    "PWM_HI_VALUE",
    "PWM_MAX_VALUE",
    "PWM_MIN_START",
    "PWM_RPM_SCALE",
    "FET_BRAKING",
    "PNFAC",
    "INFAC",
    "THR1TERM",
    "THR2TERM",
    "START_ALIGN_TIME",
    "START_ALIGN_VOLTAGE",
    "START_STEPS_NUM",
    "START_STEPS_PERIOD",
    "START_STEPS_ACCEL",
    "PWM_LOWPASS",
    "RPM_MEAS_LP",
    "SERVO_DUTY",
    "SERVO_P",
    "SERVO_D",
    "SERVO_MAX_RATE",
    "SERVO_SCALE",
    "ESC_ID",
    "DIRECTION"
};

static float esc32ReadParamTransaction(uint8_t paramId) {
    uint8_t *p;
    float value;

    yield(5);

    owData.buf[0] = OW_PARAM_READ;
    owData.buf[1] = paramId;
    owTransaction(2, 6);

    p = (uint8_t *)&value;
    p[0] = owData.buf[2];
    p[1] = owData.buf[3];
    p[2] = owData.buf[4];
    p[3] = owData.buf[5];

    return value;
}

// require parameter to be read twice the same
static float esc32ReadParam(uint8_t paramId, canNodes_t *canNode) {
    float value = NAN;

    // OW
    if (canNode == 0) {
	float tmp;
	int i;

	i = 0;
	while (i < 10) {
	    i++;
	    tmp = esc32ReadParamTransaction(paramId);

	    if (tmp != NAN && tmp != value)
		value = tmp;
	    else
		return value;
	}
    }
    // CAN
    else {
	value = *canGetParam(canNode->nodeId, paramId);

	return value;
    }

    return NAN;
}

static float esc32WriteParam(uint8_t paramId, float value, canNodes_t *canNode) {
    uint8_t *p;

    // OW
    if (canNode == 0) {
	owData.buf[0] = OW_PARAM_WRITE;
	owData.buf[1] = paramId;

	p = (uint8_t *)&value;
	owData.buf[2] = p[0];
	owData.buf[3] = p[1];
	owData.buf[4] = p[2];
	owData.buf[5] = p[3];
	owTransaction(6, 6);
    }
    // CAN
    else {
	canSetParam(CAN_TT_NODE, canNode->nodeId, paramId, value);
    }

    // moment to update
    yield(5);

    return esc32ReadParam(paramId, canNode);
}

static int8_t esc32ParamIdByName(char *param) {
    int i;

    for (i = 0; i < ESC32_CONFIG_NUM_PARAMS; i++)
	if (!strncasecmp(param, esc32ParameterStrings[i], strlen(esc32ParameterStrings[i])))
	    return i;

    return -1;
}

// set ESC32 mode
static uint8_t esc32SetMode(uint8_t mode, canNodes_t *canNode) {
    // OW
    if (canNode == 0) {
	owData.buf[0] = OW_SET_MODE;
	owData.buf[1] = mode;
	owTransaction(2, 2);

	owData.buf[0] = OW_GET_MODE;
	owData.buf[1] = 0;
	owTransaction(1, 2);

	if (owData.buf[0] != OW_GET_MODE)
	    return -1;
	else
	    return owData.buf[1];
    }
    // CAN
    else {
	if (*canSetRunMode(CAN_TT_NODE, canNode->nodeId, mode) != 0)
	    return mode;
	else
	    return -1;
    }
}

static uint32_t esc32ConfigWrite(canNodes_t *canNode) {
    uint32_t ret = 0;

    // OW
    if (canNode == 0) {
	// write to flash
	owData.buf[0] = OW_CONFIG_WRITE;
	owTransaction(1, 0);

	ret = 1;
    }
    // CAN
    else {
	ret = (uint32_t)canCommandConfigWrite(CAN_TT_NODE, canNode->nodeId);
    }

    // wait for flash to finish
    if (ret)
	yield(100);

    return ret;
}

// read esc32 params from uSD
static int8_t esc32ReadFile(char *fname, canNodes_t *canNode) {
    char *fileBuf;
    char *lineBuf;
    char param[24];
    int paramId;
    float value, retValue;
    int8_t needsConfigWrite = 0;
    int8_t fh;
    char c;
    int ret;
    int i, p1, p2;
    int n;

    if (fname == 0)
	fname = ESC32_FILE_NAME;

    if ((fh = filerGetHandle(fname)) < 0) {
	AQ_NOTICE("esc32: cannot get read file handle\n");
	return -1;
    }

    fileBuf = (char *)aqCalloc(ESC32_FILE_BUF_SIZE, sizeof(char));
    lineBuf = (char *)aqCalloc(ESC32_LINE_BUF_SIZE, sizeof(char));

    needsConfigWrite = 0;

    if (fileBuf && lineBuf) {
	p1 = 0;
	do {
	    ret = filerRead(fh, fileBuf, -1, ESC32_FILE_BUF_SIZE);

	    p2 = 0;
	    for (i = 0; i < ret; i++) {
		c = fileBuf[p2++];
		if (c == '\n' || p1 == (ESC32_LINE_BUF_SIZE-1)) {
		    lineBuf[p1] = 0;

		    n = sscanf(lineBuf, "#define DEFAULT_%23s %f", param, &value);
		    if (n != 2) {
			n = sscanf(lineBuf, "#define %23s %f", param, &value);
			if (n != 2)
			    n = sscanf(lineBuf, "%23s %f", param, &value);
		    }

		    if (n == 2) {
			// lookup parameter id
			paramId = esc32ParamIdByName(param);

			// valid id?
			if (paramId >= 0) {
			    // read
			    retValue = esc32ReadParam(paramId, canNode);

			    // current value differs from ours?
			    if (retValue != value) {
				// write
				esc32WriteParam(paramId, value, canNode);
				// read back
				retValue = esc32ReadParam(paramId, canNode);

				// successful write?
				if (retValue == value) {
				    needsConfigWrite++;
				}
				else {
				    AQ_NOTICE("esc32: parameter write failed!\n");
				}
			    }
			}
		    }
		    p1 = 0;
		}
		else {
		    lineBuf[p1++] = c;
		}
	    }

	} while (ret > 0);

	filerClose(fh);
    }

    if (lineBuf)
	aqFree(lineBuf, ESC32_LINE_BUF_SIZE, sizeof(char));
    if (fileBuf)
	aqFree(fileBuf, ESC32_FILE_BUF_SIZE, sizeof(char));

    return needsConfigWrite;
}

void esc32SetupCan(canNodes_t *canNode, uint8_t mode) {
    char *s;
    int i = 0;

    if ((s = canGetVersion(1)) != 0) {
	AQ_PRINTF("ESC32: CAN ID: %d, ver: %s\n", canNode->canId, s);

	// set parameters
	if (esc32ReadParam(ESC32_STARTUP_MODE, canNode) != (float)mode) {
	    if (esc32SetMode(mode, canNode) != mode)
		AQ_NOTICE("ESC32: failed to set run mode\n");

	    esc32WriteParam(ESC32_STARTUP_MODE, (float)mode, canNode);
	    i++;
	}

	i += esc32ReadFile(0, canNode);
	if (i > 0) {
	    if (esc32ConfigWrite(canNode)) {
		AQ_PRINTF("ESC32: updated %d param(s) in flash\n", i);
	    }
	    else {
		AQ_NOTICE("ESC32: failed to flash params\n");
	    }
	}
    }
    else {
	AQ_PRINTF("ESC32: cannot detect ESC CAN ID %d\n", canNode->canId);
    }
}

void esc32SetupOw(const GPIO_TypeDef *port, const uint16_t pin, uint8_t mode) {
    int i = 0;

    owInit((GPIO_TypeDef *)port, pin);

    // get ESC32 version
    owData.buf[0] = OW_VERSION;
    owTransaction(1, 16);

    if (owData.status != OW_STATUS_NO_PRESENSE) {
	AQ_PRINTF("ESC32: OW ver: %s\n", owData.buf);

	// set parameters
	if (esc32ReadParam(ESC32_STARTUP_MODE, 0) != (float)mode) {
	    if (esc32SetMode(mode, 0) != mode)
		AQ_NOTICE("ESC32: failed to set run mode\n");

	    esc32WriteParam(ESC32_STARTUP_MODE, (float)mode, 0);
	    i++;
	}

	i += esc32ReadFile(0, 0);
	if (i > 0) {
	    if (esc32ConfigWrite(0)) {
		AQ_PRINTF("ESC32: updated %d param(s) in flash\n", i);
	    }
	    else {
		AQ_NOTICE("ESC32: failed to flash params\n");
	    }
	}
    }
    else {
	AQ_NOTICE("ESC32: cannot detect ESC via OW\n");
    }
}
