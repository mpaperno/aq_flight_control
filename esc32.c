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

#include "aq.h"
#include "esc32.h"
#include "1wire.h"
#include "notice.h"
#include "filer.h"
#include "util.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

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
    "FET_BRAKING"
};

float esc32ReadParamTransaction(uint8_t paramId) {
    uint8_t *p;
    float value;

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
float esc32ReadParam(uint8_t paramId) {
    float value = __float32_nan;
    float tmp;
    int i;

    i = 0;
    while (i < 10) {
	i++;
	tmp = esc32ReadParamTransaction(paramId);

	if (tmp != value)
	    value = tmp;
	else
	    return value;
    }

    return __float32_nan;
}

float esc32WriteParam(uint8_t paramId, float value) {
    uint8_t *p;

    owData.buf[0] = OW_PARAM_WRITE;
    owData.buf[1] = paramId;

    p = (uint8_t *)&value;
    owData.buf[2] = p[0];
    owData.buf[3] = p[1];
    owData.buf[4] = p[2];
    owData.buf[5] = p[3];
    owTransaction(6, 6);

    return esc32ReadParam(paramId);
}

int8_t esc32ParamIdByName(char *param) {
    int i;

    for (i = 0; i < ESC32_CONFIG_NUM_PARAMS; i++)
	if (!strncasecmp(param, esc32ParameterStrings[i], strlen(esc32ParameterStrings[i])))
	    return i;

    return -1;
}

float esc32ReadParamByName(char *param) {
    float value = __float32_nan;
    int paramId;

    paramId = esc32ParamIdByName(param);

    if (paramId >= 0)
	value  = esc32ReadParam(paramId);

    return value;
}

int8_t esc32WriteParamByName(char *param, float value) {
    char s[64];
    int paramId;

    paramId = esc32ParamIdByName(param);

    if (paramId >= 0) {
	// 3 tries
	if (esc32WriteParam(paramId, value) == value || esc32WriteParam(paramId, value) == value || esc32WriteParam(paramId, value) == value)
	    return 1;
    }

    sprintf(s, "ESC32: failed to write %s param\n", param);
    AQ_NOTICE(s);

    return -1;
}

// set ESC32 mode
uint8_t esc32Mode(uint8_t mode) {
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

// read esc32 params from uSD
int8_t esc32ReadFile(char *fname) {
    char *fileBuf;
    char *lineBuf;
    char param[16];
    int paramId;
    float value;
    int8_t needsConfigWrite;
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

		    n = sscanf(lineBuf, "#define DEFAULT_%15s %f", param, &value);
		    if (n != 2) {
			n = sscanf(lineBuf, "#define %15s %f", param, &value);
			if (n != 2)
			    n = sscanf(lineBuf, "%15s %f", param, &value);
		    }

		    if (n == 2) {
			// lookup parameter id
			paramId = esc32ParamIdByName(param);
			// valid id?
			if (paramId >= 0)
			    // current value differs from ours?
			    if (esc32ReadParam(paramId) != value) {
				// successful write?
				if (esc32WriteParam(paramId, value)) {
				    AQ_NOTICE("esc32: wrote parameter from file\n");
				    needsConfigWrite = 1;
				}
				else {
				    AQ_NOTICE("esc32: parameter write failed!\n");
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

void esc32Setup(const GPIO_TypeDef *port, const uint16_t pin, uint8_t mode) {
    char s[32];

    owInit((GPIO_TypeDef *)port, pin);

    // get ESC32 version
    owData.buf[0] = OW_VERSION;
    owTransaction(1, 16);

    if (owData.status != OW_STATUS_NO_PRESENSE) {
	sprintf(s, "ESC32 ver: %s\n", owData.buf);
	AQ_NOTICE(s);

	// set parameters
	if (esc32ReadParam(ESC32_STARTUP_MODE) != (float)mode) {
	    if (esc32Mode(mode) != mode)
		    AQ_NOTICE("ESC32: failed to set mode\n");
	    esc32WriteParamByName("ESC32_STARTUP_MODE", (float)mode);
	}

	if (esc32ReadFile(0)) {
	    // write to flash
	    owData.buf[0] = OW_CONFIG_WRITE;
	    owTransaction(1, 0);

	    AQ_NOTICE("ESC32: stored config to flash\n");

	    // wait for flash to finish
	    yield(100);
	}
    }
}
