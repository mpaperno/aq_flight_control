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
#include "esc32.h"
#include "1wire.h"
#include "comm.h"
#include "filer.h"
#include "util.h"
#include <stdio.h>
#include <string.h>
#include <math.h>

static float esc32OwReadParamTransaction(uint8_t paramId) {
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

static int16_t esc32OwGetParamId(char *name) {
    uint8_t *p;
    int16_t value = -1;
    int16_t tmp;
    int i;

    i = 0;
    while (i < 10) {
        i++;

        owData.buf[0] = OW_GET_PARAM_ID;

        for (i = 0; i < 16; i++)
            owData.buf[1+i] = name[i];

        owTransaction(17, 3);

        p = (uint8_t *)&tmp;
        p[0] = owData.buf[1];
        p[1] = owData.buf[2];

        if (tmp >= 0) {
            if (tmp != value)
                value = tmp;
            else
                return value;
        }
        else {
            value = -1;
        }
    }

    return value;
}

// require parameter to be read twice the same
static float esc32OwReadParamById(uint8_t paramId) {
    float value = NAN;
    float tmp;
    int i;

    i = 0;
    while (i < 10) {
        i++;
        tmp = esc32OwReadParamTransaction(paramId);

        if (tmp != NAN && tmp != value)
            value = tmp;
        else
            return value;
    }

    return value;
}

static float esc32OwWriteParam(uint8_t paramId, float value) {
    uint8_t *p;

    owData.buf[0] = OW_PARAM_WRITE;
    owData.buf[1] = paramId;

    p = (uint8_t *)&value;
    owData.buf[2] = p[0];
    owData.buf[3] = p[1];
    owData.buf[4] = p[2];
    owData.buf[5] = p[3];
    owTransaction(6, 6);

    // moment to update
    yield(5);

    return esc32OwReadParamById(paramId);
}

// set ESC32 mode
static uint8_t esc32OwSetMode(uint8_t mode) {
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

static void esc32OwConfigWrite(void) {
    // write to flash
    owData.buf[0] = OW_CONFIG_WRITE;
    owTransaction(1, 0);

    // wait for flash to finish
    yield(100);
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
	AQ_NOTICE("ESC32: cannot get read file handle\n");
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
                        if (canNode)
                            paramId = canGetParamIdByName(canNode->networkId, (uint8_t *)param);
                        else
                            paramId = esc32OwGetParamId(param);

			// valid id?
			if (paramId >= 0) {
			    // read
                            if (canNode)
                                retValue = canGetParamById(canNode->networkId, paramId);
                            else
                                retValue = esc32OwReadParamById(paramId);

			    // current value differs from ours?
			    if (retValue != value) {
				// write
                                if (canNode)
                                    canSetParamById(canNode->networkId, paramId, value);
                                else
                                    esc32OwWriteParam(paramId, value);

				// read back
                                if (canNode)
                                    retValue = canGetParamById(canNode->networkId, paramId);
                                else
                                    retValue = esc32OwReadParamById(paramId);

				// successful write?
				if (retValue == value) {
				    needsConfigWrite++;
				}
				else {
				    AQ_NOTICE("ESC32: parameter write failed!\n");
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

float esc32SetupCan(canNodes_t *canNode, uint8_t mode) {
    char *s;
    int16_t paramId;
    int i = 0;
    float ret = 0.0f;

    if ((s = canGetVersion(canNode->networkId)) != 0) {
	AQ_PRINTF("ESC32: CAN ID: %2d, ver: %s\n", canNode->canId, s);

        paramId = canGetParamIdByName(canNode->networkId, (uint8_t *)"STARTUP_MODE");

        if (paramId < 0) {
            AQ_NOTICE("ESC32: CAN cannot read parameters\n");
        }
        else if (canGetParamById(canNode->networkId, paramId) != (float)mode) {
	    if (*canSetRunMode(CAN_TT_NODE, canNode->networkId, mode) == 0)
		AQ_NOTICE("ESC32: CAN failed to set run mode\n");

	    if (*canSetParamById(canNode->networkId, paramId, (float)mode) == 0)
                AQ_NOTICE("ESC32: CAN failed to set parameter\n");
            else
                i++;
	}

	i += esc32ReadFile(0, canNode);
	if (i > 0) {
	    if (canCommandConfigWrite(CAN_TT_NODE, canNode->networkId)) {
		AQ_PRINTF("ESC32: CAN updated %d param(s) in flash\n", i);
	    }
	    else {
		AQ_NOTICE("ESC32: CAN failed to flash params\n");
	    }
	}

	ret = strtof(s, NULL);
    }
    else {
	AQ_PRINTF("ESC32: cannot detect ESC CAN ID %d\n", canNode->canId);
    }

    if (!isfinite(ret))
        ret = 0.0f;

    return ret;
}

void esc32SetupOw(const GPIO_TypeDef *port, const uint16_t pin, uint8_t mode) {
    int16_t paramId;
    int i = 0;

    owInit((GPIO_TypeDef *)port, pin);

    // get ESC32 version
    owData.buf[0] = OW_VERSION;
    owTransaction(1, 16);

    if (owData.status != OW_STATUS_NO_PRESENSE) {
	AQ_PRINTF("ESC32: OW ver: %s\n", owData.buf);

        paramId = esc32OwGetParamId("STARTUP_MODE");

        if (paramId < 0) {
            AQ_NOTICE("ESC32: OW cannot read parameters\n");
        }
	else if (esc32OwReadParamById(paramId) != (float)mode) {
	    if (esc32OwSetMode(mode) != mode)
		AQ_NOTICE("ESC32: OW failed to set run mode\n");

	    if (esc32OwWriteParam(paramId, (float)mode) != (float)mode)
                AQ_NOTICE("ESC32: OW failed to write parameter\n");
            else
                i++;
	}

	i += esc32ReadFile(0, 0);
	if (i > 0) {
	    esc32OwConfigWrite();
            AQ_PRINTF("ESC32: OW updated %d param(s) in flash\n", i);
	}
    }
    else {
	AQ_NOTICE("ESC32: cannot detect ESC via OW\n");
    }
}
