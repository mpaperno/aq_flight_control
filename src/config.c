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
    Copyright 2013-2016 Maxim Paperno
*/

#include "aq.h"
#include "config.h"
#include "flash.h"
#include "filer.h"
#include "comm.h"
#include "supervisor.h"
#include "util.h"
#include "rc.h"
#include "motors.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

// param definitions
#include "config_params.h"

float p[CONFIG_NUM_PARAMS] __attribute__((section(".ccm"))) = {0};
configData_t configData __attribute__((section(".ccm")));

//
// private functions
//

configToken_t *configTokenFindEmpty(void) {
    configToken_t *p = (configToken_t *)(FLASH_END_ADDR + 1);

    do {
        p--;
    } while (p->key != 0xffffffff);

    return p;
}

// public
void configTokenStore(configToken_t *token) {
    flashAddress((uint32_t)configTokenFindEmpty(), (uint32_t *)token, sizeof(configToken_t)/sizeof(uint32_t));
}

// public
configToken_t *configTokenGet(uint32_t key) {
    configToken_t *p, *t;

    p = (configToken_t *)(FLASH_END_ADDR + 1);
    t = 0;

    do {
        p--;

        if (p->key == key)
            t = p;
    } while (p->key != 0xffffffff);

    return t;
}

configToken_t *configTokenIterate(configToken_t *t) {
    if (t == 0)
        t = (configToken_t *)(FLASH_END_ADDR + 1);

    t--;

    if (t->key != 0xffffffff)
        return t;
    else
        return 0;
}

// param storage in flash
bool configFlashRead(void) {
    configRec_t *recs;
    int i;

    // validate there is a reasonable config version number at start of flash
    float flashVer = *(float *)(flashStartAddr() + CONFIG_PNAME_MAX_LEN);
    if (isnan(flashVer) || flashVer <= 0 || flashVer > CONFIG_CURRENT_VERSION + 100)
	return false;

    recs = (void *)flashStartAddr();

    for (i = 0; i < CONFIG_NUM_PARAMS; i++) {
	// avoid reading past end of populated flash storage
	if (!memcmp(recs + i, "\xFF", 1))
	    break;
	configSetParamByName(recs[i].name, recs[i].val);
    }

    AQ_NOTICE("config: Parameters restored from flash memory.\n");

    return true;
}

uint8_t configFlashWrite(void) {
    configRec_t *recs;
    uint8_t ret = 0;
    int i;

    recs = (void *)aqCalloc(CONFIG_NUM_PARAMS, sizeof(configRec_t));

    if (recs) {
        configToken_t *tr = (configToken_t *)recs;
        configToken_t *tf = 0;

        // read all tokens
        do {
            tf = configTokenIterate(tf);

            // copy to RAM
            if (tf) {
                // only one instance per key
                do {
                    if (tr->key == 0 || tr->key == tf->key) {
                        memcpy(tr, tf, sizeof(configToken_t));
                        break;
                    }
                    tr++;
                } while (1);
            }
        } while (tf);

        ret = flashErase(flashStartAddr(), CONFIG_NUM_PARAMS * sizeof(configRec_t) / sizeof(uint32_t));

        // invalidate the flash data cache
        FLASH_DataCacheCmd(DISABLE);
        FLASH_DataCacheReset();
        FLASH_DataCacheCmd(ENABLE);

        if (ret) {
            tr = (configToken_t *)recs;

            // copy tokens back to flash
            while (tr->key)
                configTokenStore(tr++);

            // create param list in RAM
            for (i = 0; i < CONFIG_NUM_PARAMS; i++) {
                strncpy(recs[i].name, configParamMeta[i].name, CONFIG_PNAME_MAX_LEN);
                recs[i].val = configGetParamValueForSave(i);
            }

            ret = flashAddress(flashStartAddr(), (uint32_t *)recs, CONFIG_NUM_PARAMS*sizeof(configRec_t)/sizeof(uint32_t));
        }

        aqFree(recs, CONFIG_NUM_PARAMS, sizeof(configRec_t));

	AQ_NOTICE("config: Parameters saved to flash memory.\n");
    }
    else {
        AQ_NOTICE("config: Error writing params to flash, cannot allocate memory.\n");
    }

    return ret;
}

// Read config from uSD
// Returns >= 0 on success, -1 if file load not attempted (not found, no disk, etc), <= -2 if failure during file read (means current running params might be corrupted)
// A safer version of this would read params into a temporary buffer first, then copy to running params once loading is successful.
int8_t configReadFile(char *fname) {
    char *fileBuf;
    int8_t fh;
    int ret;
    int p1;

    if (fname == 0)
	fname = CONFIG_FILE_NAME;

    if ((fh = filerGetHandle(fname)) < 0) {
	AQ_NOTICE("config: cannot get read file handle\n");
	return -1;
    }

    if (!(fileBuf = (char *)aqCalloc(CONFIG_FILE_BUF_SIZE, sizeof(char)))) {
	AQ_NOTICE("config: Error reading from file, cannot allocate memory.\n");
	filerClose(fh);
	return -1;
    }

    p1 = 0;
    while ((ret = filerRead(fh, fileBuf, -1, CONFIG_FILE_BUF_SIZE)) > 0) {
	p1 = configParseParams((char *)fileBuf, ret, p1);
	if (p1 < 0) {
	    ret = -2;
	    break;
	}
    }

    filerClose(fh);

    if (fileBuf)
	aqFree(fileBuf, CONFIG_FILE_BUF_SIZE, sizeof(char));

    if (ret >= FILER_STATUS_OK)
	AQ_NOTICE("config: Parameters loaded from local storage file.\n");
    else if (ret < FILER_STATUS_ERR_FNF)
	AQ_NOTICE("config: Failed to read parameters from local file.");
    else {
	AQ_PRINTF("config: Params file not found: (%s)", fname);
	ret = -1;  // soft error
    }

    return ret;
}

// write config to uSD
int8_t configWriteFile(char *fname) {
    char *buf;
    int8_t fh;
    int8_t ret;
    int n;
    int i;

    if (fname == 0)
	fname = CONFIG_FILE_NAME;

    if ((fh = filerGetHandle(fname)) < 0) {
	AQ_NOTICE("config: cannot get write file handle\n");
	return -1;
    }

    if (!(buf = (char *)aqCalloc(128, sizeof(char)))) {
	AQ_NOTICE("config: Error writing to file, cannot allocate memory.\n");
	filerClose(fh);
	return -1;
    }

    for (i = 0; i < CONFIG_NUM_PARAMS; i++) {
	n = configFormatParam(buf, i);
	if (n)
	    ret = filerWrite(fh, buf, -1, n);

	if (!n || ret < n) {
	    ret = -1;
	    break;
	}
    }

    filerClose(fh);

    if (buf)
	aqFree(buf, 128, sizeof(char));

    if (ret > -1)
	AQ_NOTICE("config: Parameters saved to local storage file.\n");
    else
	AQ_NOTICE("config: Error writing parameters to file.\n");

    return ret;
}

void configLoadDefault(bool all) {
    configData.numPossibleAdjParams = 0;

    for (int i = 0; i < CONFIG_NUM_PARAMS; ++i) {
	if (all || configParamMeta[i].sinceVersion > configGetParamValue(CONFIG_VERSION))
	    configSetParamByID(i, configParamMeta[i].defaultVal);

	if (configParamMeta[i].adjustable)
	    ++configData.numPossibleAdjParams;
    }
    if (all)
	AQ_NOTICE("config: Loaded default parameters.\n");
}

// process freshly loaded/changed params
void configParamsLoaded(void) {
    float scal;
    uint32_t pId;
    uint8_t chan, j;

    // setup adjustable parameters, if any
    memset((void *)&configData.adjustParams, 0, sizeof(configData.adjustParams));  // in case we're reloading
    j = 0;
    for (int i = CONFIG_ADJUST_P1; i < CONFIG_ADJUST_P1 + CONFIG_MAX_ADJUSTABLE_PARAMS; ++i) {
	// allow adjusting params only if config versions match (to be sure param IDs are correct)
	if (p[CONFIG_VERSION] == CONFIG_CURRENT_VERSION) {
	    pId = configGetAdjParamId(i);
	    chan = ((uint32_t)p[i] >> 10) & 0x3F;
	    scal = (((uint32_t)p[i] >> 16) & 0xFF) / 10000.0f;
	    if (pId && pId < CONFIG_NUM_PARAMS && configParamMeta[pId].adjustable && chan && scal) {
		configData.adjustParams[j].adjChan = chan;
		configData.adjustParams[j].adjScale = scal;
		configData.adjustParams[j].minVal = configParamMeta[pId].minVal;
		configData.adjustParams[j].maxVal = configParamMeta[pId].maxVal;
		configData.paramFlags[pId] = ++j;
		AQ_PRINTF("WARNING: %s is adjustable on channel %d", configParamMeta[pId].name, chan);
	    }
	}
	// if versions don't match, clear the configurable param definition entirely
	else
	    p[i] = configParamMeta[i].defaultVal;
    }

#ifdef HAS_QUATOS
    // validate we have any Quatos key at all
    if (!p[LIC_KEY1])
	p[QUATOS_ENABLE] = 0;
#endif

}


//
// public getters
//

int16_t configGetParamIdByName(char *name) {
    int i;

    for (i = 0; i < CONFIG_NUM_PARAMS; i++)
	if (!strncmp(name, configParamMeta[i].name, CONFIG_PNAME_MAX_LEN))
	    break;

    if (i < CONFIG_NUM_PARAMS)
	return i;
    else {
	char buf[CONFIG_PNAME_MAX_LEN];
	strncpy(buf, name, CONFIG_PNAME_MAX_LEN);
	AQ_PRINTF("config: cannot find parmeter '%s'\n", buf);
	return -1;
    }
}

char *configGetParamName(uint16_t id) {
    if (id < CONFIG_NUM_PARAMS)
	return configParamMeta[id].name;
    else
	return 0;
}

// Returns parameter value, possibly adjusted via radio channel.
__attribute__((always_inline))
inline float configGetParamValue(uint16_t id) {

    if (id >= CONFIG_NUM_PARAMS)
	return 0.0f;

    float ret = p[id];
    uint8_t flg = configData.paramFlags[id];
    if (flg) {
	uint8_t idx = flg - 1;
	ret += (ret * rcGetChannelValue(configData.adjustParams[idx].adjChan) * configData.adjustParams[idx].adjScale);
	ret = constrainFloat(ret, configData.adjustParams[idx].minVal, configData.adjustParams[idx].maxVal);
    }

    return ret;
}

__attribute__((always_inline))
inline float configGetParamValueByName(char *name) {
    return configGetParamValue(configGetParamIdByName(name));
}

// Returns parameter value as currently defined in running params config
__attribute__((always_inline))
inline float configGetParamValueRaw(uint16_t id) {
    if (id < CONFIG_NUM_PARAMS)
	return p[id];
    else
	return 0.0f;
}

float configGetParamValueForSave(uint16_t id) {
    if (configCheckFlag(CONFIG_FLAG_SAVE_ADJUSTED))
	return configGetParamValue(id);
    else
	return configGetParamValueRaw(id);

}

// Returns a pointer to memory in params storage array starting at given param ID
__attribute__((always_inline))
inline float *configGetParamPtr(uint16_t id) {
    if (id < CONFIG_NUM_PARAMS)
	return &p[id];
    else
	return 0;
}

uint8_t configGetParamDataType(uint16_t id) {
    if (id < CONFIG_NUM_PARAMS)
	return configParamMeta[id].dataType;
    else
	return AQ_TYPE_FLT;
}

// this is used to iterate through the list of possible adjustable params, eg. when requested by a GCS
int16_t configGetNextAdjustableParam(uint16_t startId) {
    for (int i = startId; i < CONFIG_NUM_PARAMS; ++i) {
	if (configParamMeta[i].adjustable)
	    return configParamMeta[i].id;
    }
    return -1;
}

uint16_t configGetNumPossibleAdjParams(void) {
    return configData.numPossibleAdjParams;
}


//
// public setters
//

bool configSetParamByID(uint16_t id, float value) {

    if (id >= CONFIG_NUM_PARAMS) {
	AQ_PRINTF("ERROR: Parameter ID out of range: %d\n", id);
	return false;
    }
    if (value < configParamMeta[id].minVal || value > configParamMeta[id].maxVal || isnan(value) || isinf(value)) {
	AQ_PRINTF("ERROR: %s value out of range!\n", configGetParamName(id));
	AQ_PRINTF("%s Mn:%+.4e Mx:%+.4e V:%+.4e \n",
		configGetParamName(id), (double)configParamMeta[id].minVal, (double)configParamMeta[id].maxVal, (double)value);
	return false;
    }

    p[id] = value;
    return true;
}

bool configSetParamByName(char *name, float value) {
    int id = configGetParamIdByName(name);
    if (id > -1)
	return configSetParamByID((uint16_t)id, value);
    else
	return false;
}


//
// public load/save
//

bool configLoadParamsFromDefault(void) {
    configLoadDefault(true);
    configParamsLoaded();
    return true;
}

bool configLoadParamsFromFile(void) {
    bool ret = false;

    if (configReadFile(0) > -1) {
	configParamsLoaded();
	ret = true;
    }
    return ret;
}

bool configLoadParamsFromFlash(void) {
    bool ret = configFlashRead();
    if (ret)
	configParamsLoaded();

    return ret;
}

bool configSaveParamsToFile(void) {
    bool ret = false;
    // never save esc calibration bit to SD card
    p[MOT_ESC_TYPE] = ((uint32_t)p[MOT_ESC_TYPE] & ~(MOT_ESC_TYPE_CALIB_ENABLE));
    if (configWriteFile(0) > -1)
	ret = true;

    return ret;
}

bool configSaveParamsToFlash(void) {
    return configFlashWrite();
}


//
// init
//

void configInit(void) {
    int8_t fileRet;

    memset((void *)&configData, 0, sizeof(configData));

    // start with defaults
    configLoadDefault(true);

    // load what's in flash
    configFlashRead();

    // try to load any config params from uSD card
    fileRet = configReadFile(0);
    if (fileRet > -1)
	supervisorConfigRead();
    else if (fileRet < -1) {
	// start over if fatal file read error
	memset(p, 0, sizeof(p));
	configLoadDefault(true);
	configFlashRead();
    } // else no params file found, use loaded defaults/flash values

    // finally, make sure sensible defaults are loaded for any params that need to be reset since last user version
    configLoadDefault(false);

    configParamsLoaded();

    // indicate that we have the latest version of everything loaded
    p[CONFIG_VERSION] = CONFIG_CURRENT_VERSION;
}


//
// public utils
//


int8_t configFormatParam(char *buf, int n) {
    return sprintf(buf, "%-17s\t\t%.10e\n", configParamMeta[n].name, (double)configGetParamValueForSave(n));
}

// Scan a chunk of char data for param definitions and copy any found to the running config. Mulitple params can be delimited with \n
// Returns -1 if error, otherwise a reference position (p1) to pass back into configParseParams with next chunk.
int configParseParams(char *fileBuf, int size, int p1) {
    static char lineBuf[CONFIG_LINE_BUF_SIZE];
    char *param;
    float value;
    char c;
    int p2;
    int n;
    int i;

    if (!(param = (char *)aqCalloc(17, sizeof(char))))
	return -1;

    p2 = 0;
    for (i = 0; i < size; i++) {
	c = fileBuf[p2++];
	if (c == '\n' || p1 == (CONFIG_LINE_BUF_SIZE-1)) {
	    lineBuf[p1] = 0;

	    n = sscanf(lineBuf, "#define DEFAULT_%17s %f", param, &value);
	    if (n != 2) {
		n = sscanf(lineBuf, "%17s %f", param, &value);
		if (n != 2)
		    n = sscanf(lineBuf, "#define %17s %f", param, &value);
	    }

	    if (n == 2)
		configSetParamByName(param, value);

	    p1 = 0;
	}
	else {
	    lineBuf[p1++] = c;
	}
    }

    if (param)
	aqFree(param, 17, sizeof(char));

    return p1;
}

// read data->num number of current param values into data->values, starting with data->paramId
// used by optional binary command interface
unsigned int configParameterRead(void *data) {
    paramStruct_t *par = (paramStruct_t *)data;

    if (par->paramId + par->num > CONFIG_NUM_PARAMS)
	par->num = CONFIG_NUM_PARAMS - par->paramId;

    memcpy((char *)par->values, (char *)&p[par->paramId], par->num * sizeof(float));

    return par->num * sizeof(float);
}

// write data->num number of param values from data->values to running params, starting with data->paramId
// used by optional binary command interface
unsigned int configParameterWrite(void *data) {
    paramStruct_t *par = (paramStruct_t *)data;

    memcpy((char *)&p[par->paramId], (char *)par->values, par->num * sizeof(float));

    return configParameterRead(data);
}

