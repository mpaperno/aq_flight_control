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

#ifndef _logger_h
#define _logger_h

#include "ff.h"
#include "adc.h"
#include <CoOS.h>

#define LOGGER_FNAME			"AQL"
#define LOGGER_ERRORS			3
#define LOGGER_BUF_SIZE			200

typedef struct {
    uint8_t logHandle;
} loggerStruct_t;

typedef struct {
    char sig[3];
    unsigned long lastUpdate;
    float voltages[ADC_SENSORS];
    float rate[3];
    float acc[3];
    float mag[3];
    float pressure[2];
    float temp[4];
    float vIn;
    float quat[4];
    float rateAux[3];
    float accAux[3];
    float magAux[3];
    unsigned long gpsPosUpdate;
    double lat, lon;
    float gpsAlt;
    float gpsPosAcc;
    unsigned long gpsVelUpdate;
    float gpsVel[3];
    float gpsVelAcc;
    float pos[3];
    float vel[3];
    short int motors[14];
    short int throttle;
    float extra[4];
    char ckA, ckB;
} __attribute__((packed)) loggerOutput_t;

extern loggerStruct_t loggerData;
extern void loggerInit(void);
extern void loggerDo(void);

#endif
