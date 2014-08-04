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
#include "config.h"
#include "radio.h"
#include "spektrum.h"
#include "futaba.h"
#include "ppm.h"
#include "grhott.h"
#include "mlinkrx.h"
#include "dsm.h"
#include "util.h"
#include "aq_timer.h"
#include "comm.h"
#include <string.h>

radioStruct_t radioData __attribute__((section(".ccm")));

OS_STK *radioTaskStack;

// calculate radio reception quality
static void radioReceptionQuality(radioInstance_t *r, int8_t q) {
    r->quality = utilFilter(&r->qualityFilter, (float)(q + 1)) * 0.5f * 100.0f;
}

static void radioMakeCurrent(radioInstance_t *r) {
    radioData.quality = &r->quality;
    radioData.errorCount = &r->errorCount;
    radioData.channels = r->channels;
    radioData.lastUpdate = &r->lastUpdate;
    radioData.binding = &r->binding;
}

static void radioProcessInstance(radioInstance_t *r) {
    serialPort_t *s = r->serialPort;
    int8_t q;

    switch (r->radioType) {
    case RADIO_TYPE_SPEKTRUM11:
    case RADIO_TYPE_SPEKTRUM10:
    case RADIO_TYPE_DELTANG:
        while (serialAvailable(s))
            if (spektrumCharIn(r, serialRead(s))) {
                r->lastUpdate = timerMicros();
                radioReceptionQuality(r, 1);
            }
        break;

    case RADIO_TYPE_SBUS:
        while (serialAvailable(s))
            if ((q = futabaCharIn(r, serialRead(s)))) {
                r->lastUpdate = timerMicros();
                radioReceptionQuality(r, q);
            }
        break;

    case RADIO_TYPE_PPM:
        if (ppmDataAvailable(r))
            r->lastUpdate = timerMicros();
        break;

    case RADIO_TYPE_SUMD:
        while (serialAvailable(s))
            if ((q = grhottCharIn(r, serialRead(s)))) {
                r->lastUpdate = timerMicros();
                radioReceptionQuality(r, q);
            }
        break;

    case RADIO_TYPE_MLINK:
        while (serialAvailable(s))
        if ((q = mlinkrxCharIn(r, serialRead(s)))) {
            r->lastUpdate = timerMicros();
            radioReceptionQuality(r, q);
        }
        break;

    case RADIO_TYPE_CYRF6936:
        if ((q = dsmReceive(r))) {
            r->lastUpdate = timerMicros();
            radioReceptionQuality(r, q);
        }
        break;
    }

    // no radio signal?
    if (timerMicros() - r->lastUpdate > RADIO_UPDATE_TIMEOUT)
        radioReceptionQuality(r, -1);                       // minimum signal quality (0%) if no updates within timeout value
    else if (r->radioType == RADIO_TYPE_PPM)
        radioReceptionQuality(r, ppmGetSignalQuality(r));   // signal quality based on PPM status
}

void radioTaskCode(void *unused) {
    int i;

    AQ_NOTICE("Radio task started\n");

    while (1) {
	// wait for data
	yield(2); // 2ms

        for (i = 0; i < RADIO_NUM; i++) {
            radioInstance_t *r = &radioData.radioInstances[i];

            if (r->radioType > RADIO_TYPE_NONE) {
                radioProcessInstance(r);

                // find best signal
                if (radioData.mode == RADIO_MODE_DIVERSITY && r->quality > *radioData.quality)
                    radioMakeCurrent(r);
            }
        }
    }
}

static void radioRCSelect(uint8_t rcPort, uint8_t level) {
#ifdef RADIO_RC1_SELECT_PORT
    if (rcPort == 0)
        radioData.select[0] = digitalInit(RADIO_RC1_SELECT_PORT, RADIO_RC1_SELECT_PIN, level);
#endif

#ifdef RADIO_RC2_SELECT_PORT
    if (rcPort == 1)
        radioData.select[1] = digitalInit(RADIO_RC2_SELECT_PORT, RADIO_RC2_SELECT_PIN, level);
#endif
}

void radioInit(void) {
    uint16_t radioType = (uint16_t)p[RADIO_SETUP];
    int i;

    AQ_NOTICE("Radio init\n");

    // TODO: remove RADIO_TYPE
    if (radioType == 0 && (int8_t)p[RADIO_TYPE] >= 0)
        radioType = (uint16_t)p[RADIO_TYPE] + 1;

    memset((void *)&radioData, 0, sizeof(radioData));

    radioData.mode = (radioType>>12) & 0x0f;

    for (i = 0; i < RADIO_NUM; i++) {
        radioInstance_t *r = &radioData.radioInstances[i];
        USART_TypeDef *uart;

        // determine UART
        switch (i) {
            case 0:
                uart = RC1_UART;
                break;
#ifdef RC2_UART
            case 1:
                uart = RC2_UART;
                break;
#endif
#ifdef RC3_UART
            case 2:
                uart = RC3_UART;
                break;
#endif
            default:
                uart = 0;
                break;
        }

        r->radioType = (radioType>>(i*4)) & 0x0f;
        r->channels = &radioData.allChannels[RADIO_MAX_CHANNELS * i];

        utilFilterInit(&r->qualityFilter, (1.0f / 50.0f), 0.75f, 0.0f);

        switch (r->radioType) {
        case RADIO_TYPE_SPEKTRUM11:
        case RADIO_TYPE_SPEKTRUM10:
        case RADIO_TYPE_DELTANG:
            if (uart) {
                spektrumInit(r, uart);
                radioRCSelect(i, 0);
                AQ_PRINTF("Spektrum on RC port %d\n", i);
            }
            break;

        case RADIO_TYPE_SBUS:
            if (uart) {
                futabaInit(r, uart);
                radioRCSelect(i, 1);
                AQ_PRINTF("Futaba on RC port %d\n", i);
            }
            break;

        case RADIO_TYPE_PPM:
            ppmInit(r);
            AQ_PRINTF("PPM on PWM port %d\n", PPM_PWM_CHANNEL);
            break;

        case RADIO_TYPE_SUMD:
            if (uart) {
                grhottInit(r, uart);
                radioRCSelect(i, 0);
                AQ_PRINTF("GrHott on RC port %d\n", i);
            }
            break;

        case RADIO_TYPE_MLINK:
            if (uart) {
                mlinkrxInit(r, uart);
                radioRCSelect(i, 0);
                AQ_PRINTF("Mlink on RC port %d\n", i);
            }
            break;

        case RADIO_TYPE_CYRF6936:
            if (dsmInit())
                AQ_PRINTF("CYRF6936 on RC port %d\n", i);
            break;

        case RADIO_TYPE_NONE:
            break;

        default:
            AQ_NOTICE("WARNING: Invalid radio type!\n");
            break;
        }
    }

    // ensure zero default channel values
    radioData.channels = radioData.radioInstances[0].channels;

    switch (radioData.mode) {
        case RADIO_MODE_DIVERSITY:
            // select first available radio to start with
            for (i = 0; i < RADIO_NUM; i++) {
                if (radioData.radioInstances[i].radioType > RADIO_TYPE_NONE) {
                    radioMakeCurrent(&radioData.radioInstances[i]);
                    break;
                }
            }
            break;

        case RADIO_MODE_SPLIT:
            radioMakeCurrent(&radioData.radioInstances[0]);
            break;
    }

    // set mode default
    radioData.channels[(int)p[RADIO_FLAP_CH]] = -700;

    radioTaskStack = aqStackInit(RADIO_STACK_SIZE, "RADIO");

    radioData.radioTask = CoCreateTask(radioTaskCode, (void *)0, RADIO_PRIORITY, &radioTaskStack[RADIO_STACK_SIZE-1], RADIO_STACK_SIZE);
}
