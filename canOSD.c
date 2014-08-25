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

#include "canOSD.h"
#include "can.h"
#include "supervisor.h"
#include "nav.h"
#include "gps.h"
#include "nav_ukf.h"
#include "run.h"
#include "radio.h"

canOSDStruct_t canOSDData;

void canOSDRequestValue(canNodes_t *node, uint8_t seqId, uint8_t *data) {
    canOSDData.slots[data[0]] = data[1];
    canAck(node->networkId, seqId);
}

void canOSDRequstRate(canNodes_t *node, uint8_t seqId, uint8_t *data) {
    canOSDData.rate = *(uint16_t *)data;
    if (canOSDData.rate > 0)
        canOSDData.rate = 1000 / canOSDData.rate;

    canAck(node->networkId, seqId);
}

void canOSDTelemetry(uint32_t loop) {
    static int i = OSD_TELEM_NUM;
    float dataFloat[2];
    uint32_t *dataInt = (uint32_t *)dataFloat;
    uint8_t n = 0;

    if (!(loop % canOSDData.rate))
        i = 1;

    if (i < OSD_TELEM_NUM) {
        switch (i) {
            case OSD_TELEM_STATUS:
                ((uint8_t *)dataFloat)[0] = supervisorData.state;
                ((uint8_t *)dataFloat)[1] = navData.mode;
                n = 2;
                break;

            case OSD_TELEM_LAT_LON:
                dataInt[0] = gpsData.lat * (double)1e7;
                dataInt[1] = gpsData.lon * (double)1e7;
                n = 8;
                break;

            case OSD_TELEM_VELNE:
                dataFloat[0] = UKF_VELN;
                dataFloat[1] = UKF_VELE;
                n = 8;
                break;

            case OSD_TELEM_VELD:
                dataFloat[0] = VELOCITYD;
                n = 4;
                break;

            case OSD_TELEM_ALT:
                dataFloat[0] = UKF_PRES_ALT;
                n = 4;
                break;

            case OSD_TELEM_HOME:
                dataFloat[0] = navCalcDistance(gpsData.lat, gpsData.lon, navData.homeLeg.targetLat, navData.homeLeg.targetLon);
                dataFloat[1] = navCalcBearing(navData.homeLeg.targetLat, navData.homeLeg.targetLon, gpsData.lat, gpsData.lon);
                n = 8;
                break;

            case OSD_TELEM_RC_QUALITY:
                dataFloat[0] = RADIO_QUALITY;
                n = 4;
                break;

            case OSD_TELEM_GPS_HACC:
                dataFloat[0] = gpsData.hAcc;
                n = 4;
                break;

            case OSD_TELEM_Q1_Q2:
                dataFloat[0] = UKF_Q1;
                dataFloat[1] = UKF_Q2;
                n = 8;
                break;

            case OSD_TELEM_Q3_Q4:
                dataFloat[0] = UKF_Q3;
                dataFloat[1] = UKF_Q4;
                n = 8;
                break;
        }

        canSend(CAN_LCC_NORMAL | CAN_TT_NODE | CAN_FID_TELEM | (CAN_DOC_MASK & (i<<19)), canOSDData.node->networkId, n, dataFloat);
        i++;
    }
}

void canOSDInit(void) {
    int i;

    for (i = 0; i < (CAN_TID_MASK>>9)+1; i++) {
        if (canData.nodes[i].type == CAN_TYPE_OSD) {
            canOSDData.node = &canData.nodes[i];

            canCommandArm(CAN_TT_NODE, canOSDData.node->networkId);

            break;
        }
    }
}
