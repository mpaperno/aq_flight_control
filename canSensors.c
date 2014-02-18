#include "canSensors.h"
#include "aq_timer.h"

canSensorsStruct_t canSensorsData;

void canSensorsReceiveTelem(uint8_t canId, void *data) {
    canSensorsData.values[canId] = *(float *)data;

    // record reception time
    canSensorsData.rcvTimes[canId] = timerMicros();
}

void canSensorsInit(void) {
    int i;

    for (i = 0; i < CAN_SENSORS_NUM; i++) {
        if ((canSensorsData.nodes[i] = canFindNode(CAN_TYPE_SENSOR, i)) != 0) {
            canTelemRegister(canSensorsReceiveTelem, CAN_TYPE_SENSOR);

            // request telemetry
            canSetTelemetryValue(CAN_TT_NODE, canSensorsData.nodes[i]->networkId, 0, CAN_TELEM_VALUE);
            canSetTelemetryRate(CAN_TT_NODE, canSensorsData.nodes[i]->networkId, CAN_SENSORS_RATE);
        }
    }
}