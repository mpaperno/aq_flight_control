#ifndef _cansensors_h
#define _cansensors_h

#include "can.h"

#define CAN_SENSORS_RATE    10    // Hz

// sensor types
enum {
    CAN_SENSORS_PDB_BATV = 0,
    CAN_SENSORS_PDB_BATA,
    CAN_SENSORS_PDB_TEMP,
    CAN_SENSORS_GIMBAL_ACCX,
    CAN_SENSORS_GIMBAL_ACCY,
    CAN_SENSORS_GIMBAL_ACCZ,
    CAN_SENSORS_GIMBAL_GYOX,
    CAN_SENSORS_GIMBAL_GYOY,
    CAN_SENSORS_GIMBAL_GYOZ,
    CAN_SENSORS_NUM
};

typedef struct {
    canNodes_t *nodes[CAN_SENSORS_NUM];
    float values[CAN_SENSORS_NUM];
    uint32_t rcvTimes[CAN_SENSORS_NUM];
} canSensorsStruct_t;

extern canSensorsStruct_t canSensorData;

extern void canSensorsInit(void);

#endif