#ifndef _calib_h
#define _calib_h

#include "imu.h"
#include "aq_math.h"

#define CALIB_SAMPLES       20    // per octant
#define CALIB_MIN_ANGLE     25    // degrees
#define CALIB_EMPTY_SLOT    100.0f
#define CALIB_SCALE	    2.0f

typedef struct {
    float32_t *calibSamples;
    float32_t lastVec[3];
    float32_t min[3];
    float32_t max[3];
    float32_t bias[3];
    float32_t U[10];
    float32_t percentComplete;
} calibStruct_t;

extern calibStruct_t calibData;

extern void calibInit(void);
extern void calibDeinit(void);
extern void calibrate(void);

#endif
