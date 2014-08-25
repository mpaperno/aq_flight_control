#ifndef _alt_ukf_h
#define _alt_ukf_h

#include "srcdkf.h"

#define ALT_S           3   // states
#define ALT_M           1   // measurements
#define ALT_V           2   // process noise
#define ALT_N           1   // measurement noise

#define ALT_STATE_POS   0
#define ALT_STATE_VEL   1
#define ALT_STATE_BIAS  2

#define ALT_NOISE_BIAS  0
#define ALT_NOISE_VEL   1

#define ALT_POS         altUkfData.x[ALT_STATE_POS]
#define ALT_VEL         altUkfData.x[ALT_STATE_VEL]
#define ALT_BIAS        altUkfData.x[ALT_STATE_BIAS]

#define ALT_PRES_NOISE  0.02f
#define ALT_BIAS_NOISE  5e-4f//5e-5f
#define ALT_VEL_NOISE   5e-4f

typedef struct {
    srcdkf_t *kf;
    float *x;               // states
} altUkfStruct_t;

extern altUkfStruct_t altUkfData;

extern void altUkfInit(void);
extern void altUkfProcess(float measuredPres);

#endif