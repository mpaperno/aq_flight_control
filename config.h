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

#ifndef _config_h
#define _config_h

#include "aq.h"

#if BOARD_VERSION == 6 && defined DIMU_VERSION
    #if DIMU_VERSION == 10
        #include "board_dimu_v1.h"
    #elif DIMU_VERSION == 11
        #include "board_dimu_v1_1.h"
    #elif DIMU_VERSION == 20
        #include "board_dimu_v2.h"
    #else
        #error "Unknown DIMU_VERSION for BOARD_VERSION == 6"
    #endif
#endif

#ifndef CONFIG_DEFAULTS_FILE
    #if BOARD_VERSION == 8
	#define CONFIG_DEFAULTS_FILE    "config_default_m4.h"
    #else
	#define CONFIG_DEFAULTS_FILE    "config_default.h"
    #endif
#endif

#include CONFIG_DEFAULTS_FILE

#ifndef DEFAULT_CONFIG_VERSION
    #error "Configuration defaults not properly defined."
#endif

#define CONFIG_FILE_NAME	    "params.txt"
#define CONFIG_FILE_BUF_SIZE	    512
#define CONFIG_LINE_BUF_SIZE	    128

// param names must be kept <= 16 chars for mavlink
enum configParameters {
    CONFIG_VERSION = 0,
    RADIO_TYPE,
    RADIO_SETUP,
    RADIO_THRO_CH,
    RADIO_ROLL_CH,
    RADIO_PITC_CH,
    RADIO_RUDD_CH,
    RADIO_GEAR_CH,
    RADIO_FLAP_CH,
    RADIO_AUX2_CH,
    RADIO_AUX3_CH,
    RADIO_AUX4_CH,
    RADIO_AUX5_CH,
    RADIO_AUX6_CH,
    RADIO_AUX7_CH,
    PPM_SCALER,
    PPM_THROT_LOW,
    PPM_CHAN_MID,
    CTRL_PID_TYPE,
    CTRL_FACT_THRO,
    CTRL_FACT_PITC,
    CTRL_FACT_ROLL,
    CTRL_MAN_YAW_RT,
    CTRL_DEAD_BAND,
    CTRL_DBAND_THRO,
    CTRL_MIN_THROT,
    CTRL_MAX,
    CTRL_NAV_YAW_RT,
    CTRL_TLT_RTE_P,
    CTRL_TLT_RTE_I,
    CTRL_TLT_RTE_D,
    CTRL_TLT_RTE_F,
    CTRL_TLT_RTE_PM,
    CTRL_TLT_RTE_IM,
    CTRL_TLT_RTE_DM,
    CTRL_TLT_RTE_OM,
    CTRL_YAW_RTE_P,
    CTRL_YAW_RTE_I,
    CTRL_YAW_RTE_D,
    CTRL_YAW_RTE_F,
    CTRL_YAW_RTE_PM,
    CTRL_YAW_RTE_IM,
    CTRL_YAW_RTE_DM,
    CTRL_YAW_RTE_OM,
    CTRL_TLT_ANG_P,
    CTRL_TLT_ANG_I,
    CTRL_TLT_ANG_D,
    CTRL_TLT_ANG_F,
    CTRL_TLT_ANG_PM,
    CTRL_TLT_ANG_IM,
    CTRL_TLT_ANG_DM,
    CTRL_TLT_ANG_OM,
    CTRL_YAW_ANG_P,
    CTRL_YAW_ANG_I,
    CTRL_YAW_ANG_D,
    CTRL_YAW_ANG_F,
    CTRL_YAW_ANG_PM,
    CTRL_YAW_ANG_IM,
    CTRL_YAW_ANG_DM,
    CTRL_YAW_ANG_OM,
    MOT_CANL,
    MOT_CANH,
    MOT_FRAME,
    MOT_ARM,
    MOT_MIN,
    MOT_START,
    MOT_VALUE2T_A1,
    MOT_VALUE2T_A2,
    MOT_VALUE_SCAL,
    MOT_MAX,
    MOT_PWRD_01_T,
    MOT_PWRD_01_P,
    MOT_PWRD_01_R,
    MOT_PWRD_01_Y,
    MOT_PWRD_02_T,
    MOT_PWRD_02_P,
    MOT_PWRD_02_R,
    MOT_PWRD_02_Y,
    MOT_PWRD_03_T,
    MOT_PWRD_03_P,
    MOT_PWRD_03_R,
    MOT_PWRD_03_Y,
    MOT_PWRD_04_T,
    MOT_PWRD_04_P,
    MOT_PWRD_04_R,
    MOT_PWRD_04_Y,
    MOT_PWRD_05_T,
    MOT_PWRD_05_P,
    MOT_PWRD_05_R,
    MOT_PWRD_05_Y,
    MOT_PWRD_06_T,
    MOT_PWRD_06_P,
    MOT_PWRD_06_R,
    MOT_PWRD_06_Y,
    MOT_PWRD_07_T,
    MOT_PWRD_07_P,
    MOT_PWRD_07_R,
    MOT_PWRD_07_Y,
    MOT_PWRD_08_T,
    MOT_PWRD_08_P,
    MOT_PWRD_08_R,
    MOT_PWRD_08_Y,
    MOT_PWRD_09_T,
    MOT_PWRD_09_P,
    MOT_PWRD_09_R,
    MOT_PWRD_09_Y,
    MOT_PWRD_10_T,
    MOT_PWRD_10_P,
    MOT_PWRD_10_R,
    MOT_PWRD_10_Y,
    MOT_PWRD_11_T,
    MOT_PWRD_11_P,
    MOT_PWRD_11_R,
    MOT_PWRD_11_Y,
    MOT_PWRD_12_T,
    MOT_PWRD_12_P,
    MOT_PWRD_12_R,
    MOT_PWRD_12_Y,
    MOT_PWRD_13_T,
    MOT_PWRD_13_P,
    MOT_PWRD_13_R,
    MOT_PWRD_13_Y,
    MOT_PWRD_14_T,
    MOT_PWRD_14_P,
    MOT_PWRD_14_R,
    MOT_PWRD_14_Y,
    MOT_PWRD_15_T,
    MOT_PWRD_15_P,
    MOT_PWRD_15_R,
    MOT_PWRD_15_Y,
    MOT_PWRD_16_T,
    MOT_PWRD_16_P,
    MOT_PWRD_16_R,
    MOT_PWRD_16_Y,
    COMM_BAUD1,
    COMM_BAUD2,
    COMM_BAUD3,
    COMM_BAUD4,
    COMM_BAUD5,
    COMM_BAUD6,
    COMM_BAUD7,
    COMM_STREAM_TYP1,
    COMM_STREAM_TYP2,
    COMM_STREAM_TYP3,
    COMM_STREAM_TYP4,
    COMM_STREAM_TYP5,
    COMM_STREAM_TYP6,
    COMM_STREAM_TYP7,
    TELEMETRY_RATE,
    NAV_MAX_SPEED,
    NAV_MAX_DECENT,
    NAV_CEILING,
    NAV_HDFRE_CHAN,
    NAV_LANDING_VEL,
    NAV_SPEED_P,
    NAV_SPEED_I,
    NAV_SPEED_PM,
    NAV_SPEED_IM,
    NAV_SPEED_OM,
    NAV_DIST_P,
    NAV_DIST_I,
    NAV_DIST_PM,
    NAV_DIST_IM,
    NAV_DIST_OM,
    NAV_ALT_SPED_P,
    NAV_ALT_SPED_I,
    NAV_ALT_SPED_PM,
    NAV_ALT_SPED_IM,
    NAV_ALT_SPED_OM,
    NAV_ALT_POS_P,
    NAV_ALT_POS_I,
    NAV_ALT_POS_PM,
    NAV_ALT_POS_IM,
    NAV_ALT_POS_OM,
    IMU_ROT,
    IMU_FLIP,
    IMU_ACC_BIAS_X,
    IMU_ACC_BIAS_Y,
    IMU_ACC_BIAS_Z,
    IMU_ACC_BIAS1_X,
    IMU_ACC_BIAS1_Y,
    IMU_ACC_BIAS1_Z,
    IMU_ACC_BIAS2_X,
    IMU_ACC_BIAS2_Y,
    IMU_ACC_BIAS2_Z,
    IMU_ACC_BIAS3_X,
    IMU_ACC_BIAS3_Y,
    IMU_ACC_BIAS3_Z,
    IMU_ACC_SCAL_X,
    IMU_ACC_SCAL_Y,
    IMU_ACC_SCAL_Z,
    IMU_ACC_SCAL1_X,
    IMU_ACC_SCAL1_Y,
    IMU_ACC_SCAL1_Z,
    IMU_ACC_SCAL2_X,
    IMU_ACC_SCAL2_Y,
    IMU_ACC_SCAL2_Z,
    IMU_ACC_SCAL3_X,
    IMU_ACC_SCAL3_Y,
    IMU_ACC_SCAL3_Z,
    IMU_ACC_ALGN_XY,
    IMU_ACC_ALGN_XZ,
    IMU_ACC_ALGN_YX,
    IMU_ACC_ALGN_YZ,
    IMU_ACC_ALGN_ZX,
    IMU_ACC_ALGN_ZY,
    IMU_MAG_BIAS_X,
    IMU_MAG_BIAS_Y,
    IMU_MAG_BIAS_Z,
    IMU_MAG_BIAS1_X,
    IMU_MAG_BIAS1_Y,
    IMU_MAG_BIAS1_Z,
    IMU_MAG_BIAS2_X,
    IMU_MAG_BIAS2_Y,
    IMU_MAG_BIAS2_Z,
    IMU_MAG_BIAS3_X,
    IMU_MAG_BIAS3_Y,
    IMU_MAG_BIAS3_Z,
    IMU_MAG_SCAL_X,
    IMU_MAG_SCAL_Y,
    IMU_MAG_SCAL_Z,
    IMU_MAG_SCAL1_X,
    IMU_MAG_SCAL1_Y,
    IMU_MAG_SCAL1_Z,
    IMU_MAG_SCAL2_X,
    IMU_MAG_SCAL2_Y,
    IMU_MAG_SCAL2_Z,
    IMU_MAG_SCAL3_X,
    IMU_MAG_SCAL3_Y,
    IMU_MAG_SCAL3_Z,
    IMU_MAG_ALGN_XY,
    IMU_MAG_ALGN_XZ,
    IMU_MAG_ALGN_YX,
    IMU_MAG_ALGN_YZ,
    IMU_MAG_ALGN_ZX,
    IMU_MAG_ALGN_ZY,
    IMU_GYO_BIAS_X,
    IMU_GYO_BIAS_Y,
    IMU_GYO_BIAS_Z,
    IMU_GYO_BIAS1_X,
    IMU_GYO_BIAS1_Y,
    IMU_GYO_BIAS1_Z,
    IMU_GYO_BIAS2_X,
    IMU_GYO_BIAS2_Y,
    IMU_GYO_BIAS2_Z,
    IMU_GYO_BIAS3_X,
    IMU_GYO_BIAS3_Y,
    IMU_GYO_BIAS3_Z,
    IMU_GYO_SCAL_X,
    IMU_GYO_SCAL_Y,
    IMU_GYO_SCAL_Z,
    IMU_GYO_ALGN_XY,
    IMU_GYO_ALGN_XZ,
    IMU_GYO_ALGN_YX,
    IMU_GYO_ALGN_YZ,
    IMU_GYO_ALGN_ZX,
    IMU_GYO_ALGN_ZY,
    IMU_MAG_INCL,
    IMU_MAG_DECL,
    IMU_PRESS_SENSE,
    GMBL_PITCH_PORT,
    GMBL_ROLL_PORT,
    GMBL_PWM_MAX_RL,
    GMBL_PWM_MIN_RL,
    GMBL_PWM_MAX_PT,
    GMBL_PWM_MIN_PT,
    GMBL_PWM_FREQ,
    GMBL_NTRL_PITCH,
    GMBL_NTRL_ROLL,
    GMBL_SCAL_PITCH,
    GMBL_SCAL_ROLL,
    GMBL_SLEW_RATE,
    GMBL_ROLL_EXPO,
    GMBL_TILT_PORT,
    GMBL_TRIG_PORT,
    GMBL_TRIG_CH_NEU,
    GMBL_TRIG_ON_PWM,
    GMBL_TRIG_ON_TIM,
    GMBL_TRIG_DIST,
    GMBL_TRIG_TIME,
    GMBL_PSTHR_CHAN,
    GMBL_PSTHR_PORT,
    SPVR_LOW_BAT1,
    SPVR_LOW_BAT2,
    SPVR_BAT_CRV1,
    SPVR_BAT_CRV2,
    SPVR_BAT_CRV3,
    SPVR_BAT_CRV4,
    SPVR_BAT_CRV5,
    SPVR_BAT_CRV6,
    SPVR_FS_RAD_ST1,
    SPVR_FS_RAD_ST2,
    SPVR_FS_ADD_ALT,
    SPVR_VIN_SOURCE,
    QUATOS_J_ROLL,
    QUATOS_J_PITCH,
    QUATOS_J_YAW,
    QUATOS_AM1,
    QUATOS_AM2,
    QUATOS_PROP_K1,
    QUATOS_M_TLT_RT,
    QUATOS_M_YAW_RT,
    QUATOS_MAX_OUT,
    QUATOS_QUAT_TAU,
    QUATOS_L1_ASP,
    QUATOS_L1_K1,
    QUATOS_AM1_KNOB,
    QUATOS_AM2_KNOB,
    QUATOS_K1_KNOB,
    QUATOS_PT_KNOB,
    QUATOS_MM_R01,
    QUATOS_MM_P01,
    QUATOS_MM_Y01,
    QUATOS_MM_R02,
    QUATOS_MM_P02,
    QUATOS_MM_Y02,
    QUATOS_MM_R03,
    QUATOS_MM_P03,
    QUATOS_MM_Y03,
    QUATOS_MM_R04,
    QUATOS_MM_P04,
    QUATOS_MM_Y04,
    QUATOS_MM_R05,
    QUATOS_MM_P05,
    QUATOS_MM_Y05,
    QUATOS_MM_R06,
    QUATOS_MM_P06,
    QUATOS_MM_Y06,
    QUATOS_MM_R07,
    QUATOS_MM_P07,
    QUATOS_MM_Y07,
    QUATOS_MM_R08,
    QUATOS_MM_P08,
    QUATOS_MM_Y08,
    QUATOS_MM_R09,
    QUATOS_MM_P09,
    QUATOS_MM_Y09,
    QUATOS_MM_R10,
    QUATOS_MM_P10,
    QUATOS_MM_Y10,
    QUATOS_MM_R11,
    QUATOS_MM_P11,
    QUATOS_MM_Y11,
    QUATOS_MM_R12,
    QUATOS_MM_P12,
    QUATOS_MM_Y12,
    QUATOS_MM_R13,
    QUATOS_MM_P13,
    QUATOS_MM_Y13,
    QUATOS_MM_R14,
    QUATOS_MM_P14,
    QUATOS_MM_Y14,
    QUATOS_MM_R15,
    QUATOS_MM_P15,
    QUATOS_MM_Y15,
    QUATOS_MM_R16,
    QUATOS_MM_P16,
    QUATOS_MM_Y16,
    SIG_LED_1_PRT,
    SIG_LED_2_PRT,
    SIG_BEEP_PRT,
    SIG_PWM_PRT,
    LIC_KEY1,
    LIC_KEY2,
    LIC_KEY3,
    CONFIG_NUM_PARAMS
};

typedef struct {
    uint32_t key;
    char data[24];
} configToken_t;

typedef struct {
    char name[16];
    float val;
} configRec_t;

typedef struct {
    unsigned int paramId;
    unsigned int num;
    float values[CONFIG_NUM_PARAMS];
} __attribute__((packed)) paramStruct_t;

extern float p[CONFIG_NUM_PARAMS];
extern const char *configParameterStrings[];

extern void configInit(void);
extern void configFlashRead(void);
extern uint8_t configFlashWrite(void);
extern void configLoadDefault(void);
extern unsigned int configParameterRead(void *data);
extern unsigned int configParameterWrite(void *data);
extern int8_t configReadFile(char *fname);
extern int8_t configWriteFile(char *fname);
extern void configSetParamByID(int id, float value);
extern int configGetParamIdByName(char *name);
extern int8_t configFormatParam(char *buf, int n);
extern int configParseParams(char *fileBuf, int size, int p1);
extern configToken_t *configTokenGet(uint32_t key);
extern void configTokenStore(configToken_t *token);

#endif
