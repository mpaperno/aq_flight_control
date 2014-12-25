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
#include "flash.h"
#include "filer.h"
#include "comm.h"
#include "supervisor.h"
#include "util.h"
//#include CONFIG_HEADER
#include <string.h>
#include <stdio.h>
#include <math.h>

float p[CONFIG_NUM_PARAMS] __attribute__((section(".ccm")));

const char *configParameterStrings[] = {
    "CONFIG_VERSION",
    "RADIO_TYPE",
    "RADIO_SETUP",
    "RADIO_THRO_CH",
    "RADIO_ROLL_CH",
    "RADIO_PITC_CH",
    "RADIO_RUDD_CH",
    "RADIO_GEAR_CH",
    "RADIO_FLAP_CH",
    "RADIO_AUX2_CH",
    "RADIO_AUX3_CH",
    "RADIO_AUX4_CH",
    "RADIO_AUX5_CH",
    "RADIO_AUX6_CH",
    "RADIO_AUX7_CH",
    "PPM_SCALER",
    "PPM_THROT_LOW",
    "PPM_CHAN_MID",
    "CTRL_PID_TYPE",
    "CTRL_FACT_THRO",
    "CTRL_FACT_PITC",
    "CTRL_FACT_ROLL",
    "CTRL_MAN_YAW_RT",
    "CTRL_DEAD_BAND",
    "CTRL_DBAND_THRO",
    "CTRL_MIN_THROT",
    "CTRL_MAX",
    "CTRL_NAV_YAW_RT",
    "CTRL_TLT_RTE_P",
    "CTRL_TLT_RTE_I",
    "CTRL_TLT_RTE_D",
    "CTRL_TLT_RTE_F",
    "CTRL_TLT_RTE_PM",
    "CTRL_TLT_RTE_IM",
    "CTRL_TLT_RTE_DM",
    "CTRL_TLT_RTE_OM",
    "CTRL_YAW_RTE_P",
    "CTRL_YAW_RTE_I",
    "CTRL_YAW_RTE_D",
    "CTRL_YAW_RTE_F",
    "CTRL_YAW_RTE_PM",
    "CTRL_YAW_RTE_IM",
    "CTRL_YAW_RTE_DM",
    "CTRL_YAW_RTE_OM",
    "CTRL_TLT_ANG_P",
    "CTRL_TLT_ANG_I",
    "CTRL_TLT_ANG_D",
    "CTRL_TLT_ANG_F",
    "CTRL_TLT_ANG_PM",
    "CTRL_TLT_ANG_IM",
    "CTRL_TLT_ANG_DM",
    "CTRL_TLT_ANG_OM",
    "CTRL_YAW_ANG_P",
    "CTRL_YAW_ANG_I",
    "CTRL_YAW_ANG_D",
    "CTRL_YAW_ANG_F",
    "CTRL_YAW_ANG_PM",
    "CTRL_YAW_ANG_IM",
    "CTRL_YAW_ANG_DM",
    "CTRL_YAW_ANG_OM",
    "MOT_CANL",
    "MOT_CANH",
    "MOT_FRAME",
    "MOT_ARM",
    "MOT_MIN",
    "MOT_START",
    "MOT_VALUE2T_A1",
    "MOT_VALUE2T_A2",
    "MOT_VALUE_SCAL",
    "MOT_MAX",
    "MOT_PWRD_01_T",
    "MOT_PWRD_01_P",
    "MOT_PWRD_01_R",
    "MOT_PWRD_01_Y",
    "MOT_PWRD_02_T",
    "MOT_PWRD_02_P",
    "MOT_PWRD_02_R",
    "MOT_PWRD_02_Y",
    "MOT_PWRD_03_T",
    "MOT_PWRD_03_P",
    "MOT_PWRD_03_R",
    "MOT_PWRD_03_Y",
    "MOT_PWRD_04_T",
    "MOT_PWRD_04_P",
    "MOT_PWRD_04_R",
    "MOT_PWRD_04_Y",
    "MOT_PWRD_05_T",
    "MOT_PWRD_05_P",
    "MOT_PWRD_05_R",
    "MOT_PWRD_05_Y",
    "MOT_PWRD_06_T",
    "MOT_PWRD_06_P",
    "MOT_PWRD_06_R",
    "MOT_PWRD_06_Y",
    "MOT_PWRD_07_T",
    "MOT_PWRD_07_P",
    "MOT_PWRD_07_R",
    "MOT_PWRD_07_Y",
    "MOT_PWRD_08_T",
    "MOT_PWRD_08_P",
    "MOT_PWRD_08_R",
    "MOT_PWRD_08_Y",
    "MOT_PWRD_09_T",
    "MOT_PWRD_09_P",
    "MOT_PWRD_09_R",
    "MOT_PWRD_09_Y",
    "MOT_PWRD_10_T",
    "MOT_PWRD_10_P",
    "MOT_PWRD_10_R",
    "MOT_PWRD_10_Y",
    "MOT_PWRD_11_T",
    "MOT_PWRD_11_P",
    "MOT_PWRD_11_R",
    "MOT_PWRD_11_Y",
    "MOT_PWRD_12_T",
    "MOT_PWRD_12_P",
    "MOT_PWRD_12_R",
    "MOT_PWRD_12_Y",
    "MOT_PWRD_13_T",
    "MOT_PWRD_13_P",
    "MOT_PWRD_13_R",
    "MOT_PWRD_13_Y",
    "MOT_PWRD_14_T",
    "MOT_PWRD_14_P",
    "MOT_PWRD_14_R",
    "MOT_PWRD_14_Y",
    "MOT_PWRD_15_T",
    "MOT_PWRD_15_P",
    "MOT_PWRD_15_R",
    "MOT_PWRD_15_Y",
    "MOT_PWRD_16_T",
    "MOT_PWRD_16_P",
    "MOT_PWRD_16_R",
    "MOT_PWRD_16_Y",
    "COMM_BAUD1",
    "COMM_BAUD2",
    "COMM_BAUD3",
    "COMM_BAUD4",
    "COMM_BAUD5",
    "COMM_BAUD6",
    "COMM_BAUD7",
    "COMM_STREAM_TYP1",
    "COMM_STREAM_TYP2",
    "COMM_STREAM_TYP3",
    "COMM_STREAM_TYP4",
    "COMM_STREAM_TYP5",
    "COMM_STREAM_TYP6",
    "COMM_STREAM_TYP7",
    "TELEMETRY_RATE",
    "NAV_MAX_SPEED",
    "NAV_MAX_DECENT",
    "NAV_CEILING",
    "NAV_HDFRE_CHAN",
    "NAV_LANDING_VEL",
    "NAV_SPEED_P",
    "NAV_SPEED_I",
    "NAV_SPEED_PM",
    "NAV_SPEED_IM",
    "NAV_SPEED_OM",
    "NAV_DIST_P",
    "NAV_DIST_I",
    "NAV_DIST_PM",
    "NAV_DIST_IM",
    "NAV_DIST_OM",
    "NAV_ALT_SPED_P",
    "NAV_ALT_SPED_I",
    "NAV_ALT_SPED_PM",
    "NAV_ALT_SPED_IM",
    "NAV_ALT_SPED_OM",
    "NAV_ALT_POS_P",
    "NAV_ALT_POS_I",
    "NAV_ALT_POS_PM",
    "NAV_ALT_POS_IM",
    "NAV_ALT_POS_OM",
    "IMU_ROT",
    "IMU_FLIP",
    "IMU_ACC_BIAS_X",
    "IMU_ACC_BIAS_Y",
    "IMU_ACC_BIAS_Z",
    "IMU_ACC_BIAS1_X",
    "IMU_ACC_BIAS1_Y",
    "IMU_ACC_BIAS1_Z",
    "IMU_ACC_BIAS2_X",
    "IMU_ACC_BIAS2_Y",
    "IMU_ACC_BIAS2_Z",
    "IMU_ACC_BIAS3_X",
    "IMU_ACC_BIAS3_Y",
    "IMU_ACC_BIAS3_Z",
    "IMU_ACC_SCAL_X",
    "IMU_ACC_SCAL_Y",
    "IMU_ACC_SCAL_Z",
    "IMU_ACC_SCAL1_X",
    "IMU_ACC_SCAL1_Y",
    "IMU_ACC_SCAL1_Z",
    "IMU_ACC_SCAL2_X",
    "IMU_ACC_SCAL2_Y",
    "IMU_ACC_SCAL2_Z",
    "IMU_ACC_SCAL3_X",
    "IMU_ACC_SCAL3_Y",
    "IMU_ACC_SCAL3_Z",
    "IMU_ACC_ALGN_XY",
    "IMU_ACC_ALGN_XZ",
    "IMU_ACC_ALGN_YX",
    "IMU_ACC_ALGN_YZ",
    "IMU_ACC_ALGN_ZX",
    "IMU_ACC_ALGN_ZY",
    "IMU_MAG_BIAS_X",
    "IMU_MAG_BIAS_Y",
    "IMU_MAG_BIAS_Z",
    "IMU_MAG_BIAS1_X",
    "IMU_MAG_BIAS1_Y",
    "IMU_MAG_BIAS1_Z",
    "IMU_MAG_BIAS2_X",
    "IMU_MAG_BIAS2_Y",
    "IMU_MAG_BIAS2_Z",
    "IMU_MAG_BIAS3_X",
    "IMU_MAG_BIAS3_Y",
    "IMU_MAG_BIAS3_Z",
    "IMU_MAG_SCAL_X",
    "IMU_MAG_SCAL_Y",
    "IMU_MAG_SCAL_Z",
    "IMU_MAG_SCAL1_X",
    "IMU_MAG_SCAL1_Y",
    "IMU_MAG_SCAL1_Z",
    "IMU_MAG_SCAL2_X",
    "IMU_MAG_SCAL2_Y",
    "IMU_MAG_SCAL2_Z",
    "IMU_MAG_SCAL3_X",
    "IMU_MAG_SCAL3_Y",
    "IMU_MAG_SCAL3_Z",
    "IMU_MAG_ALGN_XY",
    "IMU_MAG_ALGN_XZ",
    "IMU_MAG_ALGN_YX",
    "IMU_MAG_ALGN_YZ",
    "IMU_MAG_ALGN_ZX",
    "IMU_MAG_ALGN_ZY",
    "IMU_GYO_BIAS_X",
    "IMU_GYO_BIAS_Y",
    "IMU_GYO_BIAS_Z",
    "IMU_GYO_BIAS1_X",
    "IMU_GYO_BIAS1_Y",
    "IMU_GYO_BIAS1_Z",
    "IMU_GYO_BIAS2_X",
    "IMU_GYO_BIAS2_Y",
    "IMU_GYO_BIAS2_Z",
    "IMU_GYO_BIAS3_X",
    "IMU_GYO_BIAS3_Y",
    "IMU_GYO_BIAS3_Z",
    "IMU_GYO_SCAL_X",
    "IMU_GYO_SCAL_Y",
    "IMU_GYO_SCAL_Z",
    "IMU_GYO_ALGN_XY",
    "IMU_GYO_ALGN_XZ",
    "IMU_GYO_ALGN_YX",
    "IMU_GYO_ALGN_YZ",
    "IMU_GYO_ALGN_ZX",
    "IMU_GYO_ALGN_ZY",
    "IMU_MAG_INCL",
    "IMU_MAG_DECL",
    "IMU_PRESS_SENSE",
    "GMBL_PITCH_PORT",
    "GMBL_ROLL_PORT",
    "GMBL_PWM_MAX_RL",
    "GMBL_PWM_MIN_RL",
    "GMBL_PWM_MAX_PT",
    "GMBL_PWM_MIN_PT",
    "GMBL_PWM_FREQ",
    "GMBL_NTRL_PITCH",
    "GMBL_NTRL_ROLL",
    "GMBL_SCAL_PITCH",
    "GMBL_SCAL_ROLL",
    "GMBL_SLEW_RATE",
    "GMBL_ROLL_EXPO",
    "GMBL_TILT_PORT",
    "GMBL_TRIG_PORT",
    "GMBL_TRIG_CH_NEU",
    "GMBL_TRIG_ON_PWM",
    "GMBL_TRIG_ON_TIM",
    "GMBL_TRIG_DIST",
    "GMBL_TRIG_TIME",
    "GMBL_PSTHR_CHAN",
    "GMBL_PSTHR_PORT",
    "SPVR_LOW_BAT1",
    "SPVR_LOW_BAT2",
    "SPVR_BAT_CRV1",
    "SPVR_BAT_CRV2",
    "SPVR_BAT_CRV3",
    "SPVR_BAT_CRV4",
    "SPVR_BAT_CRV5",
    "SPVR_BAT_CRV6",
    "SPVR_FS_RAD_ST1",
    "SPVR_FS_RAD_ST2",
    "SPVR_FS_ADD_ALT",
    "SPVR_VIN_SOURCE",
    "QUATOS_J_ROLL",
    "QUATOS_J_PITCH",
    "QUATOS_J_YAW",
    "QUATOS_AM1",
    "QUATOS_AM2",
    "QUATOS_PROP_K1",
    "QUATOS_M_TLT_RT",
    "QUATOS_M_YAW_RT",
    "QUATOS_MAX_OUT",
    "QUATOS_QUAT_TAU",
    "QUATOS_L1_ASP",
    "QUATOS_L1_K1",
    "QUATOS_AM1_KNOB",
    "QUATOS_AM2_KNOB",
    "QUATOS_K1_KNOB",
    "QUATOS_PT_KNOB",
    "QUATOS_MM_R01",
    "QUATOS_MM_P01",
    "QUATOS_MM_Y01",
    "QUATOS_MM_R02",
    "QUATOS_MM_P02",
    "QUATOS_MM_Y02",
    "QUATOS_MM_R03",
    "QUATOS_MM_P03",
    "QUATOS_MM_Y03",
    "QUATOS_MM_R04",
    "QUATOS_MM_P04",
    "QUATOS_MM_Y04",
    "QUATOS_MM_R05",
    "QUATOS_MM_P05",
    "QUATOS_MM_Y05",
    "QUATOS_MM_R06",
    "QUATOS_MM_P06",
    "QUATOS_MM_Y06",
    "QUATOS_MM_R07",
    "QUATOS_MM_P07",
    "QUATOS_MM_Y07",
    "QUATOS_MM_R08",
    "QUATOS_MM_P08",
    "QUATOS_MM_Y08",
    "QUATOS_MM_R09",
    "QUATOS_MM_P09",
    "QUATOS_MM_Y09",
    "QUATOS_MM_R10",
    "QUATOS_MM_P10",
    "QUATOS_MM_Y10",
    "QUATOS_MM_R11",
    "QUATOS_MM_P11",
    "QUATOS_MM_Y11",
    "QUATOS_MM_R12",
    "QUATOS_MM_P12",
    "QUATOS_MM_Y12",
    "QUATOS_MM_R13",
    "QUATOS_MM_P13",
    "QUATOS_MM_Y13",
    "QUATOS_MM_R14",
    "QUATOS_MM_P14",
    "QUATOS_MM_Y14",
    "QUATOS_MM_R15",
    "QUATOS_MM_P15",
    "QUATOS_MM_Y15",
    "QUATOS_MM_R16",
    "QUATOS_MM_P16",
    "QUATOS_MM_Y16",
    "SIG_LED_1_PRT",
    "SIG_LED_2_PRT",
    "SIG_BEEP_PRT",
    "SIG_PWM_PRT",
    "LIC_KEY1",
    "LIC_KEY2",
    "LIC_KEY3"
};

void configLoadDefault(void) {
    p[CONFIG_VERSION] = DEFAULT_CONFIG_VERSION;
    p[RADIO_TYPE] = DEFAULT_RADIO_TYPE;
    p[RADIO_SETUP] = DEFAULT_RADIO_SETUP;
    p[RADIO_THRO_CH] = DEFAULT_RADIO_THRO_CH;
    p[RADIO_ROLL_CH] = DEFAULT_RADIO_ROLL_CH;
    p[RADIO_PITC_CH] = DEFAULT_RADIO_PITC_CH;
    p[RADIO_RUDD_CH] = DEFAULT_RADIO_RUDD_CH;
    p[RADIO_GEAR_CH] = DEFAULT_RADIO_GEAR_CH;
    p[RADIO_FLAP_CH] = DEFAULT_RADIO_FLAP_CH;
    p[RADIO_AUX2_CH] = DEFAULT_RADIO_AUX2_CH;
    p[RADIO_AUX3_CH] = DEFAULT_RADIO_AUX3_CH;
    p[RADIO_AUX4_CH] = DEFAULT_RADIO_AUX4_CH;
    p[RADIO_AUX5_CH] = DEFAULT_RADIO_AUX5_CH;
    p[RADIO_AUX6_CH] = DEFAULT_RADIO_AUX6_CH;
    p[RADIO_AUX7_CH] = DEFAULT_RADIO_AUX7_CH;
    p[PPM_SCALER] = DEFAULT_PPM_SCALER;
    p[PPM_THROT_LOW] = DEFAULT_PPM_THROT_LOW;
    p[PPM_CHAN_MID] = DEFAULT_PPM_CHAN_MID;
    p[CTRL_PID_TYPE] = DEFAULT_CTRL_PID_TYPE;
    p[CTRL_FACT_THRO] = DEFAULT_CTRL_FACT_THRO;
    p[CTRL_FACT_PITC] = DEFAULT_CTRL_FACT_PITC;
    p[CTRL_FACT_ROLL] = DEFAULT_CTRL_FACT_ROLL;
    p[CTRL_MAN_YAW_RT] = DEFAULT_CTRL_MAN_YAW_RT;
    p[CTRL_DEAD_BAND] = DEFAULT_CTRL_DEAD_BAND;
    p[CTRL_DBAND_THRO] = DEFAULT_CTRL_DBAND_THRO;
    p[CTRL_MIN_THROT] = DEFAULT_CTRL_MIN_THROT;
    p[CTRL_MAX] = DEFAULT_CTRL_MAX;
    p[CTRL_NAV_YAW_RT] = DEFAULT_CTRL_NAV_YAW_RT;
    p[CTRL_TLT_RTE_P] = DEFAULT_CTRL_TLT_RTE_P;
    p[CTRL_TLT_RTE_I] = DEFAULT_CTRL_TLT_RTE_I;
    p[CTRL_TLT_RTE_D] = DEFAULT_CTRL_TLT_RTE_D;
    p[CTRL_TLT_RTE_F] = DEFAULT_CTRL_TLT_RTE_F;
    p[CTRL_TLT_RTE_PM] = DEFAULT_CTRL_TLT_RTE_PM;
    p[CTRL_TLT_RTE_IM] = DEFAULT_CTRL_TLT_RTE_IM;
    p[CTRL_TLT_RTE_DM] = DEFAULT_CTRL_TLT_RTE_DM;
    p[CTRL_TLT_RTE_OM] = DEFAULT_CTRL_TLT_RTE_OM;
    p[CTRL_YAW_RTE_P] = DEFAULT_CTRL_YAW_RTE_P;
    p[CTRL_YAW_RTE_I] = DEFAULT_CTRL_YAW_RTE_I;
    p[CTRL_YAW_RTE_D] = DEFAULT_CTRL_YAW_RTE_D;
    p[CTRL_YAW_RTE_F] = DEFAULT_CTRL_YAW_RTE_F;
    p[CTRL_YAW_RTE_PM] = DEFAULT_CTRL_YAW_RTE_PM;
    p[CTRL_YAW_RTE_IM] = DEFAULT_CTRL_YAW_RTE_IM;
    p[CTRL_YAW_RTE_DM] = DEFAULT_CTRL_YAW_RTE_DM;
    p[CTRL_YAW_RTE_OM] = DEFAULT_CTRL_YAW_RTE_OM;
    p[CTRL_TLT_ANG_P] = DEFAULT_CTRL_TLT_ANG_P;
    p[CTRL_TLT_ANG_I] = DEFAULT_CTRL_TLT_ANG_I;
    p[CTRL_TLT_ANG_D] = DEFAULT_CTRL_TLT_ANG_D;
    p[CTRL_TLT_ANG_F] = DEFAULT_CTRL_TLT_ANG_F;
    p[CTRL_TLT_ANG_PM] = DEFAULT_CTRL_TLT_ANG_PM;
    p[CTRL_TLT_ANG_IM] = DEFAULT_CTRL_TLT_ANG_IM;
    p[CTRL_TLT_ANG_DM] = DEFAULT_CTRL_TLT_ANG_DM;
    p[CTRL_TLT_ANG_OM] = DEFAULT_CTRL_TLT_ANG_OM;
    p[CTRL_YAW_ANG_P] = DEFAULT_CTRL_YAW_ANG_P;
    p[CTRL_YAW_ANG_I] = DEFAULT_CTRL_YAW_ANG_I;
    p[CTRL_YAW_ANG_D] = DEFAULT_CTRL_YAW_ANG_D;
    p[CTRL_YAW_ANG_F] = DEFAULT_CTRL_YAW_ANG_F;
    p[CTRL_YAW_ANG_PM] = DEFAULT_CTRL_YAW_ANG_PM;
    p[CTRL_YAW_ANG_IM] = DEFAULT_CTRL_YAW_ANG_IM;
    p[CTRL_YAW_ANG_DM] = DEFAULT_CTRL_YAW_ANG_DM;
    p[CTRL_YAW_ANG_OM] = DEFAULT_CTRL_YAW_ANG_OM;
    p[MOT_CANL] = DEFAULT_MOT_CANL;
    p[MOT_CANH] = DEFAULT_MOT_CANH;
    p[MOT_FRAME] = DEFAULT_MOT_FRAME;
    p[MOT_ARM] = DEFAULT_MOT_ARM;
    p[MOT_MIN] = DEFAULT_MOT_MIN;
    p[MOT_START] = DEFAULT_MOT_START;
    p[MOT_VALUE2T_A1] = DEFAULT_MOT_VALUE2T_A1;
    p[MOT_VALUE2T_A2] = DEFAULT_MOT_VALUE2T_A2;
    p[MOT_VALUE_SCAL] = DEFAULT_MOT_VALUE_SCAL;
    p[MOT_MAX] = DEFAULT_MOT_MAX;
    p[MOT_PWRD_01_T] = DEFAULT_MOT_PWRD_01_T;
    p[MOT_PWRD_01_P] = DEFAULT_MOT_PWRD_01_P;
    p[MOT_PWRD_01_R] = DEFAULT_MOT_PWRD_01_R;
    p[MOT_PWRD_01_Y] = DEFAULT_MOT_PWRD_01_Y;
    p[MOT_PWRD_02_T] = DEFAULT_MOT_PWRD_02_T;
    p[MOT_PWRD_02_P] = DEFAULT_MOT_PWRD_02_P;
    p[MOT_PWRD_02_R] = DEFAULT_MOT_PWRD_02_R;
    p[MOT_PWRD_02_Y] = DEFAULT_MOT_PWRD_02_Y;
    p[MOT_PWRD_03_T] = DEFAULT_MOT_PWRD_03_T;
    p[MOT_PWRD_03_P] = DEFAULT_MOT_PWRD_03_P;
    p[MOT_PWRD_03_R] = DEFAULT_MOT_PWRD_03_R;
    p[MOT_PWRD_03_Y] = DEFAULT_MOT_PWRD_03_Y;
    p[MOT_PWRD_04_T] = DEFAULT_MOT_PWRD_04_T;
    p[MOT_PWRD_04_P] = DEFAULT_MOT_PWRD_04_P;
    p[MOT_PWRD_04_R] = DEFAULT_MOT_PWRD_04_R;
    p[MOT_PWRD_04_Y] = DEFAULT_MOT_PWRD_04_Y;
    p[MOT_PWRD_05_T] = DEFAULT_MOT_PWRD_05_T;
    p[MOT_PWRD_05_P] = DEFAULT_MOT_PWRD_05_P;
    p[MOT_PWRD_05_R] = DEFAULT_MOT_PWRD_05_R;
    p[MOT_PWRD_05_Y] = DEFAULT_MOT_PWRD_05_Y;
    p[MOT_PWRD_06_T] = DEFAULT_MOT_PWRD_06_T;
    p[MOT_PWRD_06_P] = DEFAULT_MOT_PWRD_06_P;
    p[MOT_PWRD_06_R] = DEFAULT_MOT_PWRD_06_R;
    p[MOT_PWRD_06_Y] = DEFAULT_MOT_PWRD_06_Y;
    p[MOT_PWRD_07_T] = DEFAULT_MOT_PWRD_07_T;
    p[MOT_PWRD_07_P] = DEFAULT_MOT_PWRD_07_P;
    p[MOT_PWRD_07_R] = DEFAULT_MOT_PWRD_07_R;
    p[MOT_PWRD_07_Y] = DEFAULT_MOT_PWRD_07_Y;
    p[MOT_PWRD_08_T] = DEFAULT_MOT_PWRD_08_T;
    p[MOT_PWRD_08_P] = DEFAULT_MOT_PWRD_08_P;
    p[MOT_PWRD_08_R] = DEFAULT_MOT_PWRD_08_R;
    p[MOT_PWRD_08_Y] = DEFAULT_MOT_PWRD_08_Y;
    p[MOT_PWRD_09_T] = DEFAULT_MOT_PWRD_09_T;
    p[MOT_PWRD_09_P] = DEFAULT_MOT_PWRD_09_P;
    p[MOT_PWRD_09_R] = DEFAULT_MOT_PWRD_09_R;
    p[MOT_PWRD_09_Y] = DEFAULT_MOT_PWRD_09_Y;
    p[MOT_PWRD_10_T] = DEFAULT_MOT_PWRD_10_T;
    p[MOT_PWRD_10_P] = DEFAULT_MOT_PWRD_10_P;
    p[MOT_PWRD_10_R] = DEFAULT_MOT_PWRD_10_R;
    p[MOT_PWRD_10_Y] = DEFAULT_MOT_PWRD_10_Y;
    p[MOT_PWRD_11_T] = DEFAULT_MOT_PWRD_11_T;
    p[MOT_PWRD_11_P] = DEFAULT_MOT_PWRD_11_P;
    p[MOT_PWRD_11_R] = DEFAULT_MOT_PWRD_11_R;
    p[MOT_PWRD_11_Y] = DEFAULT_MOT_PWRD_11_Y;
    p[MOT_PWRD_12_T] = DEFAULT_MOT_PWRD_12_T;
    p[MOT_PWRD_12_P] = DEFAULT_MOT_PWRD_12_P;
    p[MOT_PWRD_12_R] = DEFAULT_MOT_PWRD_12_R;
    p[MOT_PWRD_12_Y] = DEFAULT_MOT_PWRD_12_Y;
    p[MOT_PWRD_13_T] = DEFAULT_MOT_PWRD_13_T;
    p[MOT_PWRD_13_P] = DEFAULT_MOT_PWRD_13_P;
    p[MOT_PWRD_13_R] = DEFAULT_MOT_PWRD_13_R;
    p[MOT_PWRD_13_Y] = DEFAULT_MOT_PWRD_13_Y;
    p[MOT_PWRD_14_T] = DEFAULT_MOT_PWRD_14_T;
    p[MOT_PWRD_14_P] = DEFAULT_MOT_PWRD_14_P;
    p[MOT_PWRD_14_R] = DEFAULT_MOT_PWRD_14_R;
    p[MOT_PWRD_14_Y] = DEFAULT_MOT_PWRD_14_Y;
    p[MOT_PWRD_15_T] = DEFAULT_MOT_PWRD_15_T;
    p[MOT_PWRD_15_P] = DEFAULT_MOT_PWRD_15_P;
    p[MOT_PWRD_15_R] = DEFAULT_MOT_PWRD_15_R;
    p[MOT_PWRD_15_Y] = DEFAULT_MOT_PWRD_15_Y;
    p[MOT_PWRD_16_T] = DEFAULT_MOT_PWRD_16_T;
    p[MOT_PWRD_16_P] = DEFAULT_MOT_PWRD_16_P;
    p[MOT_PWRD_16_R] = DEFAULT_MOT_PWRD_16_R;
    p[MOT_PWRD_16_Y] = DEFAULT_MOT_PWRD_16_Y;
    p[COMM_BAUD1] = DEFAULT_COMM_BAUD1;
    p[COMM_BAUD2] = DEFAULT_COMM_BAUD2;
    p[COMM_BAUD3] = DEFAULT_COMM_BAUD3;
    p[COMM_BAUD4] = DEFAULT_COMM_BAUD4;
    p[COMM_BAUD5] = DEFAULT_COMM_BAUD5;
    p[COMM_BAUD6] = DEFAULT_COMM_BAUD6;
    p[COMM_BAUD7] = DEFAULT_COMM_BAUD7;
    p[COMM_STREAM_TYP1] = DEFAULT_COMM_STREAM_TYP1;
    p[COMM_STREAM_TYP2] = DEFAULT_COMM_STREAM_TYP2;
    p[COMM_STREAM_TYP3] = DEFAULT_COMM_STREAM_TYP3;
    p[COMM_STREAM_TYP4] = DEFAULT_COMM_STREAM_TYP4;
    p[COMM_STREAM_TYP5] = DEFAULT_COMM_STREAM_TYP5;
    p[COMM_STREAM_TYP6] = DEFAULT_COMM_STREAM_TYP6;
    p[COMM_STREAM_TYP7] = DEFAULT_COMM_STREAM_TYP7;
    p[TELEMETRY_RATE] = DEFAULT_TELEMETRY_RATE;
    p[NAV_MAX_SPEED] = DEFAULT_NAV_MAX_SPEED;
    p[NAV_MAX_DECENT] = DEFAULT_NAV_MAX_DECENT;
    p[NAV_CEILING] = DEFAULT_NAV_CEILING;
    p[NAV_HDFRE_CHAN] = DEFAULT_NAV_HDFRE_CHAN;
    p[NAV_LANDING_VEL] = DEFAULT_NAV_LANDING_VEL;
    p[NAV_SPEED_P] = DEFAULT_NAV_SPEED_P;
    p[NAV_SPEED_I] = DEFAULT_NAV_SPEED_I;
    p[NAV_SPEED_PM] = DEFAULT_NAV_SPEED_PM;
    p[NAV_SPEED_IM] = DEFAULT_NAV_SPEED_IM;
    p[NAV_SPEED_OM] = DEFAULT_NAV_SPEED_OM;
    p[NAV_DIST_P] = DEFAULT_NAV_DIST_P;
    p[NAV_DIST_I] = DEFAULT_NAV_DIST_I;
    p[NAV_DIST_PM] = DEFAULT_NAV_DIST_PM;
    p[NAV_DIST_IM] = DEFAULT_NAV_DIST_IM;
    p[NAV_DIST_OM] = DEFAULT_NAV_DIST_OM;
    p[NAV_ALT_SPED_P] = DEFAULT_NAV_ALT_SPED_P;
    p[NAV_ALT_SPED_I] = DEFAULT_NAV_ALT_SPED_I;
    p[NAV_ALT_SPED_PM] = DEFAULT_NAV_ALT_SPED_PM;
    p[NAV_ALT_SPED_IM] = DEFAULT_NAV_ALT_SPED_IM;
    p[NAV_ALT_SPED_OM] = DEFAULT_NAV_ALT_SPED_OM;
    p[NAV_ALT_POS_P] = DEFAULT_NAV_ALT_POS_P;
    p[NAV_ALT_POS_I] = DEFAULT_NAV_ALT_POS_I;
    p[NAV_ALT_POS_PM] = DEFAULT_NAV_ALT_POS_PM;
    p[NAV_ALT_POS_IM] = DEFAULT_NAV_ALT_POS_IM;
    p[NAV_ALT_POS_OM] = DEFAULT_NAV_ALT_POS_OM;
    p[IMU_ROT] = DEFAULT_IMU_ROT;
    p[IMU_FLIP] = DEFAULT_IMU_FLIP;
    p[IMU_ACC_BIAS_X] = DEFAULT_IMU_ACC_BIAS_X;
    p[IMU_ACC_BIAS_Y] = DEFAULT_IMU_ACC_BIAS_Y;
    p[IMU_ACC_BIAS_Z] = DEFAULT_IMU_ACC_BIAS_Z;
    p[IMU_ACC_BIAS1_X] = DEFAULT_IMU_ACC_BIAS1_X;
    p[IMU_ACC_BIAS1_Y] = DEFAULT_IMU_ACC_BIAS1_Y;
    p[IMU_ACC_BIAS1_Z] = DEFAULT_IMU_ACC_BIAS1_Z;
    p[IMU_ACC_BIAS2_X] = DEFAULT_IMU_ACC_BIAS2_X;
    p[IMU_ACC_BIAS2_Y] = DEFAULT_IMU_ACC_BIAS2_Y;
    p[IMU_ACC_BIAS2_Z] = DEFAULT_IMU_ACC_BIAS2_Z;
    p[IMU_ACC_BIAS3_X] = DEFAULT_IMU_ACC_BIAS3_X;
    p[IMU_ACC_BIAS3_Y] = DEFAULT_IMU_ACC_BIAS3_Y;
    p[IMU_ACC_BIAS3_Z] = DEFAULT_IMU_ACC_BIAS3_Z;
    p[IMU_ACC_SCAL_X] = DEFAULT_IMU_ACC_SCAL_X;
    p[IMU_ACC_SCAL_Y] = DEFAULT_IMU_ACC_SCAL_Y;
    p[IMU_ACC_SCAL_Z] = DEFAULT_IMU_ACC_SCAL_Z;
    p[IMU_ACC_SCAL1_X] = DEFAULT_IMU_ACC_SCAL1_X;
    p[IMU_ACC_SCAL1_Y] = DEFAULT_IMU_ACC_SCAL1_Y;
    p[IMU_ACC_SCAL1_Z] = DEFAULT_IMU_ACC_SCAL1_Z;
    p[IMU_ACC_SCAL2_X] = DEFAULT_IMU_ACC_SCAL2_X;
    p[IMU_ACC_SCAL2_Y] = DEFAULT_IMU_ACC_SCAL2_Y;
    p[IMU_ACC_SCAL2_Z] = DEFAULT_IMU_ACC_SCAL2_Z;
    p[IMU_ACC_SCAL3_X] = DEFAULT_IMU_ACC_SCAL3_X;
    p[IMU_ACC_SCAL3_Y] = DEFAULT_IMU_ACC_SCAL3_Y;
    p[IMU_ACC_SCAL3_Z] = DEFAULT_IMU_ACC_SCAL3_Z;
    p[IMU_ACC_ALGN_XY] = DEFAULT_IMU_ACC_ALGN_XY;
    p[IMU_ACC_ALGN_XZ] = DEFAULT_IMU_ACC_ALGN_XZ;
    p[IMU_ACC_ALGN_YX] = DEFAULT_IMU_ACC_ALGN_YX;
    p[IMU_ACC_ALGN_YZ] = DEFAULT_IMU_ACC_ALGN_YZ;
    p[IMU_ACC_ALGN_ZX] = DEFAULT_IMU_ACC_ALGN_ZX;
    p[IMU_ACC_ALGN_ZY] = DEFAULT_IMU_ACC_ALGN_ZY;
    p[IMU_MAG_BIAS_X] = DEFAULT_IMU_MAG_BIAS_X;
    p[IMU_MAG_BIAS_Y] = DEFAULT_IMU_MAG_BIAS_Y;
    p[IMU_MAG_BIAS_Z] = DEFAULT_IMU_MAG_BIAS_Z;
    p[IMU_MAG_BIAS1_X] = DEFAULT_IMU_MAG_BIAS1_X;
    p[IMU_MAG_BIAS1_Y] = DEFAULT_IMU_MAG_BIAS1_Y;
    p[IMU_MAG_BIAS1_Z] = DEFAULT_IMU_MAG_BIAS1_Z;
    p[IMU_MAG_BIAS2_X] = DEFAULT_IMU_MAG_BIAS2_X;
    p[IMU_MAG_BIAS2_Y] = DEFAULT_IMU_MAG_BIAS2_Y;
    p[IMU_MAG_BIAS2_Z] = DEFAULT_IMU_MAG_BIAS2_Z;
    p[IMU_MAG_BIAS3_X] = DEFAULT_IMU_MAG_BIAS3_X;
    p[IMU_MAG_BIAS3_Y] = DEFAULT_IMU_MAG_BIAS3_Y;
    p[IMU_MAG_BIAS3_Z] = DEFAULT_IMU_MAG_BIAS3_Z;
    p[IMU_MAG_SCAL_X] = DEFAULT_IMU_MAG_SCAL_X;
    p[IMU_MAG_SCAL_Y] = DEFAULT_IMU_MAG_SCAL_Y;
    p[IMU_MAG_SCAL_Z] = DEFAULT_IMU_MAG_SCAL_Z;
    p[IMU_MAG_SCAL1_X] = DEFAULT_IMU_MAG_SCAL1_X;
    p[IMU_MAG_SCAL1_Y] = DEFAULT_IMU_MAG_SCAL1_Y;
    p[IMU_MAG_SCAL1_Z] = DEFAULT_IMU_MAG_SCAL1_Z;
    p[IMU_MAG_SCAL2_X] = DEFAULT_IMU_MAG_SCAL2_X;
    p[IMU_MAG_SCAL2_Y] = DEFAULT_IMU_MAG_SCAL2_Y;
    p[IMU_MAG_SCAL2_Z] = DEFAULT_IMU_MAG_SCAL2_Z;
    p[IMU_MAG_SCAL3_X] = DEFAULT_IMU_MAG_SCAL3_X;
    p[IMU_MAG_SCAL3_Y] = DEFAULT_IMU_MAG_SCAL3_Y;
    p[IMU_MAG_SCAL3_Z] = DEFAULT_IMU_MAG_SCAL3_Z;
    p[IMU_MAG_ALGN_XY] = DEFAULT_IMU_MAG_ALGN_XY;
    p[IMU_MAG_ALGN_XZ] = DEFAULT_IMU_MAG_ALGN_XZ;
    p[IMU_MAG_ALGN_YX] = DEFAULT_IMU_MAG_ALGN_YX;
    p[IMU_MAG_ALGN_YZ] = DEFAULT_IMU_MAG_ALGN_YZ;
    p[IMU_MAG_ALGN_ZX] = DEFAULT_IMU_MAG_ALGN_ZX;
    p[IMU_MAG_ALGN_ZY] = DEFAULT_IMU_MAG_ALGN_ZY;
    p[IMU_GYO_BIAS_X] = DEFAULT_IMU_GYO_BIAS_X;
    p[IMU_GYO_BIAS_Y] = DEFAULT_IMU_GYO_BIAS_Y;
    p[IMU_GYO_BIAS_Z] = DEFAULT_IMU_GYO_BIAS_Z;
    p[IMU_GYO_BIAS1_X] = DEFAULT_IMU_GYO_BIAS1_X;
    p[IMU_GYO_BIAS1_Y] = DEFAULT_IMU_GYO_BIAS1_Y;
    p[IMU_GYO_BIAS1_Z] = DEFAULT_IMU_GYO_BIAS1_Z;
    p[IMU_GYO_BIAS2_X] = DEFAULT_IMU_GYO_BIAS2_X;
    p[IMU_GYO_BIAS2_Y] = DEFAULT_IMU_GYO_BIAS2_Y;
    p[IMU_GYO_BIAS2_Z] = DEFAULT_IMU_GYO_BIAS2_Z;
    p[IMU_GYO_BIAS3_X] = DEFAULT_IMU_GYO_BIAS3_X;
    p[IMU_GYO_BIAS3_Y] = DEFAULT_IMU_GYO_BIAS3_Y;
    p[IMU_GYO_BIAS3_Z] = DEFAULT_IMU_GYO_BIAS3_Z;
    p[IMU_GYO_SCAL_X] = DEFAULT_IMU_GYO_SCAL_X;
    p[IMU_GYO_SCAL_Y] = DEFAULT_IMU_GYO_SCAL_Y;
    p[IMU_GYO_SCAL_Z] = DEFAULT_IMU_GYO_SCAL_Z;
    p[IMU_GYO_ALGN_XY] = DEFAULT_IMU_GYO_ALGN_XY;
    p[IMU_GYO_ALGN_XZ] = DEFAULT_IMU_GYO_ALGN_XZ;
    p[IMU_GYO_ALGN_YX] = DEFAULT_IMU_GYO_ALGN_YX;
    p[IMU_GYO_ALGN_YZ] = DEFAULT_IMU_GYO_ALGN_YZ;
    p[IMU_GYO_ALGN_ZX] = DEFAULT_IMU_GYO_ALGN_ZX;
    p[IMU_GYO_ALGN_ZY] = DEFAULT_IMU_GYO_ALGN_ZY;
    p[IMU_MAG_INCL] = DEFAULT_IMU_MAG_INCL;
    p[IMU_MAG_DECL] = DEFAULT_IMU_MAG_DECL;
    p[IMU_PRESS_SENSE] = DEFAULT_IMU_PRESS_SENSE;
    p[GMBL_PITCH_PORT] = DEFAULT_GMBL_PITCH_PORT;
    p[GMBL_ROLL_PORT] = DEFAULT_GMBL_ROLL_PORT;
    p[GMBL_PWM_MAX_RL] = DEFAULT_GMBL_PWM_MAX_RL;
    p[GMBL_PWM_MIN_RL] = DEFAULT_GMBL_PWM_MIN_RL;
    p[GMBL_PWM_MAX_PT] = DEFAULT_GMBL_PWM_MAX_PT;
    p[GMBL_PWM_MIN_PT] = DEFAULT_GMBL_PWM_MIN_PT;
    p[GMBL_PWM_FREQ] = DEFAULT_GMBL_PWM_FREQ;
    p[GMBL_NTRL_PITCH] = DEFAULT_GMBL_NTRL_PITCH;
    p[GMBL_NTRL_ROLL] = DEFAULT_GMBL_NTRL_ROLL;
    p[GMBL_SCAL_PITCH] = DEFAULT_GMBL_SCAL_PITCH;
    p[GMBL_SCAL_ROLL] = DEFAULT_GMBL_SCAL_ROLL;
    p[GMBL_SLEW_RATE] = DEFAULT_GMBL_SLEW_RATE;
    p[GMBL_ROLL_EXPO] = DEFAULT_GMBL_ROLL_EXPO;
    p[GMBL_TILT_PORT] = DEFAULT_GMBL_TILT_PORT;
    p[GMBL_TRIG_PORT] = DEFAULT_GMBL_TRIG_PORT;
    p[GMBL_TRIG_CH_NEU] = DEFAULT_GMBL_TRIG_CH_NEU;
    p[GMBL_TRIG_ON_PWM] = DEFAULT_GMBL_TRIG_ON_PWM;
    p[GMBL_TRIG_ON_TIM] = DEFAULT_GMBL_TRIG_ON_TIM;
    p[GMBL_TRIG_DIST] = DEFAULT_GMBL_TRIG_DIST;
    p[GMBL_TRIG_TIME] = DEFAULT_GMBL_TRIG_TIME;
    p[GMBL_PSTHR_CHAN] = DEFAULT_GMBL_PSTHR_CHAN;
    p[GMBL_PSTHR_PORT] = DEFAULT_GMBL_PSTHR_PORT;
    p[SPVR_LOW_BAT1] = DEFAULT_SPVR_LOW_BAT1;
    p[SPVR_LOW_BAT2] = DEFAULT_SPVR_LOW_BAT2;
    p[SPVR_BAT_CRV1] = DEFAULT_SPVR_BAT_CRV1;
    p[SPVR_BAT_CRV2] = DEFAULT_SPVR_BAT_CRV2;
    p[SPVR_BAT_CRV3] = DEFAULT_SPVR_BAT_CRV3;
    p[SPVR_BAT_CRV4] = DEFAULT_SPVR_BAT_CRV4;
    p[SPVR_BAT_CRV5] = DEFAULT_SPVR_BAT_CRV5;
    p[SPVR_BAT_CRV6] = DEFAULT_SPVR_BAT_CRV6;
    p[SPVR_FS_RAD_ST1] = DEFAULT_SPVR_FS_RAD_ST1;
    p[SPVR_FS_RAD_ST2] = DEFAULT_SPVR_FS_RAD_ST2;
    p[SPVR_FS_ADD_ALT] = DEFAULT_SPVR_FS_ADD_ALT;
    p[SPVR_VIN_SOURCE] = DEFAULT_SPVR_VIN_SOURCE;
    p[QUATOS_J_ROLL] = DEFAULT_QUATOS_J_ROLL;
    p[QUATOS_J_PITCH] = DEFAULT_QUATOS_J_PITCH;
    p[QUATOS_J_YAW] = DEFAULT_QUATOS_J_YAW;
    p[QUATOS_AM1] = DEFAULT_QUATOS_AM1;
    p[QUATOS_AM2] = DEFAULT_QUATOS_AM2;
    p[QUATOS_PROP_K1] = DEFAULT_QUATOS_PROP_K1;
    p[QUATOS_M_TLT_RT] = DEFAULT_QUATOS_M_TLT_RT;
    p[QUATOS_M_YAW_RT] = DEFAULT_QUATOS_M_YAW_RT;
    p[QUATOS_MAX_OUT] = DEFAULT_QUATOS_MAX_OUT;
    p[QUATOS_QUAT_TAU] = DEFAULT_QUATOS_QUAT_TAU;
    p[QUATOS_L1_ASP] = DEFAULT_QUATOS_L1_ASP;
    p[QUATOS_L1_K1] = DEFAULT_QUATOS_L1_K1;
    p[QUATOS_AM1_KNOB] = DEFAULT_QUATOS_AM1_KNOB;
    p[QUATOS_AM2_KNOB] = DEFAULT_QUATOS_AM2_KNOB;
    p[QUATOS_K1_KNOB] = DEFAULT_QUATOS_K1_KNOB;
    p[QUATOS_PT_KNOB] = DEFAULT_QUATOS_PT_KNOB;
    p[QUATOS_MM_R01] = DEFAULT_QUATOS_MM_R01;
    p[QUATOS_MM_P01] = DEFAULT_QUATOS_MM_P01;
    p[QUATOS_MM_Y01] = DEFAULT_QUATOS_MM_Y01;
    p[QUATOS_MM_R02] = DEFAULT_QUATOS_MM_R02;
    p[QUATOS_MM_P02] = DEFAULT_QUATOS_MM_P02;
    p[QUATOS_MM_Y02] = DEFAULT_QUATOS_MM_Y02;
    p[QUATOS_MM_R03] = DEFAULT_QUATOS_MM_R03;
    p[QUATOS_MM_P03] = DEFAULT_QUATOS_MM_P03;
    p[QUATOS_MM_Y03] = DEFAULT_QUATOS_MM_Y03;
    p[QUATOS_MM_R04] = DEFAULT_QUATOS_MM_R04;
    p[QUATOS_MM_P04] = DEFAULT_QUATOS_MM_P04;
    p[QUATOS_MM_Y04] = DEFAULT_QUATOS_MM_Y04;
    p[QUATOS_MM_R05] = DEFAULT_QUATOS_MM_R05;
    p[QUATOS_MM_P05] = DEFAULT_QUATOS_MM_P05;
    p[QUATOS_MM_Y05] = DEFAULT_QUATOS_MM_Y05;
    p[QUATOS_MM_R06] = DEFAULT_QUATOS_MM_R06;
    p[QUATOS_MM_P06] = DEFAULT_QUATOS_MM_P06;
    p[QUATOS_MM_Y06] = DEFAULT_QUATOS_MM_Y06;
    p[QUATOS_MM_R07] = DEFAULT_QUATOS_MM_R07;
    p[QUATOS_MM_P07] = DEFAULT_QUATOS_MM_P07;
    p[QUATOS_MM_Y07] = DEFAULT_QUATOS_MM_Y07;
    p[QUATOS_MM_R08] = DEFAULT_QUATOS_MM_R08;
    p[QUATOS_MM_P08] = DEFAULT_QUATOS_MM_P08;
    p[QUATOS_MM_Y08] = DEFAULT_QUATOS_MM_Y08;
    p[QUATOS_MM_R09] = DEFAULT_QUATOS_MM_R09;
    p[QUATOS_MM_P09] = DEFAULT_QUATOS_MM_P09;
    p[QUATOS_MM_Y09] = DEFAULT_QUATOS_MM_Y09;
    p[QUATOS_MM_R10] = DEFAULT_QUATOS_MM_R10;
    p[QUATOS_MM_P10] = DEFAULT_QUATOS_MM_P10;
    p[QUATOS_MM_Y10] = DEFAULT_QUATOS_MM_Y10;
    p[QUATOS_MM_R11] = DEFAULT_QUATOS_MM_R11;
    p[QUATOS_MM_P11] = DEFAULT_QUATOS_MM_P11;
    p[QUATOS_MM_Y11] = DEFAULT_QUATOS_MM_Y11;
    p[QUATOS_MM_R12] = DEFAULT_QUATOS_MM_R12;
    p[QUATOS_MM_P12] = DEFAULT_QUATOS_MM_P12;
    p[QUATOS_MM_Y12] = DEFAULT_QUATOS_MM_Y12;
    p[QUATOS_MM_R13] = DEFAULT_QUATOS_MM_R13;
    p[QUATOS_MM_P13] = DEFAULT_QUATOS_MM_P13;
    p[QUATOS_MM_Y13] = DEFAULT_QUATOS_MM_Y13;
    p[QUATOS_MM_R14] = DEFAULT_QUATOS_MM_R14;
    p[QUATOS_MM_P14] = DEFAULT_QUATOS_MM_P14;
    p[QUATOS_MM_Y14] = DEFAULT_QUATOS_MM_Y14;
    p[QUATOS_MM_R15] = DEFAULT_QUATOS_MM_R15;
    p[QUATOS_MM_P15] = DEFAULT_QUATOS_MM_P15;
    p[QUATOS_MM_Y15] = DEFAULT_QUATOS_MM_Y15;
    p[QUATOS_MM_R16] = DEFAULT_QUATOS_MM_R16;
    p[QUATOS_MM_P16] = DEFAULT_QUATOS_MM_P16;
    p[QUATOS_MM_Y16] = DEFAULT_QUATOS_MM_Y16;
    p[SIG_LED_1_PRT] = DEFAULT_SIG_LED_1_PRT;
    p[SIG_LED_2_PRT] = DEFAULT_SIG_LED_2_PRT;
    p[SIG_BEEP_PRT] = DEFAULT_SIG_BEEP_PRT;
    p[SIG_PWM_PRT] = DEFAULT_SIG_PWM_PRT;
    p[LIC_KEY1] = DEFAULT_LIC_KEY1;
    p[LIC_KEY2] = DEFAULT_LIC_KEY2;
    p[LIC_KEY3] = DEFAULT_LIC_KEY3;

    AQ_NOTICE("config: Loaded default parameters.\n");
}

configToken_t *configTokenFindEmpty(void) {
    configToken_t *p = (configToken_t *)(FLASH_END_ADDR + 1);

    do {
        p--;
    } while (p->key != 0xffffffff);

    return p;
}

void configTokenStore(configToken_t *token) {
    flashAddress((uint32_t)configTokenFindEmpty(), (uint32_t *)token, sizeof(configToken_t)/sizeof(uint32_t));
}

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

void configFlashRead(void) {
    configRec_t *recs;
    int i, j;

    recs = (void *)flashStartAddr();

    for (i = 0; i < CONFIG_NUM_PARAMS; i++) {
        for (j = 0; j < CONFIG_NUM_PARAMS; j++)
            if (!strncasecmp(recs[i].name, configParameterStrings[j], 16))
                p[j] = recs[i].val;
    }

    AQ_NOTICE("config: Parameters restored from flash memory.\n");
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

        ret = flashErase(flashStartAddr(), CONFIG_NUM_PARAMS*sizeof(configRec_t)/sizeof(uint32_t));

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
                memcpy(recs[i].name, configParameterStrings[i], 16);
                recs[i].val = p[i];
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

void configInit(void) {
    float ver;

    // start with what's in flash
    configFlashRead();

    // try to load any config params from uSD card
    if (configReadFile(0) < 0)
	// clear config if read error
	memset(p, 0, sizeof(p));
    else
	supervisorConfigRead();

    // get flash version
    ver = *(float *)(flashStartAddr()+16);
    if (isnan(ver))
	ver = 0.0f;

    // if compiled defaults are greater than flash version and loaded version
    if (DEFAULT_CONFIG_VERSION > ver && DEFAULT_CONFIG_VERSION > p[CONFIG_VERSION])
	configLoadDefault();
    // if flash version greater than or equal to currently loaded version
    else if (ver >= p[CONFIG_VERSION])
	configFlashRead();

    // if loaded version greater than flash version
    if (p[CONFIG_VERSION] > ver)
	configFlashWrite();
}

unsigned int configParameterRead(void *data) {
    paramStruct_t *par = (paramStruct_t *)data;

    if (par->paramId + par->num > CONFIG_NUM_PARAMS)
	par->num = CONFIG_NUM_PARAMS - par->paramId;

    memcpy((char *)par->values, (char *)&p[par->paramId], par->num * sizeof(float));

    return par->num * sizeof(float);
}

unsigned int configParameterWrite(void *data) {
    paramStruct_t *par = (paramStruct_t *)data;

    memcpy((char *)&p[par->paramId], (char *)par->values, par->num * sizeof(float));

    return configParameterRead(data);
}

int configParseParams(char *fileBuf, int size, int p1) {
    static char lineBuf[CONFIG_LINE_BUF_SIZE];
    char *param;
    float value;
    char c;
    int p2;
    int n;
    int i, j;

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
		if (n != 2) {
		    n = sscanf(lineBuf, "#define %17s %f", param, &value);
		}
	    }

	    if (n == 2) {
		for (j = 0; j < CONFIG_NUM_PARAMS; j++) {
		    if (!strncasecmp(param, configParameterStrings[j], sizeof(char)*17))
			p[j] = value;
		}
	    }
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

// read config from uSD
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
	    ret = -1;
	    break;
	}
    }

    filerClose(fh);

    if (fileBuf)
	aqFree(fileBuf, CONFIG_FILE_BUF_SIZE, sizeof(char));

    if (ret > -1)
	AQ_NOTICE("config: Parameters loaded from local storage file.\n");
    else
	AQ_NOTICE("config: Failed to read parameters from local file.");

    return ret;
}

int8_t configFormatParam(char *buf, int n) {
    char *str;
    int8_t ret = 0;

    if (!(str = (char *)aqCalloc(16, sizeof(char))))
	return ret;

    ftoa(str, p[n], 10);
    ret = sprintf(buf, "%-17s\t\t%s\n", configParameterStrings[n], str);

    if (str)
	aqFree(str, 16, sizeof(char));

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

void configSetParamByID(int id, float value) {
    p[id] = value;
}

int configGetParamIdByName(char *name) {
    int i = -1;

    for (i = 0; i < CONFIG_NUM_PARAMS; i++)
	if (!strncmp(name, configParameterStrings[i], 16))
	    break;

    if (i == -1)
	AQ_PRINTF("config: cannot find parmeter '%s'\n", name);

    return i;
}
