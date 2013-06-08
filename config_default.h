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

    Copyright © 2011, 2012, 2013  Bill Nesbitt
*/

#define DEFAULT_CONFIG_VERSION	    107

#define DEFAULT_RADIO_TYPE	    0		// 0 == Spektrum 11bit, 1 == Spektrum 10bit, 2 == SBUS, 3 == PPM, 4 == SUMD
#define DEFAULT_RADIO_THRO_CH	    0
#define DEFAULT_RADIO_ROLL_CH	    1
#define DEFAULT_RADIO_PITC_CH	    2
#define DEFAULT_RADIO_RUDD_CH	    3
#define DEFAULT_RADIO_GEAR_CH	    4
#define DEFAULT_RADIO_FLAP_CH	    5
#define DEFAULT_RADIO_AUX2_CH	    6
#define DEFAULT_RADIO_AUX3_CH	    7
#define DEFAULT_RADIO_AUX4_CH	    8
#define DEFAULT_RADIO_AUX5_CH	    9
#define DEFAULT_RADIO_AUX6_CH	    10
#define DEFAULT_RADIO_AUX7_CH	    11

#define DEFAULT_PPM_SCALER	    3		// good for FrSky & Graupner HOTT
#define DEFAULT_PPM_THROT_LOW	    1090	// throttle value at low stick
#define DEFAULT_PPM_CHAN_MID	    1512	// middle of the stick range

#define DEFAULT_SIG_LED_1_PRT       0		// External Led(1) port, 0 to disable
#define DEFAULT_SIG_LED_2_PRT       0		// External Led(2) port, 0 to disable
#define DEFAULT_SIG_BEEP_PRT        0		// negative sign before port number when using a piezo speaker, no sign when using a piezo buzzer
#define DEFAULT_SIG_PWM_PRT         0		// External PWM-controlled signaling port, 0 to disable

#define DEFAULT_CTRL_PID_TYPE	    0		// 0 == parallel TILT/RATE PIDs, 1 == cascading TILT/RATE PIDs

#define DEFAULT_CTRL_FACT_THRO	    0.70f	// user throttle multiplier
#define DEFAULT_CTRL_FACT_PITC	    0.05f	// user pitch multiplier
#define DEFAULT_CTRL_FACT_ROLL	    0.05f	// user roll multiplier
#define DEFAULT_CTRL_FACT_RUDD	    0.0004f	// user rudder multiplier
#define DEFAULT_CTRL_DEAD_BAND	    40.0f	// rc control dead band (for pitch, roll, & rudder control)
#define DEFAULT_CTRL_DBAND_THRO	    40.0f	// rc control dead band (for throttle channel only)
#define DEFAULT_CTRL_MIN_THROT	    20.0f	// minimum user throttle to activate motors
#define DEFAULT_CTRL_MAX	    300.0f	// maximum control applied to motors +- throttle
#define DEFAULT_CTRL_NAV_YAW_RT	    180.0f	// maximum navigation yaw rate deg/s

// TILT rate PID
#define DEFAULT_CTRL_TLT_RTE_P	    0.0f
#define DEFAULT_CTRL_TLT_RTE_I	    0.0f
#define DEFAULT_CTRL_TLT_RTE_D	    7180.0f
#define DEFAULT_CTRL_TLT_RTE_F	    0.25f
#define DEFAULT_CTRL_TLT_RTE_PM	    999.0f
#define DEFAULT_CTRL_TLT_RTE_IM	    999.0f
#define DEFAULT_CTRL_TLT_RTE_DM	    999.0f
#define DEFAULT_CTRL_TLT_RTE_OM	    250.0f

// YAW rate PID
#define DEFAULT_CTRL_YAW_RTE_P	    300.0f
#define DEFAULT_CTRL_YAW_RTE_I	    0.15f
#define DEFAULT_CTRL_YAW_RTE_D	    50.0f
#define DEFAULT_CTRL_YAW_RTE_F	    0.25f
#define DEFAULT_CTRL_YAW_RTE_PM	    80.0f
#define DEFAULT_CTRL_YAW_RTE_IM	    80.0f
#define DEFAULT_CTRL_YAW_RTE_DM	    80.0f
#define DEFAULT_CTRL_YAW_RTE_OM	    180.0f

// TILT angle PID
#define DEFAULT_CTRL_TLT_ANG_P	    60.0f
#define DEFAULT_CTRL_TLT_ANG_I	    0.0005f
#define DEFAULT_CTRL_TLT_ANG_D	    1744.0f
#define DEFAULT_CTRL_TLT_ANG_F	    0.25f
#define DEFAULT_CTRL_TLT_ANG_PM	    150.0f
#define DEFAULT_CTRL_TLT_ANG_IM	    75.0f
#define DEFAULT_CTRL_TLT_ANG_DM	    150.0f
#define DEFAULT_CTRL_TLT_ANG_OM	    250.0f

// YAW angle PID
#define DEFAULT_CTRL_YAW_ANG_P	    0.05f
#define DEFAULT_CTRL_YAW_ANG_I	    0.00002f
#define DEFAULT_CTRL_YAW_ANG_D	    0.0f
#define DEFAULT_CTRL_YAW_ANG_F	    0.0f
#define DEFAULT_CTRL_YAW_ANG_PM	    1.25f
#define DEFAULT_CTRL_YAW_ANG_IM	    0.04f
#define DEFAULT_CTRL_YAW_ANG_DM	    0.0f
#define DEFAULT_CTRL_YAW_ANG_OM	    1.25f

#define DEFAULT_MOT_FRAME	    0		// used as hint for frame config GUI
#define DEFAULT_MOT_START	    1125
#define DEFAULT_MOT_MIN		    975
#define DEFAULT_MOT_MAX		    1950
#define DEFAULT_MOT_HOV_THROT	    500
#define DEFAULT_MOT_EXP_FACT	    0.0f
#define DEFAULT_MOT_EXP_MIN	    0.0f
#define DEFAULT_MOT_EXP_MAX	    0.0f

#define DEFAULT_MOT_PWRD_01_T	    0.0
#define DEFAULT_MOT_PWRD_01_P	    0.0
#define DEFAULT_MOT_PWRD_01_R	    0.0
#define DEFAULT_MOT_PWRD_01_Y	    0.0
#define DEFAULT_MOT_PWRD_02_T	    0.0
#define DEFAULT_MOT_PWRD_02_P	    0.0
#define DEFAULT_MOT_PWRD_02_R	    0.0
#define DEFAULT_MOT_PWRD_02_Y	    0.0
#define DEFAULT_MOT_PWRD_03_T	    0.0
#define DEFAULT_MOT_PWRD_03_P	    0.0
#define DEFAULT_MOT_PWRD_03_R	    0.0
#define DEFAULT_MOT_PWRD_03_Y	    0.0
#define DEFAULT_MOT_PWRD_04_T	    0.0
#define DEFAULT_MOT_PWRD_04_P	    0.0
#define DEFAULT_MOT_PWRD_04_R	    0.0
#define DEFAULT_MOT_PWRD_04_Y	    0.0
#define DEFAULT_MOT_PWRD_05_T	    0.0
#define DEFAULT_MOT_PWRD_05_P	    0.0
#define DEFAULT_MOT_PWRD_05_R	    0.0
#define DEFAULT_MOT_PWRD_05_Y	    0.0
#define DEFAULT_MOT_PWRD_06_T	    0.0
#define DEFAULT_MOT_PWRD_06_P	    0.0
#define DEFAULT_MOT_PWRD_06_R	    0.0
#define DEFAULT_MOT_PWRD_06_Y	    0.0
#define DEFAULT_MOT_PWRD_07_T	    0.0
#define DEFAULT_MOT_PWRD_07_P	    0.0
#define DEFAULT_MOT_PWRD_07_R	    0.0
#define DEFAULT_MOT_PWRD_07_Y	    0.0
#define DEFAULT_MOT_PWRD_08_T	    0.0
#define DEFAULT_MOT_PWRD_08_P	    0.0
#define DEFAULT_MOT_PWRD_08_R	    0.0
#define DEFAULT_MOT_PWRD_08_Y	    0.0
#define DEFAULT_MOT_PWRD_09_T	    0.0
#define DEFAULT_MOT_PWRD_09_P	    0.0
#define DEFAULT_MOT_PWRD_09_R	    0.0
#define DEFAULT_MOT_PWRD_09_Y	    0.0
#define DEFAULT_MOT_PWRD_10_T	    0.0
#define DEFAULT_MOT_PWRD_10_P	    0.0
#define DEFAULT_MOT_PWRD_10_R	    0.0
#define DEFAULT_MOT_PWRD_10_Y	    0.0
#define DEFAULT_MOT_PWRD_11_T	    0.0
#define DEFAULT_MOT_PWRD_11_P	    0.0
#define DEFAULT_MOT_PWRD_11_R	    0.0
#define DEFAULT_MOT_PWRD_11_Y	    0.0
#define DEFAULT_MOT_PWRD_12_T	    0.0
#define DEFAULT_MOT_PWRD_12_P	    0.0
#define DEFAULT_MOT_PWRD_12_R	    0.0
#define DEFAULT_MOT_PWRD_12_Y	    0.0
#define DEFAULT_MOT_PWRD_13_T	    0.0
#define DEFAULT_MOT_PWRD_13_P	    0.0
#define DEFAULT_MOT_PWRD_13_R	    0.0
#define DEFAULT_MOT_PWRD_13_Y	    0.0
#define DEFAULT_MOT_PWRD_14_T	    0.0
#define DEFAULT_MOT_PWRD_14_P	    0.0
#define DEFAULT_MOT_PWRD_14_R	    0.0
#define DEFAULT_MOT_PWRD_14_Y	    0.0


#define DEFAULT_COMM_BAUD1	    115200
#define DEFAULT_COMM_BAUD2	    230400
#define DEFAULT_COMM_BAUD3	    3000000
#define DEFAULT_COMM_BAUD4	    460800
#define DEFAULT_COMM_STREAM_TYP1    COMM_TYPE_MAVLINK
#define DEFAULT_COMM_STREAM_TYP2    0
#define DEFAULT_COMM_STREAM_TYP3    0
#define DEFAULT_COMM_STREAM_TYP4    0


#define DEFAULT_TELEMETRY_RATE	    20		// loops between reports


#define DEFAULT_NAV_MAX_SPEED	    5.0f	// m/s
#define DEFAULT_NAV_MAX_DECENT	    1.5f	// m/s
#define DEFAULT_NAV_CEILING         0.0f	// m relative to home alt. Maximum altitude in alt/pos/mission/dvh modes

// speed => tilt PID
#define DEFAULT_NAV_SPEED_P	    7.0f
#define DEFAULT_NAV_SPEED_I	    0.005f
#define DEFAULT_NAV_SPEED_PM	    20.0f
#define DEFAULT_NAV_SPEED_IM	    20.0f
#define DEFAULT_NAV_SPEED_OM	    30.0f

// distance => speed PID
#define DEFAULT_NAV_DIST_P	    0.5f
#define DEFAULT_NAV_DIST_I	    0.0f
#define DEFAULT_NAV_DIST_PM	    999.0f
#define DEFAULT_NAV_DIST_IM	    0.0f
#define DEFAULT_NAV_DIST_OM	    999.0f

// Altitude hold Speed PID
#define DEFAULT_NAV_ATL_SPED_P	    200.0f
#define DEFAULT_NAV_ATL_SPED_I	    2.85f
#define DEFAULT_NAV_ATL_SPED_PM	    150.0f
#define DEFAULT_NAV_ATL_SPED_IM	    700.0f
#define DEFAULT_NAV_ATL_SPED_OM	    700.0f

// Altitude hold Position PID
#define DEFAULT_NAV_ALT_POS_P	    0.20f
#define DEFAULT_NAV_ALT_POS_I	    0.0f
#define DEFAULT_NAV_ALT_POS_PM	    2.5f
#define DEFAULT_NAV_ALT_POS_IM	    0.0f
#define DEFAULT_NAV_ALT_POS_OM	    2.5f

#define DEFAULT_IMU_ROT		    +0.0		// degrees to rotate the IMU to align with the frame

#define DEFAULT_IMU_ACC_BIAS_X	    -1.65
#define DEFAULT_IMU_ACC_BIAS_Y	    -1.65
#define DEFAULT_IMU_ACC_BIAS_Z	    -1.65
#define DEFAULT_IMU_ACC_BIAS1_X	    0.0
#define DEFAULT_IMU_ACC_BIAS1_Y	    0.0
#define DEFAULT_IMU_ACC_BIAS1_Z	    0.0
#define DEFAULT_IMU_ACC_BIAS2_X	    0.0
#define DEFAULT_IMU_ACC_BIAS2_Y	    0.0
#define DEFAULT_IMU_ACC_BIAS2_Z	    0.0
#define DEFAULT_IMU_ACC_BIAS3_X	    0.0
#define DEFAULT_IMU_ACC_BIAS3_Y	    0.0
#define DEFAULT_IMU_ACC_BIAS3_Z	    0.0

#define DEFAULT_IMU_ACC_SCAL_X	    +0.019
#define DEFAULT_IMU_ACC_SCAL_Y	    +0.019
#define DEFAULT_IMU_ACC_SCAL_Z	    +0.019
#define DEFAULT_IMU_ACC_SCAL1_X	    0.0
#define DEFAULT_IMU_ACC_SCAL1_Y	    0.0
#define DEFAULT_IMU_ACC_SCAL1_Z	    0.0
#define DEFAULT_IMU_ACC_SCAL2_X	    0.0
#define DEFAULT_IMU_ACC_SCAL2_Y	    0.0
#define DEFAULT_IMU_ACC_SCAL2_Z	    0.0
#define DEFAULT_IMU_ACC_SCAL3_X	    0.0
#define DEFAULT_IMU_ACC_SCAL3_Y	    0.0
#define DEFAULT_IMU_ACC_SCAL3_Z	    0.0

#define DEFAULT_IMU_ACC_ALGN_XY	    0.0
#define DEFAULT_IMU_ACC_ALGN_XZ	    0.0
#define DEFAULT_IMU_ACC_ALGN_YX	    0.0
#define DEFAULT_IMU_ACC_ALGN_YZ	    0.0
#define DEFAULT_IMU_ACC_ALGN_ZX	    0.0
#define DEFAULT_IMU_ACC_ALGN_ZY	    0.0

#define DEFAULT_IMU_MAG_BIAS_X	    0.0
#define DEFAULT_IMU_MAG_BIAS_Y	    0.0
#define DEFAULT_IMU_MAG_BIAS_Z	    0.0
#define DEFAULT_IMU_MAG_BIAS1_X	    0.0
#define DEFAULT_IMU_MAG_BIAS1_Y	    0.0
#define DEFAULT_IMU_MAG_BIAS1_Z	    0.0
#define DEFAULT_IMU_MAG_BIAS2_X	    0.0
#define DEFAULT_IMU_MAG_BIAS2_Y	    0.0
#define DEFAULT_IMU_MAG_BIAS2_Z	    0.0
#define DEFAULT_IMU_MAG_BIAS3_X	    0.0
#define DEFAULT_IMU_MAG_BIAS3_Y	    0.0
#define DEFAULT_IMU_MAG_BIAS3_Z	    0.0

#define DEFAULT_IMU_MAG_SCAL_X	    +0.19
#define DEFAULT_IMU_MAG_SCAL_Y	    +0.19
#define DEFAULT_IMU_MAG_SCAL_Z	    +0.19
#define DEFAULT_IMU_MAG_SCAL1_X	    0.0
#define DEFAULT_IMU_MAG_SCAL1_Y	    0.0
#define DEFAULT_IMU_MAG_SCAL1_Z	    0.0
#define DEFAULT_IMU_MAG_SCAL2_X	    0.0
#define DEFAULT_IMU_MAG_SCAL2_Y	    0.0
#define DEFAULT_IMU_MAG_SCAL2_Z	    0.0
#define DEFAULT_IMU_MAG_SCAL3_X	    0.0
#define DEFAULT_IMU_MAG_SCAL3_Y	    0.0
#define DEFAULT_IMU_MAG_SCAL3_Z	    0.0

#define DEFAULT_IMU_MAG_ALGN_XY	    0.0
#define DEFAULT_IMU_MAG_ALGN_XZ	    0.0
#define DEFAULT_IMU_MAG_ALGN_YX	    0.0
#define DEFAULT_IMU_MAG_ALGN_YZ	    0.0
#define DEFAULT_IMU_MAG_ALGN_ZX	    0.0
#define DEFAULT_IMU_MAG_ALGN_ZY	    0.0

#define DEFAULT_IMU_GYO_BIAS_X	    -1.33
#define DEFAULT_IMU_GYO_BIAS_Y	    -1.33
#define DEFAULT_IMU_GYO_BIAS_Z	    -1.33
#define DEFAULT_IMU_GYO_BIAS1_X	    0.0
#define DEFAULT_IMU_GYO_BIAS1_Y	    0.0
#define DEFAULT_IMU_GYO_BIAS1_Z	    0.0
#define DEFAULT_IMU_GYO_BIAS2_X	    0.0
#define DEFAULT_IMU_GYO_BIAS2_Y	    0.0
#define DEFAULT_IMU_GYO_BIAS2_Z	    0.0
#define DEFAULT_IMU_GYO_BIAS3_X	    0.0
#define DEFAULT_IMU_GYO_BIAS3_Y	    0.0
#define DEFAULT_IMU_GYO_BIAS3_Z	    0.0

#define DEFAULT_IMU_GYO_SCAL_X	    +0.1145
#define DEFAULT_IMU_GYO_SCAL_Y	    +0.1145
#define DEFAULT_IMU_GYO_SCAL_Z	    +0.1145

#define DEFAULT_IMU_GYO_ALGN_XY	    0.0
#define DEFAULT_IMU_GYO_ALGN_XZ	    0.0
#define DEFAULT_IMU_GYO_ALGN_YX	    0.0
#define DEFAULT_IMU_GYO_ALGN_YZ	    0.0
#define DEFAULT_IMU_GYO_ALGN_ZX	    0.0
#define DEFAULT_IMU_GYO_ALGN_ZY	    0.0

#define DEFAULT_IMU_MAG_INCL	    -65.0
#define DEFAULT_IMU_MAG_DECL	    0.0

#define DEFAULT_IMU_PRESS_SENSE	    0.0f		// 0 == sensor #1, 1 == sensor #2, 2 == both


#define DEFAULT_GMBL_PITCH_PORT	    0.0f		// 0 == disabled
#define DEFAULT_GMBL_ROLL_PORT	    0.0f		// 0 == disabled
#define DEFAULT_GMBL_PWM_MAX_RL    2250
#define DEFAULT_GMBL_PWM_MIN_RL    750
#define DEFAULT_GMBL_PWM_MAX_PT    2250
#define DEFAULT_GMBL_PWM_MIN_PT    750
#define DEFAULT_GMBL_PWM_FREQ	    200
#define DEFAULT_GMBL_NTRL_PITCH	    1575
#define DEFAULT_GMBL_NTRL_ROLL	    1442
#define DEFAULT_GMBL_SCAL_PITCH	    (1.0f / 91.0f)
#define DEFAULT_GMBL_SCAL_ROLL	    (1.0f / 70.0f)
#define DEFAULT_GMBL_SLEW_RATE	    0.005f
#define DEFAULT_GMBL_ROLL_EXPO      0.0f


#define DEFAULT_SPVR_LOW_BAT1	    3.5f	    // cell volts
#define DEFAULT_SPVR_LOW_BAT2	    3.3f	    // cell volts
#define DEFAULT_SPVR_BAT_CRV1	    +9.210144e+00f
#define DEFAULT_SPVR_BAT_CRV2	    +1.481796e+01f
#define DEFAULT_SPVR_BAT_CRV3	    -5.890255e+01f
#define DEFAULT_SPVR_BAT_CRV4	    +1.166082e+02f
#define DEFAULT_SPVR_BAT_CRV5	    -1.074193e+02f
#define DEFAULT_SPVR_BAT_CRV6	    +3.779082e+01f
#define DEFAULT_SPVR_FS_RAD_ST1     0           // radio failsafe stage 1 action (0 = position hold)
#define DEFAULT_SPVR_FS_RAD_ST2     0           // radio failsafe stage 2 action (0 = slow decent, 1 = RTH + slow decent)



#define DEFAULT_UKF_VEL_Q		+7.6020e-02	// +0.076019661680	 0.000109304521 -0.001443424436
#define DEFAULT_UKF_VEL_ALT_Q		+1.4149e-01	// +0.141489724652	 0.000109419473 +0.000987597731
#define DEFAULT_UKF_POS_Q		+6.0490e+03	// +6048.951523179588	 0.000109199532 +97.58772834123110
#define DEFAULT_UKF_POS_ALT_Q		+4.5576e+03	// +4557.622475819297	 0.000109580650 +19.44625975731340
#define DEFAULT_UKF_ACC_BIAS_Q		+9.3722e-04	// +0.000937220476	 0.000109347614 +0.000009865862
#define DEFAULT_UKF_GYO_BIAS_Q		+4.6872e-02	// +0.046871534288	 0.000109380732 -0.000123894440
#define DEFAULT_UKF_QUAT_Q		+7.3021e-04	// +0.000730213283	 0.000109472899 +0.000000995669
#define DEFAULT_UKF_PRES_ALT_Q		+6.5172e+01	// +65.171935456104	 0.000109418082 -0.2151891180844
#define DEFAULT_UKF_ACC_BIAS_V		+2.7535e-07	// +0.000000275353	 0.000109561088 -0.000000004212
#define DEFAULT_UKF_GYO_BIAS_V		+8.2738e-07	// +0.000000827379	 0.000107923369 +0.000000009107
#define DEFAULT_UKF_RATE_V		+6.0568e-05	// +0.000060568461	 0.000109458065 +0.000000498081
#define DEFAULT_UKF_PRES_ALT_V		+1.0204e-04	// +0.000102039667	 0.000109254406 -0.000002050090
#define DEFAULT_UKF_POS_V		+6.4505e-08	// +0.000000064505	 0.000109587486 -0.000000000240
#define DEFAULT_UKF_VEL_V		+1.0980e-07	// +0.000000109802	 0.000109537353 -0.000000000134
#define DEFAULT_UKF_ALT_POS_V		+5.3821e-09	// +0.000000005382	 0.000109525531 +0.000000000093
#define DEFAULT_UKF_ALT_VEL_V		+2.8103e-07	// +0.000000281035	 0.000109279082 +0.000000000639
#define DEFAULT_UKF_GPS_POS_N		+1.7620e-05	// +0.000017619672	 0.000109467204 -0.000000022679
#define DEFAULT_UKF_GPS_POS_M_N		+4.7413e-05	// +0.000047413187	 0.000108906551 -0.000000419440
#define DEFAULT_UKF_GPS_ALT_N		+7.6558e-05	// +0.000076558177	 0.000109472033 -0.000000162714
#define DEFAULT_UKF_GPS_ALT_M_N		+3.8535e-05	// +0.000038534766	 0.000109525552 +0.000000007101
#define DEFAULT_UKF_GPS_VEL_N		+4.6256e-02	// +0.046255979499	 0.000109061365 +0.000395208418
#define DEFAULT_UKF_GPS_VEL_M_N		+1.2336e-02	// +0.012336395925	 0.000109431436 +0.000140398236
#define DEFAULT_UKF_GPS_VD_N		+3.7820e+00	// +3.782028700864	 0.000109323731 -0.028830318912
#define DEFAULT_UKF_GPS_VD_M_N		+1.5841e-02	// +0.015840936058	 0.000109475273 -0.000030160915
#define DEFAULT_UKF_ALT_N		+1.7077e-01	// +0.170768080733	 0.000109571562 +0.000084225765
#define DEFAULT_UKF_ACC_N		+9.5468e-05	// +0.000095468045	 0.000109331710 -0.000000932407
#define DEFAULT_UKF_DIST_N		+1.8705e-02	// +0.018704747883	 0.000109457024 +0.000038618360
#define DEFAULT_UKF_MAG_N		+3.8226e-01	// +0.382258731690	 0.000109407461 +0.002851611558
#define DEFAULT_UKF_POS_DELAY		+2.0574e+03	// +2057.421963899194	 0.001097611925 -48.37809534324900
#define DEFAULT_UKF_VEL_DELAY		-1.0373e+05	// -103727.997010331557	 0.000109494449 -293.522967971236500


#define DEFAULT_VN100_MAG_BIAS_X	0.0f
#define DEFAULT_VN100_MAG_BIAS_Y	0.0f
#define DEFAULT_VN100_MAG_BIAS_Z	0.0f
#define DEFAULT_VN100_MAG_SCAL_X	1.0f
#define DEFAULT_VN100_MAG_SCAL_Y	1.0f
#define DEFAULT_VN100_MAG_SCAL_Z	1.0f
#define DEFAULT_VN100_MAG_ALGN_XY	0.0f
#define DEFAULT_VN100_MAG_ALGN_XZ	0.0f
#define DEFAULT_VN100_MAG_ALGN_YX	0.0f
#define DEFAULT_VN100_MAG_ALGN_YZ	0.0f
#define DEFAULT_VN100_MAG_ALGN_ZX	0.0f
#define DEFAULT_VN100_MAG_ALGN_ZY	0.0f

#define DEFAULT_L1_ATT_J_ROLL		0.0127f		    // J matrix
#define DEFAULT_L1_ATT_J_PITCH		0.0127f		    // J matrix
#define DEFAULT_L1_ATT_J_YAW		0.024661f	    // J matrix
#define DEFAULT_L1_ATT_AM1		-10.0f
#define DEFAULT_L1_ATT_AM2		-25.0f
#define DEFAULT_L1_ATT_T2R_A1		0.0
#define DEFAULT_L1_ATT_T2R_A2		0.0
#define DEFAULT_L1_ATT_PROP_K1		14.62f
#define DEFAULT_L1_ATT_M_TLT_RT		1.0f		    // rads/s
#define DEFAULT_L1_ATT_M_YAW_RT		1.0f		    // rads/s
#define DEFAULT_L1_ATT_MAX_OUT		4.0f		    // total moment demand allowed (n_a + n_b)
#define DEFAULT_L1_ATT_QUAT_TAU		0.05f
#define DEFAULT_L1_ATT_L1_ASP		-10.0f
#define DEFAULT_L1_ATT_L1_K1		18.6f
#define DEFAULT_L1_ATT_PWM_LO		1000.0f		    // us
#define DEFAULT_L1_ATT_PWM_HI		1950.0f		    // us
#define DEFAULT_L1_ATT_PWM_SCAL 	6500.0f		    // RPM
#define DEFAULT_L1_ATT_AM1_KNOB		0.0f		    // AM1
#define DEFAULT_L1_ATT_AM2_KNOB		0.0f		    // AM2
#define DEFAULT_L1_ATT_K1_KNOB		0.0f		    // L1_K1
#define DEFAULT_L1_ATT_PT_KNOB		0.0f		    // prop torque

#define DEFAULT_L1_ATT_MM_R01		0.0f
#define DEFAULT_L1_ATT_MM_P01		0.0f
#define DEFAULT_L1_ATT_MM_Y01		0.0f
#define DEFAULT_L1_ATT_MM_R02		0.0f
#define DEFAULT_L1_ATT_MM_P02		0.0f
#define DEFAULT_L1_ATT_MM_Y02		0.0f
#define DEFAULT_L1_ATT_MM_R03		0.0f
#define DEFAULT_L1_ATT_MM_P03		0.0f
#define DEFAULT_L1_ATT_MM_Y03		0.0f
#define DEFAULT_L1_ATT_MM_R04		0.0f
#define DEFAULT_L1_ATT_MM_P04		0.0f
#define DEFAULT_L1_ATT_MM_Y04		0.0f
#define DEFAULT_L1_ATT_MM_R05		0.0f
#define DEFAULT_L1_ATT_MM_P05		0.0f
#define DEFAULT_L1_ATT_MM_Y05		0.0f
#define DEFAULT_L1_ATT_MM_R06		0.0f
#define DEFAULT_L1_ATT_MM_P06		0.0f
#define DEFAULT_L1_ATT_MM_Y06		0.0f
#define DEFAULT_L1_ATT_MM_R07		0.0f
#define DEFAULT_L1_ATT_MM_P07		0.0f
#define DEFAULT_L1_ATT_MM_Y07		0.0f
#define DEFAULT_L1_ATT_MM_R08		0.0f
#define DEFAULT_L1_ATT_MM_P08		0.0f
#define DEFAULT_L1_ATT_MM_Y08		0.0f
#define DEFAULT_L1_ATT_MM_R09		0.0f
#define DEFAULT_L1_ATT_MM_P09		0.0f
#define DEFAULT_L1_ATT_MM_Y09		0.0f
#define DEFAULT_L1_ATT_MM_R10		0.0f
#define DEFAULT_L1_ATT_MM_P10		0.0f
#define DEFAULT_L1_ATT_MM_Y10		0.0f
#define DEFAULT_L1_ATT_MM_R11		0.0f
#define DEFAULT_L1_ATT_MM_P11		0.0f
#define DEFAULT_L1_ATT_MM_Y11		0.0f
#define DEFAULT_L1_ATT_MM_R12		0.0f
#define DEFAULT_L1_ATT_MM_P12		0.0f
#define DEFAULT_L1_ATT_MM_Y12		0.0f
#define DEFAULT_L1_ATT_MM_R13		0.0f
#define DEFAULT_L1_ATT_MM_P13		0.0f
#define DEFAULT_L1_ATT_MM_Y13		0.0f
#define DEFAULT_L1_ATT_MM_R14		0.0f
#define DEFAULT_L1_ATT_MM_P14		0.0f
#define DEFAULT_L1_ATT_MM_Y14		0.0f
