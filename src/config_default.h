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
    Copyright 2013-2016 Maxim Paperno
*/

// Misc config bitfield, 24b max.
//    b0: 1 = save adjusted params back to flash/SD; 0 = save defined param value
//    b1: enable/disable heading-free mode option in all flight modes (not just DVH)
//    b2: 1 = cascading TILT/RATE PIDs, 0 = parallel TILT/RATE PIDs
//    b3: 1 = disable mass storage component (MSC) on USB connection (allows logging/etc)
//    b4: scale down (cut) throttle when inverted and in altitude-hold mode
//    b5: scale down throttle when inverted in manual modes
#define DEFAULT_CONFIG_FLAGS	    (0 | CONFIG_FLAG_INVRT_TCUT_AUTO)

// Remote control adjustable parameters.  24 bits: SSSS_SSSS_CCCC_CCPP_PPPP_PPPP
// 	8 high bits = Scale of adjustment in 100ths of 1% (0-255, eg. 2 = 0.02%, 20 = 0.2% and 200 = 2%), next 6b = Channel number (0-63, zero = no channel/disabled), 10 low bits = Parameter ID to adjust (0-1024).
// Example: #define DEFAULT_CONFIG_ADJUST_P1  ((20 <<16) | (9 <<10) | 43)  // (= 1319979) Adjust parameter CTRL_TLT_ANG_P (#43) using radio channel 9 by 0.2% per radio value step.
#define DEFAULT_CONFIG_ADJUST_P1    (20<<16)  // default adjustment scale is 0.2%
#define DEFAULT_CONFIG_ADJUST_P2    (20<<16)
#define DEFAULT_CONFIG_ADJUST_P3    (20<<16)
#define DEFAULT_CONFIG_ADJUST_P4    (20<<16)
#define DEFAULT_CONFIG_ADJUST_P5    (20<<16)
#define DEFAULT_CONFIG_ADJUST_P6    (20<<16)

#define DEFAULT_RADIO_SETUP         0		// 0 = NONE, 1 == Spektrum 11bit, 2 == Spektrum 10bit, 3 == SBUS, 4 == PPM, 5 == SUMD, 6 == M-Link, 7 == Deltang, 8 == CYRF6936
#define DEFAULT_RADIO_THRO_CH	    0
#define DEFAULT_RADIO_ROLL_CH	    1
#define DEFAULT_RADIO_PITC_CH	    2
#define DEFAULT_RADIO_RUDD_CH	    3

#define DEFAULT_PPM_SCALER	    3		// good for FrSky & Graupner HOTT
#define DEFAULT_PPM_THROT_LOW	    1090	// throttle value at low stick
#define DEFAULT_PPM_CHAN_MID	    1512	// middle of the stick range

#define DEFAULT_SIG_LED_1_PRT       0		// External Led(1) port, 0 to disable
#define DEFAULT_SIG_LED_2_PRT       0		// External Led(2) port, 0 to disable
#define DEFAULT_SIG_BEEP_PRT        0		// negative sign before port number when using a piezo speaker, no sign when using a piezo buzzer
#define DEFAULT_SIG_PWM_PRT         0		// External PWM-controlled signaling port, 0 to disable

#define DEFAULT_CTRL_FACT_THRO	    0.70f	// user throttle multiplier
#define DEFAULT_CTRL_FACT_PITC	    0.05f	// user pitch multiplier
#define DEFAULT_CTRL_FACT_ROLL	    0.05f	// user roll multiplier
#define DEFAULT_CTRL_MAN_TLT_RT	    180.0f	// maximum rotation speed around pitch/roll axes in rate-control mode deg/s
#define DEFAULT_CTRL_MAN_YAW_RT	    60.0f	// deg/s
#define DEFAULT_CTRL_DEAD_BAND	    40.0f	// rc control dead band for pitch & roll axes to enter DVH mode from pos. hold
#define DEFAULT_CTRL_DBAND_YAW	    40.0f	// rc control dead band for yaw override when in all angle-control modes (not rate mode)
#define DEFAULT_CTRL_DBAND_THRO	    40.0f	// rc control dead band for throttle override in altitude-hold modes
#define DEFAULT_CTRL_DBAND_SWTCH    250.0f	// rc control dead band for switch positions (eg. mode control)
#define DEFAULT_CTRL_MIN_THROT	    20.0f	// minimum user throttle to activate motors
#define DEFAULT_CTRL_MAX	    1446.0f	// maximum control applied to motors +- throttle
#define DEFAULT_CTRL_NAV_YAW_RT	    180.0f	// maximum navigation yaw rate deg/s
#define DEFAULT_CTRL_TLT_ANG_TAU    0.1f	// filter constant for tilt angle requests (user or nav), larger value is smoother, min: 0.0025
#define DEFAULT_CTRL_TLT_RTE_TAU    0.005f	// filter constant for tilt rotation rate requests (user in rate mode only), larger value is smoother, min: 0.0025
#define DEFAULT_CTRL_YAW_RTE_TAU    0.01f	// filter constant for yaw rotation rate requests (user only, manual yaw is always rate-based), larger value is smoother, min: 0.0025

// TILT rate PID for angle-control mode
#define DEFAULT_CTRL_TLT_RTE_P	    0.0f
#define DEFAULT_CTRL_TLT_RTE_I	    0.0f
#define DEFAULT_CTRL_TLT_RTE_D	    34600.0f
#define DEFAULT_CTRL_TLT_RTE_F	    0.25f
// TILT rate PID for rate-of-rotation manual control mode
#define DEFAULT_CTRL_TLT_RTE_R_P    1000.0f
#define DEFAULT_CTRL_TLT_RTE_R_I    0.5f
#define DEFAULT_CTRL_TLT_RTE_R_D    3500.0f
#define DEFAULT_CTRL_TLT_RTE_R_F    0.25f
// TILT rate PID overall maximum outputs per timestep
#define DEFAULT_CTRL_TLT_RTE_PM	    9999.0f
#define DEFAULT_CTRL_TLT_RTE_IM	    9999.0f
#define DEFAULT_CTRL_TLT_RTE_DM	    9999.0f
#define DEFAULT_CTRL_TLT_RTE_OM	    1200.0f

// YAW rate PID
#define DEFAULT_CTRL_YAW_RTE_P	    1445.0f
#define DEFAULT_CTRL_YAW_RTE_I	    0.725f
#define DEFAULT_CTRL_YAW_RTE_D	    240.0f
#define DEFAULT_CTRL_YAW_RTE_F	    0.25f
#define DEFAULT_CTRL_YAW_RTE_PM	    386.0f
#define DEFAULT_CTRL_YAW_RTE_IM	    386.0f
#define DEFAULT_CTRL_YAW_RTE_DM	    386.0f
#define DEFAULT_CTRL_YAW_RTE_OM	    870.0f

// TILT angle PID
#define DEFAULT_CTRL_TLT_ANG_P	    290.0f
#define DEFAULT_CTRL_TLT_ANG_I	    0.0024f
#define DEFAULT_CTRL_TLT_ANG_D	    8400.0f
#define DEFAULT_CTRL_TLT_ANG_F	    0.25f
#define DEFAULT_CTRL_TLT_ANG_PM	    725.0f
#define DEFAULT_CTRL_TLT_ANG_IM	    360.0f
#define DEFAULT_CTRL_TLT_ANG_DM	    725.0f
#define DEFAULT_CTRL_TLT_ANG_OM	    1200.0f

// YAW angle PID
#define DEFAULT_CTRL_YAW_ANG_P      0.05f
#define DEFAULT_CTRL_YAW_ANG_I      0.00002f
#define DEFAULT_CTRL_YAW_ANG_D      0.0f
#define DEFAULT_CTRL_YAW_ANG_F      0.0f
#define DEFAULT_CTRL_YAW_ANG_PM     1.25f
#define DEFAULT_CTRL_YAW_ANG_IM     0.04f
#define DEFAULT_CTRL_YAW_ANG_DM     0.0f
#define DEFAULT_CTRL_YAW_ANG_OM     1.25f

#define DEFAULT_MOT_CANL	    0           //  1 - 16
#define DEFAULT_MOT_CANH	    0           // 17 - 32
#define DEFAULT_MOT_FRAME	    0		// used as hint for frame config GUI
#define DEFAULT_MOT_ARM		    975
#define DEFAULT_MOT_MIN		    1000
#define DEFAULT_MOT_START	    1125
#define DEFAULT_MOT_MAX		    1950
#define DEFAULT_MOT_VALUE2T_A1	    0.0f
#define DEFAULT_MOT_VALUE2T_A2	    0.0f
#define DEFAULT_MOT_VALUE_SCAL	    0.0f

#define DEFAULT_MOT_ESC_TYPE        0        // ESC Type; 0 = standard PWM, 1 = M4 onboard, 2 = AQ ESC32; add 24th bit to run PWM ESC endpoint calibration on next bootup (eg: 8388608 for std ESC); bits 20-24 are reserved

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
#define DEFAULT_MOT_PWRD_15_T	    0.0
#define DEFAULT_MOT_PWRD_15_P	    0.0
#define DEFAULT_MOT_PWRD_15_R	    0.0
#define DEFAULT_MOT_PWRD_15_Y	    0.0
#define DEFAULT_MOT_PWRD_16_T	    0.0
#define DEFAULT_MOT_PWRD_16_P	    0.0
#define DEFAULT_MOT_PWRD_16_R	    0.0
#define DEFAULT_MOT_PWRD_16_Y	    0.0


#define DEFAULT_COMM_BAUD1	    115200
#define DEFAULT_COMM_BAUD2	    230400
#define DEFAULT_COMM_BAUD3	    3000000
#define DEFAULT_COMM_BAUD4	    460800
#define DEFAULT_COMM_BAUD5	    115200                       // CAN UART stream ID 1
#define DEFAULT_COMM_BAUD6	    115200                       // CAN UART stream ID 2
#define DEFAULT_COMM_BAUD7	    115200                       // CAN UART stream ID 3
#define DEFAULT_COMM_STREAM_TYP1    COMM_STREAM_TYPE_MAVLINK
#define DEFAULT_COMM_STREAM_TYP2    0
#define DEFAULT_COMM_STREAM_TYP3    0
#define DEFAULT_COMM_STREAM_TYP4    0
#define DEFAULT_COMM_STREAM_TYP5    COMM_STREAM_TYPE_MAVLINK    // CAN UART stream ID 1
#define DEFAULT_COMM_STREAM_TYP6    0                           // CAN UART stream ID 2
#define DEFAULT_COMM_STREAM_TYP7    0                           // CAN UART stream ID 3

#define DEFAULT_TELEMETRY_RATE	    20		// loops between reports

// RC receiver telemetry config bits (up to 24b can be used).
//    4 LSB denote telemetry type to use: 0 = none, 1 = FrSky SmartPort
//    Remaining 20 bits can be used for settings specific to telemetry protocol.
//    For FrSky SmartPort:
//	b4:enable custom data;
//      b5:send ACCX/Y/Z;
//      b6:require 3D fix before reporting location for first time (otherwise 2D);
//      b7:wait for GPS fix before reporting UKF altitude;
//      b8: send text messages
#define DEFAULT_TELEMETRY_RX_CFG    0b011000000


#define DEFAULT_NAV_MAX_SPEED	    5.0f	// m/s
#define DEFAULT_NAV_MAX_ASCENT      2.5f	// m/s
#define DEFAULT_NAV_MAX_DECENT	    1.5f	// m/s
#define DEFAULT_NAV_CEILING         0.0f	// m relative to home alt. Maximum altitude in alt/pos/mission/dvh modes
#define DEFAULT_NAV_LANDING_VEL	    1.0f	// m/s

// speed => tilt PID
#define DEFAULT_NAV_SPEED_P	    10.5f
#define DEFAULT_NAV_SPEED_I	    0.0075f
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
#define DEFAULT_NAV_ALT_SPED_P	    200.0f
#define DEFAULT_NAV_ALT_SPED_I	    2.85f
#define DEFAULT_NAV_ALT_SPED_PM	    150.0f
#define DEFAULT_NAV_ALT_SPED_IM	    700.0f
#define DEFAULT_NAV_ALT_SPED_OM	    700.0f

// Altitude hold Position PID
#define DEFAULT_NAV_ALT_POS_P	    0.20f
#define DEFAULT_NAV_ALT_POS_I	    0.0f
#define DEFAULT_NAV_ALT_POS_PM	    2.5f
#define DEFAULT_NAV_ALT_POS_IM	    0.0f
#define DEFAULT_NAV_ALT_POS_OM	    2.5f

#define DEFAULT_NAV_ALT_GPS_ACC     0.8f     // Maximum (worst) GPS accuracy at which GPS vertical position is preferred over pressure sensor. 0 to always use PS.

// Control definitions for flight mode, etc.  20 bits: SVVV_VVVV_VVVV_CCCC_CCCC
// 	high bit = value Sign (0=neg,1=pos), next 11b = absolute channel Value (0-2047), 8 low bits = Channel number (0-255, zero == no channel/disabled)
//  Set bit 23 high (0x800000) to reverse a radio channel.

#define DEFAULT_NAV_CTRL_AH	    0                         // disabled
#define DEFAULT_NAV_CTRL_PH	    6                         // ch.6, middle (zero value) position
#define DEFAULT_NAV_CTRL_MISN	    ((1<<19) | (501<<8) | 6)  // ch.6, high (+501 +/-CTRL_DBAND_SWTCH)
#define DEFAULT_NAV_CTRL_HOM_SET    ((1<<19) | (501<<8) | 7)  // ch.7 high
#define DEFAULT_NAV_CTRL_HOM_GO     ((0<<19) | (501<<8) | 7)  // ch.7 low (-501 +/-CTRL_DBAND_SWTCH)
#define DEFAULT_NAV_CTRL_HF_SET     ((1<<19) | (501<<8) | 0)  // disabled, high
#define DEFAULT_NAV_CTRL_HF_LOCK    ((0<<19) | (501<<8) | 0)  // disabled, low
#define DEFAULT_NAV_CTRL_WP_REC     ((1<<19) | (501<<8) | 0)  // disabled, high
#define DEFAULT_NAV_CTRL_RT_MODE    ((0<<19) | (501<<8) | 0)  // rate control mode switch
#define DEFAULT_NAV_CTRL_LR_MODE    ((0<<19) | (501<<8) | 0)  // limited-rate (with maximum angles) control mode switch  ("beginner acro")


#define DEFAULT_IMU_FLIP            0                   // flip DIMU: 0 == none, 1 == around roll axis (left becomes right), 2 == around pitch axis (front becomes back)
#define DEFAULT_IMU_ROT		    +0.0		// degrees to rotate the IMU to align with the frame (applied after FLIP)
#define DEFAULT_IMU_MAG_INCL	    -65.0
#define DEFAULT_IMU_MAG_DECL	    0.0
#define DEFAULT_IMU_PRESS_SENSE	    0.0f		// 0 == sensor #1, 1 == sensor #2, 2 == both


#define DEFAULT_GMBL_PITCH_PORT		0		// Gimbal pitch stabilization output port. 0 == disabled
#define DEFAULT_GMBL_CTRL_TILT		0		// Gimbal manual tilt control definition (channel number).
#define DEFAULT_GMBL_TILT_PORT		0		// Gimbal manual/PoI tilt control output port (can be same as PITCH_PORT to combine the 2 functions). 0 == disabled
#define DEFAULT_GMBL_ROLL_PORT		0		// Gimbal roll stabilization output port. 0 == disabled
#define DEFAULT_GMBL_PWM_MAX_RL		2250
#define DEFAULT_GMBL_PWM_MIN_RL		750
#define DEFAULT_GMBL_PWM_MAX_PT		2250
#define DEFAULT_GMBL_PWM_MIN_PT		750
#define DEFAULT_GMBL_PWM_FREQ		200
#define DEFAULT_GMBL_NTRL_PITCH		1575
#define DEFAULT_GMBL_NTRL_ROLL		1442
#define DEFAULT_GMBL_SCAL_PITCH		(1.0f / 91.0f)
#define DEFAULT_GMBL_SCAL_ROLL		(1.0f / 70.0f)
#define DEFAULT_GMBL_SLEW_RATE		0.005f
#define DEFAULT_GMBL_ROLL_EXPO		0.0f
#define DEFAULT_GMBL_TRIG_PORT		0		// Triggering/passthrough output port. 0 == disabled
#define DEFAULT_GMBL_CTRL_TRG_ON	0		// Channel & position control definition for trigger (see comment on NAV_CTRL params). Used for passthrough or with automated triggering.
#define DEFAULT_GMBL_TRIG_ON_PWM	2100		// PWM output of trigger at full-press (on) position (for automated triggering)
#define DEFAULT_GMBL_TRIG_ON_TIM	1000		// Time to keep shutter at on position, in ms
#define DEFAULT_GMBL_TRIG_DIST		0.0f		// Activate trigger every this many meters. 0 == disabled
#define DEFAULT_GMBL_TRIG_TIME		0		// Activate trigger every this many seconds. 0 == disabled
#define DEFAULT_GMBL_PSTHR_CHAN		0		// Pure passthrough radio channel (eg. camera zoom, pan axis, lights, etc). 0 == disabled
#define DEFAULT_GMBL_PSTHR_PORT		0		// Pure passthrough output port. 0 == disabled


#define DEFAULT_SPVR_LOW_BAT1	    3.5f	    // cell volts
#define DEFAULT_SPVR_LOW_BAT2	    3.3f	    // cell volts
#define DEFAULT_SPVR_FS_RAD_TO1     1.0f        // radio failsafe stage 1 timeout (seconds)
#define DEFAULT_SPVR_FS_RAD_TO2     15.0f       // radio failsafe stage 2 timeout (seconds)
#define DEFAULT_SPVR_FS_RAD_ST1     0           // radio failsafe stage 1 action (0 = position hold)
#define DEFAULT_SPVR_FS_RAD_ST2     0           // radio failsafe stage 2 action (0 = land, 1 = RTH, land, 2 = Ascend if needed, RTH, land)
#define DEFAULT_SPVR_FS_ADD_ALT     0.0f        // meters to add to home alt. for falisafe stg.2 action == 2
#define DEFAULT_SPVR_VIN_SOURCE     0           // source to use for measuring primary voltage (0 = mains voltage on all boards; 1 = onboard external voltage divider)


#define DEFAULT_QUATOS_ENABLE		1		    // 0 = do not use Quatos; 1 = use Quatos
#define DEFAULT_QUATOS_J_ROLL		0.0127f		    // J matrix
#define DEFAULT_QUATOS_J_PITCH		0.0127f		    // J matrix
#define DEFAULT_QUATOS_J_YAW		0.024661f	    // J matrix
#define DEFAULT_QUATOS_AM1		-10.0f
#define DEFAULT_QUATOS_AM2		-25.0f
#define DEFAULT_QUATOS_PROP_K1		14.62f
#define DEFAULT_QUATOS_M_TLT_RT		1.0f		    // rads/s
#define DEFAULT_QUATOS_M_YAW_RT		1.0f		    // rads/s
#define DEFAULT_QUATOS_MAX_OUT		4.0f		    // total moment demand allowed (n_a + n_b)
#define DEFAULT_QUATOS_QUAT_TAU		0.05f
#define DEFAULT_QUATOS_L1_ASP		-10.0f
#define DEFAULT_QUATOS_L1_K1		18.6f

#define DEFAULT_QUATOS_MM_R01		0.0f
#define DEFAULT_QUATOS_MM_P01		0.0f
#define DEFAULT_QUATOS_MM_Y01		0.0f
#define DEFAULT_QUATOS_MM_R02		0.0f
#define DEFAULT_QUATOS_MM_P02		0.0f
#define DEFAULT_QUATOS_MM_Y02		0.0f
#define DEFAULT_QUATOS_MM_R03		0.0f
#define DEFAULT_QUATOS_MM_P03		0.0f
#define DEFAULT_QUATOS_MM_Y03		0.0f
#define DEFAULT_QUATOS_MM_R04		0.0f
#define DEFAULT_QUATOS_MM_P04		0.0f
#define DEFAULT_QUATOS_MM_Y04		0.0f
#define DEFAULT_QUATOS_MM_R05		0.0f
#define DEFAULT_QUATOS_MM_P05		0.0f
#define DEFAULT_QUATOS_MM_Y05		0.0f
#define DEFAULT_QUATOS_MM_R06		0.0f
#define DEFAULT_QUATOS_MM_P06		0.0f
#define DEFAULT_QUATOS_MM_Y06		0.0f
#define DEFAULT_QUATOS_MM_R07		0.0f
#define DEFAULT_QUATOS_MM_P07		0.0f
#define DEFAULT_QUATOS_MM_Y07		0.0f
#define DEFAULT_QUATOS_MM_R08		0.0f
#define DEFAULT_QUATOS_MM_P08		0.0f
#define DEFAULT_QUATOS_MM_Y08		0.0f
#define DEFAULT_QUATOS_MM_R09		0.0f
#define DEFAULT_QUATOS_MM_P09		0.0f
#define DEFAULT_QUATOS_MM_Y09		0.0f
#define DEFAULT_QUATOS_MM_R10		0.0f
#define DEFAULT_QUATOS_MM_P10		0.0f
#define DEFAULT_QUATOS_MM_Y10		0.0f
#define DEFAULT_QUATOS_MM_R11		0.0f
#define DEFAULT_QUATOS_MM_P11		0.0f
#define DEFAULT_QUATOS_MM_Y11		0.0f
#define DEFAULT_QUATOS_MM_R12		0.0f
#define DEFAULT_QUATOS_MM_P12		0.0f
#define DEFAULT_QUATOS_MM_Y12		0.0f
#define DEFAULT_QUATOS_MM_R13		0.0f
#define DEFAULT_QUATOS_MM_P13		0.0f
#define DEFAULT_QUATOS_MM_Y13		0.0f
#define DEFAULT_QUATOS_MM_R14		0.0f
#define DEFAULT_QUATOS_MM_P14		0.0f
#define DEFAULT_QUATOS_MM_Y14		0.0f
#define DEFAULT_QUATOS_MM_R15		0.0f
#define DEFAULT_QUATOS_MM_P15		0.0f
#define DEFAULT_QUATOS_MM_Y15		0.0f
#define DEFAULT_QUATOS_MM_R16		0.0f
#define DEFAULT_QUATOS_MM_P16		0.0f
#define DEFAULT_QUATOS_MM_Y16		0.0f


#define DEFAULT_LIC_KEY1		0.0f
#define DEFAULT_LIC_KEY2		0.0f
#define DEFAULT_LIC_KEY3		0.0f
