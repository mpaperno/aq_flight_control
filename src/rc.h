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

    Copyright 2015-2016 Maxim Paperno
*/

#ifndef _rc_h
#define _rc_h

#include "radio.h"
#include "config.h"
#include <stdint.h>

#define RC_MIN_RADIO_QUALITY_ARM          40          // minimum RADIO_QUALITY needed to consider input as valid


/* Return the configured channel number from a control param ID. */
#define rcGetControlChannel(Pid_p)        ( (uint32_t)p[Pid_p] & 0xFF )

/* Return true if control is configured to reverse given radio channel value (bit 23 set high). */
#define rcIsControlReversed(Pid_p)        ( (uint32_t)p[Pid_p] & (1<<23) )

/* Returns true if any control channel (eg. radio channel) is configured for a given control param ID. */
#define rcIsControlConfigured(Pid_p)      rcGetControlChannel(Pid_p)

/* Returns true if a control channel number is valid, based on current radio mode type. */
#define rcIsControlChannelValid(Chan_p)   ( Chan_p > 0 && Chan_p <= RADIO_MAX_CHANNELS * (radioData.mode == RADIO_MODE_SPLIT ? RADIO_NUM : 1) )

/* Returns the raw value of a control channel (eg. radio channel). Default value is zero. */
#define rcGetControlValue(Pid_p)          ( rcGetChannelValue(rcGetControlChannel(Pid_p)) * (rcIsControlReversed(Pid_p) ? -1 : 1) )

/* Return the configured channel value for a switch based on control param ID */
#define rcGetSwitchTargetValue(Pid_p)     ( (((uint32_t)p[Pid_p] >> 8) & 0x7FF) * (((uint32_t)p[Pid_p] & (1<<19)) ? 1 : -1) )

/* Alias the deadband config */
#define rcGetSwitchDeadband()             (int)p[CTRL_DBAND_SWTCH]

/* Sets a controller channel to the given value.
 * This would be overwritten if an active radio is controlling the same channel. */
#define rcSetControlValue(Pid_p, Chv_p)   rcSetChannelValue(rcGetControlChannel(Pid_p), Chv_p)

/* Sets a controller channel value to reflect the active state of a given switch-type control.
 * This would be overwritten if an active radio is controlling the same channel. */
#define rcSetSwitchActive(Pid_p)          rcSetControlValue(Pid_p, rcGetSwitchTargetValue(Pid_p))

enum rcErrorCodes {
    RC_ERROR_NONE = 0,
    RC_ERROR_LOW_RADIO_QUAL    = 0x01,
    RC_ERROR_CTRL_OVERLP_MODE  = 0x02,
    RC_ERROR_CTRL_OVERLP_HOME  = 0x04,
};

extern int16_t rcGetChannelValue(int chan);
extern void rcSetChannelValue(int chan, int val);
extern uint8_t rcIsSwitchActive(int paramId);
extern uint8_t rcCheckValidController(void);
extern void rcReportAllErrors(uint8_t errs);

#endif /* _rc_h */
