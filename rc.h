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

    Copyright © 2011-2015  Bill Nesbitt
*/

#ifndef _rc_h
#define _rc_h

#include "radio.h"
#include "config.h"
#include <stdint.h>

#define RC_MIN_RADIO_QUALITY_ARM          40          // minimum RADIO_QUALITY needed to consider input as valid


/* Return the configured channel number from a control param ID. */
#define rcGetControlChannel(Pid_p)        ((uint32_t)p[Pid_p] & 0xFF)

/* Returns true if any control channel (eg. radio channel) is configured for a given control param ID. */
#define rcIsControlConfigured(Pid_p)      rcGetControlChannel(Pid_p)

/* Returns the raw value of a control channel (eg. radio channel). Default value is zero. */
#define rcGetControlValue(Pid_p)          _rcGetChannelValue(rcGetControlChannel(Pid_p)-1)

/* Return the configured channel value for a switch based on control param ID */
#define rcGetSwitchTargetValue(Pid_p)     ((((uint32_t)p[Pid_p] >> 8) & 0x7FF) * (((uint32_t)p[Pid_p] & (1<<19)) ? 1 : -1))

/* Alias the deadband config */
#define rcGetSwitchDeadband()             (int)p[CTRL_DBAND_SWTCH]

/* Sets a controller channel to the given value.
 * This would be overwritten if an active radio is controlling the same channel. */
#define rcSetControlValue(Pid_p, Chv_p)   _rcSetChannelValue(rcGetControlChannel(Pid_p)-1, Chv_p)

/* Sets a controller channel value to reflect the active state of a given switch-type control.
 * This would be overwritten if an active radio is controlling the same channel. */
#define rcSetSwitchActive(Pid_p)          { if (rcIsControlConfigured(Pid_p)) rcSetControlValue(Pid_p, rcGetSwitchTargetValue(Pid_p)); }


enum rcErrorCodes {
    RC_ERROR_NONE = 0,
    RC_ERROR_LOW_RADIO_QUAL    = 0x01,
    RC_ERROR_CTRL_OVERLP_MODE  = 0x02,
    RC_ERROR_CTRL_OVERLP_HOME  = 0x04,
};

/* Return the current value of a radio channel. Typically you'd wan to use rcGetControlValue() or rcIsSwitchActive() instead. */
__attribute__((always_inline))
static inline int16_t _rcGetChannelValue(int chan) {
    if (chan >= 0) {
	if (radioData.mode == RADIO_MODE_SPLIT) {
	    if (chan < RADIO_MAX_CHANNELS * RADIO_NUM)
		return radioData.allChannels[chan];
	}
	else if (chan < RADIO_MAX_CHANNELS)
	    return radioData.channels[chan];
    }
    return 0;
}

/* Sets a radio channel to a specified value. This would get overwritten if an active radio is controlling the same channel.
 * Typically you'd wan to use rcSetControlValue() or rcSetSwitchActive() instead. */
__attribute__((always_inline))
static inline void _rcSetChannelValue(int chan, int val) {
    if (chan >= 0) {
	if (radioData.mode == RADIO_MODE_SPLIT) {
	    if (chan < RADIO_MAX_CHANNELS * RADIO_NUM)
		radioData.allChannels[chan] = val;
	} else if (chan < RADIO_MAX_CHANNELS)
	    radioData.channels[chan] = val;
    }
}


/* Returns true if current RC controller value (eg. radio channel) matches configured value (channel and position) for a given control parameter. */
__attribute__((always_inline))
static inline uint8_t rcIsSwitchActive(int paramId) {
    if (rcIsControlConfigured(paramId)) {
	int16_t chanVal = rcGetControlValue(paramId);
	int16_t targetVal = rcGetSwitchTargetValue(paramId);

	return (chanVal >= targetVal - rcGetSwitchDeadband() && chanVal <= targetVal + rcGetSwitchDeadband());
    }
    return 0;
}

extern uint8_t rcCheckValidController(void);
extern void rcReportAllErrors(uint8_t errs);

#endif /* _rc_h */
