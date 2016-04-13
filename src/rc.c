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

#include "rc.h"
#include "comm.h"

/* Return the current value of a radio channel. Typically you'd wan to use rcGetControlValue() or rcIsSwitchActive() instead. */
__attribute__((always_inline))
inline int16_t rcGetChannelValue(int chan) {
    if (rcIsControlChannelValid(chan)) {
	--chan;
	if (radioData.mode == RADIO_MODE_DIVERSITY)
	    return radioData.channels[chan];
	else if (chan < RADIO_MAX_CHANNELS)
	    return radioData.allChannels[chan];
    }
    return 0;
}

/* Sets a radio channel to a specified value. This would get overwritten if an active radio is controlling the same channel.
 * Typically you'd wan to use rcSetControlValue() or rcSetSwitchActive() instead. */
__attribute__((always_inline))
inline void rcSetChannelValue(int chan, int val) {
    if (rcIsControlChannelValid(chan)) {
	--chan;
	if (radioData.mode == RADIO_MODE_DIVERSITY)
	    radioData.channels[chan] = val;
	else if (chan < RADIO_MAX_CHANNELS)
	    radioData.allChannels[chan] = val;
    }
}


/* Returns true if current RC controller value (eg. radio channel) matches configured value (channel and position) for a given control parameter. */
__attribute__((always_inline))
inline uint8_t rcIsSwitchActive(int paramId) {
    if (rcIsControlConfigured(paramId)) {
	int16_t chanVal = rcGetControlValue(paramId);
	int16_t targetVal = rcGetSwitchTargetValue(paramId);

	return (chanVal >= targetVal - rcGetSwitchDeadband() && chanVal <= targetVal + rcGetSwitchDeadband());
    }
    return 0;
}

/* Return true if given controls have overlapping switch positions (eg. if same channel value might mean PH and MISN) */
static uint8_t rcCheckSwitchRangeOverlap(int *swarry, uint8_t alen) {
    int i, j;

    for (i=0; i < alen; ++i) {
	if (!rcIsControlConfigured(swarry[i]))
	    continue;

	for (j=0; j < alen; ++j) {
	    if (swarry[j] == swarry[i] || !rcIsControlConfigured(swarry[j]) || rcGetControlChannel(swarry[i]) != rcGetControlChannel(swarry[j]))
		continue;

	    if (rcGetSwitchTargetValue(swarry[i]) - rcGetSwitchDeadband() <= rcGetSwitchTargetValue(swarry[j]) + rcGetSwitchDeadband() &&
		    rcGetSwitchTargetValue(swarry[j]) - rcGetSwitchDeadband() <= rcGetSwitchTargetValue(swarry[i]) + rcGetSwitchDeadband())
		return 1;
	}
    }

    return 0;
}

/* Validate that all important controls don't overlap/contradict each other */
static uint8_t rcCheckControlsOverlap() {
    uint8_t ret = RC_ERROR_NONE;
    int switches[] = { NAV_CTRL_AH, NAV_CTRL_PH, NAV_CTRL_MISN };

    if (rcCheckSwitchRangeOverlap(switches, 3))
	ret |= RC_ERROR_CTRL_OVERLP_MODE;

    switches[0] = NAV_CTRL_HOM_SET;
    switches[1] = NAV_CTRL_HOM_GO;
    if (rcCheckSwitchRangeOverlap(switches, 2))
	ret |= RC_ERROR_CTRL_OVERLP_HOME;

    return ret;
}

/* Returns zero if any valid remote control is active and no problems detected, or an error code otherwise. */
uint8_t rcCheckValidController(void) {
    uint8_t ret = RC_ERROR_NONE;
    if (RADIO_QUALITY < RC_MIN_RADIO_QUALITY_ARM)
	ret |= RC_ERROR_LOW_RADIO_QUAL;

    ret |= rcCheckControlsOverlap();

    return ret;
}

static const char *rcGetErrorString(uint8_t ec) {
    switch (ec) {
	case RC_ERROR_LOW_RADIO_QUAL :
	    return (const char *)"Radio Quality too low";
	case RC_ERROR_CTRL_OVERLP_MODE :
	    return (const char *)"Mode controls overlap";
	case RC_ERROR_CTRL_OVERLP_HOME :
	    return (const char *)"Home controls overlap";
	default :
	    return (const char *)"no error";
    }
}

/* Prints all errors contained in errs bitfield, if any. */
void rcReportAllErrors(uint8_t errs) {
    for (int i=0; i < 8; ++i) {
	if (errs & (1 << i))
	    AQ_NOTICE(rcGetErrorString(1<<i));
    }
}
