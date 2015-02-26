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

#include "rc.h"

// Returns zero if any valid remote control is active, or an error code otherwise.
uint8_t rcCheckValidController(void) {
    if (RADIO_QUALITY < RC_MIN_RADIO_QUALITY_ARM)
	return RC_ERROR_LOW_RADIO_QUAL;
    else
	return RC_ERROR_NONE;
}

// Returns true if current controller value (eg. radio channel) matches configured value (channel and position) for a given control parameter.
uint8_t rcIsSwitchActive(int paramId) {
    if (rcIsControlConfigured(paramId)) {
	int16_t chanVal = rcGetControlValue(paramId);
	int16_t targetVal = (((uint32_t)p[paramId] >> 6) & 0x1FF);

	if (!((uint32_t)p[paramId] & (1<<15)))  // check sign bit
	    targetVal *= -1;

	return (
#if defined(RC_SWITCH_VALUE_BOUNDS) && RC_SWITCH_VALUE_BOUNDS
	    (chanVal < -RC_SWITCH_VALUE_BOUNDS && targetVal - (int)p[CTRL_DBAND_SWTCH] <= -RC_SWITCH_VALUE_BOUNDS) ||
	    (chanVal > RC_SWITCH_VALUE_BOUNDS && targetVal + (int)p[CTRL_DBAND_SWTCH] >= RC_SWITCH_VALUE_BOUNDS) ||
#endif
	    (chanVal > targetVal - (int)p[CTRL_DBAND_SWTCH] && chanVal < targetVal + (int)p[CTRL_DBAND_SWTCH])
	);
    }

    return 0;
}

// Sets a controller channel value to reflect the active state of a given control parameter.
// Currently this would be overwritten if any actual RC is present (eg. an active radio connection).
/* unused for now
void rcSetSwitchActive(int paramId) {
    if (rcIsControlConfigured(paramId)) {
	int16_t targetVal = (((uint32_t)p[paramId] >> 6) & 0x1FF);
	if (!((uint32_t)p[paramId] & (1<<15)))  // check sign bit
	    targetVal *= -1;

	rcSetChannelValue(((uint32_t)p[paramId] & 0x3F)-1, targetVal);
    }
}
*/
