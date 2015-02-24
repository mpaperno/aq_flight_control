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
#include "radio.h"
#include "config.h"

// Returns zero if any valid remote control is active, or an error code otherwise.
uint8_t rcCheckValidController(void) {
    if (RADIO_QUALITY < RC_MIN_RADIO_QUALITY_ARM)
	return RC_ERROR_LOW_RADIO_QUAL;
    else
	return RC_ERROR_NONE;
}

// Returns true if current controller value (eg. radio channel) matches configured value (channel and position) for a given control parameter.
uint8_t rcIsSwitchActive(int paramId) {
    uint32_t param = (uint32_t)p[paramId];
    uint8_t chan = (param & 0x3F);
    int16_t val, chanVal;

    if (chan--) {
	chanVal = rcGetChannelValue(chan);
	val = ((param >> 6) & 0x1FF);
	if (!(param & (1<<15)))  // check sign bit
	    val *= -1;

#ifdef RC_SWITCH_VALUE_BOUNDS
	if ((chanVal < -RC_SWITCH_VALUE_BOUNDS && val - (int)p[CTRL_DBAND_SWTCH] <= -RC_SWITCH_VALUE_BOUNDS) ||
		(chanVal > RC_SWITCH_VALUE_BOUNDS && val + (int)p[CTRL_DBAND_SWTCH] >= RC_SWITCH_VALUE_BOUNDS))
	    return 1;
	else
#endif
	if (chanVal > val - (int)p[CTRL_DBAND_SWTCH] && chanVal < val + (int)p[CTRL_DBAND_SWTCH])
	    return 1;

    }

    return 0;
}

// Sets a controller channel value to reflect the active state of a given control parameter.
// Currently this would be overwritten if any actual RC is present (eg. an active radio connection).
/* unused for now
void rcSetSwitchActive(int paramId) {
    uint32_t param = (uint32_t)p[paramId];
    uint8_t chan = (param & 0x3F);
    int16_t val;

    if (chan--) {
	val = ((param >> 6) & 0x1FF);
	if (!(param & (1<<15)))  // check sign bit
	    val *= -1;
	rcSetChannelValue(chan, val);
    }
}
 */
