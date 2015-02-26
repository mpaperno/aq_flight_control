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

#ifndef RC_H_
#define RC_H_

#include "radio.h"
#include "config.h"
#include <stdint.h>

/* Define RC_SWITCH_VALUE_BOUNDS > 0 to use "loose" upper/lower bounds checking on switch type controls,
 *  where a channel value and configured activation value over/under +/-RC_SWITCH_VALUE_BOUNDS counts as activation.
 * Eg. if mission mode configuration is on channel 6 with a range of 450 to 550 (NAV_CTRL_MISN=500 +/- CTRL_DBAND_SWTCH=50),
 *  then any channel 6 pulse value > 550 would also count as mission mode being activated.
 * Undef this value or set it to zero to use strict bounds checking. See #ifdef code in rcIsSwitchActive().
 */
#define RC_SWITCH_VALUE_BOUNDS		550
// minimum RADIO_QUALITY needed to allow arming
#define RC_MIN_RADIO_QUALITY_ARM	40


/* private functions (do not use externally) */

#define rcGetChannelValue(X)		((X >= 0 && X < RADIO_MAX_CHANNELS) ? radioData.channels[X] : 0)
// TODO: RADIO_MODE_SPLIT not tested -- concept only!
//#define rcGetChannelValue(X)		((X >= 0) ? ((radioData.mode == RADIO_MODE_SPLIT && X < RADIO_MAX_CHANNELS*RADIO_NUM) ? radioData.allChannels[X] : ((X < RADIO_MAX_CHANNELS) ? radioData.channels[X] : 0)) : 0)

// unused for now
//#define rcSetChannelValue(X, Y)	if (X >= 0 && X < RADIO_MAX_CHANNELS) radioData.channels[X] = Y;


/* public functions */

// Returns true if any control channel (eg. radio channel) is configured for a given control parameter.
#define rcIsControlConfigured(X)	((uint32_t)p[X] & 0x3F)

// Returns the raw value of a control channel (eg. radio channel). Default value is zero.
#define rcGetControlValue(X)		(rcGetChannelValue(((uint32_t)p[X] & 0x3F)-1))

/* end function macros */


enum rcErrorCodes {
    RC_ERROR_NONE = 0,
    RC_ERROR_LOW_RADIO_QUAL
};

extern uint8_t rcCheckValidController(void);
extern uint8_t rcIsSwitchActive(int paramId);
//extern void    rcSetSwitchActive(int paramId);

#endif /* RC_H_ */
