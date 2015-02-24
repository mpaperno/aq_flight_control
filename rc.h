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

#include <stdint.h>

/* Define RC_SWITCH_VALUE_BOUNDS to use "loose" upper/lower bounds checking on switch type controls,
 *  where a channel value and configured activation value over/under +/-RC_SWITCH_VALUE_BOUNDS counts as activation.
 * Eg. if mission mode configuration is on channel 6 with a range of 450 to 550 (NAV_CTRL_MISN=500 +/- CTRL_DBAND_SWTCH=50),
 *  then any channel 6 pulse value > 550 would also count as mission mode being activated.
 * Undef this value to use strict bounds checking. See #ifdef code in rcIsControlActive().
 */
#define RC_SWITCH_VALUE_BOUNDS		550

// minimum RADIO_QUALITY needed to allow arming
#define RC_MIN_RADIO_QUALITY_ARM	40

enum rcErrorCodes {
    RC_ERROR_NONE = 0,
    RC_ERROR_LOW_RADIO_QUAL
};

extern uint8_t rcCheckValidController(void);
extern uint8_t rcIsSwitchActive(int paramId);
//extern void    rcSetSwitchActive(int paramId);
extern int16_t rcGetControlValue(int paramId);
extern uint8_t rcIsControlConfigured(int paramId);

#endif /* RC_H_ */
