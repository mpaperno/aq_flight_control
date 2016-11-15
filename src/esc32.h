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

#ifndef _esc32_h
#define _esc32_h

#include "can.h"

#define ESC32_FILE_NAME			"esc32.txt"
#define ESC32_FILE_BUF_SIZE		512
#define ESC32_LINE_BUF_SIZE		128

enum esc32States {
    ESC32_STATE_DISARMED = 0,
    ESC32_STATE_STOPPED,
    ESC32_STATE_NOCOMM,
    ES32C_STATE_STARTING,
    ESC32_STATE_RUNNING
};

typedef struct {
    unsigned int state :    3;
    unsigned int vin :	    12;	// x 100
    unsigned int amps :	    14;	// x 100
    unsigned int rpm :	    15;
    unsigned int duty :	    8;	// x (255/100)
    unsigned int temp :     9;  // (Deg C + 32) * 4
    unsigned int errCode :  3;
} __attribute__((packed)) esc32CanStatus_t;

extern void esc32SetupOw(const GPIO_TypeDef *port, const uint16_t pin, uint8_t mode);
extern float esc32SetupCan(canNodes_t *canNode, uint8_t mode);

#endif
