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

#ifndef _DIGITAL_H
#define _DIGITAL_H

#include "aq.h"

typedef uint32_t digitalPin;

#define digitalHi(p)        *p = 1
#define digitalLo(p)        *p = 0
#define digitalSet(p, n)    *p = n
#define digitalGet(p)       (*p)
#define digitalTogg(p)      *p = !(*p)

extern uint32_t *digitalInit(GPIO_TypeDef* port, const uint16_t pin, uint8_t initial);

#endif
