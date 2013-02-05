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

    Copyright © 2011, 2012  Bill Nesbitt
*/

#ifndef _DIGITAL_H
#define _DIGITAL_H

#include "aq.h"

typedef struct {
    GPIO_TypeDef* port;
    uint16_t pin;
} digitalPin;

#define digitalHi(p)	{ p->port->BSRRL = p->pin; }
#define digitalLo(p)	{ p->port->BSRRH = p->pin; }
#define digitalGet(p)	((p->port->ODR & p->pin) != 0)

extern digitalPin *digitalInit(GPIO_TypeDef* port, const uint16_t pin);
extern void digitalTogg(digitalPin *p);

#endif
