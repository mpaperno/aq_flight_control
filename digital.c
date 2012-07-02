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

#include "digital.h"
#include "util.h"
#include <stdlib.h>

digitalPin *digitalInit(GPIO_TypeDef* port, const uint16_t pin) {
    digitalPin *p;
    GPIO_InitTypeDef GPIO_InitStructure;

    p = (digitalPin *)aqDataCalloc(1, sizeof(digitalPin));
    p->port = port;
    p->pin = pin;

    digitalLo(p);

    GPIO_StructInit(&GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = pin;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(port, &GPIO_InitStructure);

    return p;
}

void digitalTogg(digitalPin *p) {
    if (digitalGet(p)) {
	digitalLo(p);
    }
    else {
	digitalHi(p);
    }
}
