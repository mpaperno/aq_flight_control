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

#ifndef _ext_irq_h
#define _ext_irq_h

#define EXT_NUM_EXTI        23

typedef void extCallback_t(void);

typedef struct {
    extCallback_t *callbacks[EXT_NUM_EXTI];
} extStruct_t;

extern int extRegisterCallback(GPIO_TypeDef *port, uint16_t pin, EXTITrigger_TypeDef polarity, uint8_t priority, GPIOPuPd_TypeDef pull, extCallback_t *callback);

#endif