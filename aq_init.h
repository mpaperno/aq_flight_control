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

#ifndef _aq_init_h
#define _aq_init_h

#include "aq.h"
#include "digital.h"
#include <CoOS.h>

#define AQINIT_STACK_SIZE   400
#define AQINIT_PRIORITY	    12

extern digitalPin *tp;

extern OS_STK   *aqInitStack;

extern void aqInit(void *pdata);

#endif
