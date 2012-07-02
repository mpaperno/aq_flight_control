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

#ifndef fpu_h_
#define fpu_h_

#include "stm32f4xx.h"
#include <CoOS.h>

//#define FPU_SOFT
#define FPU_HARD
//#define FPU_LAZY_SWITCH

#define __fpu_enable()	    __asm__ __volatile__ ("LDR.W    R0, =0xE000ED88 \n"		\
						    "LDR    R1, [R0] \n"		\
						    "ORR    R1, R1, #(0xF << 20) \n"	\
						    "STR    R1, [R0]")
#define __fpu_disable()	    __asm__ __volatile__ ("LDR.W    R0, =0xE000ED88 \n"		\
						    "LDR    R1, [R0] \n"		\
						    "BIC    R1, R1, #(0xF << 20) \n"	\
						    "STR    R1, [R0]")
// Store VFP registers
#define	__vfp_store(loc)    __asm__ __volatile__ ("fstmiad %0, {d0-d15} \n"		\
						    "fmrx r2, fpscr \n"			\
						    "str r2, [%0, #32*4] \n"		\
						    : :"r"(loc))
// Retrieve VFP registers
#define	__vfp_restore(loc)  __asm__ __volatile__ ("fldmiad %0, {d0-d15} \n"		\
						    "ldr r2, [%0, #32*4] \n"		\
						    "fmxr fpscr, r2 \n"			\
						    : :"r"(loc))

extern void fpuInit(void);
extern uint32_t fpuRegisters[CFG_MAX_USER_TASKS*33];

#endif
