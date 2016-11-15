/*****************************************************************************
 * Copyright (c) 2007, 2009 Rowley Associates Limited.                       *
 *                                                                           *
 * This file may be distributed under the terms of the License Agreement     *
 * provided with this software.                                              *
 *                                                                           *
 * THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND, INCLUDING THE   *
 * WARRANTY OF DESIGN, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. *
 *****************************************************************************/

/*****************************************************************************
 *                           Preprocessor Definitions
 *                           ------------------------
 *
 * STARTUP_FROM_RESET
 *
 *   If defined, the program will startup from power-on/reset. If not defined
 *   the program will just loop endlessly from power-on/reset.
 *
 *   This definition is not defined by default on this target because the
 *   debugger is unable to reset this target and maintain control of it over the
 *   JTAG interface. The advantage of doing this is that it allows the debugger
 *   to reset the CPU and run programs from a known reset CPU state on each run.
 *   It also acts as a safety net if you accidently download a program in FLASH
 *   that crashes and prevents the debugger from taking control over JTAG
 *   rendering the target unusable over JTAG. The obvious disadvantage of doing
 *   this is that your application will not startup without the debugger.
 *
 *   We advise that on this target you keep STARTUP_FROM_RESET undefined whilst
 *   you are developing and only define STARTUP_FROM_RESET when development is
 *   complete.
 *
 * __NO_SYSTEM_INIT
 *
 *   If defined, the SystemInit() function will NOT be called. By default SystemInit()
 *   is called after reset to enable the clocks and memories to be initialised 
 *   prior to any C startup initialisation.
 *
 * VECTORS_IN_RAM
 *
 *   If defined then the exception vectors are copied from Flash to RAM
 *
 * __TARGET_*
 *
 *  Interrupt vectors table selection
 *
 * __NO_FPU
 *
 *   If defined do NOT turn on the FPU for F4xx/F3xx parts
 *
 * __NO_RUNFAST_MODE
 *
 *   If defined do NOT turn on flush-to-zero and default NaN modes
 *
 *****************************************************************************/

.macro ISR_HANDLER name=
  .section .vectors, "ax"
  .word \name
  .section .init, "ax"
  .thumb_func
  .weak \name
\name:
1: b 1b /* endless loop */
.endm

.macro ISR_RESERVED
  .section .vectors, "ax"
  .word 0
.endm

  .syntax unified
  .global reset_handler

  .section .vectors, "ax"
  .code 16  
  .global _vectors  

_vectors:
  .word __stack_end__
#ifdef STARTUP_FROM_RESET
  .word reset_handler
#else
  .word reset_wait
#endif /* STARTUP_FROM_RESET */
ISR_HANDLER NMI_Handler
ISR_HANDLER HardFault_Handler
ISR_HANDLER MemManage_Handler 
ISR_HANDLER BusFault_Handler
ISR_HANDLER UsageFault_Handler
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_RESERVED
ISR_HANDLER SVC_Handler
ISR_HANDLER DebugMon_Handler
ISR_RESERVED
ISR_HANDLER PendSV_Handler
ISR_HANDLER SysTick_Handler 
#if defined(STM32F0XX_MD) || defined(STM32F030X8)
  #include "STM32F0xx.vec"
#elif defined(STM32F0XX_LD) || defined(STM32F030X6)
  #include "STM32F0xx_LD.vec"
#elif defined(STM32F10X_LD)
  #include "STM32F10X_LD.vec"
#elif defined(STM32F10X_MD)
  #include "STM32F10X_MD.vec"
#elif defined(STM32F10X_HD)
  #include "STM32F10X_HD.vec"
#elif defined(STM32F10X_XL)
  #include "STM32F10X_XL.vec"
#elif defined(STM32F10X_CL)
  #include "STM32F10X_CL.vec"
#elif defined(STM32F10X_LD_VL)
  #include "STM32F10X_LD_VL.vec"
#elif defined(STM32F10X_MD_VL)
  #include "STM32F10X_MD_VL.vec"
#elif defined(STM32F10X_HD_VL)
  #include "STM32F10X_HD_VL.vec"
#elif defined(STM32F2XX)
  #include "STM32F2XX.vec"
#elif defined(STM32F30X)
  #include "STM32F30x.vec"
#elif defined(STM32F37X)
  #include "STM32F37x.vec"
#elif defined(STM32F40_41xxx)
  #include "STM32F40_41xxx.vec"
#elif defined(STM32F401xx)
  #include "STM32F401xx.vec"
#elif defined(STM32F427_437xx)
  #include "STM32F427_437xx.vec"
#elif defined(STM32F429_439xx)
  #include "STM32F429_439xx.vec"
#elif defined(STM32L1XX_MD)
  #include "STM32L1XX_MD.vec"
#elif defined(STM32L1XX_MDP)
  #include "STM32L1XX_MDP.vec"
#elif defined(STM32L1XX_HD)
  #include "STM32L1XX_HD.vec"
#elif defined (STM32W108C8) && defined (STM32W108CB) && defined (STM32W108CC) && defined (STM32W108CZ) && defined (STM32W108HB)
  #include "STM32W108.vec"
#else 
  #error target not defined
#endif
  .section .vectors, "ax"
_vectors_end:


#ifdef VECTORS_IN_RAM
  .section .vectors_ram, "ax"
_vectors_ram:
  .space _vectors_end-_vectors, 0
#endif

  .section .init, "ax"
  .thumb_func

reset_handler:

#ifndef __NO_SYSTEM_INIT
  ldr r0, =__RAM_segment_end__
  mov sp, r0
  bl SystemInit
#endif

#ifdef VECTORS_IN_RAM
  ldr r0, =__vectors_load_start__
  ldr r1, =__vectors_load_end__
  ldr r2, =_vectors_ram
l0:
  cmp r0, r1
  beq l1
  ldr r3, [r0]
  str r3, [r2]
  adds r0, r0, #4
  adds r2, r2, #4
  b l0
l1:
#endif

#if defined(__FPU_PRESENT)
#ifndef __NO_FPU
  // Enable CP11 and CP10 with CPACR |= (0xf<<20)
  movw r0, 0xED88
  movt r0, 0xE000
  ldr r1, [r0]
  orrs r1, r1, #(0xf << 20)
  str r1, [r0]
#ifndef __NO_RUNFAST_MODE
  nop
  nop
  nop  
  vmrs r0, fpscr
  orrs r0, r0, #(0x3 << 24) // FZ and DN
  vmsr fpscr, r0
  // clear the CONTROL.FPCA bit
  mov r0, #0
  msr control, r0 
  // FPDSCR similarly
  movw r1, 0xEF3C
  movt r1, 0xE000
  ldr r0, [r1]
  orrs r0, r0, #(0x3 << 24) // FZ and DN
  str r0, [r1]
#endif
#endif
#endif

#ifndef __ARM_ARCH_6M__
  /* Configure vector table offset register */
  ldr r0, =0xE000ED08
#ifdef VECTORS_IN_RAM
  ldr r1, =_vectors_ram
#else
  ldr r1, =_vectors
#endif
  str r1, [r0]
#endif

  b _start

#ifndef __NO_SYSTEM_INIT
  .thumb_func
  .weak SystemInit
SystemInit:
  bx lr
#endif

#ifndef STARTUP_FROM_RESET
  .thumb_func
reset_wait:
1: b 1b /* endless loop */
#endif /* STARTUP_FROM_RESET */
