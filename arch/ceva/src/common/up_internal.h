/****************************************************************************
 * arch/ceva/src/common/up_internal.h
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_CEVA_SRC_COMMON_UP_INTERNAL_H
#define __ARCH_CEVA_SRC_COMMON_UP_INTERNAL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <sys/types.h>
#  include <stdint.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Determine which (if any) console driver to use.  If a console is enabled
 * and no other console device is specified, then a serial console is
 * assumed.
 */

#if !defined(CONFIG_DEV_CONSOLE) || CONFIG_NFILE_DESCRIPTORS == 0
#  undef  USE_SERIALDRIVER
#  undef  USE_EARLYSERIALINIT
#  undef  CONFIG_DEV_LOWCONSOLE
#  undef  CONFIG_RAMLOG_CONSOLE
#else
#  if defined(CONFIG_RAMLOG_CONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#    undef  CONFIG_DEV_LOWCONSOLE
#  elif defined(CONFIG_DEV_LOWCONSOLE)
#    undef  USE_SERIALDRIVER
#    undef  USE_EARLYSERIALINIT
#  else
#    define USE_SERIALDRIVER 1
#    define USE_EARLYSERIALINIT 1
#  endif
#endif

/* If some other device is used as the console, then the serial driver may
 * still be needed.  Let's assume that if the upper half serial driver is
 * built, then the lower half will also be needed.  There is no need for
 * the early serial initialization in this case.
 */

#if !defined(USE_SERIALDRIVER) && defined(CONFIG_STANDARD_SERIAL)
#  define USE_SERIALDRIVER 1
#endif

/* Linker defined section addresses */

#define _START_TEXT    ((const void *)&_stext)
#define _END_TEXT      ((const void *)&_etext)
#define _START_BSS     ((void *)&_sbss)
#define _END_BSS       ((void *)&_ebss)
#define _DATA_INIT     ((const void *)&_eronly)
#define _START_DATA    ((void *)&_sdata)
#define _END_DATA      ((void *)&_edata)
#define _START_HEAP    ((void *)&_ebss + B2C(CONFIG_IDLETHREAD_STACKSIZE))
#define _END_HEAP      ((void *)&_eheap)
#define _END_MEM       ((void *)~0)

/* This is the value used to mark the stack for subsequent stack monitoring
 * logic.
 */

#define STACK_COLOR    0xdeadbeef
#define INTSTACK_COLOR 0xdeadbeef
#define HEAP_COLOR     'h'

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

#ifdef CONFIG_SMP
/* For the case of architectures with multiple CPUs, then there must be one
 * such value for each processor that can receive an interrupt.
 */

int up_cpu_index(void); /* See include/nuttx/arch.h */

EXTERN uint32_t *volatile g_current_regs[CONFIG_SMP_NCPUS];
#  define CURRENT_REGS (g_current_regs[up_cpu_index()])

#else

EXTERN uint32_t *volatile g_current_regs[1];
#  define CURRENT_REGS (g_current_regs[0])

#endif

/* This is the beginning of heap as provided from up_head.S.
 * This is the first address in DRAM after the loaded
 * program+bss+idle stack.  The end of the heap is
 * CONFIG_RAM_END
 */

EXTERN void *g_idle_basestack;
EXTERN void *g_idle_topstack;

/* Address of the interrupt stack pointer */

EXTERN char g_intstackalloc; /* Allocated stack base */
EXTERN char g_intstackbase;  /* Initial top of interrupt stack */

/* These 'addresses' of these values are setup by the linker script.  They are
 * not actual char storage locations! They are only used meaningfully in the
 * following way:
 *
 *  - The linker script defines, for example, the symbol _sdata.
 *  - The declareion extern char _sdata; makes C happy.  C will believe
 *    that the value _sdata is the address of a char variable _data (it is
 *    not!).
 *  - We can recover the linker value then by simply taking the address of
 *    of _data.  like:  char *pdata = &_sdata;
 */

EXTERN const char _stext , _stext2 , _stext3 , _stext4 ; /* Start of .text */
EXTERN const char _etext , _etext2 , _etext3 , _etext4 ; /* End+1 of .text */
EXTERN const char _eronly, _eronly2, _eronly3, _eronly4; /* End+1 of read only section (.text + .rodata) */
EXTERN char _sdata, _sdata2, _sdata3, _sdata4;           /* Start of .data */
EXTERN char _edata, _edata2, _edata3, _edata4;           /* End+1 of .data */
EXTERN char _sbss , _sbss2 , _sbss3 , _sbss4 ;           /* Start of .bss */
EXTERN char _ebss , _ebss2 , _ebss3 , _ebss4 ;           /* End+1 of .bss */
EXTERN char _eheap, _eheap2, _eheap3, _eheap4;           /* End+1 of the memory */

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Context switching */

int  up_saveusercontext(uint32_t *saveregs);
void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
void up_switchcontext(uint32_t **saveregs, uint32_t *restoreregs);

/* Signal handling **********************************************************/

void up_sigdeliver(void);

/* Arch specific ************************************************************/

void up_earlyinitialize(void);
void up_lateinitialize(void);
void up_finalinitialize(void);

/* Power management *********************************************************/

#ifdef CONFIG_PM
void up_pminitialize(void);
#else
#  define up_pminitialize()
#endif

void up_reset(void);

void up_cpu_doze(void);
void up_cpu_idle(void);
void up_cpu_standby(void);
void up_cpu_sleep(void);
void up_cpu_normal(void);

/* Interrupt handling *******************************************************/

void up_irqinitialize(void);

/* Interrupt acknowledge and dispatch */

uint32_t *up_doirq(int irq, uint32_t *regs);

/* Exception Handlers */

int  up_svcall(int irq, FAR void *context, FAR void *arg);
int  up_hardfault(int irq, FAR void *context, FAR void *arg);

void up_svcall_handler(void);

/* System timer *************************************************************/

void ceva_timer_initialize(void);

/* Low level serial output **************************************************/

#ifdef USE_SERIALDRIVER
void up_serialinit(void);
#else
#  define up_serialinit()
#endif

#ifdef USE_EARLYSERIALINIT
void up_earlyserialinit(void);
#else
#  define up_earlyserialinit()
#endif

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void);
#else
#  define rpmsg_serialinit()
#endif

/* Defined in drivers/lowconsole.c */

#ifdef CONFIG_DEV_LOWCONSOLE
void lowconsole_init(void);
#else
#  define lowconsole_init()
#endif

/* DMA **********************************************************************/

#ifdef CONFIG_ARCH_DMA
void up_dma_initialize(void);
#endif

/* Memory management ********************************************************/

#if CONFIG_MM_REGIONS > 1
void up_addregion(void);
#else
# define up_addregion()
#endif

/* Watchdog timer ***********************************************************/

void up_wdtinit(void);

/* Networking ***************************************************************/

#if defined(CONFIG_NET) && !defined(CONFIG_NETDEV_LATEINIT)
void up_netinitialize(void);
#else
# define up_netinitialize()
#endif

/* USB **********************************************************************/

#ifdef CONFIG_USBDEV
void up_usbinitialize(void);
void up_usbuninitialize(void);
#else
# define up_usbinitialize()
# define up_usbuninitialize()
#endif

/* Debug ********************************************************************/
#ifdef CONFIG_HEAP_COLORATION
#  define up_heap_color(start, size) memset(start, HEAP_COLOR, size)
#else
#  define up_heap_color(start, size)
#endif

#ifdef CONFIG_STACK_COLORATION
void up_stack_color(FAR void *stackbase, size_t nbytes);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif /* __ASSEMBLY__ */

#endif /* __ARCH_CEVA_SRC_COMMON_UP_INTERNAL_H */
