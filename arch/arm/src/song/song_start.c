/****************************************************************************
 * arch/arm/src/song/song_start.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/init.h>
#include <nuttx/userspace.h>

#include "arm_fpu.h"
#include "arm_mpu.h"
#include "chip.h"
#include "up_internal.h"

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

static void init_kernelspace(void);

#ifdef CONFIG_BUILD_PROTECTED
static void init_userspace(void);
#endif

#ifdef CONFIG_STACK_COLORATION
static void color_start(FAR void *stack, unsigned int size)
  __attribute__ ((naked, no_instrument_function, noreturn));
#endif

#ifdef CONFIG_SONG_COPY_TABLE
extern uint32_t _scopytable;
extern uint32_t _ecopytable;
#endif

#ifdef CONFIG_SONG_ZERO_TABLE
extern uint32_t _szerotable;
extern uint32_t _ezerotable;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void init_kernelspace(void)
{
  const uint32_t *src;
  uint32_t *dest;

#if defined CONFIG_SONG_COPY_TABLE || defined CONFIG_SONG_ZERO_TABLE
  uint32_t *table;
#endif

  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs
   */

#ifdef CONFIG_ARCH_RAMFUNCS
  for (src = &_framfuncs, dest = &_sramfuncs; dest < &_eramfuncs; )
    {
      *dest++ = *src++;
    }
#endif

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

#ifdef CONFIG_BOOT_RUNFROMFLASH
  for (src = &_eronly, dest = &_sdata; dest < &_edata; )
    {
      *dest++ = *src++;
    }
#endif

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  for (dest = &_sbss; dest < &_ebss; )
    {
      *dest++ = 0;
    }

#ifdef CONFIG_SONG_COPY_TABLE
  /* Between symbol address _scopytable and _ecopytable, there are
   * array of triplets, each of which specify:
   *    offset 0: Start LMA of a section
   *    offset 4: Start VMA of a section
   *    offset 8: End VMA of a section
   */

  for (table = &_scopytable; table < &_ecopytable; table += 3)
    {
      for (src = (uint32_t *)table[0], dest = (uint32_t *)table[1];
          dest < (uint32_t *)table[2]; )
        {
          *dest++ = *src++;
        }
    }
#endif

#ifdef CONFIG_SONG_ZERO_TABLE
  /* Between symbol address _szerotable and _ezerotable is an array
   * of pairs, and each pair specifies:
   *    offset 0: start address of the section to clear
   *    offset 4: end address (exclusive) of the section to clear
   */
  for (table = &_szerotable; table < &_ezerotable; table += 2)
    {
      for (dest = (uint32_t *)table[0]; dest < (uint32_t *)table[1]; )
        {
          *dest++ = 0;
        }
    }
#endif
}

#ifdef CONFIG_BUILD_PROTECTED
static void init_userspace(void)
{
  const uint32_t *src;
  uint32_t *dest;
  uint32_t *end;

  /* Initialize all of user-space .data */

  DEBUGASSERT(USERSPACE->us_datasource != 0 &&
              USERSPACE->us_datastart != 0 && USERSPACE->us_dataend != 0 &&
              USERSPACE->us_datastart <= USERSPACE->us_dataend);

  src  = (uint32_t *)USERSPACE->us_datasource;
  dest = (uint32_t *)USERSPACE->us_datastart;
  end  = (uint32_t *)USERSPACE->us_dataend;

  if (src != dest)
    {
      while (dest < end)
        {
          *dest++ = *src++;
        }
    }

  /* Clear all of user-space .bss */

  DEBUGASSERT(USERSPACE->us_bssstart != 0 && USERSPACE->us_bssend != 0 &&
              USERSPACE->us_bssstart <= USERSPACE->us_bssend);

  dest = (uint32_t *)USERSPACE->us_bssstart;
  end  = (uint32_t *)USERSPACE->us_bssend;

  while (dest < end)
    {
      *dest++ = 0;
    }
}
#else
#  define init_userspace()
#endif

/****************************************************************************
 * Name: color_start
 *
 * Description:
 *   Set the IDLE stack to the coloration value and jump to os_start
 *
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static void color_start(FAR void *stack, unsigned int size)
{
  /* Set the IDLE stack to the stack coloration value then jump to
   * os_start().  We take extreme care here because were currently
   * executing on this stack.
   *
   * We want to avoid sneak stack access generated by the compiler.
   */

  __asm__ __volatile__
  (
    "\tmovs r1, r1, lsr #2\n"   /* R1 = nwords = size >> 2 */
    "\tbeq  2f\n"               /* (should not happen) */

    "\tbic  r0, r0, #3\n"       /* R0 = Aligned stackptr */
    "\tmovw r2, #0xbeef\n"      /* R2 = STACK_COLOR = 0xdeadbeef */
    "\tmovt r2, #0xdead\n"

    "1:\n"                      /* Top of the loop */
    "\tsub  r1, r1, #1\n"       /* R1 nwords-- */
    "\tcmp  r1, #0\n"           /* Check (nwords == 0) */
    "\tstr  r2, [r0], #4\n"     /* Save stack color word, increment stackptr */
    "\tbne  1b\n"               /* Bottom of the loop */

    "2:\n"
    "\tmov  r14, #0\n"          /* LR = return address (none) */
    "\tb    os_start\n"         /* Branch to os_start */
  );
}
#else
#  define color_start(stack, size) os_start()
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: __start
 *
 * Description:
 *   This is the reset entry point.
 *
 ****************************************************************************/

void weak_function up_earlyinitialize(void)
{
}

void weak_function board_earlyinitialize(void)
{
}

#ifdef CONFIG_ARMV7M_STACKCHECK
/* we need to get r10 set before we can allow instrumentation calls */

void __start(void) __attribute__ ((no_instrument_function));
#endif

void __start(void)
{
#ifdef CONFIG_ARMV7M_STACKCHECK
  /* Set the stack limit before we attempt to call any functions */

  __asm__ volatile ("sub r10, sp, %0" : : "r" (CONFIG_IDLETHREAD_STACKSIZE - 64) : );
#endif

  init_kernelspace();

  /* Will be enabled again by up_irqinitialize */
  up_irq_disable();

  up_earlyserialinit();
  up_earlyinitialize();
  board_earlyinitialize();

  up_enable_icache();
  up_enable_dcache();
  up_fpuinitialize();

  init_userspace();
  up_mpuinitialize();

  color_start((FAR void *)&_ebss, CONFIG_IDLETHREAD_STACKSIZE);
}
