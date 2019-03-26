/****************************************************************************
 * arch/csky/src/song/song_start.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/init.h>
#include <nuttx/arch.h>
#include <arch/board/board.h>

#include <string.h>

#include "chip.h"
#include "up_internal.h"

/****************************************************************************
 * Private Types
 ****************************************************************************/

#ifdef CONFIG_SONG_COPY_TABLE
/* Between symbol address _scopytable and _ecopytable, there are
 * array of triplets, each of which specify:
 *    offset 0: Start LMA of a section
 *    offset 4: Start VMA of a section
 *    offset 8: End VMA of a section
 */

begin_packed_struct struct copytable_s
{
  void *src;
  void *dest;
  void *end;
} end_packed_struct;
#endif

#ifdef CONFIG_SONG_ZERO_TABLE
/* Between symbol address _szerotable and _ezerotable is an array
 * of pairs, and each pair specifies:
 *    offset 0: start address of the section to clear
 *    offset 4: end address (exclusive) of the section to clear
 */

begin_packed_struct struct zerotable_s
{
  void *dest;
  void *end;
} end_packed_struct;
#endif

/****************************************************************************
 * Private Function prototypes
 ****************************************************************************/

static void init_kernelspace(void);

#ifdef CONFIG_SONG_COPY_TABLE
extern struct copytable_s _scopytable;
extern struct copytable_s _ecopytable;
#endif

#ifdef CONFIG_SONG_ZERO_TABLE
extern struct zerotable_s _szerotable;
extern struct zerotable_s _ezerotable;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static void init_kernelspace(void)
{
#ifdef CONFIG_SONG_COPY_TABLE
  struct copytable_s *copytable;
#endif
#ifdef CONFIG_SONG_ZERO_TABLE
  struct zerotable_s *zerotable;
#endif

  /* Copy any necessary code sections from FLASH to RAM.  The correct
   * destination in SRAM is given by _sramfuncs and _eramfuncs.  The
   * temporary location is in flash after the data initialization code
   * at _framfuncs
   */

#ifdef CONFIG_ARCH_RAMFUNCS
  memcpy(&_sramfuncs, &_framfuncs, (uintptr_t)&_eramfuncs - (uintptr_t)&_sramfuncs);
#endif

  /* Move the initialized data section from his temporary holding spot in
   * FLASH into the correct place in SRAM.  The correct place in SRAM is
   * give by _sdata and _edata.  The temporary location is in FLASH at the
   * end of all of the other read-only data (.text, .rodata) at _eronly.
   */

  if (&_eronly != &_sdata)
    {
      memcpy(&_sdata, &_eronly, (uintptr_t)&_edata - (uintptr_t)&_sdata);
    }

  /* Clear .bss.  We'll do this inline (vs. calling memset) just to be
   * certain that there are no issues with the state of global variables.
   */

  memset(&_sbss, 0, (uintptr_t)&_ebss - (uintptr_t)&_sbss);

#ifdef CONFIG_SONG_COPY_TABLE
  for (copytable = &_scopytable; copytable < &_ecopytable; copytable++)
    {
      memcpy(copytable->dest, copytable->src, copytable->end - copytable->dest);
    }
#endif

#ifdef CONFIG_SONG_ZERO_TABLE
  for (zerotable = &_szerotable; zerotable < &_ezerotable; zerotable++)
    {
      memset(zerotable->dest, 0, zerotable->end - zerotable->dest);
    }
#endif
}

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

void up_start(void)
{
  up_enable_icache();
  up_enable_dcache();

  init_kernelspace();

  up_earlyserialinit();
  up_earlyinitialize();
  board_earlyinitialize();

  nx_start();
}
