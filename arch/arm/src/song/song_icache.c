/****************************************************************************
 * arch/arm/src/song/song_icache.c
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

#ifdef CONFIG_SONG_ICACHE

#include <nuttx/arch.h>

#include "chip.h"
#include "up_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_ICACHE_CTL           (CONFIG_SONG_ICACHE_BASE + 0x00)
#define SONG_ICACHE_PRO_CTRL      (CONFIG_SONG_ICACHE_BASE + 0x04)
#define SONG_ICACHE_HCNT          (CONFIG_SONG_ICACHE_BASE + 0x08)
#define SONG_ICACHE_MCNT          (CONFIG_SONG_ICACHE_BASE + 0x0c)

#define SONG_ICACHE_LP_EN         (1 << 3)
#define SONG_ICACHE_PREFET        (1 << 2)
#define SONG_ICACHE_EN            (1 << 1)
#define SONG_ICACHE_FLUSH         (1 << 0)

#define NOP()                     __asm__ __volatile__ ("nop")

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_enable_icache
 *
 * Description:
 *   Enable the I-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

void up_enable_icache(void)
{
  up_disable_icache();

  modifyreg32(SONG_ICACHE_CTL, 0, SONG_ICACHE_LP_EN | SONG_ICACHE_EN);
  NOP();
}

/****************************************************************************
 * Name: up_disable_icache
 *
 * Description:
 *   Disable the I-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_disable_icache(void)
{
  modifyreg32(SONG_ICACHE_CTL, SONG_ICACHE_EN, 0);
  NOP();

  modifyreg32(SONG_ICACHE_CTL, 0, SONG_ICACHE_FLUSH);
  while (getreg32(SONG_ICACHE_CTL) & SONG_ICACHE_FLUSH)
    {
      /* Wait until FLUSH bit get clear */;
    }
}

/****************************************************************************
 * Name: up_invalidate_icache
 *
 * Description:
 *   Invalidate the instruction cache within the specified region.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_invalidate_icache(uintptr_t start, uintptr_t end)
{
  /* Just can invalidate the entire cache */

  up_invalidate_icache_all();
}

/****************************************************************************
 * Name: up_invalidate_icache_all
 *
 * Description:
 *   Invalidate the entire contents of I cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

__ramfunc__ void up_invalidate_icache_all(void)
{
  irqstate_t flags;

  flags = spin_lock_irqsave();

  modifyreg32(SONG_ICACHE_CTL, 0, SONG_ICACHE_FLUSH);
  while (getreg32(SONG_ICACHE_CTL) & SONG_ICACHE_FLUSH)
    {
      /* Wait until FLUSH bit get clear */;
    }

  spin_unlock_irqrestore(flags);
}

/****************************************************************************
 * Name: up_coherent_dcache
 *
 * Description:
 *   Ensure that the I and D caches are coherent within specified region
 *   by cleaning the D cache (i.e., flushing the D cache contents to memory
 *   and invalidating the I cache. This is typically used when code has been
 *   written to a memory region, and will be executed.
 *
 * Input Parameters:
 *   addr - virtual start address of region
 *   len  - Size of the address region in bytes
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_coherent_dcache(uintptr_t addr, size_t len)
{
  /* Invalidate instruction cache is enough */

  up_invalidate_icache(addr, addr + len);
}

#endif /* CONFIG_SONG_ICACHE */
