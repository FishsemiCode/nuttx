/****************************************************************************
 * arch/ceva/src/tl4/up_cache.c
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

#include <nuttx/cache.h>
#include <nuttx/irq.h>

#include <string.h>

#include "cpm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PMSS_CIR                                0x0004
#define PMSS_PCR                                0x0008
#define DMSS_CHRR1                              0x0300
#define DMSS_CCR                                0x030c
#define DMSS_CCOSAR                             0x0310
#define DMSS_CCOSLR                             0x0314
#define DMSS_CCOCR                              0x0318

#define PMSS_CIR_INV                            0x00000001

#define PMSS_PCR_CAC_EN                         0x00000001
#define PMSS_PCR_CAC_LCK                        0x00000002
#define PMSS_PCR_PREF_EN                        0x00000004

#define DMSS_CHRR1_L1DCS_0KW                    0x00000000
#define DMSS_CHRR1_L1DCS_8KW                    0x00020000
#define DMSS_CHRR1_L1DCS_16KW                   0x00030000
#define DMSS_CHRR1_L1DCS_32KW                   0x00040000
#define DMSS_CHRR1_L1DCS_MASK                   0x00070000

#define DMSS_CCR_L1DCE                          0x00000002
#define DMSS_CCR_L1W_CB                         0x00000020
#define DMSS_CCR_L1DC                           0x00000040
#define DMSS_CCR_MOM_SO                         0x00000080

#define DMSS_CCOCR_L1DCO                        0x00000002
#define DMSS_CCOCR_OT_LOCK                      0x00000010
#define DMSS_CCOCR_OT_UNLOCK                    0x00000018
#define DMSS_CCOCR_OT_INVALIDATE                0x00000020
#define DMSS_CCOCR_OT_FLUSH                     0x00000028
#define DMSS_CCOCR_OT_FLUSH_INVALIDATE          0x00000030
#define DMSS_CCOCR_OT_MASK                      0x00000038
#define DMSS_CCOCR_OS_ENTIRE                    0x00000080
#define DMSS_CCOCR_NOBPL_SHIFT                  16
#define DMSS_CCOCR_NOBPL_MASK                   0xffff0000

#define DMSS_CACHE_BLOCK_SIZE                   B2C(64)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_CEVA_ICACHE
static void invalidate_disabled_icache_all(void)
{
  /* Start the invalidation */
  putcpm(PMSS_CIR, PMSS_CIR_INV);
  while (getcpm(PMSS_CIR) & PMSS_CIR_INV)
    {
      /* Loop until the operation finish */;
    }
}

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
  /* Invalidate the entire icache */
  invalidate_disabled_icache_all();

  /* Enable icache/prefetch and disable lock */
  modifycpm(PMSS_PCR, PMSS_PCR_CAC_LCK, PMSS_PCR_CAC_EN | PMSS_PCR_PREF_EN);
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
  /* Disable icache */
  modifycpm(PMSS_PCR, PMSS_PCR_CAC_EN, 0);

  /* Invalidate the entire icache */
  invalidate_disabled_icache_all();
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
  /* Skip itcm since it never put into icache */
  if (end > B2C(CONFIG_ARCH_ITCM_SIZE))
    {
      /* Just can invalidate the entire icache */
      up_invalidate_icache_all();
    }
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

void up_invalidate_icache_all(void)
{
  irqstate_t flags;

  /* Disable irq and icache */
  flags = up_irq_save();
  modifycpm(PMSS_PCR, PMSS_PCR_CAC_EN, 0);

  /* Invalidate icache */
  invalidate_disabled_icache_all();

  /* Restore icache and irq */
  modifycpm(PMSS_PCR, 0, PMSS_PCR_CAC_EN);
  up_irq_restore(flags);
}
#endif

#ifdef CONFIG_CEVA_DCACHE
static void maintain_dcache_all(uint32_t op)
{
  irqstate_t flags;

  /* Disable irq */
  flags = up_irq_save();

  /* Start the operation on the entire cache */
  putcpm(DMSS_CCOCR, DMSS_CCOCR_L1DCO | op | DMSS_CCOCR_OS_ENTIRE);

  while (getcpm(DMSS_CCOCR) & DMSS_CCOCR_L1DCO)
    {
      /* Loop until the operation finish */;
    }

  /* Restore irq */
  up_irq_restore(flags);
}

static void maintain_dcache(uint32_t op, uintptr_t start, uintptr_t end)
{
  static size_t op_maxblocks;

  /* Initialize op_maxblocks if not yet */
  if (op_maxblocks == 0)
    {
      switch (getcpm(DMSS_CHRR1) & DMSS_CHRR1_L1DCS_MASK)
        {
        case DMSS_CHRR1_L1DCS_8KW:
          op_maxblocks = 512;
          break;
        case DMSS_CHRR1_L1DCS_16KW:
          op_maxblocks = 1024;
          break;
        case DMSS_CHRR1_L1DCS_32KW:
          op_maxblocks = 2048;
          break;
        default:
          op_maxblocks = 1;
          break;
        }
    }

  /* Align the address to the cache block boundary */
  start &= ~(DMSS_CACHE_BLOCK_SIZE - 1);
  end   +=  (DMSS_CACHE_BLOCK_SIZE - 1);
  end   &= ~(DMSS_CACHE_BLOCK_SIZE - 1);

  /* Skip dtcm since it never put into dcache */
  if (end > B2C(CONFIG_ARCH_DTCM_SIZE))
    {
      if (start < B2C(CONFIG_ARCH_DTCM_SIZE))
        {
          start = B2C(CONFIG_ARCH_DTCM_SIZE);
        }

      while (start < end)
        {
          irqstate_t flags;
          size_t op_blocks;

          /* Get the max blocks we can do in one iteration */
          op_blocks = (end - start) / DMSS_CACHE_BLOCK_SIZE;
          if (op_blocks > op_maxblocks)
            {
              op_blocks = op_maxblocks;
            }

          /* Disable irq */
          flags = up_irq_save();

          /* Set the cache address */
          putcpm(DMSS_CCOSAR, C2B(start));
          putcpm(DMSS_CCOSLR, 1); /* 1D operation */

          /* Start the cache operation */
          putcpm(DMSS_CCOCR, /* Address based operation */
            DMSS_CCOCR_L1DCO | op | (op_blocks << DMSS_CCOCR_NOBPL_SHIFT));

          while (getcpm(DMSS_CCOCR) & DMSS_CCOCR_L1DCO)
            {
              /* Loop until the operation finish */;
            }

          /* Restore irq */
          up_irq_restore(flags);

          /* Prepare the next loop */
          start += op_blocks * DMSS_CACHE_BLOCK_SIZE;
        }
    }
}

 /****************************************************************************
 * Name: up_enable_dcache
 *
 * Description:
 *   Enable the D-Cache
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

void up_enable_dcache(void)
{
  /* Invalidate the entire dcache */
  maintain_dcache_all(DMSS_CCOCR_OT_INVALIDATE);

  /* Enable dcache and set the default region:
   * no cacheable and strong order
   */
  modifycpm(DMSS_CCR, DMSS_CCR_L1DC, DMSS_CCR_L1DCE | DMSS_CCR_MOM_SO);
}

/****************************************************************************
 * Name: up_disable_dcache
 *
 * Description:
 *   Disable the D-Cache
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_disable_dcache(void)
{
  /* Disable dcache */
  modifycpm(DMSS_CCR, DMSS_CCR_L1DCE, 0);

  /* Flush and invalidate the entire dcache */
  maintain_dcache_all(DMSS_CCOCR_OT_FLUSH_INVALIDATE);
}

/****************************************************************************
 * Name: up_invalidate_dcache
 *
 * Description:
 *   Invalidate the data cache within the specified region; we will be
 *   performing a DMA operation in this region and we want to purge old data
 *   in the cache.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_invalidate_dcache(uintptr_t start, uintptr_t end)
{
  if (start & (DMSS_CACHE_BLOCK_SIZE - 1))
    {
      maintain_dcache(DMSS_CCOCR_OT_FLUSH, start, start);
    }
  if (end & (DMSS_CACHE_BLOCK_SIZE - 1))
    {
      maintain_dcache(DMSS_CCOCR_OT_FLUSH, end, end);
    }

  maintain_dcache(DMSS_CCOCR_OT_INVALIDATE, start, end);
}

/****************************************************************************
 * Name: up_invalidate_dcache_all
 *
 * Description:
 *   Invalidate the entire contents of D cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_invalidate_dcache_all(void)
{
  maintain_dcache_all(DMSS_CCOCR_OT_INVALIDATE);
}

/****************************************************************************
 * Name: up_clean_dcache
 *
 * Description:
 *   Clean the data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_clean_dcache(uintptr_t start, uintptr_t end)
{
  maintain_dcache(DMSS_CCOCR_OT_FLUSH, start, end);
}

/****************************************************************************
 * Name: up_clean_dcache_all
 *
 * Description:
 *   Clean the entire data cache within the specified region by flushing the
 *   contents of the data cache to memory.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_clean_dcache_all(void)
{
  maintain_dcache_all(DMSS_CCOCR_OT_FLUSH);
}

/****************************************************************************
 * Name: up_flush_dcache
 *
 * Description:
 *   Flush the data cache within the specified region by cleaning and
 *   invalidating the D cache.
 *
 * Input Parameters:
 *   start - virtual start address of region
 *   end   - virtual end address of region + 1
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_flush_dcache(uintptr_t start, uintptr_t end)
{
  maintain_dcache(DMSS_CCOCR_OT_FLUSH_INVALIDATE, start, end);
}

/****************************************************************************
 * Name: up_flush_dcache_all
 *
 * Description:
 *   Flush the entire data cache by cleaning and invalidating the D cache.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_flush_dcache_all(void)
{
  maintain_dcache_all(DMSS_CCOCR_OT_FLUSH_INVALIDATE);
}
#endif

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

#ifdef CONFIG_CEVA_ICACHE
void up_coherent_dcache(uintptr_t addr, size_t len)
{
  up_clean_dcache(addr, addr + len);
  up_invalidate_icache(addr, addr + len);
}
#endif
