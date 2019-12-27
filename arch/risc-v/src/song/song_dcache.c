/****************************************************************************
 * arch/risc-v/src/song/song_dcache.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
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

#ifdef CONFIG_ARCH_DCACHE

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifndef CONFIG_SONG_DCACHE_LINEBYTES
#error "DCACHE_LINEBYTES must be configured"
#endif

#define DCACHE_LINEMASK     (CONFIG_SONG_DCACHE_LINEBYTES - 1)
#define DCACHE_ALIGNUP(a)   (((a) + DCACHE_LINEMASK) & ~DCACHE_LINEMASK)
#define DCACHE_ALIGNDOWN(a) ((a) & ~DCACHE_LINEMASK)

#define STR(x) XSTR(x)
#define XSTR(x) #x

#define CFLUSH_D_L1_INST(rs1)       \
        0x73                   |    \
        (0x0     << (7))       |    \
        (0x0     << (7+5))     |    \
        (0x0     << (7+5+2))   |    \
        (rs1     << (7+5+3))   |    \
        (0x0     << (7+5+3+5)) |    \
        (0x7e    << (7+5+3+5+5))

#define CFLUSH_D_L1(rs1)                                \
{                                                       \
  register uint64_t rs1_ asm ("x11") = (uint64_t) rs1;  \
  asm volatile (                                        \
            ".word " STR(CFLUSH_D_L1_INST(11)) "\n\t"   \
            :: [_rs1] "r" (rs1_));                      \
}

#define CFLUSH_D_ALL(rs1)                               \
{                                                       \
    register uint32_t rs1_ asm ("x0") = (uint32_t) rs1; \
    asm volatile (                                      \
            ".word " STR(CFLUSH_D_L1_INST(11)) "\n\t"   \
            ::[_rs1] "r" (rs1_));                       \
}

#define CFLUSH_FENCE()                                  \
{                                                       \
    asm volatile (                                      \
            "fence \n\t"                                \
            ::);                                        \
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

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
  start = DCACHE_ALIGNDOWN(start);
  end   = DCACHE_ALIGNUP(end);

  for (; start < end; start += CONFIG_SONG_DCACHE_LINEBYTES)
    {
      CFLUSH_D_L1(start);
    }

  CFLUSH_FENCE();
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
  CFLUSH_D_ALL(0);
  CFLUSH_FENCE();
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
  up_invalidate_dcache(start, end);
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
  up_invalidate_dcache_all();
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
  up_invalidate_dcache(start, end);
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
  up_invalidate_dcache_all();
}

#endif /* CONFIG_ARCH_DCACHE */
