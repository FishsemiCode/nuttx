/*****************************************************************************
 * arch/ceva/src/tl4/up_mpu.c
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

#include <nuttx/config.h>

#include <assert.h>
#include <string.h>

#include "cpm.h"
#include "mpu.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DMSS_CDACR0L                            0x036c
#define DMSS_CDACR0H                            0x0370

#define DMSS_CDACRL_V                           0x00000001
#define DMSS_CDACRL_BA_SHIFT                    1
#define DMSS_CDACRL_BA_MASK                     0x001ffffe
#define DMSS_CDACRL_PS_16KB                     0x00000000
#define DMSS_CDACRL_PS_32KB                     0x00200000
#define DMSS_CDACRL_PS_64KB                     0x00400000
#define DMSS_CDACRL_PS_512KB                    0x00600000
#define DMSS_CDACRL_PS_1MB                      0x00a00000
#define DMSS_CDACRL_PS_2MB                      0x01c00000
#define DMSS_CDACRL_PS_4MB                      0x01e00000
#define DMSS_CDACRL_PS_MASK                     0x01e00000
#define DMSS_CDACRL_L1DC                        0x02000000
#define DMSS_CDACRL_L1W_CB                      0x04000000
#define DMSS_CDACRL_MOM_SO                      0x80000000

#define DMSS_CDACRH_DPRAW                       0x00000001

#define DMSS_NR_CDACR                           8

#ifdef CONFIG_ARCH_MPU

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t attr_from_cdacrl(uint32_t cdacrl)
{
  return cdacrl & (DMSS_CDACRL_MOM_SO | DMSS_CDACRL_L1W_CB | DMSS_CDACRL_L1DC);
}

static inline uintptr_t base_from_cdacrl(uint32_t cdacrl)
{
  return B2C(((cdacrl & DMSS_CDACRL_BA_MASK) >> DMSS_CDACRL_BA_SHIFT) << 12);
}

static inline size_t size_from_cdacrl(uint32_t cdacrl)
{
  switch (cdacrl & DMSS_CDACRL_PS_MASK)
    {
    case DMSS_CDACRL_PS_16KB:
      return B2C(16 * 1024);
    case DMSS_CDACRL_PS_32KB:
      return B2C(32 * 1024);
    case DMSS_CDACRL_PS_64KB:
      return B2C(64 * 1024);
    case DMSS_CDACRL_PS_512KB:
      return B2C(512 * 1024);
    case DMSS_CDACRL_PS_1MB:
      return B2C(1024 * 1024);
    case DMSS_CDACRL_PS_2MB:
      return B2C(2 * 1024 * 1024);
    case DMSS_CDACRL_PS_4MB:
      return B2C(4 * 1024 * 1024);
    default:
      return 0;
    }
}

static inline uint32_t range_to_cdacrl(uintptr_t base, size_t size)
{
  uint32_t psflag;

  base = C2B(base);
  size = C2B(size);

  if (size <= 16 * 1024)
    {
      base  &= ~(16 * 1024 - 1);
      psflag = DMSS_CDACRL_PS_16KB;
    }
  else if (size <= 32 * 1024)
    {
      base  &= ~(32 * 1024 - 1);
      psflag = DMSS_CDACRL_PS_32KB;
    }
  else if (size <= 64 * 1024)
    {
      base  &= ~(64 * 1024 - 1);
      psflag = DMSS_CDACRL_PS_64KB;
    }
  else if (size <= 512 * 1024)
    {
      base  &= ~(512 * 1024 - 1);
      psflag = DMSS_CDACRL_PS_512KB;
    }
  else if (size <= 1024 * 1024)
    {
      base  &= ~(1024 * 1024 - 1);
      psflag = DMSS_CDACRL_PS_1MB;
    }
  else if (size <= 2 * 1024 * 1024)
    {
      base  &= ~(2 * 1024 * 1024 - 1);
      psflag = DMSS_CDACRL_PS_2MB;
    }
  else
    {
      DEBUGASSERT(size <= 4 * 1024 * 1024);
      base  &= ~(4 * 1024 * 1024 - 1);
      psflag = DMSS_CDACRL_PS_4MB;
    }

  return psflag | ((base >> 12) << DMSS_CDACRL_BA_SHIFT);
}

static void mpu_apply_region(uintptr_t base, size_t size, uint32_t attr)
{
  int i, j;

  if (base + size <= B2C(CONFIG_ARCH_DTCM_SIZE))
    {
      /* Don't need enable dcache since dtcm is fast as dcache */
      return;
    }

  for (i = 0, j = -1; i < DMSS_NR_CDACR; i++)
    {
      uint32_t cdacrl = getcpm(DMSS_CDACR0L + 8 * i);
      if (!(cdacrl & DMSS_CDACRL_V))
        {
          /* Remember the first unused region */
          if (j == -1)
            {
              j = i;
            }
        }
      else if (attr == attr_from_cdacrl(cdacrl))
        {
          uintptr_t nbase = base_from_cdacrl(cdacrl);
          size_t nsize = size_from_cdacrl(cdacrl);
          uintptr_t nend = nbase + nsize;
          uintptr_t end = base + size;

          /* Do these two regions overlap? */
          if (end >= nbase || nend >= base)
            {
              j = i; /* Extend the current region */

              if (nbase < base)
                {
                  base = nbase;
                }
              if (nend > end)
                {
                  end = nend;
                }
              size = end - base;
              break;
            }
        }
    }

  DEBUGASSERT(j != -1);
  putcpm(DMSS_CDACR0L + 8 * j,
    attr | range_to_cdacrl(base, size) | DMSS_CDACRL_V);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: mpu_control
 *
 * Description:
 *   Configure and enable (or disable) the MPU
 *
 ****************************************************************************/

void mpu_control(bool enable)
{
  /* No special action here */
}

/****************************************************************************
 * Name: mpu_user_code
 *
 * Description:
 *   Configure a region for user code
 *
 ****************************************************************************/

void mpu_user_code(const void *base, size_t size)
{
  /* The user mode is same as the privilege mode */
  mpu_priv_code(base, size);
}

/****************************************************************************
 * Name: mpu_priv_code
 *
 * Description:
 *   Configure a region for privileged code
 *
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

void mpu_priv_code(const void *base, size_t size)
{
  /* Code is always cacheable regardless the location
   * and icache is already enabled in up_cache.c.
   */
}

/****************************************************************************
 * Name: mpu_user_data
 *
 * Description:
 *   Configure a region as user data
 *
 ****************************************************************************/

void mpu_user_data(void *base, size_t size)
{
  /* The user mode is same as the privilege mode */
  mpu_priv_data(base, size);
}

/****************************************************************************
 * Name: mpu_priv_data
 *
 * Description:
 *   Configure a region as privileged data
 *
 * Caution:
 *   The writable global variables aren't initialized yet.
 *
 ****************************************************************************/

void mpu_priv_data(void *base, size_t size)
{
  mpu_apply_region((uintptr_t)base, size,
    DMSS_CDACRL_L1DC | DMSS_CDACRL_L1W_CB);
}

/****************************************************************************
 * Name: mpu_peripheral
 *
 * Description:
 *   Configure a region as privileged peripheral address space
 *
 ****************************************************************************/

void mpu_peripheral(void *base, size_t size)
{
  mpu_apply_region((uintptr_t)base, size, 0);
}

/****************************************************************************
 * Name: mpu_stronglyordered
 *
 * Description:
 *   Configure a region for privileged, strongly ordered memory
 *
 ****************************************************************************/

void mpu_stronglyordered(void *base, size_t size)
{
  mpu_apply_region((uintptr_t)base, size, DMSS_CDACRL_MOM_SO);
}

#endif
