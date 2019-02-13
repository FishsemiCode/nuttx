/****************************************************************************
 * arch/arm/src/song/arm_mpu.c
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

#ifdef CONFIG_ARM_MPU

#include <assert.h>

#include <nuttx/power/pm.h>
#include <nuttx/userspace.h>

#include "chip.h"
#include "mpu.h"
#include "up_internal.h"

/****************************************************************************
 * Private Function Declarations
 ****************************************************************************/

static size_t up_mpu_round_size(uintptr_t start, uintptr_t end);

#ifdef CONFIG_PM
static void up_mpu_pm_notify(struct pm_callback_s *cb, int domain,
                           enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PM
static struct pm_callback_s g_up_mpu_pm_cb =
{
  .notify  = up_mpu_pm_notify,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static size_t up_mpu_round_size(uintptr_t start, uintptr_t end)
{
  size_t size = end - start;
  size_t newsize = 1 << mpu_log2regionceil(size);
  if ((start & (newsize - 1)) + size > newsize)
    {
      newsize *= 2;
    }
  return newsize;
}

#ifdef CONFIG_PM
static void up_mpu_pm_notify(struct pm_callback_s *cb, int domain,
                           enum pm_state_e pmstate)
{
  static uint32_t mpu_regsave[2 * CONFIG_ARM_MPU_NREGIONS + 1];
  int region;
  int i;

  switch (pmstate)
    {
      case PM_STANDBY:
      case PM_SLEEP:
        /* Following MPU registers are saved during standby:
          *  RBAR & RASR[8]
          *  CONTROL
          */

        for (region = 0, i = 0; region < CONFIG_ARM_MPU_NREGIONS; region++)
          {
            putreg32(region, MPU_RNR);
            mpu_regsave[i++] = getreg32(MPU_RBAR);
            mpu_regsave[i++] = getreg32(MPU_RASR);
          }
        mpu_regsave[i] = getreg32(MPU_CTRL);
        break;

      case PM_RESTORE:
        if (pm_querystate(PM_IDLE_DOMAIN) >= PM_STANDBY)
          {
            for (region = 0, i = 0; region < CONFIG_ARM_MPU_NREGIONS; region++)
              {
                putreg32(region, MPU_RNR);
                putreg32(mpu_regsave[i++], MPU_RBAR);
                putreg32(mpu_regsave[i++], MPU_RASR);
              }
            putreg32(mpu_regsave[i], MPU_CTRL);
          }
        break;

      default:
        break;
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_mpuinitialize(void)
{
  /* Show MPU information */

  mpu_showtype();

  /* Configure kernel flash and SRAM space */

  DEBUGASSERT((uintptr_t)_END_TEXT >= (uintptr_t)_START_TEXT);
  mpu_priv_flash((uintptr_t)_START_TEXT,
    up_mpu_round_size((uintptr_t)_START_TEXT, (uintptr_t)_END_TEXT));

#ifdef CONFIG_ARCH_RAMFUNCS
  DEBUGASSERT((uintptr_t)&_eramfuncs >= (uintptr_t)&_sramfuncs);
  mpu_priv_flash((uintptr_t)&_sramfuncs,
    up_mpu_round_size((uintptr_t)&_sramfuncs, (uintptr_t)&_eramfuncs));
#endif

  DEBUGASSERT((uintptr_t)_END_BSS >= (uintptr_t)_START_DATA);
  mpu_priv_intsram((uintptr_t)_START_DATA,
    up_mpu_round_size((uintptr_t)_START_DATA,
      (uintptr_t)_END_BSS + CONFIG_IDLETHREAD_STACKSIZE));

#ifdef CONFIG_BUILD_PROTECTED
  /* Configure user flash and SRAM space */

  DEBUGASSERT(USERSPACE->us_textend >= USERSPACE->us_textstart);
  mpu_user_flash(USERSPACE->us_textstart,
    up_mpu_round_size(USERSPACE->us_textstart, USERSPACE->us_textend));

  DEBUGASSERT(USERSPACE->us_bssend >= USERSPACE->us_datastart);
  mpu_user_intsram(USERSPACE->us_datastart,
    up_mpu_round_size(USERSPACE->us_datastart, USERSPACE->us_bssend));
#endif

#ifdef CONFIG_PM
  pm_register(&g_up_mpu_pm_cb);
#endif

  /* Then enable the MPU */

  mpu_control(true, false, true);
}

void up_mpu_user_heap(uintptr_t start, size_t size)
{
  mpu_user_intsram(start, up_mpu_round_size(start, start + size));
}

void up_mpu_priv_heap(uintptr_t start, size_t size)
{
  mpu_priv_intsram(start, up_mpu_round_size(start, start + size));
}
#endif
