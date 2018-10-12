/****************************************************************************
 * arch/arm/src/song/u1_sp_regulator.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: zhuyanlin <zhuyanlin@pinecone.net>
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

#include <nuttx/power/consumer.h>
#include <nuttx/power/regulator.h>
#include <nuttx/power/pm.h>

#include "chip.h"

#if defined(CONFIG_ARCH_CHIP_U1_SP) && defined(CONFIG_SONG_PMIC_APB)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_PM
struct pm_regulator_s
{
  struct pm_callback_s cb;
  struct regulator *reg;
  uint32_t voltage[2];
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PM
static struct pm_regulator_s g_pm_regulator;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_PM
static void regulator_pm_notify(struct pm_callback_s *cb, int domain,
                           enum pm_state_e pmstate)
{
  FAR struct pm_regulator_s *pm = (FAR struct pm_regulator_s *)cb;

  switch (pmstate)
    {
      case PM_RESTORE:
        if (pm->reg)
          {
            regulator_set_voltage(pm->reg, pm->voltage[1], pm->voltage[1]);
          }
        break;

      case PM_SLEEP:
        if (pm->reg)
          {
            regulator_set_voltage(pm->reg, pm->voltage[0], pm->voltage[0]);
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

void up_regulator_initialize(void)
{
  spmu_regulator_apb_initialize(0xb2010000, 0xb0180000);

#ifdef CONFIG_PM
  g_pm_regulator.reg = regulator_get(NULL, "ldo0");
  if (g_pm_regulator.reg)
    {
      g_pm_regulator.voltage[0] = 625000;
      g_pm_regulator.voltage[1] = 900000;
      g_pm_regulator.cb.notify = regulator_pm_notify;
      pm_register(&g_pm_regulator.cb);
    }
#endif
}

#endif /* CONFIG_ARCH_CHIP_U1_SP */
