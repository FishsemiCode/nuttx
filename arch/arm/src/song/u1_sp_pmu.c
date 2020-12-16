/****************************************************************************
 * arch/arm/src/song/u1_sp_pmu.c
 *
 *   Copyright (C) 2019 Fishsemi Inc. All rights reserved.
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

#if (defined(CONFIG_ARCH_CHIP_U1_SP) || defined(CONFIG_ARCH_CHIP_U1_RECOVERY)) \
    && defined(CONFIG_SONG_PMIC_APB)

#include <stdint.h>
#include <nuttx/power/regulator.h>

#include "up_arch.h"
#include "u1_common.h"
#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

enum {
  SPMU_REG_BUCK0 = 0,
  SPMU_REG_BUCK1,
  SPMU_REG_LDO0,
  SPMU_REG_LDO1,
  SPMU_REG_LDO2,
  SPMU_REG_LDO3,
  SPMU_REG_LDO4,
  SPMU_REG_LDO5,
  SPMU_REG_LDO6,
  /* Total number of regulators */
  SPMU_NUM_REGS,
};

static const struct regulator_desc spmu_regulator_desc[SPMU_NUM_REGS] = {
  [SPMU_REG_BUCK0] = {
      .name = "buck0",
      .id = SPMU_REG_BUCK0,
      .vsel_reg = 0x20,
      .vsel_mask = 0x3f00,
      .enable_reg = 0x20,
      .enable_mask = 0x1,
      .enable_time = 500,
      .ramp_delay = 20000,
      .uV_step = 0,
      .min_uV = 0,
      .max_uV = UINT_MAX,
      .boot_on = true,
  },
  [SPMU_REG_BUCK1] = {
      .name = "buck1",
      .id = SPMU_REG_BUCK1,
      .vsel_reg = 0x24,
      .vsel_mask = 0x3f00,
      .enable_reg = 0x24,
      .enable_mask = 0x1,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 12500,
      .min_uV = 650000,
      .max_uV = 1437500,
      .boot_on = true,
  },
  [SPMU_REG_LDO0] = {
      .name = "ldo0",
      .id = SPMU_REG_LDO0,
      .vsel_reg = 0x28,
      .vsel_mask = 0x3f00,
      .enable_reg = 0x28,
      .enable_mask = 0x1,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 25000,
      .min_uV = 600000,
      .max_uV = 1375000,
      .boot_on = true,
  },
  [SPMU_REG_LDO1] = {
      .name = "ldo1",
      .id = SPMU_REG_LDO1,
      .vsel_reg = 0x2c,
      .vsel_mask = 0x3f00,
      .enable_reg = 0x2c,
      .enable_mask = 0x1,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 50000,
      .min_uV = 1750000,
      .max_uV = 3300000,
      .boot_on = true,
  },
  [SPMU_REG_LDO2] = {
      .name = "ldo2",
      .id = SPMU_REG_LDO2,
      .vsel_reg = 0x30,
      .vsel_mask = 0x3f00,
      .enable_reg = 0x30,
      .enable_mask = 0x1,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 50000,
      .min_uV = 1750000,
      .max_uV = 3300000,
  },
  [SPMU_REG_LDO3] = {
      .name = "ldo3",
      .id = SPMU_REG_LDO3,
      .vsel_reg = 0x34,
      .vsel_mask = 0x3f00,
      .enable_reg = 0x34,
      .enable_mask = 0x1,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 50000,
      .min_uV = 1750000,
      .max_uV = 3300000,
      .boot_on = true,
  },
  [SPMU_REG_LDO4] = {
      .name = "ldo4",
      .id = SPMU_REG_LDO4,
      .vsel_reg = 0x38,
      .vsel_mask = 0x3f00,
      .enable_reg = 0x38,
      .enable_mask = 0x1,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 50000,
      .min_uV = 1750000,
      .max_uV = 3300000,
  },
  [SPMU_REG_LDO5] = {
      .name = "ldo5",
      .id = SPMU_REG_LDO5,
      .vsel_reg = 0x3c,
      .vsel_mask = 0x3f00,
      .enable_reg = 0x3c,
      .enable_mask = 0x1,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 50000,
      .min_uV = 1750000,
      .max_uV = 3300000,
  },
  [SPMU_REG_LDO6] = {
      .name = "ldo6",
      .id = SPMU_REG_LDO6,
      .vsel_reg = 0x40,
      .vsel_mask = 0x3f00,
      .enable_reg = 0x40,
      .enable_mask = 0x1,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 50000,
      .min_uV = 1800000,
      .max_uV = 3000000,
      .boot_on = true,
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_pmu_initialize(void)
{
  spmu_apb_init(TOP_PMICFSM_BASE, 0xb0180000,
      spmu_regulator_desc, ARRAY_SIZE(spmu_regulator_desc));
}
#endif /* (defined(CONFIG_ARCH_CHIP_U1_SP) || defined(CONFIG_ARCH_CHIP_U1_RECOVERY))
          && defined(CONFIG_SONG_PMIC_APB) */
