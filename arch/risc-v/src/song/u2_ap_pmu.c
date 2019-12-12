/****************************************************************************
 * arch/riscv/src/song/u2_ap_pmu.c
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

#if (defined(CONFIG_ARCH_CHIP_U2_AP) && defined(CONFIG_SONG_PMIC_I2C))

#include <stdint.h>
#include <nuttx/i2c/i2c_master.h>
#include <nuttx/power/regulator.h>

#include "up_arch.h"
#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

enum {
  SPMU_REG_BUCK1 = 0,
  SPMU_REG_BUCK2,
  SPMU_REG_BUCK3,
  SPMU_REG_LDO1,
  SPMU_REG_LDO2,
  SPMU_REG_LDO3,
  SPMU_REG_LDO4,
  SPMU_REG_LDO5,
  /* Total number of regulators */
  SPMU_NUM_REGS,
};

static const struct regulator_desc spmu_regulator_desc[SPMU_NUM_REGS] = {
  [SPMU_REG_BUCK1] = {
      .name = "buck1",
      .id = SPMU_REG_BUCK1,
      .vsel_reg = 0x10,
      .vsel_mask = 0x3f,
      .enable_reg = 0x10,
      .enable_mask = 0x80,
      .enable_time = 500,
      .ramp_delay = 20000,
      .uV_step = 12500,
      .min_uV = 650000,
      .max_uV = 1437500,
      .boot_on = true,
      .apply_uV = 1100000,
  },
  [SPMU_REG_BUCK2] = {
      .name = "buck2",
      .id = SPMU_REG_BUCK2,
      .vsel_reg = 0x13,
      .vsel_mask = 0x3f,
      .enable_reg = 0x13,
      .enable_mask = 0x80,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 25000,
      .min_uV = 800000,
      .max_uV = 2375000,
      .apply_uV = 1800000,
      .boot_on = true,
  },
  [SPMU_REG_BUCK3] = {
      .name = "buck3",
      .id = SPMU_REG_BUCK3,
      .vsel_reg = 0x16,
      .vsel_mask = 0x3f,
      .enable_reg = 0x16,
      .enable_mask = 0x80,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 25000,
      .min_uV = 800000,
      .max_uV = 2375000,
      .boot_on = true,
  },
  [SPMU_REG_LDO1] = {
      .name = "ldo1",
      .id = SPMU_REG_LDO1,
      .vsel_reg = 0x22,
      .vsel_mask = 0x1f,
      .enable_reg = 0x22,
      .enable_mask = 0x80,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 25000,
      .min_uV = 1150000,
      .max_uV = 1900000,
      .boot_on = true,
  },
  [SPMU_REG_LDO2] = {
      .name = "ldo2",
      .id = SPMU_REG_LDO2,
      .vsel_reg = 0x24,
      .vsel_mask = 0x1f,
      .enable_reg = 0x24,
      .enable_mask = 0x80,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 25000,
      .min_uV = 1150000,
      .max_uV = 1900000,
      .boot_on = true,
  },
  [SPMU_REG_LDO3] = {
      .name = "ldo3",
      .id = SPMU_REG_LDO3,
      .vsel_reg = 0x26,
      .vsel_mask = 0x1f,
      .enable_reg = 0x26,
      .enable_mask = 0x80,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 25000,
      .min_uV = 1150000,
      .max_uV = 1900000,
      .boot_on = true,
  },
  [SPMU_REG_LDO4] = {
      .name = "ldo4",
      .id = SPMU_REG_LDO4,
      .vsel_reg = 0x28,
      .vsel_mask = 0x1f,
      .enable_reg = 0x28,
      .enable_mask = 0x80,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 25000,
      .min_uV = 1150000,
      .max_uV = 1900000,
      .boot_on = true,
  },
  [SPMU_REG_LDO5] = {
      .name = "ldo5",
      .id = SPMU_REG_LDO5,
      .vsel_reg = 0x2a,
      .vsel_mask = 0x1f,
      .enable_reg = 0x2a,
      .enable_mask = 0x80,
      .enable_time = 100,
      .ramp_delay = 20000,
      .uV_step = 25000,
      .min_uV = 1150000,
      .max_uV = 1900000,
      .boot_on = true,
  },
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern struct i2c_master_s *g_i2c[4];

void up_pmu_initialize(void)
{
  spmu_i2c_init((g_i2c[0]), 0x70, 400000,
      spmu_regulator_desc, ARRAY_SIZE(spmu_regulator_desc));
}
#endif /* (defined(CONFIG_ARCH_CHIP_U2_AP) && defined(CONFIG_SONG_PMIC_I2C) */
