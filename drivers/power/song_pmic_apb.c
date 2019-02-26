/****************************************************************************
 * drivers/power/spmu_regulator_apb.c
 * Lower half driver for song pmic regulator driver with apb interface
 *
 *   Copyright (C) 2018 Pinecone Electronics Inc. All rights reserved.
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

#include <sys/types.h>

#include <limits.h>
#include <unistd.h>
#include <stdint.h>
#include <stdbool.h>
#include <strings.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/kmalloc.h>
#include <nuttx/power/regulator.h>

/* This driver requires:
 *
 * CONFIG_REGULATOR - Upper half battery driver support
 */

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/****************************************************************************
 * Private
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

struct spmu_regulator {
  uintptr_t base;
  uintptr_t rf_base;
  struct regulator_dev *rdev[SPMU_NUM_REGS];
};
/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static int spmu_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV, unsigned *selector);
static int spmu_get_voltage(struct regulator_dev *rdev);
static int spmu_enable(struct regulator_dev *rdev);
static int spmu_disable(struct regulator_dev *rdev);
static int spmu_is_enabled(struct regulator_dev *rdev);

/****************************************************************************
 * Private Data
 ****************************************************************************/
static const struct regulator_ops  spmu_regulator_ops =
{
  .set_voltage = spmu_set_voltage,
  .get_voltage = spmu_get_voltage,
  .enable = spmu_enable,
  .disable = spmu_disable,
  .is_enabled = spmu_is_enabled,
};

static const struct regulator_desc spmu_regulator_desc[SPMU_NUM_REGS] = {
  [SPMU_REG_BUCK0] = {
      .name = "buck0",
      .id = SPMU_REG_BUCK0,
      .ops = &spmu_regulator_ops,
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
      .ops = &spmu_regulator_ops,
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
      .ops = &spmu_regulator_ops,
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
      .ops = &spmu_regulator_ops,
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
      .ops = &spmu_regulator_ops,
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
      .ops = &spmu_regulator_ops,
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
      .ops = &spmu_regulator_ops,
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
      .ops = &spmu_regulator_ops,
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
      .ops = &spmu_regulator_ops,
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
 * Private Functions
 ****************************************************************************/
static inline int spmu_read(struct spmu_regulator *priv, unsigned int offset, unsigned int *val)
{
  *val = *(volatile uint32_t *)(priv->base + offset);
  return 0;
}

static inline int spmu_write(struct spmu_regulator *priv, unsigned int offset, unsigned int val)
{
  *(volatile uint32_t *)(priv->base + offset) = val;
  return 0;
}

static inline int spmu_update_bits(struct spmu_regulator *priv, unsigned int offset,
                            unsigned int mask, unsigned int val)
{
  int ret;
  unsigned int tmp, orig;

  ret = spmu_read(priv, offset, &orig);
  if (ret < 0)
      return ret;

  tmp = orig & ~mask;
  tmp |= val & mask;

  return spmu_write(priv, offset, tmp);
}

static int spmu_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV, unsigned *selector)
{
  unsigned int val;

  if (min_uV > max_uV || min_uV < rdev->desc->min_uV || max_uV > rdev->desc->max_uV) {
      pwrerr("%s invalid uV, min %d max %d\n", __func__, min_uV, max_uV);
      return -EINVAL;
  }

  if (rdev->desc->id == SPMU_REG_BUCK0 || rdev->desc->id == SPMU_REG_LDO6) {
      pwrerr("%s not supported %d\n", __func__, rdev->desc->id);
      return -EINVAL;
  }

  val = (min_uV - rdev->desc->min_uV) / rdev->desc->uV_step;
  /* For ldo0, 0x0 is for 900mv and 0xc 600mv */
  if (rdev->desc->id == SPMU_REG_LDO0 && (min_uV == 600000 || min_uV == 900000)) {
      if (min_uV == 600000)
          val = 0xc;
      else if (min_uV == 900000)
          val = 0x0;
  }

  *selector = val;

  val <<= ffs(rdev->desc->vsel_mask) - 1;
  return spmu_update_bits(rdev->priv, rdev->desc->vsel_reg, rdev->desc->vsel_mask, val);
}

static int spmu_get_voltage(struct regulator_dev *rdev)
{
  int ret;
  unsigned int val;

  ret = spmu_read(rdev->priv, rdev->desc->vsel_reg, &val);
  if (ret < 0)
      return ret;

  val &= rdev->desc->vsel_mask;
  val >>= ffs(rdev->desc->vsel_mask) - 1;

  /* For ldo0, 0x0 is for 900mv and 0xc 600mv */
  if (rdev->desc->id == SPMU_REG_LDO0 && (val == 0x0 || val == 0xc)) {
      if (val == 0x0)
          return 900000;
      else if (val == 0xc)
          return 600000;
  }

  return (val * rdev->desc->uV_step + rdev->desc->min_uV);
}

static int spmu_enable(struct regulator_dev *rdev)
{
  unsigned int val = 1;

  val <<= ffs(rdev->desc->enable_mask) - 1;
  return spmu_update_bits(rdev->priv, rdev->desc->enable_reg, rdev->desc->enable_mask, val);
}

static int spmu_disable(struct regulator_dev *rdev)
{
  unsigned int val = 0;

  val <<= ffs(rdev->desc->enable_mask) - 1;
  return spmu_update_bits(rdev->priv, rdev->desc->enable_reg, rdev->desc->enable_mask, val);
}

static int spmu_is_enabled(struct regulator_dev *rdev)
{
  unsigned int val;
  int ret;

  ret = spmu_read(rdev->priv, rdev->desc->enable_reg, &val);
  if (ret < 0)
      return 0;

  return !!(val & rdev->desc->enable_mask);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/
int spmu_regulator_apb_initialize(uintptr_t base, uintptr_t rf_base)
{
  struct spmu_regulator *priv;
  int i, ret = 0;

  if (!base || !rf_base) {
      pwrerr("%s invalid base\n", __func__);
      return -EINVAL;
  }

  priv = kmm_zalloc(sizeof(struct spmu_regulator));
  if (priv == NULL) {
      pwrerr("%s failed to get memory\n", __func__);
      return -EINVAL;
  }

  priv->base = base;
  priv->rf_base = rf_base;

  for (i = 0; i < SPMU_NUM_REGS; i++) {
      priv->rdev[i] = regulator_register(&spmu_regulator_desc[i], (void *)priv);
      if (priv->rdev[i] == NULL) {
          pwrerr("%s failed to register %d\n", __func__, i);
          ret = -EINVAL;
          goto err;
      }
  }

  return ret;

err:
  while (--i >= 0) {
      if (priv->rdev[i])
          regulator_unregister(priv->rdev[i]);
  }

  kmm_free(priv);
  return ret;
}
