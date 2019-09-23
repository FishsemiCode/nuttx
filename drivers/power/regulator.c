/****************************************************************************
 * drivers/power/regulator.c
 * Upper-half, common core driver for pmic regulators.
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

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <semaphore.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>
#include <nuttx/power/regulator.h>

#include "internal.h"

/* This driver requires:
 *
 * CONFIG_REGULATOR - Upper half regulator driver support
 */

#if defined(CONFIG_REGULATOR)

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Debug ********************************************************************/

/****************************************************************************
 * Private
 ****************************************************************************/
static struct list_node regulator_list = LIST_INITIAL_VALUE(regulator_list);
static sem_t regulator_list_sem = SEM_INITIALIZER(1);

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int _regulator_is_enabled(struct regulator_dev *rdev);
static int _regulator_do_enable(struct regulator_dev *rdev);
static int _regulator_do_disable(struct regulator_dev *rdev);
static int regulator_check_consumers(struct regulator_dev *rdev, int *min_uV, int *max_uV);
static struct regulator_dev *regulator_dev_lookup(void *dev, const char *supply);
static int regulator_map_voltage_iterate(struct regulator_dev *rdev, int min_uV, int max_uV);
static int _regulator_get_voltage(struct regulator_dev *rdev);
static int _regulator_do_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV);
static int _regulator_set_voltage_unlocked(struct regulator *regulator, int min_uV, int max_uV);

/****************************************************************************
 * Private Functions
 ****************************************************************************/
static int _regulator_is_enabled(struct regulator_dev *rdev)
{
  if (!rdev->ops->is_enabled)
      return 1;

  return rdev->ops->is_enabled(rdev);
}

static int _regulator_do_enable(struct regulator_dev *rdev)
{
  int ret = 0;

  if (rdev->ops->enable) {
      ret = rdev->ops->enable(rdev);
      if (ret < 0) {
          pwrerr("failed to enable %d\n", ret);
          return ret;
      }
  }

  if (rdev->desc->enable_time > 0)
      up_udelay(rdev->desc->enable_time);

  return ret;
}

static int _regulator_do_disable(struct regulator_dev *rdev)
{
  int ret = 0;

  if (rdev->ops->disable) {
      ret = rdev->ops->disable(rdev);
      if (ret < 0) {
          pwrerr("failed to disable %d\n", ret);
      }
  }

  return ret;
}

static int regulator_check_consumers(struct regulator_dev *rdev, int *min_uV, int *max_uV)
{
  struct regulator *regulator;

  list_for_every_entry(&rdev->consumer_list, regulator, struct regulator, list) {
      /*
       * Assume consumers that didn't say anything are OK
       * with anything in the constraint range.
       */
      if (!regulator->min_uV && !regulator->max_uV)
          continue;

      if (*max_uV > regulator->max_uV)
          *max_uV = regulator->max_uV;
      if (*min_uV < regulator->min_uV)
          *min_uV = regulator->min_uV;
  }

  if (*min_uV > *max_uV) {
      pwrerr("Restricting voltage, %d-%d uV\n", *min_uV, *max_uV);
      return -EINVAL;
  }

  return 0;
}

static struct regulator_dev *regulator_dev_lookup(void *dev, const char *supply)
{
  struct regulator_dev *rdev, *rdev_found = NULL;

  nxsem_wait_uninterruptible(&regulator_list_sem);
  list_for_every_entry(&regulator_list, rdev, struct regulator_dev, list) {
      if (rdev->desc->name && strcmp(rdev->desc->name, supply) == 0) {
          rdev_found = rdev;
          break;
      }
  }
  nxsem_post(&regulator_list_sem);

  return rdev_found;
}

static int regulator_map_voltage_iterate(struct regulator_dev *rdev, int min_uV, int max_uV)
{
  int best_val = INT_MAX;
  int selector = 0;
  int i, ret;

  /* Find the smallest voltage that falls within the specified range. */
  for (i = 0; i < rdev->desc->n_voltages; i++) {
      ret = rdev->ops->list_voltage(rdev, i);
      if (ret < 0)
          continue;

      if (ret < best_val && ret >= min_uV && ret <= max_uV) {
          best_val = ret;
          selector = i;
      }
  }

  if (best_val != INT_MAX)
      return selector;
  else
      return -EINVAL;
}

static int _regulator_get_voltage(struct regulator_dev *rdev)
{
  int sel, ret;

  if (rdev->ops->get_voltage_sel) {
      sel = rdev->ops->get_voltage_sel(rdev);
      if (sel < 0)
          return sel;
      ret = rdev->ops->list_voltage(rdev, sel);
  } else if (rdev->ops->get_voltage) {
      ret = rdev->ops->get_voltage(rdev);
  } else if (rdev->ops->list_voltage) {
      ret = rdev->ops->list_voltage(rdev, 0);
  } else {
      return -EINVAL;
  }

  return ret;
}

static int _regulator_do_set_voltage(struct regulator_dev *rdev, int min_uV, int max_uV)
{
  const struct regulator_ops *ops = rdev->ops;
  unsigned int selector;
  int new_uV = 0, old_uV = _regulator_get_voltage(rdev);
  int ret = 0, delay = 0, best_val;

  if (ops->set_voltage) {
      ret = ops->set_voltage(rdev, min_uV, max_uV, &selector);
      if (ret >= 0) {
          if (ops->list_voltage)
              new_uV = ops->list_voltage(rdev, selector);
          else
              new_uV = _regulator_get_voltage(rdev);
      }
  } else if (ops->set_voltage_sel) {
      ret = regulator_map_voltage_iterate(rdev, min_uV, max_uV);
      if (ret >= 0) {
          best_val = ops->list_voltage(rdev, ret);
          if (min_uV <= best_val && max_uV >= best_val) {
              selector = ret;
              ret = ops->set_voltage_sel(rdev, selector);
          }
      } else {
          ret = -EINVAL;
      }
  } else {
      ret = -EINVAL;
  }

  if (ret < 0)
      return ret;

  if (rdev->desc->ramp_delay)
      delay = abs(new_uV - old_uV) / rdev->desc->ramp_delay + 1;

  up_udelay(delay);

  return ret;
}

static int _regulator_set_voltage_unlocked(struct regulator *regulator, int min_uV, int max_uV)
{
  struct regulator_dev *rdev = regulator->rdev;
  const struct regulator_ops *ops = rdev->ops;
  int old_min_uV, old_max_uV;
  int ret = 0;

  if (min_uV > max_uV) {
      pwrerr("invalid min %d max %d\n", min_uV, max_uV);
      return -EINVAL;
  }

  if (regulator->min_uV == min_uV && regulator->max_uV == max_uV)
      goto out;

  if (!ops->set_voltage && !ops->set_voltage_sel) {
      pwrerr("set voltage is null\n");
      ret = -EINVAL;
      goto out;
  }

  if (max_uV > rdev->desc->max_uV)
     max_uV = rdev->desc->max_uV;
  if (min_uV < rdev->desc->min_uV)
     min_uV = rdev->desc->min_uV;

  if (min_uV > max_uV) {
      pwrerr("invalid min %d max %d\n", min_uV, max_uV);
      ret = -EINVAL;
      goto out;
  }

  /* rstore original values in case of error */
  old_min_uV = regulator->min_uV;
  old_max_uV = regulator->max_uV;
  regulator->min_uV = min_uV;
  regulator->max_uV = max_uV;

  ret = regulator_check_consumers(rdev, &min_uV, &max_uV);
  if (ret < 0)
      goto out2;

  ret = _regulator_do_set_voltage(rdev, min_uV, max_uV);
  if (ret < 0)
      goto out2;

out:
  return ret;

out2:
  regulator->min_uV = old_min_uV;
  regulator->max_uV = old_max_uV;

  return ret;
}
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: regulator_get
 *
 * Description:
 *   Lookup and obtain a reference to a regulator.
 *
 * Input parameters:
 *   dev - The device for regulator consumer, NULL is ok.
 *   id - Supply name or the regulator ID.
 *
 * Returned value:
 *    A struct regulator pointer on success or NULL on failure
 *
 ****************************************************************************/

struct regulator *regulator_get(void *dev, const char *id)
{
  struct regulator_dev *rdev;
  struct regulator *regulator = NULL;

  if (id == NULL) {
      pwrerr("get() with no identifier\n");
      return NULL;
  }

  rdev = regulator_dev_lookup(dev, id);
  if (rdev == NULL) {
      pwrerr("regulator %s not found\n", id);
      return NULL;
  }

  regulator = kmm_zalloc(sizeof(struct regulator));
  if (regulator == NULL) {
      pwrerr("failed to get memory\n");
      return NULL;
  }

  regulator->rdev = rdev;
  list_initialize(&regulator->list);

  nxsem_wait_uninterruptible(&rdev->regulator_sem);
  rdev->open_count++;
  list_add_tail(&rdev->consumer_list, &regulator->list);
  nxsem_post(&rdev->regulator_sem);

  return regulator;
}

/****************************************************************************
 * Name: regulator_put
 *
 * Description:
 *   Free the regulator resource.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *
 ****************************************************************************/

void regulator_put(struct regulator *regulator)
{
  struct regulator_dev *rdev;

  if (regulator == NULL)
      return;

  rdev = regulator->rdev;

  nxsem_wait_uninterruptible(&regulator_list_sem);

  nxsem_wait_uninterruptible(&rdev->regulator_sem);
  list_delete(&regulator->list);
  rdev->open_count--;
  nxsem_post(&rdev->regulator_sem);

  nxsem_post(&regulator_list_sem);

  kmm_free(regulator);
}

/****************************************************************************
 * Name: regulator_is_enabled
 *
 * Description:
 *   Is the regulator output enabled.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *   1 is enabled and zero for disabled.
 *
 ****************************************************************************/

int regulator_is_enabled(struct regulator *regulator)
{
  struct regulator_dev *rdev;
  int ret = 0;

  if (regulator == NULL) {
      pwrerr("regulator is null\n");
      return -EINVAL;
  }

  rdev = regulator->rdev;

  nxsem_wait_uninterruptible(&rdev->regulator_sem);
  ret = _regulator_is_enabled(rdev);
  nxsem_post(&rdev->regulator_sem);

  return ret;
}

/****************************************************************************
 * Name: regulator_enable
 *
 * Description:
 *   Enable the regulator output.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_enable(struct regulator *regulator)
{
  struct regulator_dev *rdev;
  int ret = 0;

  if (regulator == NULL) {
      pwrerr("enable regulator is null\n");
      return -EINVAL;
  }

  rdev = regulator->rdev;

  nxsem_wait_uninterruptible(&rdev->regulator_sem);
  if (rdev->use_count == 0) {
      ret = _regulator_do_enable(rdev);
      if (ret < 0)
          return ret;
  }

  rdev->use_count++;
  nxsem_post(&rdev->regulator_sem);

  return 0;
}

/****************************************************************************
 * Name: regulator_disable
 *
 * Description:
 *   Disable the regulator output.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_disable(struct regulator *regulator)
{
  struct regulator_dev *rdev;
  int ret = 0;

  if (regulator == NULL) {
      pwrerr("disable regulator is null\n");
      return -EINVAL;
  }

  rdev = regulator->rdev;

  nxsem_wait_uninterruptible(&rdev->regulator_sem);
  if (rdev->use_count <= 0) {
      pwrerr("unbalanced disable for %s\n", rdev->desc->name);
      return -EIO;
  }

  if (rdev->use_count == 1) {
      ret = _regulator_do_disable(rdev);
      if (ret < 0)
          return ret;
  }

  rdev->use_count--;
  nxsem_post(&rdev->regulator_sem);

  return 0;
}

/****************************************************************************
 * Name: regulator_set_voltage
 *
 * Description:
 *   Set the regulator output voltage.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *   min_uV - Minimum required voltage in uV
 *   max_uV - Maximum acceptable voltage in uV
 *
 * Returned value:
 *   Zero on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_set_voltage(struct regulator *regulator, int min_uV, int max_uV)
{
  struct regulator_dev *rdev;
  int ret = 0;

  if (regulator == NULL) {
      pwrerr("get regulator is null\n");
      return -EINVAL;
  }

  rdev = regulator->rdev;

  nxsem_wait_uninterruptible(&rdev->regulator_sem);
  ret = _regulator_set_voltage_unlocked(regulator, min_uV, max_uV);
  nxsem_post(&rdev->regulator_sem);

  return ret;
}

/****************************************************************************
 * Name: regulator_get_voltage
 *
 * Description:
 *   Obtain the regulator output voltage.
 *
 * Input parameters:
 *   regulator - The regulator consumer representative
 *
 * Returned value:
 *   Positive on success or a negated errno value on failure.
 *
 ****************************************************************************/

int regulator_get_voltage(struct regulator *regulator)
{
  struct regulator_dev *rdev;
  int ret = 0;

  if (regulator == NULL) {
      pwrerr("get regulator is null\n");
      return -EINVAL;
  }

  rdev = regulator->rdev;

  nxsem_wait_uninterruptible(&rdev->regulator_sem);
  ret = _regulator_get_voltage(rdev);
  nxsem_post(&rdev->regulator_sem);

  return ret;
}

/****************************************************************************
 * Name: regulator_register
 *
 * Description:
 *   This routine is called by the specific regulator drivers to register a
 *   regulator.
 *
 ****************************************************************************/

struct regulator_dev *regulator_register(const struct regulator_desc *regulator_desc,
                          const struct regulator_ops *regulator_ops, void *priv)
{
  struct regulator_dev *rdev;

  if (regulator_desc == NULL) {
      pwrerr("regulator desc is null\n");
      return NULL;
  }

  if (regulator_desc->name == NULL || regulator_ops == NULL) {
      pwrerr("regulator name or ops is null\n");
      return NULL;
  }

  if (regulator_ops->get_voltage && regulator_ops->get_voltage_sel) {
      pwrerr("get_voltage and get_voltage_sel are assigned\n");
      return NULL;
  }

  if (regulator_ops->set_voltage && regulator_ops->set_voltage_sel) {
      pwrerr("set_voltage and set_voltage_sel are assigned\n");
      return NULL;
  }

  if (regulator_ops->get_voltage_sel && !regulator_ops->list_voltage) {
      pwrerr("list voltage is null\n");
      return NULL;
  }

  if (regulator_ops->set_voltage_sel && !regulator_ops->list_voltage) {
      pwrerr("list voltage is null\n");
      return NULL;
  }

  rdev = kmm_zalloc(sizeof(struct regulator_dev));
  if (rdev == NULL) {
      pwrerr("failed to get memory\n");
      return NULL;
  }

  rdev->desc = regulator_desc;
  rdev->ops = regulator_ops;
  rdev->priv = priv;
  nxsem_init(&rdev->regulator_sem, 0, 1);
  list_initialize(&rdev->consumer_list);
  list_initialize(&rdev->list);

  if (rdev->desc->boot_on && !_regulator_is_enabled(rdev)) {
      _regulator_do_enable(rdev);
  } else if (!rdev->desc->boot_on && _regulator_is_enabled(rdev)) {
      _regulator_do_disable(rdev);
  }

  if (rdev->desc->apply_uV)
      _regulator_do_set_voltage(rdev, rdev->desc->apply_uV, rdev->desc->apply_uV);

  nxsem_wait_uninterruptible(&regulator_list_sem);
  list_add_tail(&regulator_list, &rdev->list);
  nxsem_post(&regulator_list_sem);

  return rdev;
}

/****************************************************************************
 * Name: regulator_unregister
 *
 * Description:
 *   This routine is called by the specific regulator drivers to unregister a
 *   regulator.
 *
 ****************************************************************************/

void regulator_unregister(struct regulator_dev *rdev)
{
  if (rdev == NULL)
      return;

  nxsem_wait_uninterruptible(&regulator_list_sem);
  if (rdev->open_count) {
      pwrerr("unregister, open %d\n", rdev->open_count);
      nxsem_post(&regulator_list_sem);
      return;
  }
  list_delete(&rdev->list);
  nxsem_post(&regulator_list_sem);

  kmm_free(rdev);
}

#endif /* CONFIG_REGULATOR */
