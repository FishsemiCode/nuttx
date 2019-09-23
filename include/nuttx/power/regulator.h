/****************************************************************************
 * include/nuttx/power/regulator.h
 * NuttX PMIC Regulator Driver Interfaces
 *
 *   Copyright (C) 2018, Pinecone Electronics Inc. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_POWER_REGULATOR_H
#define __INCLUDE_NUTTX_POWER_REGULATOR_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <semaphore.h>

#include <nuttx/list.h>

#ifdef CONFIG_REGULATOR

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* CONFIG_REGULATOR - Upper half PMIC regulator driver support
 *
 * Specific, lower-half drivers will have other configuration requirements
 * such as:
 *
 *   CONFIG_I2C - I2C support *may* be needed
 */

/* IOCTL Commands ***********************************************************/
/*
 */


/****************************************************************************
 * Public Types
 ****************************************************************************/
 /* This structure defines the lower half regulator interface */

struct regulator_dev;
struct regulator_ops
{
  /* enumerate supported voltage */
  int (*list_voltage)(struct regulator_dev *rdev, unsigned selector);

  /* set regulator voltages */
  int (*set_voltage)(struct regulator_dev *rdev, int min_uV, int max_uV, unsigned *selector);

  int (*set_voltage_sel)(struct regulator_dev *rdev, unsigned selector);

  /* get regulator voltage */
  int (*get_voltage)(struct regulator_dev *rdev);

  int (*get_voltage_sel)(struct regulator_dev *rdev);

  /* enable regulator */
  int (*enable)(struct regulator_dev *rdev);

  /* is enabled */
  int (*is_enabled)(struct regulator_dev *rdev);

  /* disable regulator */
  int (*disable)(struct regulator_dev *rdev);
};

/* This structure defines the regulator state structure */

struct regulator_desc
{
  const char *name;
  int id;
  unsigned int n_voltages;

  unsigned int vsel_reg;
  unsigned int vsel_mask;
  unsigned int enable_reg;
  unsigned int enable_mask;

  unsigned int enable_time; /* us */
  unsigned int ramp_delay; /* us */
  unsigned int uV_step; /* us */
  unsigned int min_uV;
  unsigned int max_uV;
  unsigned int apply_uV;
  bool boot_on;
};

struct regulator_dev
{
  /* Fields required by the upper-half driver */
  const struct regulator_desc *desc; /* regulator descriptor struct */
  const struct regulator_ops *ops; /* regulator operations */
  uint32_t use_count; /* the use count by consumer */
  uint32_t open_count; /* the open count by consumer */
  sem_t regulator_sem;  /* Enforce mutually exclusive access */
  struct list_node list; /* list of all regulators  */
  struct list_node consumer_list; /* consumers we supply*/

  /* Data fields specific to the lower-half driver may follow */
  void *priv; /* driver private data */
};

#endif /* CONFIG_REGULATOR */

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#ifdef CONFIG_REGULATOR

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/****************************************************************************
 * Name: regulator_register
 *
 * Description:
 *   Register a lower half regulator driver with the common, upper-half
 *   regulator driver.
 *
 * Input parameters:
 *   desc - The regulator descriptor struct.
 *   ops  - The regulator operations pointer
 *
 * Returned value:
 *    The pointer to struct regulator_dev on success or NULL on failure.
 *
 ****************************************************************************/

struct regulator_dev *regulator_register(const struct regulator_desc *desc,
        const struct regulator_ops *ops, void *priv);

/****************************************************************************
 * Name: regulator_unregister
 *
 * Description:
 *   Unregister a lower half regulator driver with the common, upper-half
 *   regulator driver.
 *
 * Input parameters:
 *   rdev - The regulator dev pointer.
 *
 * Returned value:
 *    N/A
 *
 ****************************************************************************/

void regulator_unregister(struct regulator_dev *rdev);

/****************************************************************************
 * Name: song_pmic_regulator_apb_initialize
 *
 * Description:
 *   Initialize song pmic module with apb interface.
 *
 * Input Parameters:
 *   base    - the base addr for song pmic module
 *   rf_base - the base addr for rf power module
 *   rdesc   - the regulator descriptors
 *   len     - the number of regulator descriptors
 *
 * Returned Value:
 *
 ****************************************************************************/
#if defined(CONFIG_SONG_PMIC_APB)
int spmu_apb_init(uintptr_t base, uintptr_t rf_base,
         const struct regulator_desc *rdesc, int len);
#endif

/****************************************************************************
 * Name: song_pmic_regulator_i2c_initialize
 *
 * Description:
 *   Initialize song pmic module with i2c interface.
 *
 * Input Parameters:
 *   i2c  - the i2c master
 *   addr - the device i2c address
 *   freq - the i2c frequency

 * Returned Value:
 *
 ****************************************************************************/
#if defined(CONFIG_SONG_PMIC_I2C)
int spmu_regulator_i2c_initialize(struct i2c_master_s *i2c, uint8_t addr, uint32_t freq);
#endif

#endif /* CONFIG_REGULATOR */

/****************************************************************************
 * Name: rpmsg_regulator_init
 *
 * Description:
 *
 * Input Parameters:
 *
 * Returned Value:
 *
 ****************************************************************************/
#if defined(CONFIG_RPMSG_REGULATOR)
int rpmsg_regulator_init(const char *cpu_name, bool server);
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_POWER_REGULATOR_H */
