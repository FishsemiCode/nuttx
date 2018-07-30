/****************************************************************************
 * include/nuttx/pinctrl/pinctrl.h
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
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

#ifndef __INCLUDE_NUTTX_PINCTRL_PINCTRL_H
#define __INCLUDE_NUTTX_PINCTRL_PINCTRL_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

#include <nuttx/fs/ioctl.h>

#ifdef CONFIG_PINCTRL

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Command:     PINCTRLC_SETFUNC
 * Description: Set the mux function of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SETDS
 * Description: Set the driver strength of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SETDT
 * Description: Set the driver type of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SETSLEW
 * Description: Set slewrate of the pinctrl pin
 * Argument:    A pointer to an instance of struct pinctrl_iotrans_s
 *
 * Command:     PINCTRLC_SELGPIO
 * Description: Select gpio function of pinctrl pin
 * Argument:    The uint32_t pinctrl number
 *
 */

#define PINCTRLC_SETFUNC      _PINCTRLC(1)
#define PINCTRLC_SETDS        _PINCTRLC(2)
#define PINCTRLC_SETDT        _PINCTRLC(3)
#define PINCTRLC_SETSLEW      _PINCTRLC(4)
#define PINCTRLC_SELGPIO      _PINCTRLC(5)

/* Access macros ************************************************************/

/****************************************************************************
 * Name: PINCTRL_SETFUNC
 *
 * Description:
 *   Set the mux function of the pinctrl pin.
 *
 * Input Parameters:
 *   dev      - Device-specific state data.
 *   pin      - the pinctrl controller number.
 *   selector - the pinctrl pin function number.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETFUNC(dev, pin, selector) ((dev)->ops->set_mux(dev, pin, selector))

/****************************************************************************
 * Name: PINCTRL_SETDS
 *
 * Description:
 *   Set the driver strength of the pinctrl pin
 *
 * Input Parameters:
 *   dev   - Device-specific state data.
 *   pin   - the pinctrl controller number.
 *   level - the pinctrl pin driver strength number.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETDS(dev, pin, level) ((dev)->ops->set_ds(dev, pin, level))

/****************************************************************************
 * Name: PINCTRL_SETDT
 *
 * Description:
 *   Set the driver type of the pinctrl pin
 *
 * Input Parameters:
 *   dev  - Device-specific state data.
 *   pin  - the pinctrl controller number.
 *   type - the pinctrl_drivertype_e type.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETDT(dev, pin, type) ((dev)->ops->set_dt(dev, pin, type))

/****************************************************************************
 * Name: PINCTRL_SETSLEW
 *
 * Description:
 *   Set the slewrate of the pinctrl pin
 *
 * Input Parameters:
 *   dev   - Device-specific state data.
 *   pin   - the pinctrl controller number.
 *   state - the slewsrate state.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SETSLEW(dev, pin, state) ((dev)->ops->set_slew(dev, pin, state))

/****************************************************************************
 * Name: PINCTRL_SELGPIO
 *
 * Description:
 *   Select gpio function the pinctrl pin
 *
 * Input Parameters:
 *   dev   - Device-specific state data.
 *   pin   - the pinctrl controller number.
 *
 * Returned Value:
 *   0 on success, else a negative error code
 *
 ****************************************************************************/

#define PINCTRL_SELGPIO(dev, pin) ((dev)->ops->sel_gpio(dev, pin))

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Identifies the type of the pinctrl driver type */

enum pinctrl_drivertype_e
{
  BIAS_DISABLE = 0,
  BIAS_PULLUP,
  BIAS_PULLDOWN,
  BIAS_STRONG_PULLDOWN,
  BIAS_NDRIVERTYPES
};

/*
   This describes pinctrl ioctl structure in pinctrl command operation.
   the follow command use this structure:
   PINCTRLC_SETFUNC, PINCTRLC_SETDS ,PINCTRLC_SETDT, PINCTRLC_SETSLEW.
*/

struct pinctrl_iotrans_s
{
  uint32_t pin;
  union
  {
    uint32_t                  selector;
    uint32_t                  level;
    enum pinctrl_drivertype_e type;
    uint32_t                  state;
  } para;
};

/* pinctrl interface methods */

struct pinctrl_dev_s;
struct pinctrl_ops_s
{
  int (*set_mux)(FAR struct pinctrl_dev_s *dev, uint32_t pin, uint32_t selector);
  int (*set_ds)(FAR struct pinctrl_dev_s *dev,  uint32_t pin, uint32_t level);
  int (*set_dt)(FAR struct pinctrl_dev_s *dev, uint32_t pin, enum pinctrl_drivertype_e type);
  int (*set_slew)(FAR struct pinctrl_dev_s *dev, uint32_t pin, uint32_t state);
  int (*sel_gpio)(FAR struct pinctrl_dev_s *dev, uint32_t pin);
};

struct pinctrl_dev_s
{
  /* "Lower half" operations provided by the pinctrl lower half */
  FAR const struct pinctrl_ops_s *ops;

  /* Internal storage used by the pinctrl may (internal to the pinctrl
   * implementation).
   */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: pinctrl_register
 *
 * Description:
 *   Register PINCTRL device driver.
 *
 ****************************************************************************/

int pinctrl_register(FAR struct pinctrl_dev_s *dev, int minor);

/****************************************************************************
 * Name: pinctrl_unregister
 *
 * Description:
 *   Unregister PINCTRL device driver.
 *
 ****************************************************************************/

void pinctrl_unregister(FAR struct pinctrl_dev_s *dev, int minor);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_PINCTRL */
#endif /* __INCLUDE_NUTTX_PINCTRL_PINCTRL_H */
