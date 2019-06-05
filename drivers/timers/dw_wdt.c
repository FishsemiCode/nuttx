/****************************************************************************
 * drivers/timers/dw_wdt.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/clk/clk.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/nuttx.h>
#include <nuttx/power/pm.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/watchdog.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DW_WDT_CONTROL_OFFSET               0x00
#define DW_WDT_TIMEOUT_RANGE_OFFSET         0x04
#define DW_WDT_CURRENT_COUNT_OFFSET         0x08
#define DW_WDT_COUNTER_RESTART_OFFSET       0x0c
#define DW_WDT_INTERRUPT_CLEAR_OFFSET       0x14

#define DW_WDT_CONTROL_WDT_EN_MASK          0x01
#define DW_WDT_CONTROL_IRQ_MASK             0x02
#define DW_WDT_CONTROL_RPL_MASK             0x1c

#define DW_WDT_TIMEOUT_RANGE_TOP_MAX        15
#define DW_WDT_TIMEOUT_RANGE_TOPINIT_SHIFT  4

#define DW_WDT_COUNTER_RESTART_KICK_VALUE   0x76

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the watchdog lower-half driver */

struct dw_wdt_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.
   */

  FAR const struct watchdog_ops_s *ops;

  uintptr_t   base;
  struct clk *tclk;
  bool      active;
  void      *upper;
  xcpt_t   handler;

#ifdef CONFIG_PM
  struct pm_callback_s pm_cb;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dw_wdt_start(FAR struct watchdog_lowerhalf_s *lower);
static int dw_wdt_stop(FAR struct watchdog_lowerhalf_s *lower);
static int dw_wdt_keepalive(FAR struct watchdog_lowerhalf_s *lower);
static int dw_wdt_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                            FAR struct watchdog_status_s *status);
static int dw_wdt_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                             uint32_t timeout);
static xcpt_t dw_wdt_capture(FAR struct watchdog_lowerhalf_s *lower,
                             CODE xcpt_t handler);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct watchdog_ops_s g_dw_wdt_ops =
{
  .start      = dw_wdt_start,
  .stop       = dw_wdt_stop,
  .keepalive  = dw_wdt_keepalive,
  .getstatus  = dw_wdt_getstatus,
  .settimeout = dw_wdt_settimeout,
  .capture    = dw_wdt_capture,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t dw_wdt_getreg(uintptr_t base, uint32_t off)
{
  return *(FAR volatile uint32_t *)(base + B2C(off));
}

static inline void dw_wdt_putreg(uintptr_t base, uint32_t off, uint32_t val)
{
  *(FAR volatile uint32_t *)(base + B2C(off)) = val;
}

static inline void dw_wdt_modifyreg(uintptr_t base, uint32_t off,
                                    uint32_t clearbits, uint32_t setbits)
{
  uint32_t temp;

  temp  = dw_wdt_getreg(base, off);
  temp &= ~clearbits;
  temp |= setbits;
  dw_wdt_putreg(base, off, temp);
}

static inline uint32_t dw_wdt_top_in_msec(uint32_t top, uint32_t rate)
{
  /* There are 16 possible timeout values in 0..15 where the number of
   * cycles is 2 ^ (16 + i) and the watchdog counts down.
   */

  return 1000ull * (1 << (16 + top)) / rate;
}

static inline uint32_t dw_wdt_get_timeout(uintptr_t base, uint32_t rate)
{
  uint32_t timeout = dw_wdt_getreg(base, DW_WDT_TIMEOUT_RANGE_OFFSET);
  return dw_wdt_top_in_msec(timeout & DW_WDT_TIMEOUT_RANGE_TOP_MAX, rate);
}

static inline uint32_t dw_wdt_get_timeleft(uintptr_t base, uint32_t rate)
{
  return 1000ull * dw_wdt_getreg(base, DW_WDT_CURRENT_COUNT_OFFSET) / rate;
}

static int dw_wdt_start(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct dw_wdt_lowerhalf_s *wdt = (FAR struct dw_wdt_lowerhalf_s *)lower;

  if (!wdt->active)
    {
      clk_enable(wdt->tclk);

      dw_wdt_modifyreg(wdt->base, DW_WDT_CONTROL_OFFSET,
                       0, DW_WDT_CONTROL_WDT_EN_MASK);

      /* Reload the count immediately because the hardware
       * always load 0xffff after the enable bit is set
       */

      dw_wdt_putreg(wdt->base, DW_WDT_COUNTER_RESTART_OFFSET,
                    DW_WDT_COUNTER_RESTART_KICK_VALUE);

      wdt->active = true;
    }

  return 0;
}

static int dw_wdt_stop(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct dw_wdt_lowerhalf_s *wdt = (FAR struct dw_wdt_lowerhalf_s *)lower;

  if (wdt->active)
    {
      /* Disable clock here since DW_WDT_CONTROL_WDT_EN_MASK
       * can be cleared only by a system reset
       */

      clk_disable(wdt->tclk);
      wdt->active = false;
    }

  return 0;
}

static int dw_wdt_keepalive(FAR struct watchdog_lowerhalf_s *lower)
{
  FAR struct dw_wdt_lowerhalf_s *wdt = (FAR struct dw_wdt_lowerhalf_s *)lower;

  dw_wdt_putreg(wdt->base, DW_WDT_COUNTER_RESTART_OFFSET,
                DW_WDT_COUNTER_RESTART_KICK_VALUE);
  return 0;
}

static int dw_wdt_getstatus(FAR struct watchdog_lowerhalf_s *lower,
                            FAR struct watchdog_status_s *status)
{
  FAR struct dw_wdt_lowerhalf_s *wdt = (FAR struct dw_wdt_lowerhalf_s *)lower;
  uint32_t rate = clk_get_rate(wdt->tclk);

  status->flags = wdt->active ? WDFLAGS_ACTIVE : 0;
  status->flags |= wdt->handler ? WDFLAGS_CAPTURE : 0;
  status->timeout = dw_wdt_get_timeout(wdt->base, rate);
  status->timeleft = dw_wdt_get_timeleft(wdt->base, rate);

  return 0;
}

static int dw_wdt_settimeout(FAR struct watchdog_lowerhalf_s *lower,
                             uint32_t timeout)
{
  FAR struct dw_wdt_lowerhalf_s *wdt = (FAR struct dw_wdt_lowerhalf_s *)lower;
  uint32_t rate = clk_get_rate(wdt->tclk);
  uint32_t top;

  /* Iterate over the timeout values until we find the closest match.
   * We always look for >=.
   */

  for (top = 0; top < DW_WDT_TIMEOUT_RANGE_TOP_MAX; top++)
    {
      if (dw_wdt_top_in_msec(top, rate) >= timeout)
        {
          break;
        }
    }

  /* Set the new value in the watchdog. Some versions of hardware
   * have TOPINIT in the TIMEOUT_RANGE register (as per CP_WDT_DUAL_TOP
   * in WDT_COMP_PARAMS_1). On those we effectively get a pat of
   * the watchdog right here.
   */

  dw_wdt_putreg(wdt->base, DW_WDT_TIMEOUT_RANGE_OFFSET,
                top | top << DW_WDT_TIMEOUT_RANGE_TOPINIT_SHIFT);
  return 0;
}

static xcpt_t dw_wdt_capture(FAR struct watchdog_lowerhalf_s *lower,
                             CODE xcpt_t handler)
{
  FAR struct dw_wdt_lowerhalf_s *wdt = (FAR struct dw_wdt_lowerhalf_s *)lower;
  xcpt_t oldhandler = wdt->handler;

  wdt->handler = handler;
  return oldhandler;
}

#ifdef CONFIG_PM
static void dw_wdt_pm_notify(FAR struct pm_callback_s *cb,
                             int domain, enum pm_state_e pmstate)
{
  FAR struct dw_wdt_lowerhalf_s *wdt =
    container_of(cb, struct dw_wdt_lowerhalf_s, pm_cb);

  switch (pmstate)
    {
    case PM_RESTORE:
      if (wdt->active)
        {
          clk_enable(wdt->tclk);
        }
      break;

    default:
      if (wdt->active)
        {
          clk_disable(wdt->tclk);
          if (clk_is_enabled(wdt->tclk))
            {
              PANIC();
            }
        }
      break;
    }
}
#endif

static int dw_wdt_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct dw_wdt_lowerhalf_s *wdt = arg;

  /* Clear interrupt request */

  dw_wdt_getreg(wdt->base, DW_WDT_INTERRUPT_CLEAR_OFFSET);

  if (wdt->handler)
    {
      return wdt->handler(irq, context, wdt->upper);
    }
  else
    {
      PANIC();
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int dw_wdt_initialize(FAR const struct dw_wdt_config_s *config)
{
  FAR struct dw_wdt_lowerhalf_s *wdt;

  wdt = kmm_zalloc(sizeof(*wdt));
  if (wdt == NULL)
    {
      return -ENOMEM;
    }

  wdt->ops = &g_dw_wdt_ops;
  wdt->base = config->base;

  wdt->tclk = clk_get(config->tclk);
  if (wdt->tclk == NULL)
    {
      kmm_free(wdt);
      return -EINVAL;
    }

  /* set RPL(reset pulse length) to max value(256 pclk)
   * ensure the reset signal can be caught by the system
   * and disable the watchdog and irq mode.
   */

  dw_wdt_putreg(wdt->base, DW_WDT_CONTROL_OFFSET, DW_WDT_CONTROL_RPL_MASK);

  wdt->upper = watchdog_register(config->path, (FAR void *)wdt);
  if (wdt->upper == NULL)
    {
      kmm_free(wdt);
      return -EINVAL;
    }

#ifdef CONFIG_PM
  wdt->pm_cb.notify = dw_wdt_pm_notify;
  pm_register(&wdt->pm_cb);
#endif

  if (config->irq >= 0)
    {
      dw_wdt_modifyreg(wdt->base, DW_WDT_CONTROL_OFFSET,
        DW_WDT_CONTROL_IRQ_MASK, DW_WDT_CONTROL_IRQ_MASK);
      irq_attach(config->irq, dw_wdt_interrupt, wdt);
      up_enable_irq(config->irq);
    }
  else
    {
      dw_wdt_modifyreg(wdt->base, DW_WDT_CONTROL_OFFSET,
                       DW_WDT_CONTROL_IRQ_MASK, 0);
    }


  return 0;
}
