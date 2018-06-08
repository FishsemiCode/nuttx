/****************************************************************************
 * drivers/timers/cmsdk_timer.c
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

#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>

#include <nuttx/timers/cmsdk_timer.h>

#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CMSDK_TIMER_CTRL_ENABLE               (1 << 0)
#define CMSDK_TIMER_CTRL_INT_ENABLE           (1 << 3)

#define CMSDK_TIMER_INT_STATUS_CLEAR          (1 << 0)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct cmsdk_timer_s
{
  volatile uint32_t CTRL;
  volatile uint32_t VALUE;
  volatile uint32_t RELOAD;
  volatile uint32_t INT_STATUS;
};

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct cmsdk_timer_lowerhalf_s
{
  const struct timer_ops_s *ops;        /* Lower half operations */
  FAR struct cmsdk_timer_s *tim;        /* Hardware timer register */
  uint32_t                  freq;       /* Timer working clock frequency(Hz) */
  tccb_t                    callback;   /* Current user interrupt callback */
  void                     *arg;        /* Argument passed to upper half callback */
  uint32_t                 *next_interval;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int cmsdk_timer_start(FAR struct timer_lowerhalf_s *lower_);
static int cmsdk_timer_stop(FAR struct timer_lowerhalf_s *lower_);
static int cmsdk_timer_getstatus(FAR struct timer_lowerhalf_s *lower_,
                                 FAR struct timer_status_s *status);
static int cmsdk_timer_settimeout(FAR struct timer_lowerhalf_s *lower_,
                                  uint32_t timeout);
static void cmsdk_timer_setcallback(FAR struct timer_lowerhalf_s *lower_,
                                    tccb_t callback, FAR void *arg);
static int cmsdk_timer_maxtimeout(FAR struct timer_lowerhalf_s *lower_,
                                  uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct timer_ops_s g_cmsdk_timer_ops =
{
  .start       = cmsdk_timer_start,
  .stop        = cmsdk_timer_stop,
  .getstatus   = cmsdk_timer_getstatus,
  .settimeout  = cmsdk_timer_settimeout,
  .setcallback = cmsdk_timer_setcallback,
  .maxtimeout  = cmsdk_timer_maxtimeout,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint64_t usec_from_count(uint32_t count, uint32_t freq)
{
  return (uint64_t)count * USEC_PER_SEC / freq;
}

static inline uint64_t usec_to_count(uint32_t usec, uint32_t freq)
{
  return (uint64_t)usec * freq / USEC_PER_SEC;
}

static int cmsdk_timer_start(FAR struct timer_lowerhalf_s *lower_)
{
  FAR struct cmsdk_timer_lowerhalf_s *lower = (FAR struct cmsdk_timer_lowerhalf_s *)lower_;

  irqstate_t flags = enter_critical_section();
  lower->tim->CTRL |= CMSDK_TIMER_CTRL_ENABLE;
  leave_critical_section(flags);
  return 0;
}

static int cmsdk_timer_stop(FAR struct timer_lowerhalf_s *lower_)
{
  FAR struct cmsdk_timer_lowerhalf_s *lower = (FAR struct cmsdk_timer_lowerhalf_s *)lower_;

  irqstate_t flags = enter_critical_section();
  lower->tim->CTRL &= ~CMSDK_TIMER_CTRL_ENABLE;
  lower->tim->INT_STATUS = CMSDK_TIMER_INT_STATUS_CLEAR;
  leave_critical_section(flags);
  return 0;
}

static int cmsdk_timer_getstatus(FAR struct timer_lowerhalf_s *lower_,
                                 FAR struct timer_status_s *status)
{
  FAR struct cmsdk_timer_lowerhalf_s *lower = (FAR struct cmsdk_timer_lowerhalf_s *)lower_;

  irqstate_t flags = enter_critical_section();
  status->flags  = lower->callback != NULL ? TCFLAGS_HANDLER : 0;
  status->flags |= lower->tim->CTRL & CMSDK_TIMER_CTRL_ENABLE ? TCFLAGS_ACTIVE : 0;
  status->timeout = usec_from_count(lower->tim->RELOAD, lower->freq);
  status->timeleft = usec_from_count(lower->tim->VALUE, lower->freq);
  leave_critical_section(flags);

  return 0;
}

static int cmsdk_timer_settimeout(FAR struct timer_lowerhalf_s *lower_,
                                  uint32_t timeout)
{
  FAR struct cmsdk_timer_lowerhalf_s *lower = (FAR struct cmsdk_timer_lowerhalf_s *)lower_;
  uint32_t count = usec_to_count(timeout, lower->freq);

  irqstate_t flags = enter_critical_section();
  if (lower->next_interval)
    {
      /* If the timer callback is in the process,
       * delay the update to timer interrupt handler.
       */

      *lower->next_interval = timeout;
    }
  else
    {
      lower->tim->VALUE  = count;
      lower->tim->RELOAD = count;
    }
  leave_critical_section(flags);

  return 0;
}


static void cmsdk_timer_setcallback(FAR struct timer_lowerhalf_s *lower_,
                                    CODE tccb_t callback, FAR void *arg)
{
  FAR struct cmsdk_timer_lowerhalf_s *lower = (FAR struct cmsdk_timer_lowerhalf_s *)lower_;

  irqstate_t flags = enter_critical_section();
  lower->callback = callback;
  lower->arg      = arg;
  leave_critical_section(flags);
}

static int cmsdk_timer_maxtimeout(FAR struct timer_lowerhalf_s *lower_,
                                  uint32_t *maxtimeout)
{
  FAR struct cmsdk_timer_lowerhalf_s *lower = (FAR struct cmsdk_timer_lowerhalf_s *)lower_;
  uint64_t maxtimeout64 = usec_from_count(UINT32_MAX, lower->freq);

  if (maxtimeout64 > UINT32_MAX)
    {
      *maxtimeout = UINT32_MAX;
    }
  else
    {
      *maxtimeout = maxtimeout64;
    }

  return 0;
}

static int cmsdk_timer_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct cmsdk_timer_lowerhalf_s *lower = arg;

  if (lower->callback && (lower->tim->CTRL & CMSDK_TIMER_CTRL_ENABLE))
    {
      uint32_t interval, next_interval;

      interval = usec_from_count(lower->tim->RELOAD, lower->freq);
      next_interval = interval;

      lower->next_interval = &next_interval;
      if (lower->callback(&next_interval, lower->arg))
        {
          if (next_interval && next_interval != interval)
            {
              uint32_t count;

              count = usec_to_count(next_interval, lower->freq);
              lower->tim->VALUE  = count;
              lower->tim->RELOAD = count;
            }
        }
      else
        {
          lower->tim->CTRL &= ~CMSDK_TIMER_CTRL_ENABLE;
        }
      lower->next_interval = NULL;
    }

  /* Clear interrupt  */
  lower->tim->INT_STATUS = CMSDK_TIMER_INT_STATUS_CLEAR;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct timer_lowerhalf_s *
cmsdk_timer_initialize(FAR const struct cmsdk_timer_config_s *config)
{
  FAR struct cmsdk_timer_lowerhalf_s *lower;

  lower = kmm_zalloc(sizeof(*lower));
  if (lower != NULL)
    {
      lower->tim = (FAR struct cmsdk_timer_s *)config->base;
      lower->freq = config->freq;
      lower->ops = &g_cmsdk_timer_ops;

      /* Disable timer and enable interrupt */

      lower->tim->CTRL = CMSDK_TIMER_CTRL_INT_ENABLE;
      lower->tim->INT_STATUS = CMSDK_TIMER_INT_STATUS_CLEAR;

      irq_attach(config->irq, cmsdk_timer_interrupt, lower);
      up_enable_irq(config->irq);

      if (config->minor >= 0)
        {
          char devname[32];

          sprintf(devname, "/dev/timer%d", config->minor);
          timer_register(devname, (FAR struct timer_lowerhalf_s *)lower);
        }
    }

  return (FAR struct timer_lowerhalf_s *)lower;
}
