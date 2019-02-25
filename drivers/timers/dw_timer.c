/****************************************************************************
 * drivers/timers/dw_timer.c
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

#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/timers/dw_timer.h>

#include <stdio.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DW_TIMER_MINCOUNT             (4)

#define DW_TIMER_ENABLE               (1 << 0)
#define DW_TIMER_PERIODIC             (1 << 1)
#define DW_TIMER_INTMASK              (1 << 2)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dw_timer_s
{
  volatile uint32_t LOAD_COUNT;
  volatile uint32_t CURRENT_VALUE;
  volatile uint32_t CONTROL;
  volatile uint32_t EOI;
  volatile uint32_t INT_STATUS;
};

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct dw_timer_lowerhalf_s
{
  FAR const struct timer_ops_s *ops;        /* Lower half operations */
  FAR struct dw_timer_s        *tim;        /* Hardware timer register */
  uint32_t                      freq;       /* Timer working clock frequency(Hz) */
  tccb_t                        callback;   /* Current user interrupt callback */
  FAR void                     *arg;        /* Argument passed to upper half callback */
  uint32_t                     *next_interval;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int dw_timer_start(FAR struct timer_lowerhalf_s *lower_);
static int dw_timer_stop(FAR struct timer_lowerhalf_s *lower_);
static int dw_timer_getstatus(FAR struct timer_lowerhalf_s *lower_,
                              FAR struct timer_status_s *status);
static int dw_timer_settimeout(FAR struct timer_lowerhalf_s *lower_,
                               uint32_t timeout);
static void dw_timer_setcallback(FAR struct timer_lowerhalf_s *lower_,
                                 tccb_t callback, FAR void *arg);
static int dw_timer_maxtimeout(FAR struct timer_lowerhalf_s *lower_,
                               uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct timer_ops_s g_dw_timer_ops =
{
  .start       = dw_timer_start,
  .stop        = dw_timer_stop,
  .getstatus   = dw_timer_getstatus,
  .settimeout  = dw_timer_settimeout,
  .setcallback = dw_timer_setcallback,
  .ioctl       = NULL,
  .maxtimeout  = dw_timer_maxtimeout,
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
  uint64_t count = (uint64_t)usec * freq / USEC_PER_SEC;
  return count < DW_TIMER_MINCOUNT ? DW_TIMER_MINCOUNT : count;
}

static bool dw_timer_irq_pending(struct dw_timer_lowerhalf_s *lower)
{
  if (lower->next_interval)
    {
      return false; /* Interrupt is in process */
    }
  else
    {
      return !!lower->tim->INT_STATUS;
    }
}

static void dw_timer_clear_irq(struct dw_timer_lowerhalf_s* lower_)
{
  volatile uint32_t eoi = lower_->tim->EOI;
  UNUSED(eoi);
}

static int dw_timer_start(FAR struct timer_lowerhalf_s *lower_)
{
  FAR struct dw_timer_lowerhalf_s *lower = (FAR struct dw_timer_lowerhalf_s *)lower_;

  lower->tim->CONTROL |= DW_TIMER_ENABLE;
  return 0;
}

static int dw_timer_stop(struct timer_lowerhalf_s *lower_)
{
  FAR struct dw_timer_lowerhalf_s *lower = (FAR struct dw_timer_lowerhalf_s *)lower_;

  lower->tim->CONTROL &= ~DW_TIMER_ENABLE;
  dw_timer_clear_irq(lower); /* Read to clear interrupt */
  return 0;
}

static int dw_timer_getstatus(FAR struct timer_lowerhalf_s *lower_,
                              FAR struct timer_status_s *status)
{
  FAR struct dw_timer_lowerhalf_s *lower = (FAR struct dw_timer_lowerhalf_s *)lower_;
  uint32_t load, current;

  do
    {
      load = lower->tim->LOAD_COUNT;
      current = lower->tim->CURRENT_VALUE;
    }
  while (load != lower->tim->LOAD_COUNT);

  status->flags  = lower->callback != NULL ? TCFLAGS_HANDLER : 0;
  status->flags |= lower->tim->CONTROL & DW_TIMER_ENABLE ? TCFLAGS_ACTIVE : 0;
  status->timeout = usec_from_count(load, lower->freq);
  status->timeleft = usec_from_count(current, lower->freq);

  if (dw_timer_irq_pending(lower))
    {
      /* Interrupt is pending and the timer wrap happen? */
      if (status->timeleft)
        {
          /* Make timeout-timeleft equal the real elapsed time */
          status->timeout += status->timeout - status->timeleft;
          status->timeleft = 0;
        }
    }
  else if (status->timeleft == 0)
    {
      status->timeleft = status->timeout;
    }

  return 0;
}

static int dw_timer_settimeout(FAR struct timer_lowerhalf_s *lower_,
                               uint32_t timeout)
{
  FAR struct dw_timer_lowerhalf_s *lower = (FAR struct dw_timer_lowerhalf_s *)lower_;
  uint32_t load_count = usec_to_count(timeout, lower->freq);

  if (lower->next_interval)
    {
      /* If the timer callback is in the process,
       * delay the update to timer interrupt handler.
       */

      *lower->next_interval = timeout;
    }
  else if (lower->tim->CONTROL & DW_TIMER_ENABLE)
    {
      if (load_count != lower->tim->CURRENT_VALUE)
        {
          lower->tim->CONTROL &= ~DW_TIMER_ENABLE;
          lower->tim->LOAD_COUNT = load_count;
          lower->tim->CONTROL |= DW_TIMER_ENABLE;
        }
    }
  else
    {
      lower->tim->LOAD_COUNT = load_count;
    }

  return 0;
}

static void dw_timer_setcallback(FAR struct timer_lowerhalf_s *lower_,
                                 tccb_t callback, FAR void *arg)
{
  FAR struct dw_timer_lowerhalf_s *lower = (FAR struct dw_timer_lowerhalf_s *)lower_;

  lower->arg      = arg;
  lower->callback = callback;
}

static int dw_timer_maxtimeout(FAR struct timer_lowerhalf_s *lower_,
                               uint32_t *maxtimeout)
{
  FAR struct dw_timer_lowerhalf_s *lower = (FAR struct dw_timer_lowerhalf_s *)lower_;
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

static int dw_timer_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct dw_timer_lowerhalf_s *lower = arg;

  if (lower->callback && (lower->tim->CONTROL & DW_TIMER_ENABLE))
    {
      uint32_t interval, next_interval;

      interval = usec_from_count(lower->tim->LOAD_COUNT, lower->freq);
      next_interval = interval;

      lower->next_interval = &next_interval;
      if (lower->callback(&next_interval, lower->arg))
        {
          if (next_interval && next_interval != interval)
            {
              uint32_t load_count;

              load_count = usec_to_count(next_interval, lower->freq);
              lower->tim->CONTROL &= ~DW_TIMER_ENABLE;
              lower->tim->LOAD_COUNT = load_count;
              lower->tim->CONTROL |= DW_TIMER_ENABLE;
            }
        }
      else
        {
          lower->tim->CONTROL &= ~DW_TIMER_ENABLE;
        }
      lower->next_interval = NULL;
    }

  /* Clear interrupt by reading EOI */
  dw_timer_clear_irq(lower);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct timer_lowerhalf_s *
dw_timer_initialize(FAR const struct dw_timer_config_s *config)
{
  FAR struct dw_timer_lowerhalf_s *lower;

  lower = kmm_zalloc(sizeof(*lower));
  if (lower != NULL)
    {
      lower->tim = (FAR struct dw_timer_s *)config->base;
      lower->freq = config->freq;
      lower->ops = &g_dw_timer_ops;

      /* Disable timer, Enable interrupt and periodic mode */
      lower->tim->CONTROL = DW_TIMER_PERIODIC;
      dw_timer_clear_irq(lower); /* Read to clear interrupt */

      irq_attach(config->irq, dw_timer_interrupt, lower);
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
