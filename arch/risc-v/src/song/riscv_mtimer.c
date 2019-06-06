/****************************************************************************
 * arch/risc-v/src/song/riscv_mtimer.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
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

#include <stdio.h>

#include "chip.h"
#include "up_arch.h"
#include "riscv_mtimer.h"

#ifdef CONFIG_RISCV_MTIME

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define RISCV_TIMER_MINCOUNT                (4)

#define RISCV_MTIME_OFFSET                  0xbff8
#define RISCV_MTIMECMP_OFFSET               0x4000

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure provides the private representation of the "lower-half"
 * driver state structure.  This structure must be cast-compatible with the
 * timer_lowerhalf_s structure.
 */

struct riscv_mtimer_lowerhalf_s
{
  FAR const struct timer_ops_s  *ops;        /* Lower half operations */
  uintptr_t                     base;
  uint32_t                      load_count;
  uint32_t                      freq;        /* Timer working clock frequency(Hz) */
  tccb_t                        callback;    /* Current user interrupt callback */
  FAR void                      *arg;        /* Argument passed to upper half callback */
  uint32_t                      *next_interval;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int riscv_mtimer_start(FAR struct timer_lowerhalf_s *lower_);
static int riscv_mtimer_stop(FAR struct timer_lowerhalf_s *lower_);
static int riscv_mtimer_getstatus(FAR struct timer_lowerhalf_s *lower_,
                              FAR struct timer_status_s *status);
static int riscv_mtimer_settimeout(FAR struct timer_lowerhalf_s *lower_,
                               uint32_t timeout);
static void riscv_mtimer_setcallback(FAR struct timer_lowerhalf_s *lower_,
                                 tccb_t callback, FAR void *arg);
static int riscv_mtimer_maxtimeout(FAR struct timer_lowerhalf_s *lower_,
                               uint32_t *maxtimeout);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct timer_ops_s g_riscv_mtimer_ops =
{
  .start       = riscv_mtimer_start,
  .stop        = riscv_mtimer_stop,
  .getstatus   = riscv_mtimer_getstatus,
  .settimeout  = riscv_mtimer_settimeout,
  .setcallback = riscv_mtimer_setcallback,
  .maxtimeout  = riscv_mtimer_maxtimeout,
};

static struct riscv_mtimer_lowerhalf_s *mtimer_lower;

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
  return count < RISCV_TIMER_MINCOUNT ? RISCV_TIMER_MINCOUNT : count;
}

static inline bool riscv_mtimer_is_enabled(void)
{
  uint32_t mie = READ_CSR(mie);

  return !!(mie & (1 << 7));
}

static bool riscv_mtimer_irq_pending(struct riscv_mtimer_lowerhalf_s *lower)
{
  if (lower->next_interval)
    {
      return false; /* Interrupt is in process */
    }
  else
    {
      uint32_t mip = READ_CSR(mip);
      return !!(mip & (1 << 7));
    }
}

static uint64_t riscv_mtime_get(struct riscv_mtimer_lowerhalf_s *lower)
{
  uint32_t hi, lo;

  do {
      hi = getreg32(lower->base + RISCV_MTIME_OFFSET + 4);
      lo = getreg32(lower->base + RISCV_MTIME_OFFSET);
    } while (getreg32(lower->base + RISCV_MTIME_OFFSET + 4) != hi);

  return (((uint64_t)hi) << 32) | lo;
}

static uint64_t riscv_mtimecmp_get(struct riscv_mtimer_lowerhalf_s *lower)
{
  uint32_t hi, lo;

  hi = getreg32(lower->base + RISCV_MTIMECMP_OFFSET + 4);
  lo = getreg32(lower->base + RISCV_MTIMECMP_OFFSET);

  return (((uint64_t)hi) << 32) | lo;
}

static void riscv_mtimecmp_set(struct riscv_mtimer_lowerhalf_s *lower, uint64_t time)
{
  putreg32(0xffffffff, (lower->base + RISCV_MTIMECMP_OFFSET + 4));
  putreg32((uint32_t)time, (lower->base + RISCV_MTIMECMP_OFFSET));
  putreg32((uint32_t)(time >> 32), (lower->base + RISCV_MTIMECMP_OFFSET + 4));
}

static int riscv_mtimer_start(FAR struct timer_lowerhalf_s *lower_)
{
  SET_CSR(mie, (1 << 7));
  return 0;
}

static int riscv_mtimer_stop(struct timer_lowerhalf_s *lower_)
{
  CLEAR_CSR(mie, (1 << 7));
  return 0;
}

static int riscv_mtimer_getstatus(FAR struct timer_lowerhalf_s *lower_,
                              FAR struct timer_status_s *status)
{
  FAR struct riscv_mtimer_lowerhalf_s *lower = (FAR struct riscv_mtimer_lowerhalf_s *)lower_;
  uint32_t load;
  uint32_t current;

  load = lower->load_count;
  current = (uint32_t)(riscv_mtimecmp_get(lower) - riscv_mtime_get(lower));

  status->flags  = lower->callback != NULL ? TCFLAGS_HANDLER : 0;
  status->flags |= riscv_mtimer_is_enabled() ? TCFLAGS_ACTIVE : 0;
  status->timeout = usec_from_count(load, lower->freq);
  status->timeleft = usec_from_count(current, lower->freq);

  if (riscv_mtimer_irq_pending(lower))
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

static int riscv_mtimer_settimeout(FAR struct timer_lowerhalf_s *lower_,
                               uint32_t timeout)
{
  FAR struct riscv_mtimer_lowerhalf_s *lower = (FAR struct riscv_mtimer_lowerhalf_s *)lower_;
  uint32_t load_count = usec_to_count(timeout, lower->freq);

  if (lower->next_interval)
    {
      /* If the timer callback is in the process,
       * delay the update to timer interrupt handler.
       */

      *lower->next_interval = timeout;
    }
  else
    {
      lower->load_count = load_count;
      CLEAR_CSR(mie, (1 << 7));
      riscv_mtimecmp_set(lower, (uint64_t)(riscv_mtime_get(lower) + lower->load_count));
      SET_CSR(mie, (1 << 7));
    }

  return 0;
}

static void riscv_mtimer_setcallback(FAR struct timer_lowerhalf_s *lower_,
                                 tccb_t callback, FAR void *arg)
{
  FAR struct riscv_mtimer_lowerhalf_s *lower = (FAR struct riscv_mtimer_lowerhalf_s *)lower_;

  lower->arg      = arg;
  lower->callback = callback;
}

static int riscv_mtimer_maxtimeout(FAR struct timer_lowerhalf_s *lower_,
                               uint32_t *maxtimeout)
{
  FAR struct riscv_mtimer_lowerhalf_s *lower = (FAR struct riscv_mtimer_lowerhalf_s *)lower_;
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

int riscv_mtimer_interrupt(void)
{
  FAR struct riscv_mtimer_lowerhalf_s *lower = mtimer_lower;

  if (lower->callback && riscv_mtimer_is_enabled())
    {
      uint32_t interval, next_interval;

      interval = usec_from_count(lower->load_count, lower->freq);
      next_interval = interval;

      lower->next_interval = &next_interval;
      if (lower->callback(&next_interval, lower->arg))
        {
          if (next_interval)
            {
              uint32_t load_count;
              load_count = usec_to_count(next_interval, lower->freq);
              lower->load_count = load_count;

              CLEAR_CSR(mie, (1 << 7));
              riscv_mtimecmp_set(lower, (uint64_t)(riscv_mtime_get(lower) + lower->load_count));
              SET_CSR(mie, (1 << 7));

              lower->next_interval = NULL;

              return 0;
            }
        }

      lower->next_interval = NULL;
    }

  CLEAR_CSR(mie, (1 << 7));

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct timer_lowerhalf_s *
riscv_mtimer_initialize(FAR const struct riscv_mtimer_config_s *config)
{
  mtimer_lower = kmm_zalloc(sizeof(struct riscv_mtimer_lowerhalf_s));
  if (mtimer_lower != NULL)
    {
      mtimer_lower->base = config->base;
      mtimer_lower->freq = config->freq;
      mtimer_lower->ops = &g_riscv_mtimer_ops;

      if (config->minor >= 0)
        {
          char devname[32];

          sprintf(devname, "/dev/timer%d", config->minor);
          timer_register(devname, (FAR struct timer_lowerhalf_s *)mtimer_lower);
        }
    }

  return (FAR struct timer_lowerhalf_s *)mtimer_lower;
}
#endif
