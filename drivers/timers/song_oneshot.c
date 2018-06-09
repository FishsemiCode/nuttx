/****************************************************************************
 * drivers/timers/song_oneshot.c
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
#include <nuttx/power/pm.h>
#include <nuttx/timers/song_oneshot.h>

#include <stdio.h>
#include <string.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SONG_ONESHOT_RESET_BIT    0
#define SONG_ONESHOT_C1_MAX_BIT   16
#define SONG_ONESHOT_C1_MAX_MASK  0xffff

#define SONG_ONESHOT_CALIB_BIT    0

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This structure describes the state of the oneshot timer lower-half driver */

struct song_oneshot_lowerhalf_s
{
  /* This is the part of the lower half driver that is visible to the upper-
   * half client of the driver.
   */

  FAR const struct oneshot_operations_s *ops;

  /* Private lower half data may follow */

  FAR const struct song_oneshot_config_s *config;
  uint64_t offset;
  uint32_t c1_max;
  uint32_t c2_prev;

  oneshot_callback_t callback;
  FAR void *arg;

#ifdef CONFIG_PM
  struct pm_callback_s pm_cb;
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int song_oneshot_max_delay(FAR struct oneshot_lowerhalf_s *lower_,
                                  FAR struct timespec *ts);
static int song_oneshot_start(FAR struct oneshot_lowerhalf_s *lower_,
                              oneshot_callback_t callback, FAR void *arg,
                              FAR const struct timespec *ts);
static int song_oneshot_cancel(FAR struct oneshot_lowerhalf_s *lower_,
                               FAR struct timespec *ts);
static int song_oneshot_current(FAR struct oneshot_lowerhalf_s *lower_,
                                FAR struct timespec *ts);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* Lower half operations */

static const struct oneshot_operations_s g_song_oneshot_ops =
{
  .max_delay = song_oneshot_max_delay,
  .start     = song_oneshot_start,
  .cancel    = song_oneshot_cancel,
  .current   = song_oneshot_current,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint64_t msec_from_count(uint64_t count, uint32_t freq)
{
  return count * MSEC_PER_SEC / freq;
}

static inline void timespec_from_count(FAR struct timespec *ts,
                                       uint64_t count, uint32_t freq)
{
  ts->tv_sec  = count / freq;
  count      -= (uint64_t)ts->tv_sec * freq;
  ts->tv_nsec = count * NSEC_PER_SEC / freq;
}

static inline uint32_t song_oneshot_getreg(uintptr_t base, uint32_t off)
{
  return *(FAR volatile uint32_t *)(base + B2C(off));
}

static inline void song_oneshot_putreg(uintptr_t base, uint32_t off, uint32_t val)
{
   *(FAR volatile uint32_t *)(base + B2C(off)) = val;
}

static inline uint32_t song_oneshot_getbits(uintptr_t base, uint32_t off,
                                           uint32_t bit, uint32_t mask)
{
  return (song_oneshot_getreg(base, off) >> bit) & mask;
}

static inline bool song_oneshot_getbit(uintptr_t base, uint32_t off, uint32_t bit)
{
  return song_oneshot_getbits(base, off, bit, 1);
}

static inline void song_oneshot_putbits(uintptr_t base, uint32_t off, uint32_t bit,
                                        uint32_t mask, uint32_t val)
{
  uint32_t temp;

  temp  = song_oneshot_getreg(base, off);
  temp &= ~(mask << bit);
  temp |= val << bit;
  song_oneshot_putreg(base, off, temp);
}

static inline void song_oneshot_putbit(uintptr_t base, uint32_t off, uint32_t bit, bool val)
{
  song_oneshot_putbits(base, off, bit, 1, val);
}

static inline bool song_oneshot_getintr(FAR struct song_oneshot_lowerhalf_s *lower)
{
  FAR const struct song_oneshot_config_s *config = lower->config;
  return song_oneshot_getbit(config->base, config->intrst_off, config->intr_bit);
}

static inline void song_oneshot_clearintr(FAR struct song_oneshot_lowerhalf_s *lower)
{
  FAR const struct song_oneshot_config_s *config = lower->config;
  song_oneshot_putbit(config->base, config->intrst_off, config->intr_bit, true);
}

static inline void song_oneshot_enableintr(FAR struct song_oneshot_lowerhalf_s *lower)
{
  FAR const struct song_oneshot_config_s *config = lower->config;
  song_oneshot_putbit(config->base, config->intren_off, config->intr_bit, true);
}

static inline void song_oneshot_disableintr(FAR struct song_oneshot_lowerhalf_s *lower)
{
  FAR const struct song_oneshot_config_s *config = lower->config;
  song_oneshot_putbit(config->base, config->intren_off, config->intr_bit, false);
  song_oneshot_clearintr(lower); /* Avoid the residual request */
}

static void song_oneshot_startcount(FAR struct song_oneshot_lowerhalf_s *lower)
{
  FAR const struct song_oneshot_config_s *config = lower->config;

  /* Start the counting if not yet */

  if (song_oneshot_getbit(config->base, config->ctl_off, SONG_ONESHOT_RESET_BIT))
    {
      /* Setup C1 period if caller provide one */

      if (config->c1_max)
        {
          song_oneshot_putbits(config->base, config->ctl_off,
            SONG_ONESHOT_C1_MAX_BIT, SONG_ONESHOT_C1_MAX_MASK, config->c1_max);
        }

      /* Start 32KHz clock calibration first */

      song_oneshot_putbit(config->base, config->calib_off, SONG_ONESHOT_CALIB_BIT, true);

      /* Then release the reset signal */

      song_oneshot_putbit(config->base, config->ctl_off, SONG_ONESHOT_RESET_BIT, false);
    }

  while (song_oneshot_getbit(config->base, config->calib_off, SONG_ONESHOT_CALIB_BIT))
    ; /* Wait until the calibration finish */

  /* Remember C1 period for later use */

  if (config->c1_max)
    {
      lower->c1_max = config->c1_max;
    }
  else
    {
      lower->c1_max = song_oneshot_getbits(config->base, config->ctl_off,
                         SONG_ONESHOT_C1_MAX_BIT, SONG_ONESHOT_C1_MAX_MASK);
    }
}

static void song_oneshot_getcount(FAR struct song_oneshot_lowerhalf_s *lower,
                                  FAR uint32_t *c2, FAR uint32_t *c1)
{
  FAR const struct song_oneshot_config_s *config = lower->config;

  song_oneshot_startcount(lower);
  do
    {
      *c1 = song_oneshot_getreg(config->base, config->c1_off);
      *c2 = song_oneshot_getreg(config->base, config->c2_off);
      *c1 = song_oneshot_getreg(config->base, config->c1_off);
    }
  while (*c2 != song_oneshot_getreg(config->base, config->c2_off));
}

static void song_oneshot_gettime(FAR struct song_oneshot_lowerhalf_s *lower,
                                 FAR struct timespec *ts)
{
  FAR const struct song_oneshot_config_s *config = lower->config;
  uint32_t c2, c1;
  uint64_t count;

  song_oneshot_getcount(lower, &c2, &c1);
  count = (uint64_t)c2 * lower->c1_max + c1;

  /* Don't need sync since it's called very early */
  if (lower->offset == UINT64_MAX)
    {
      lower->offset = UINT64_MAX - count + 1;
    }

  if (c2 < lower->c2_prev)
    {
      /* AT counter overflow. Increment counter base. */

      lower->offset += (UINT32_MAX + 1ull) * lower->c1_max;
    }

  lower->c2_prev = c2;
  timespec_from_count(ts, lower->offset + count, config->c1_freq);
}

static void song_oneshot_getspec(FAR struct song_oneshot_lowerhalf_s *lower,
                                 FAR struct timespec *ts)
{
  FAR const struct song_oneshot_config_s *config = lower->config;
  uint64_t count;
  uint32_t spec;

  spec = song_oneshot_getreg(config->base, config->spec_off);
  count = (uint64_t)spec * lower->c1_max;
  timespec_from_count(ts, lower->offset + count, config->c1_freq);
}

static void song_oneshot_putspec(FAR struct song_oneshot_lowerhalf_s *lower,
                                 FAR const struct timespec *ts)
{
  FAR const struct song_oneshot_config_s *config = lower->config;
  uint64_t count;

  count = (uint64_t)ts->tv_sec * config->c1_freq;
  count += (uint64_t)ts->tv_nsec * config->c1_freq / NSEC_PER_SEC;
  count -= lower->offset;

  song_oneshot_putreg(config->base, config->spec_off, count / lower->c1_max + 1);
}

static int song_oneshot_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct song_oneshot_lowerhalf_s *lower = arg;

  if (song_oneshot_getintr(lower))
    {
      song_oneshot_disableintr(lower);
      if (lower->callback)
        {
          lower->callback(arg, lower->arg);
        }
    }

  return 0;
}

static int song_oneshot_max_delay(FAR struct oneshot_lowerhalf_s *lower_,
                                  FAR struct timespec *ts)
{
  FAR struct song_oneshot_lowerhalf_s *lower
    = (FAR struct song_oneshot_lowerhalf_s *)lower_;
  FAR const struct song_oneshot_config_s *config = lower->config;

  timespec_from_count(ts, (UINT32_MAX + 1ull) * lower->c1_max - 1, config->c1_freq);
  return 0;
}

static int song_oneshot_start(FAR struct oneshot_lowerhalf_s *lower_,
                              oneshot_callback_t callback, FAR void *arg,
                              FAR const struct timespec *ts)
{
  FAR struct song_oneshot_lowerhalf_s *lower
    = (FAR struct song_oneshot_lowerhalf_s *)lower_;
  struct timespec now, spec;
  irqstate_t flags;

  flags = enter_critical_section();
  lower->callback = callback;
  lower->arg = arg;

  song_oneshot_gettime(lower, &now);
  clock_timespec_add(&now, ts, &spec);
  song_oneshot_putspec(lower, &spec);
  song_oneshot_enableintr(lower);
  leave_critical_section(flags);

  return 0;
}

static int song_oneshot_cancel(FAR struct oneshot_lowerhalf_s *lower_,
                               FAR struct timespec *ts)
{
  FAR struct song_oneshot_lowerhalf_s *lower
    = (FAR struct song_oneshot_lowerhalf_s *)lower_;
  struct timespec now, spec;
  irqstate_t flags;

  flags = enter_critical_section();
  song_oneshot_disableintr(lower);
  song_oneshot_getspec(lower, &spec);
  song_oneshot_gettime(lower, &now);
  clock_timespec_subtract(&spec, &now, ts);
  leave_critical_section(flags);

  return 0;
}

static int song_oneshot_current(FAR struct oneshot_lowerhalf_s *lower_,
                                FAR struct timespec *ts)
{
  FAR struct song_oneshot_lowerhalf_s *lower =
    (FAR struct song_oneshot_lowerhalf_s *)lower_;

  song_oneshot_gettime(lower, ts);
  return 0;
}

#ifdef CONFIG_PM
static enum pm_state_e song_oneshot_pm_evaluate(FAR struct pm_callback_s *cb, int domain)
{
  FAR struct song_oneshot_lowerhalf_s *lower =
    container_of(cb, struct song_oneshot_lowerhalf_s, pm_cb);
  FAR const struct song_oneshot_config_s *config = lower->config;

  if (song_oneshot_getbit(config->base, config->intren_off, config->intr_bit))
    {
      uint32_t c2 = song_oneshot_getreg(config->base, config->c2_off);
      uint32_t spec = song_oneshot_getreg(config->base, config->spec_off);
      uint64_t count = 1ull * (spec - c2) * lower->c1_max;
      uint64_t ms = msec_from_count(count, config->c1_freq);

      if (ms >= CONFIG_ONESHOT_SONG_SLEEPENTER_THRESH)
        {
          return PM_SLEEP;
        }
      else if (ms >= CONFIG_ONESHOT_SONG_STANDBYENTER_THRESH)
        {
          return PM_STANDBY;
        }
      else if (ms >= CONFIG_ONESHOT_SONG_IDLEENTER_THRESH)
        {
          return PM_IDLE;
        }
      else
        {
          return PM_NORMAL;
        }
    }
  /* There is no OS timer. */

  return PM_SLEEP;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct oneshot_lowerhalf_s *
song_oneshot_initialize(FAR const struct song_oneshot_config_s *config)
{
  FAR struct song_oneshot_lowerhalf_s *lower;

  lower = kmm_zalloc(sizeof(*lower));
  if (lower != NULL)
    {
      lower->config = config;
      lower->offset = UINT64_MAX;
      lower->ops = &g_song_oneshot_ops;
#ifdef CONFIG_PM
      lower->pm_cb.evaluate = song_oneshot_pm_evaluate;
      pm_register(&lower->pm_cb);
#endif

      song_oneshot_disableintr(lower);
      irq_attach(config->irq, song_oneshot_interrupt, lower);
      up_enable_irq(config->irq);

      if (config->minor >= 0)
        {
          char devname[32];

          sprintf(devname, "/dev/oneshot%d", config->minor);
          oneshot_register(devname, (FAR struct oneshot_lowerhalf_s *)lower);
        }
    }

  return (FAR struct oneshot_lowerhalf_s *)lower;
}
