/****************************************************************************
 * drivers/timers/song_timer.c
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

#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS

#  ifndef CONFIG_SCHED_TICKLESS_ALARM
#    error CONFIG_SCHED_TICKLESS_ALARM must be set to use CONFIG_SCHED_TICKLESS
#  endif

#  ifndef CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP
#    error CONFIG_SCHED_TICKLESS_LIMIT_MAX_SLEEP must be set to use CONFIG_SCHED_TICKLESS
#  endif

#endif

#ifdef CONFIG_ARCH_ARM
#  define song_timer_initialize arm_timer_initialize
#endif

#define SONG_TIMER_CTL        (CONFIG_TIMERS_SONG_BASE + B2C(CONFIG_TIMERS_SONG_CTL_OFFSET))
#define SONG_TIMER_CALIB      (CONFIG_TIMERS_SONG_BASE + B2C(CONFIG_TIMERS_SONG_CALIB_OFFSET))
#define SONG_TIMER_C1         (CONFIG_TIMERS_SONG_BASE + B2C(CONFIG_TIMERS_SONG_C1_OFFSET))
#define SONG_TIMER_C2         (CONFIG_TIMERS_SONG_BASE + B2C(CONFIG_TIMERS_SONG_C2_OFFSET))
#define SONG_TIMER_SPEC       (CONFIG_TIMERS_SONG_BASE + B2C(CONFIG_TIMERS_SONG_SPEC_OFFSET))
#define SONG_TIMER_INTREN     (CONFIG_TIMERS_SONG_BASE + B2C(CONFIG_TIMERS_SONG_INTREN_OFFSET))
#define SONG_TIMER_INTRST     (CONFIG_TIMERS_SONG_BASE + B2C(CONFIG_TIMERS_SONG_INTRST_OFFSET))

#define SONG_TIMER_RESET_BIT  0
#define SONG_TIMER_CALIB_BIT  0
#define SONG_TIMER_INTR_BIT   CONFIG_TIMERS_SONG_INTR_BIT

#define SONG_TIMER_IRQ        CONFIG_TIMERS_SONG_IRQ

#define SONG_TIMER_C1_MAX     CONFIG_TIMERS_SONG_C1_MAX
#define SONG_TIMER_C1_FREQ    CONFIG_TIMERS_SONG_C1_FREQ

/****************************************************************************
 * Private Data
 ****************************************************************************/

static uint64_t g_offset = UINT64_MAX;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline uint32_t song_timer_getreg(uintptr_t addr)
{
  return *((FAR volatile uint32_t *)addr);
}

static inline void song_timer_putreg(uintptr_t addr, uint32_t val)
{
   *((FAR volatile uint32_t *)addr) = val;
}

static inline bool song_timer_getbit(uintptr_t addr, uint32_t bit)
{
  return !!(song_timer_getreg(addr) & (1 << bit));
}

static inline void song_timer_putbit(uintptr_t addr, uint32_t bit, bool val)
{
  uint32_t temp;

  temp  = song_timer_getreg(addr);
  temp &= ~(1 << bit);
  temp |= val << bit;
  song_timer_putreg(addr, temp);
}

static bool song_timer_getintr(void)
{
  return song_timer_getbit(SONG_TIMER_INTRST, SONG_TIMER_INTR_BIT);
}

static void song_timer_clearintr(void)
{
  song_timer_putbit(SONG_TIMER_INTRST, SONG_TIMER_INTR_BIT, true);
}

static void song_timer_enableintr(void)
{
  song_timer_putbit(SONG_TIMER_INTREN, SONG_TIMER_INTR_BIT, true);
}

static void song_timer_disableintr(void)
{
  song_timer_putbit(SONG_TIMER_INTREN, SONG_TIMER_INTR_BIT, false);
  song_timer_clearintr(); /* Avoid the residual request */
}

static void song_timer_start(void)
{
  /* Start the counting if not yet */
  if (song_timer_getbit(SONG_TIMER_CTL, SONG_TIMER_RESET_BIT))
    {
      /* Start 32KHz clock calibration first */
      song_timer_putbit(SONG_TIMER_CALIB, SONG_TIMER_CALIB_BIT, true);

      /* Then release the reset signal */
      song_timer_putbit(SONG_TIMER_CTL, SONG_TIMER_RESET_BIT, false);
    }

  while (song_timer_getbit(SONG_TIMER_CALIB, SONG_TIMER_CALIB_BIT))
    ; /* Wait until the calibration finish */
}

static void song_timer_getcounter32(FAR uint32_t *c2, FAR uint32_t *c1)
{
  song_timer_start();
  do
    {
      *c1 = song_timer_getreg(SONG_TIMER_C1);
      *c2 = song_timer_getreg(SONG_TIMER_C2);
      *c1 = song_timer_getreg(SONG_TIMER_C1);
    }
  while (*c2 != song_timer_getreg(SONG_TIMER_C2));
}

static uint64_t song_timer_getcounter(void)
{
  uint64_t counter;
  uint32_t c1;
  uint32_t c2;

  song_timer_getcounter32(&c2, &c1);
  counter = (uint64_t)c2 * SONG_TIMER_C1_MAX + c1;

  /* Don't need sync since it's called very early */
  if (g_offset == UINT64_MAX)
    {
      g_offset = counter;
    }

  return counter - g_offset;
}

static uint64_t song_timer_getspeccounter(void)
{
  uint64_t counter;
  uint32_t spec;

  spec = song_timer_getreg(SONG_TIMER_SPEC);
  counter = (spec + 1ull) * SONG_TIMER_C1_MAX - 1;

  return counter - g_offset;
}

static uint64_t song_timer_getusec(void)
{
  return song_timer_getcounter() * USEC_PER_SEC / SONG_TIMER_C1_FREQ;
}

static void song_timer_gettime(FAR struct timespec *ts, bool spec)
{
  uint64_t counter;

  counter = spec ? song_timer_getspeccounter() : song_timer_getcounter();
  ts->tv_sec = counter / SONG_TIMER_C1_FREQ;

  counter -= (uint64_t)ts->tv_sec * SONG_TIMER_C1_FREQ;
  ts->tv_nsec = counter * NSEC_PER_SEC / SONG_TIMER_C1_FREQ;
}

static void song_timer_putspectime(FAR const struct timespec *ts)
{
  uint64_t counter = g_offset;

  counter += (uint64_t)ts->tv_sec * SONG_TIMER_C1_FREQ;
  counter += (uint64_t)ts->tv_nsec * SONG_TIMER_C1_FREQ / NSEC_PER_SEC;

  song_timer_putreg(SONG_TIMER_SPEC, counter / SONG_TIMER_C1_MAX);
}

static int song_timer_interrupt(int irq, FAR void *context, FAR void *arg)
{
  struct timespec ts;

  if (song_timer_getintr())
    {
#ifdef CONFIG_SCHED_TICKLESS
      up_alarm_cancel(&ts);
      sched_alarm_expiration(&ts);
#else
      song_timer_gettime(&ts, true);
      ts.tv_nsec += NSEC_PER_TICK;
      song_timer_putspectime(&ts);
      song_timer_clearintr();
      sched_process_timer();
#endif
    }

  return 0;
}

#ifdef CONFIG_SCHED_TICKLESS
static uint32_t song_timer_get_maxticks(void)
{
  uint64_t maxticks;

  maxticks  = UINT32_MAX;
  maxticks *= SONG_TIMER_C1_MAX;

  maxticks *= USEC_PER_SEC;
  maxticks /= SONG_TIMER_C1_FREQ;

  maxticks /= USEC_PER_TICK;
  if (maxticks > UINT32_MAX)
    {
      maxticks = UINT32_MAX;
    }

  return maxticks;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void song_timer_initialize(void)
{
  struct timespec ts;

  song_timer_disableintr();
  song_timer_gettime(&ts, false);

#ifdef CONFIG_SCHED_TICKLESS
  g_oneshot_maxticks = song_timer_get_maxticks();
#else
  ts.tv_nsec += NSEC_PER_TICK;
  song_timer_putspectime(&ts);
  song_timer_enableintr();
#endif

  irq_attach(SONG_TIMER_IRQ, song_timer_interrupt, NULL);
  up_enable_irq(SONG_TIMER_IRQ);
}

/****************************************************************************
 * Name: up_timer_gettime
 *
 * Description:
 *   Return the elapsed time since power-up (or, more correctly, since
 *   the archtecture-specific timer was initialized).  This function is
 *   functionally equivalent to:
 *
 *      int clock_gettime(clockid_t clockid, FAR struct timespec *ts);
 *
 *   when clockid is CLOCK_MONOTONIC.
 *
 *   This function provides the basis for reporting the current time and
 *   also is used to eliminate error build-up from small errors in interval
 *   time calculations.
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Provides the location in which to return the up-time.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   Called from the normal tasking context.  The implementation must
 *   provide whatever mutual exclusion is necessary for correct operation.
 *   This can include disabling interrupts in order to assure atomic register
 *   operations.
 *
 ****************************************************************************/

#ifdef CONFIG_CLOCK_TIMEKEEPING
int up_timer_getcounter(FAR uint64_t *cycles)
{
  *cycles = song_timer_getusec() / USEC_PER_TICK;
  return 0;
}

void up_timer_getmask(FAR uint64_t *mask)
{
  *mask = UINT32_MAX;
}
#elif defined(CONFIG_SCHED_TICKLESS)
int up_timer_gettime(FAR struct timespec *ts)
{
  song_timer_gettime(ts, false);
  return 0;
}
#endif

/****************************************************************************
 * Name: up_alarm_cancel
 *
 * Description:
 *   Cancel the alarm and return the time of cancellation of the alarm.
 *   These two steps need to be as nearly atomic as possible.
 *   sched_alarm_expiration() will not be called unless the alarm is
 *   restarted with up_alarm_start().
 *
 *   If, as a race condition, the alarm has already expired when this
 *   function is called, then time returned is the current time.
 *
 *   NOTE: This function may execute at a high rate with no timer running (as
 *   when pre-emption is enabled and disabled).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - Location to return the expiration time.  The current time should
 *        returned if the alarm is not active.  ts may be NULL in which
 *        case the time is not returned.
 *
 * Returned Value:
 *   Zero (OK) is returned on success.  A call to up_alarm_cancel() when
 *   the timer is not active should also return success; a negated errno
 *   value is returned on any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
int up_alarm_cancel(FAR struct timespec *ts)
{
  song_timer_disableintr();
  song_timer_gettime(ts, false);
  return 0;
}
#endif

/****************************************************************************
 * Name: up_alarm_start
 *
 * Description:
 *   Start the alarm.  sched_alarm_expiration() will be called when the
 *   alarm occurs (unless up_alaram_cancel is called to stop it).
 *
 *   Provided by platform-specific code and called from the RTOS base code.
 *
 * Input Parameters:
 *   ts - The time in the future at the alarm is expected to occur.  When
 *        the alarm occurs the timer logic will call sched_alarm_expiration().
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure.
 *
 * Assumptions:
 *   May be called from interrupt level handling or from the normal tasking
 *   level.  Interrupts may need to be disabled internally to assure
 *   non-reentrancy.
 *
 ****************************************************************************/

#ifdef CONFIG_SCHED_TICKLESS
int up_alarm_start(FAR const struct timespec *ts)
{
  song_timer_putspectime(ts);
  song_timer_enableintr();
  return 0;
}
#endif

/****************************************************************************
 * Name: up_mdelay and up_udelay
 *
 * Description:
 *   Some device drivers may require that the plaform-specific logic
 *   provide these timing loops for short delays.
 *
 ****************************************************************************/

void up_mdelay(unsigned int milliseconds)
{
  up_udelay(1000 * milliseconds);
}

void up_udelay(useconds_t microseconds)
{
  uint32_t start = song_timer_getusec();
  while (1)
    {
      uint32_t now = song_timer_getusec();
      if (now - start >= microseconds)
        {
          break; /* Reach the time interval */
        }
    }
}
