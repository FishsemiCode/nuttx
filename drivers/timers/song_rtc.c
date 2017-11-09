/****************************************************************************
 * drivers/timers/song_rtc.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao
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

#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_rtc_alarm_s
{
  volatile uint32_t int_status;
  volatile uint32_t int_mask;
  volatile uint32_t int_en;
  volatile uint32_t cnt_lo;
  volatile uint32_t cnt_hi;
  volatile uint32_t int_update;
  volatile uint32_t reserved[10];
};

struct song_rtc_s
{
  volatile uint32_t set_cnt1;
  volatile uint32_t set_cnt2;
  volatile uint32_t set_update;
  volatile uint32_t reserved0;
  volatile uint32_t set_cyc;
  volatile uint32_t set_clk32k_adj;
  volatile uint32_t cali_update;
  volatile uint32_t time_valid;
  volatile uint32_t rtc_ctl;
  volatile uint32_t reserved1[7];
  struct song_rtc_alarm_s alarm[];
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct song_rtc_s * const g_song_rtc
  = (FAR struct song_rtc_s *)CONFIG_RTC_SONG_BASE;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* Variable determines the state of the RTC module.
 *
 * After initialization value is set to 'true' if RTC starts successfully.
 * The value can be changed to false also during operation if RTC for
 * some reason fails.
 */

volatile bool g_rtc_enabled = false;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t song_rtc_nsec2cnt1(uint32_t nsec)
{
  uint32_t usec;

  usec = nsec / NSEC_PER_USEC;
  return 1024 * usec / USEC_PER_SEC;
}

static uint32_t song_rtc_cnt12nsec(uint32_t cnt1)
{
  uint32_t usec;

  usec = USEC_PER_SEC * cnt1 / 1024;
  return NSEC_PER_USEC * usec;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_rtc_initialize
 *
 * Description:
 *   Initialize the builtin, MCU hardware RTC per the selected
 *   configuration.  This function is called once very early in the OS
 *   initialization sequence.
 *
 *   NOTE that initialization of external RTC hardware that depends on the
 *   availability of OS resources (such as SPI or I2C) must be deferred
 *   until the system has fully booted.  Other, RTC-specific initialization
 *   functions are used in that case.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ****************************************************************************/

int up_rtc_initialize(void)
{
  /* Nothing to do since RTC auto start after reset */
  g_rtc_enabled = true;
  return 0;
}

/************************************************************************************
 * Name: up_rtc_time
 *
 * Description:
 *   Get the current time in seconds.  This is similar to the standard time()
 *   function.  This interface is only required if the low-resolution RTC/counter
 *   hardware implementation selected.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC is set but neither
 *   CONFIG_RTC_HIRES nor CONFIG_RTC_DATETIME are set.
 *
 * Input Parameters:
 *   None
 *
 * Returned Value:
 *   The current time in seconds.
 *
 ************************************************************************************/

time_t up_rtc_time(void)
{
  return g_song_rtc->set_cnt2;
}

/************************************************************************************
 * Name: up_rtc_getdatetime
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: Some date/time RTC hardware is capability of sub-second accuracy.  That
 *   sub-second accuracy is lost in this interface.  However, since the system time
 *   is reinitialized on each power-up/reset, there will be no timing inaccuracy in
 *   the long run.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

int up_rtc_getdatetime(FAR struct tm *tp)
{
  time_t time;

  time = up_rtc_time();
  gmtime_r(&time, tp);

  return 0;
}

/************************************************************************************
 * Name: up_rtc_getdatetime_with_subseconds
 *
 * Description:
 *   Get the current date and time from the date/time RTC.  This interface
 *   is only supported by the date/time RTC hardware implementation.
 *   It is used to replace the system timer.  It is only used by the RTOS during
 *   initialization to set up the system time when CONFIG_RTC and CONFIG_RTC_DATETIME
 *   are selected (and CONFIG_RTC_HIRES is not).
 *
 *   NOTE: This interface exposes sub-second accuracy capability of RTC hardware.
 *   This interface allow maintaining timing accuracy when system time needs constant
 *   resynchronization with RTC, for example on MCU with low-power state that
 *   stop system timer.
 *
 * Input Parameters:
 *   tp - The location to return the high resolution time value.
 *   nsec - The location to return the subsecond time value.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno on failure
 *
 ************************************************************************************/

int up_rtc_getdatetime_with_subseconds(FAR struct tm *tp, FAR long *nsec)
{
  uint32_t cnt1;
  uint32_t cnt2;

  do
    {
      cnt2 = g_song_rtc->set_cnt2;
      cnt1 = g_song_rtc->set_cnt1;
    }
  while (cnt2 != g_song_rtc->set_cnt2);

  *nsec = song_rtc_cnt12nsec(cnt1);
  gmtime_r(&cnt2, tp);

  return 0;
}

/************************************************************************************
 * Name: up_rtc_settime
 *
 * Description:
 *   Set the RTC to the provided time.  All RTC implementations must be able to
 *   set their time based on a standard timespec.
 *
 * Input Parameters:
 *   tp - the time to use
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ************************************************************************************/

int up_rtc_settime(FAR const struct timespec *tp)
{
  g_song_rtc->set_cnt1   = song_rtc_nsec2cnt1(tp->tv_nsec);
  g_song_rtc->set_cnt2   = tp->tv_sec;
  g_song_rtc->set_update = 1; /* Trigger the update */
  g_song_rtc->time_valid = 1; /* Mark the change */

  return 0;
}

#ifdef CONFIG_RTC_DRIVER
#include "song_rtc_lowerhalf.c"
#endif
