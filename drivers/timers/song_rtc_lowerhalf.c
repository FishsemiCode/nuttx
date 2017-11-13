/****************************************************************************
 * drivers/timers/song_rtc_lowerhalf.c
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

#include <nuttx/irq.h>
#include <nuttx/timers/rtc.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

/* This is the private type for the RTC state. It must be cast compatible
 * with struct rtc_lowerhalf_s.
 */

struct song_rtc_lowerhalf_s
{
  /* This is the contained reference to the read-only, lower-half
   * operations vtable (which may lie in FLASH or ROM)
   */

  FAR const struct rtc_ops_s *ops;

  /* Data following is private to this driver and not visible outside of
   * this file.
   */

#ifdef CONFIG_RTC_ALARM
  rtc_alarm_callback_t cb;  /* Callback when the alarm expires */
  FAR void *priv;           /* Private argument to accompany callback */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int song_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                           FAR struct rtc_time *rtctime);
static int song_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                            FAR const struct rtc_time *rtctime);
static bool song_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower);

#ifdef CONFIG_RTC_ALARM
static int song_rtc_setalarm(FAR struct rtc_lowerhalf_s *lower_,
                             FAR const struct lower_setalarm_s *alarminfo);
static int song_rtc_setrelative(FAR struct rtc_lowerhalf_s *lower,
                                FAR const struct lower_setrelative_s *relinfo);
static int song_rtc_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
                                int alarmid);
static int song_rtc_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                            FAR struct lower_rdalarm_s *alarminfo);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct rtc_ops_s g_song_rtc_ops =
{
  .rdtime      = song_rtc_rdtime,
  .settime     = song_rtc_settime,
  .havesettime = song_rtc_havesettime,
#ifdef CONFIG_RTC_ALARM
  .setalarm    = song_rtc_setalarm,
  .setrelative = song_rtc_setrelative,
  .cancelalarm = song_rtc_cancelalarm,
  .rdalarm     = song_rtc_rdalarm,
#endif
};

static struct song_rtc_lowerhalf_s g_song_rtc_lowerhalf =
{
  .ops        = &g_song_rtc_ops,
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int song_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower,
                           FAR struct rtc_time *rtctime)
{
  return up_rtc_getdatetime((FAR struct tm *)rtctime);
}

static int song_rtc_settime(FAR struct rtc_lowerhalf_s *lower,
                            FAR const struct rtc_time *rtctime)
{
  struct timespec ts;

  ts.tv_sec  = mktime((FAR struct tm *)rtctime);
  ts.tv_nsec = 0;

  return up_rtc_settime(&ts);
}

static bool song_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower)
{
  return g_song_rtc->time_valid;
}

#ifdef CONFIG_RTC_ALARM

static int song_rtc_setalarm(FAR struct rtc_lowerhalf_s *lower_,
                             FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct song_rtc_lowerhalf_s *lower = (FAR struct song_rtc_lowerhalf_s *)lower_;
  FAR struct song_rtc_alarm_s *alarm = &g_song_rtc->alarm[CONFIG_RTC_SONG_ALARM_INDEX];
  time_t time = mktime((FAR struct tm *)&alarminfo->time);
  irqstate_t flags;

  flags = enter_critical_section();
  lower->cb         = alarminfo->cb;
  lower->priv       = alarminfo->priv;
  alarm->cnt_hi     = time;
  alarm->cnt_lo     = 0;
  alarm->int_update = 1;
  alarm->int_en     = 1;
  leave_critical_section(flags);

  return 0;
}

static int song_rtc_setrelative(FAR struct rtc_lowerhalf_s *lower,
                                FAR const struct lower_setrelative_s *relinfo)
{
  struct lower_setalarm_s alarminfo;
  time_t time;

  alarminfo.id   = relinfo->id;
  alarminfo.cb   = relinfo->cb;
  alarminfo.priv = relinfo->priv;

  time = up_rtc_time() + relinfo->reltime;
  gmtime_r(&time, (FAR struct tm *)&alarminfo.time);

  return song_rtc_setalarm(lower, &alarminfo);
}

static int song_rtc_cancelalarm(FAR struct rtc_lowerhalf_s *lower,
                                int alarmid)
{
  FAR struct song_rtc_alarm_s *alarm = &g_song_rtc->alarm[CONFIG_RTC_SONG_ALARM_INDEX];
  irqstate_t flags;

  flags = enter_critical_section();
  alarm->int_en     = 0;
  alarm->int_status = 1; /* Clear the request */
  leave_critical_section(flags);

  return 0;
}

static int song_rtc_rdalarm(FAR struct rtc_lowerhalf_s *lower,
                            FAR struct lower_rdalarm_s *alarminfo)
{
  FAR struct song_rtc_alarm_s *alarm = &g_song_rtc->alarm[CONFIG_RTC_SONG_ALARM_INDEX];
  uint32_t cnt_hi = alarm->cnt_hi;

  gmtime_r(&cnt_hi, (FAR struct tm *)alarminfo->time);

  return 0;
}

static int song_rtc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct song_rtc_lowerhalf_s *lower = arg;

  song_rtc_cancelalarm(arg, 0);
  if (lower->cb)
    {
      lower->cb(lower->priv, 0);
    }

  return 0;
}

#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

int up_rtc_irqinitialize(void)
{
#ifdef CONFIG_RTC_ALARM
  irq_attach(CONFIG_RTC_SONG_IRQ, song_rtc_interrupt, &g_song_rtc_lowerhalf);
  up_enable_irq(CONFIG_RTC_SONG_IRQ);
#endif

  return rtc_initialize(0, (FAR struct rtc_lowerhalf_s *)&g_song_rtc_lowerhalf);
}
