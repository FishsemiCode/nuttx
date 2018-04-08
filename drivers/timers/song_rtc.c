/****************************************************************************
 * drivers/timers/song_rtc.c
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

#include <errno.h>

#include <nuttx/kmalloc.h>
#include <nuttx/timers/song_rtc.h>

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct song_rtc_alarm_s
{
  volatile uint32_t INT_STATUS;
  volatile uint32_t INT_MASK;
  volatile uint32_t INT_EN;
  volatile uint32_t CNT_LO;
  volatile uint32_t CNT_HI;
  volatile uint32_t INT_UPDATE;
  volatile uint32_t RESERVED[10];
};

struct song_rtc_s
{
  volatile uint32_t SET_CNT1;
  volatile uint32_t SET_CNT2;
  volatile uint32_t SET_UPDATE;
  volatile uint32_t RESERVED0;
  volatile uint32_t SET_CYC;
  volatile uint32_t SET_CLK32K_ADJ;
  volatile uint32_t CALI_UPDATE;
  volatile uint32_t VERSION;
  volatile uint32_t RTC_CTL;
  volatile uint32_t USER_DEFINED;
  volatile uint32_t RESERVED1[6];
  struct song_rtc_alarm_s ALARM[];
};

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

  FAR const struct song_rtc_config_s *config;

#ifdef CONFIG_RTC_ALARM
  rtc_alarm_callback_t cb;  /* Callback when the alarm expires */
  FAR void *priv;           /* Private argument to accompany callback */
#endif
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static int song_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower_,
                           FAR struct rtc_time *rtctime);
static int song_rtc_settime(FAR struct rtc_lowerhalf_s *lower_,
                            FAR const struct rtc_time *rtctime);
static bool song_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower_);

#ifdef CONFIG_RTC_ALARM
static int song_rtc_setalarm(FAR struct rtc_lowerhalf_s *lower_,
                             FAR const struct lower_setalarm_s *alarminfo);
static int song_rtc_setrelative(FAR struct rtc_lowerhalf_s *lower,
                                FAR const struct lower_setrelative_s *relinfo);
static int song_rtc_cancelalarm(FAR struct rtc_lowerhalf_s *lower_,
                                int alarmid);
static int song_rtc_rdalarm(FAR struct rtc_lowerhalf_s *lower_,
                            FAR struct lower_rdalarm_s *alarminfo);
#endif

#ifdef CONFIG_RTC_IOCTL
static int song_rtc_ioctl(FAR struct rtc_lowerhalf_s *lower, int cmd,
                          unsigned long arg);
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
#ifdef CONFIG_RTC_IOCTL
  .ioctl       = song_rtc_ioctl,
#endif
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t song_rtc_nsec2cnt(uint32_t nsec)
{
  uint32_t usec;

  usec = nsec / NSEC_PER_USEC;
  return 1024 * usec / USEC_PER_SEC;
}

static uint32_t song_rtc_cnt2nsec(uint32_t cnt)
{
  uint32_t usec;

  usec = USEC_PER_SEC * cnt / 1024;
  return NSEC_PER_USEC * usec;
}

static int song_rtc_rdtime(FAR struct rtc_lowerhalf_s *lower_,
                           FAR struct rtc_time *rtctime)
{
  FAR struct song_rtc_lowerhalf_s *lower = (FAR struct song_rtc_lowerhalf_s *)lower_;
  FAR struct song_rtc_s *base = (FAR struct song_rtc_s *)lower->config->base;
  uint32_t cnt2, cnt1;

  do
    {
      cnt2 = base->SET_CNT2;
      cnt1 = base->SET_CNT1;
    }
  while (cnt2 != base->SET_CNT2);

  gmtime_r(&cnt2, (FAR struct tm *)rtctime);
  rtctime->tm_nsec = song_rtc_cnt2nsec(cnt1);

  return 0;
}

static int song_rtc_settime(FAR struct rtc_lowerhalf_s *lower_,
                            FAR const struct rtc_time *rtctime)
{
  FAR struct song_rtc_lowerhalf_s *lower = (FAR struct song_rtc_lowerhalf_s *)lower_;
  FAR struct song_rtc_s *base = (FAR struct song_rtc_s *)lower->config->base;
  uint32_t cnt2, cnt1;
  irqstate_t flags;

  cnt2 = mktime((FAR struct tm *)rtctime);
  cnt1 = song_rtc_nsec2cnt(rtctime->tm_nsec);

  flags = enter_critical_section();
  base->SET_CNT2     = cnt2;
  base->SET_CNT1     = cnt1;
  base->SET_UPDATE   = 1; /* Trigger the update */
  base->USER_DEFINED = 1; /* Mark the change */
  leave_critical_section(flags);

  return 0;
}

static bool song_rtc_havesettime(FAR struct rtc_lowerhalf_s *lower_)
{
  FAR struct song_rtc_lowerhalf_s *lower = (FAR struct song_rtc_lowerhalf_s *)lower_;
  FAR struct song_rtc_s *base = (FAR struct song_rtc_s *)lower->config->base;

  return base->USER_DEFINED != 0;
}

#ifdef CONFIG_RTC_ALARM
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

static int song_rtc_setalarm(FAR struct rtc_lowerhalf_s *lower_,
                             FAR const struct lower_setalarm_s *alarminfo)
{
  FAR struct song_rtc_lowerhalf_s *lower = (FAR struct song_rtc_lowerhalf_s *)lower_;
  FAR struct song_rtc_s *base = (FAR struct song_rtc_s *)lower->config->base;
  FAR struct song_rtc_alarm_s *alarm = &base->ALARM[lower->config->index];
  uint32_t cnt_hi, cnt_lo;
  irqstate_t flags;
  bool first_alarm;

  cnt_hi = mktime((FAR struct tm *)&alarminfo->time);
  cnt_lo = song_rtc_nsec2cnt(alarminfo->time.tm_nsec);

  flags = enter_critical_section();
  first_alarm       = !!lower->cb;
  lower->cb         = alarminfo->cb;
  lower->priv       = alarminfo->priv;
  alarm->CNT_HI     = cnt_hi;
  alarm->CNT_LO     = cnt_lo;
  alarm->INT_UPDATE = 1; /* Trigger the update */
  alarm->INT_EN     = 1; /* Then enable interrupt */
  leave_critical_section(flags);

  if (first_alarm)
    {
      irq_attach(lower->config->irq, song_rtc_interrupt, lower);
      up_enable_irq(lower->config->irq);
    }

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

  song_rtc_rdtime(lower, &alarminfo.time);
  time = mktime((FAR struct tm *)&alarminfo.time);
  time = time + relinfo->reltime;
  gmtime_r(&time, (FAR struct tm *)&alarminfo.time);

  return song_rtc_setalarm(lower, &alarminfo);
}

static int song_rtc_cancelalarm(FAR struct rtc_lowerhalf_s *lower_,
                                int alarmid)
{
  FAR struct song_rtc_lowerhalf_s *lower = (FAR struct song_rtc_lowerhalf_s *)lower_;
  FAR struct song_rtc_s *base = (FAR struct song_rtc_s *)lower->config->base;
  FAR struct song_rtc_alarm_s *alarm = &base->ALARM[lower->config->index];
  irqstate_t flags;

  flags = enter_critical_section();
  alarm->INT_EN     = 0; /* Disable interrupt first */
  alarm->INT_STATUS = 1; /* Clear the request */
  leave_critical_section(flags);

  return 0;
}

static int song_rtc_rdalarm(FAR struct rtc_lowerhalf_s *lower_,
                            FAR struct lower_rdalarm_s *alarminfo)
{
  FAR struct song_rtc_lowerhalf_s *lower = (FAR struct song_rtc_lowerhalf_s *)lower_;
  FAR struct song_rtc_s *base = (FAR struct song_rtc_s *)lower->config->base;
  FAR struct song_rtc_alarm_s *alarm = &base->ALARM[lower->config->index];
  uint32_t cnt_hi, cnt_lo;
  irqstate_t flags;

  flags = enter_critical_section();
  cnt_hi = alarm->CNT_HI;
  cnt_lo = alarm->CNT_LO;
  leave_critical_section(flags);

  gmtime_r(&cnt_hi, (FAR struct tm *)alarminfo->time);
  alarminfo->time->tm_nsec = song_rtc_cnt2nsec(cnt_lo);

  return 0;
}
#endif

#ifdef CONFIG_RTC_IOCTL
static int song_rtc_ioctl(FAR struct rtc_lowerhalf_s *lower, int cmd,
                          unsigned long arg)
{
  if (cmd == _RTCIOC(RTC_USER_IOCBASE))
    {
      *((FAR void **)(uintptr_t)arg) = lower;
      return 0;
    }

  return -ENOTTY;
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

FAR struct rtc_lowerhalf_s *song_rtc_initialize(FAR const struct song_rtc_config_s *config)
{
  FAR struct song_rtc_lowerhalf_s *lower;

  lower = kmm_zalloc(sizeof(*lower));
  if (lower != NULL)
    {
      lower->config = config;
      lower->ops = &g_song_rtc_ops;
    }

  return (FAR struct rtc_lowerhalf_s *)lower;
}
