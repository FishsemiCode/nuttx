/****************************************************************************
 * arch/ceva/src/song/u3_cpx.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Bo Zhang <zhangbo_a@pinecone.net>
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

#ifdef CONFIG_ARCH_CHIP_U3_CPX

#include <nuttx/drivers/addrenv.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/dw_timer.h>
#include <nuttx/timers/song_oneshot.h>

#include "up_arch.h"
#include "up_internal.h"

#define TOP_PWR_BASE                (0xb0040000)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
}

void ceva_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG
  static const struct song_oneshot_config_s config0 =
  {
      .minor      = -1,
      .base       = TOP_PWR_BASE,
      .irq        = IRQ_VINT23,
      .c1_max     = 480,
      .c1_freq    = 4800000,
      .ctl_off    = 0x170,
      .calib_off  = 0x194,
      .calib_inc  = 0x198,
      .c1_off     = 0x174,
      .c2_off     = 0x178,
      .spec_off   = 0x1a8,
      .intren_off = 0x12c,
      .intrst_off = 0x138,
      .intr_bit   = 1,
  };

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config0));
#endif

#if defined(CONFIG_CPULOAD_PERIOD) && defined(CONFIG_TIMER_DW)
  static const struct dw_timer_config_s config =
  {
    .minor      = -1,
    .base       = 0xb0080000,
    .irq        = IRQ_VINT1,
    .freq       = 1000000,
  };

  sched_period_extclk(dw_timer_initialize(&config));
#endif
}

void up_lateinitialize(void)
{
}

void up_finalinitialize(void)
{
}

void up_cpu_standby(void)
{
  up_cpu_idle();
}

void up_cpu_sleep(void)
{
  up_cpu_standby();
}

void up_cpu_normal(void)
{
}

#endif /* CONFIG_ARCH_CHIP_U3_CPX */
