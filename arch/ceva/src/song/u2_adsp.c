/****************************************************************************
 * arch/ceva/src/song/u2_adsp.c
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

#include <nuttx/irq.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/song_oneshot.h>

#include <string.h>

#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_U2_ADSP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TOP_PWR_BASE                (0xa00e0000)

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
    .base       = B2C(TOP_PWR_BASE),
    .irq        = IRQ_VINT_FIRST + 3, /* VINT3 */
    .c1_freq    = 19200000, /* 19.2Mhz */
    .ctl_off    = 0x290, /* TOP_PWR_AT_CTL */
    .calib_off  = 0x2b4, /* TOP_PWR_AT_CALIB_CTL */
    .c1_off     = 0x294, /* TOP_PWR_AT_C1 */
    .c2_off     = 0x298, /* TOP_PWR_AT_C2 */
    .spec_off   = 0x2c4, /* TOP_PWR_AT_SPEC_TIM0 */
    .intren_off = 0x218, /* TOP_PWR_INTR_EN_TL421 */
    .intrst_off = 0x220, /* TOP_PWR_INTR_ST_TL421 */
    .intr_bit   = 0,     /* TOP_PWR_AT_SPEC_TIM0_BIT */
  };

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config0, -1));

#  ifdef CONFIG_CPULOAD_ONESHOT
  static const struct song_oneshot_config_s config1 =
  {
    .base       = B2C(TOP_PWR_BASE),
    .irq        = IRQ_VINT_FIRST + 3, /* VINT3 */
    .c1_freq    = 19200000, /* 19.2Mhz */
    .ctl_off    = 0x290, /* TOP_PWR_AT_CTL */
    .calib_off  = 0x2b4, /* TOP_PWR_AT_CALIB_CTL */
    .c1_off     = 0x294, /* TOP_PWR_AT_C1 */
    .c2_off     = 0x298, /* TOP_PWR_AT_C2 */
    .spec_off   = 0x2c8, /* TOP_PWR_AT_SPEC_TIM1 */
    .intren_off = 0x218, /* TOP_PWR_INTR_EN_TL421 */
    .intrst_off = 0x220, /* TOP_PWR_INTR_ST_TL421 */
    .intr_bit   = 1,     /* TOP_PWR_AT_SPEC_TIM1_BIT */
  };

  sched_oneshot_extclk(song_oneshot_initialize(&config1, -1));
#  endif

#endif
}

void up_lateinitialize(void)
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

#endif /* CONFIG_ARCH_CHIP_U2_ADSP */
