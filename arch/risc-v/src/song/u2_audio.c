/****************************************************************************
 * arch/risc-v/src/song/u2_audio.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Wang Yanjiong <wangyanjiong@pinecone.net>
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

#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/song_oneshot.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_U2_AUDIO

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TOP_PWR_BASE                (0xa00e0000)
#define TOP_PWR_RCPU1_INTR2SLP_MK0  (TOP_PWR_BASE + 0x228)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_wic_initialize(void)
{
  putreg32(0xffffffff, TOP_PWR_RCPU1_INTR2SLP_MK0);
}

void up_wic_enable_irq(int irq)
{
  modifyreg32(TOP_PWR_RCPU1_INTR2SLP_MK0, 1 << irq, 0);
}

void up_wic_disable_irq(int irq)
{
  modifyreg32(TOP_PWR_RCPU1_INTR2SLP_MK0, 0, 1 << irq);
}

void riscv_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG
  static const struct song_oneshot_config_s config =
  {
    .minor      = -1,
    .base       = TOP_PWR_BASE,
    .irq        = 9,
    .c1_max     = 600,
    .c1_freq    = 6000000,
    .ctl_off    = 0x290,
    .calib_off  = 0x2b4,
    .c1_off     = 0x294,
    .c2_off     = 0x298,
    .spec_off   = 0x2c8,
    .intren_off = 0x218,
    .intrst_off = 0x220,
    .intr_bit   = 1,
  };

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config));
#endif
}

#endif /* CONFIG_ARCH_CHIP_U2_AUDIO */
