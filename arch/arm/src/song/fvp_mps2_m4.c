/****************************************************************************
 * arch/arm/src/song/fvp_mps2_m4.c
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

#include <nuttx/clock.h>
#include <nuttx/net/lan91c111.h>
#include <nuttx/timers/arch_timer.h>
#include <nuttx/timers/cmsdk_timer.h>

#include "systick.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_FVP_MPS2_M4

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void arm_timer_initialize(void)
{
#ifdef CONFIG_TIMER_CMSDK
  static const struct cmsdk_timer_config_s config =
  {
    .base = 0x40000000,
    .irq  = 24,
    .freq = 25000000,
  };

  up_timer_set_lowerhalf(cmsdk_timer_initialize(&config, -1));
#endif

#ifdef CONFIG_CPULOAD_PERIOD
  sched_period_extclk(systick_initialize(false, 25000, -1));
#endif
}

#ifdef CONFIG_NETDEVICES
void up_netinitialize(void)
{
#  ifdef CONFIG_NET_LAN91C111
  lan91c111_initialize(0x40200000, 29);
#  endif
}
#endif

#endif /* CONFIG_ARCH_CHIP_FVP_MPS2_M4 */
