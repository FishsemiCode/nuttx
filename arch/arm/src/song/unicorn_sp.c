/****************************************************************************
 * arch/arm/src/song/unicorn_sp.c
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

#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/song_oneshot.h>
#include <nuttx/timers/song_rtc.h>

#include "up_arch.h"
#include "up_internal.h"
#include "song_addrenv.h"

#ifdef CONFIG_ARCH_CHIP_UNICORN_SP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                 "ap"
#define CPU_NAME_CP                 "cp"

#define PWR_AP_POR_CTL              0xb00400e0
#define PWR_CP_POR_CTL              0xb00400dc
#define SECURITY_CFG_0              0xb0150030

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct rtc_lowerhalf_s *g_rtc_lower;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
  static const struct song_addrenv_s addrenv[] =
  {
    {.va = 0x21000000, .pa = 0xc1000000, .size = 0x00100000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  /* SP <--shram0--> AP
   * SP <--shram1--> CP
   * SP should enable shram1 for IPC, (shram0 default enabled)
   * then boot AP & CP
   */

  putreg32(0x00010000, SECURITY_CFG_0);
  putreg32(0x00010000, PWR_AP_POR_CTL);
  putreg32(0x00010000, PWR_CP_POR_CTL);

  /* Set up addrenv */

  up_addrenv_initialize(addrenv);
}

int up_rtc_initialize(void)
{
  static const struct song_rtc_config_s config =
  {
    .base  = 0xb2020000,
    .irq   = 16,
    .index = 2,
  };

  g_rtc_lower = song_rtc_initialize(&config);
  if (g_rtc_lower)
    {
      up_rtc_set_lowerhalf(g_rtc_lower);
    }

  return 0;
}

void arm_timer_initialize(void)
{
  static const struct song_oneshot_config_s config =
  {
    .base       = 0xb0040000,
    .irq        = 18,
    .c1_max     = 2048,
    .c1_freq    = 32768000,
    .ctl_off    = 0x170,
    .calib_off  = 0x194,
    .c1_off     = 0x174,
    .c2_off     = 0x178,
    .spec_off   = 0x1ac,
    .intren_off = 0x124,
    .intrst_off = 0x130,
    .intr_bit   = 2,
  };
  FAR struct oneshot_lowerhalf_s *lower;

  lower = song_oneshot_initialize(&config, -1);
  if (lower)
    {
      up_alarm_set_lowerhalf(lower);
    }
}

#ifdef CONFIG_OPENAMP
void up_openamp_initialize(void)
{
  struct mbox_dev_s *mbox_ap, *mbox_cp, *mbox_sp;

  static const struct song_mbox_config_s mbox_cfg_ap =
  {
    .base       = 0xb0030000,
    .set_off    = 0x10,
    .en_off     = 0x14,
    .en_bit     = 16,
    .src_en_off = 0x14,
    .sta_off    = 0x18,
    .chnl_count = 16,
    .irq        = -1,
  };

  static const struct song_mbox_config_s mbox_cfg_cp =
  {
    .base       = 0xb0030000,
    .set_off    = 0x0,
    .en_off     = 0x4,
    .en_bit     = 16,
    .src_en_off = 0x4,
    .sta_off    = 0x8,
    .chnl_count = 16,
    .irq        = -1,
  };

  static const struct song_mbox_config_s mbox_cfg_sp =
  {
    .base       = 0xb0030000,
    .set_off    = 0x20,
    .en_off     = 0x24,
    .en_bit     = 16,
    .src_en_off = 0x24,
    .sta_off    = 0x28,
    .chnl_count = 16,
    .irq        = 21,
  };

  static const struct song_rptun_config_s rptun_cfg_ap =
  {
    .cpu_name    = CPU_NAME_AP,
    .role        = RPMSG_MASTER,
    .ch_start_rx = 14,
    .ch_vring_rx = 15,
    .ch_start_tx = 14,
    .ch_vring_tx = 15,
    .rsc         =
    {
      .rsc_tab   = (void *)0xb0000000,
      .size      = sizeof(struct rptun_rsc_loadstart_s),
    },
  };

  static const struct song_rptun_config_s rptun_cfg_cp =
  {
    .cpu_name    = CPU_NAME_CP,
    .role        = RPMSG_MASTER,
    .ch_start_rx = 0,
    .ch_vring_rx = 1,
    .ch_start_tx = 0,
    .ch_vring_tx = 1,
    .rsc         =
    {
      .rsc_tab   = (void *)0xb0010000,
      .size      = sizeof(struct rptun_rsc_loadstart_s),
    },
  };

  mbox_ap = song_mbox_initialize(&mbox_cfg_ap, 0);
  mbox_cp = song_mbox_initialize(&mbox_cfg_cp, 1);
  mbox_sp = song_mbox_initialize(&mbox_cfg_sp, 2);

  song_rptun_initialize(&rptun_cfg_ap, mbox_sp, mbox_ap, 0);
  song_rptun_initialize(&rptun_cfg_cp, mbox_sp, mbox_cp, 1);

#ifdef CONFIG_SYSLOG_RPMSG_SERVER
  syslog_rpmsg_init();
#endif

#ifdef CONFIG_RPMSG_UART
  uart_rpmsg_init(0, 4096, false, NULL, 0, true);
  uart_rpmsg_init(1, 4096, false, NULL, 1, true);
#endif

#ifdef CONFIG_FS_HOSTFS_RPMSG_SERVER
  hostfs_rpmsg_server_init();
#endif
}
#endif

void up_lateinitialize(void)
{
  rtc_initialize(0, g_rtc_lower);
}

#endif
