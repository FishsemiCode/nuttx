/****************************************************************************
 * arch/arm/src/song/banks_rpm.c
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
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/rpmsg_rtc.h>
#include <nuttx/timers/song_oneshot.h>

#include "song_addrenv.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_BANKS_RPM

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                 "ap"

#define _LOGBUF_BASE                ((uintptr_t)&_slog)
#define _LOGBUF_SIZE                ((uint32_t)&_logsize)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _slog;
extern uint32_t _logsize;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
  static const struct song_addrenv_s addrenv[] =
  {
    {.va = 0x00000000, .pa = 0xf9240000, .size = 0x00040000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  up_addrenv_initialize(addrenv);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(CPU_NAME_AP, (void *)_LOGBUF_BASE, _LOGBUF_SIZE);
#endif
}

void arm_timer_initialize(void)
{
  static const struct song_oneshot_config_s config =
  {
    .base       = 0xf9210000,
    .irq        = 18,
    .c1_max     = 2600,
    .c1_freq    = 26000000,
    .ctl_off    = 0x160,
    .calib_off  = 0x174,
    .c1_off     = 0x164,
    .c2_off     = 0x168,
    .spec_off   = 0x664,
    .intren_off = 0x150,
    .intrst_off = 0x154,
    .intr_bit   = 29,
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
  struct mbox_dev_s *mbox_ap, *mbox_rpm;

  static const struct song_mbox_config_s mbox_cfg_ap =
  {
    .base       = 0xf9000000,
    .set_off    = 0x40,
    .en_off     = UINT32_MAX,
    .en_bit     = UINT32_MAX,
    .src_en_off = UINT32_MAX,
    .sta_off    = UINT32_MAX,
    .chnl_count = 64,
    .irq        = -1,
  };

  static const struct song_mbox_config_s mbox_cfg_rpm =
  {
    .base       = 0xf9000000,
    .set_off    = UINT32_MAX,
    .en_off     = 0x324,
    .en_bit     = 16,
    .src_en_off = 0x324,
    .sta_off    = 0x328,
    .chnl_count = 16,
    .irq        = 27,
  };

  static struct rptun_rsc_s rptun_rsc_ap
              __attribute__ ((section (".resource_table"))) =
  {
    .rsc_tbl_hdr     =
    {
      .ver           = 1,
      .num           = 2,
    },
    .offset          =
    {
      offsetof(struct rptun_rsc_s, log_trace),
      offsetof(struct rptun_rsc_s, rpmsg_vdev),
    },
    .log_trace       =
    {
      .type          = RSC_TRACE,
      .da            = _LOGBUF_BASE,
      .len           = _LOGBUF_SIZE,
    },
    .rpmsg_vdev      =
    {
      .type          = RSC_VDEV,
      .id            = VIRTIO_ID_RPMSG,
      .dfeatures     = 1 << VIRTIO_RPMSG_F_NS
                     | 1 << VIRTIO_RPMSG_F_BIND
                     | 1 << VIRTIO_RPMSG_F_BUFSZ,
      .config_len    = 4,
      .num_of_vrings = 2,
    },
    .rpmsg_vring0    =
    {
      .align         = 8,
      .num           = 8,
    },
    .rpmsg_vring1    =
    {
      .align         = 8,
      .num           = 8,
    },
    .buf_size = 0xe0,
  };

  static const struct song_rptun_config_s rptun_cfg_ap =
  {
    .cpu_name    = CPU_NAME_AP,
    .role        = RPMSG_REMOTE,
    .ch_start_rx = -1,
    .ch_vring_rx = 15,
    .ch_start_tx = -1,
    .ch_vring_tx = 62,
    .rsc         =
    {
      .rsc_tab   = &rptun_rsc_ap.rsc_tbl_hdr,
      .size      = sizeof(rptun_rsc_ap),
    },
  };

  mbox_ap = song_mbox_initialize(&mbox_cfg_ap, 0);
  mbox_rpm = song_mbox_initialize(&mbox_cfg_rpm, 1);

  song_rptun_initialize(&rptun_cfg_ap, mbox_rpm, mbox_ap, 0);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#endif

#ifdef CONFIG_RTC_RPMSG
  up_rtc_set_lowerhalf(rpmsg_rtc_initialize(CPU_NAME_AP, 0));
#endif

#ifdef CONFIG_RPMSG_UART
# ifdef CONFIG_SERIAL_CONSOLE
  uart_rpmsg_init(0, 4096, false, CPU_NAME_AP, 0, false);
# else
  uart_rpmsg_init(0, 4096, true, CPU_NAME_AP, 0, false);
# endif
#endif

#ifdef CONFIG_FS_HOSTFS_RPMSG
  hostfs_rpmsg_init(CPU_NAME_AP);
#endif
}
#endif

void up_lateinitialize(void)
{
}

#endif
