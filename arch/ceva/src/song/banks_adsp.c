/****************************************************************************
 * arch/ceva/src/song/banks_adsp.c
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

#include <string.h>

#include "song_addrenv.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_BANKS_ADSP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                 "ap"

#define LOGBUF_BASE                 ((uintptr_t)&_slog)

#define TOP_MAILBOX_BASE            (0xf9000000)

#define DDR_PWR_BASE                (0xf9210000)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _slog;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
  static const struct song_addrenv_s addrenv[] =
  {
    {.va = 0x00000000, .pa = 0xf8400000, .size = 0x00040000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  up_addrenv_initialize(addrenv);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(CPU_NAME_AP, (void *)LOGBUF_BASE, CONFIG_LOGBUF_SIZE);
#endif
}

void ceva_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG
  static const struct song_oneshot_config_s config0 =
  {
    .base       = B2C(DDR_PWR_BASE),
    .irq        = IRQ_INT3,
    .c1_max     = 2600,
    .c1_freq    = 26000000,
    .ctl_off    = 0x160,
    .calib_off  = 0x174,
    .c1_off     = 0x164,
    .c2_off     = 0x168,
    .spec_off   = 0x208,
    .intren_off = 0x480,
    .intrst_off = 0x484,
    .intr_bit   = 16,
  };

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config0, -1));

#  ifdef CONFIG_CPULOAD_ONESHOT
  static const struct song_oneshot_config_s config1 =
  {
    .base       = B2C(DDR_PWR_BASE),
    .irq        = IRQ_INT3,
    .c1_max     = 2600,
    .c1_freq    = 26000000,
    .ctl_off    = 0x160,
    .calib_off  = 0x174,
    .c1_off     = 0x164,
    .c2_off     = 0x168,
    .spec_off   = 0x660,
    .intren_off = 0x480,
    .intrst_off = 0x484,
    .intr_bit   = 28,
  };

  sched_oneshot_extclk(song_oneshot_initialize(&config1, -1));
#  endif

#endif
}

#ifdef CONFIG_OPENAMP
void up_openamp_initialize(void)
{
  struct mbox_dev_s *mbox_ap, *mbox_adsp;

  static const struct song_mbox_config_s mbox_cfg_ap =
  {
    .base       = B2C(TOP_MAILBOX_BASE),
    .set_off    = 0x40,
    .en_off     = 0x44,
    .en_bit     = 0,
    .src_en_off = 0x48,
    .sta_off    = 0x50,
    .chnl_count = 64,
    .irq        = -1,
  };

  static const struct song_mbox_config_s mbox_cfg_adsp =
  {
    .base       = B2C(TOP_MAILBOX_BASE),
    .set_off    = 0x70,
    .en_off     = 0x74,
    .en_bit     = 16,
    .src_en_off = 0x74,
    .sta_off    = 0x78,
    .chnl_count = 16,
    .irq        = IRQ_INT1,
  };

  static struct rptun_rsc_s rptun_rsc_ap
    __attribute__ ((section(".DSECT resource_table"))) =
  {
    .rsc_tbl_hdr     =
    {
      .ver           = 1,
      .num           = 2,
    },
    .offset          =
    {
      C2B(offsetof(struct rptun_rsc_s, log_trace)),
      C2B(offsetof(struct rptun_rsc_s, rpmsg_vdev)),
    },
    .log_trace       =
    {
      .type          = RSC_TRACE,
      .da            = LOGBUF_BASE,
      .len           = C2B(CONFIG_LOGBUF_SIZE),
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
    .buf_size        = 0xe0,
  };

  static const struct song_rptun_config_s rptun_cfg_ap =
  {
    .cpu_name    = CPU_NAME_AP,
    .role        = RPMSG_REMOTE,
    .ch_start_rx = -1,
    .ch_vring_rx = 15,
    .ch_start_tx = -1,
    .ch_vring_tx = 63,
    .rsc         =
    {
      .rsc_tab   = &rptun_rsc_ap.rsc_tbl_hdr,
      .size      = sizeof(rptun_rsc_ap),
    },
  };

  mbox_ap = song_mbox_initialize(&mbox_cfg_ap, 0);
  mbox_adsp = song_mbox_initialize(&mbox_cfg_adsp, 1);

  song_rptun_initialize(&rptun_cfg_ap, mbox_adsp, mbox_ap);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#endif

#ifdef CONFIG_RTC_RPMSG
  up_rtc_set_lowerhalf(rpmsg_rtc_initialize(CPU_NAME_AP, 0));
#endif

#ifdef CONFIG_RPMSG_UART
# ifdef CONFIG_SERIAL_CONSOLE
  uart_rpmsg_init(CPU_NAME_AP, "ADSP", B2C(1024), false);
# else
  uart_rpmsg_init(CPU_NAME_AP, "ADSP", B2C(1024), true);
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

void up_cpu_standby(void)
{
  up_cpu_idle();
}

void up_cpu_sleep(void)
{
  up_cpu_standby();
}

#endif /* CONFIG_ARCH_CHIP_BANKS_ADSP */
