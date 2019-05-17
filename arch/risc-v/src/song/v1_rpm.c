/****************************************************************************
 * arch/risc-v/src/song/v1_rpm.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
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

#ifdef CONFIG_ARCH_CHIP_V1_RPM

#include <nuttx/mbox/song_mbox.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/song_oneshot.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                 "ap"
#define CPU_INDEX_AP                0
#define CPU_INDEX_RPM               1

#define TOP_PWR_BASE                (0xf9210000)

#define TOP_MAILBOX_BASE            (0xf9000000)


#define _LOGBUF_BASE                ((uintptr_t)&_slog)
#define _LOGBUF_SIZE                ((uint32_t)&_logsize)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_SONG_MBOX
FAR struct mbox_dev_s *g_mbox[3] =
{
  [2] = DEV_END,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern uint32_t _slog;
extern uint32_t _logsize;

void riscv_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG
  static const struct song_oneshot_config_s config =
  {
    .minor      = -1,
    .base       = TOP_PWR_BASE,
    .irq        = 5,
    .c1_max     = 2600,
    .c1_freq    = 26000000,
    .ctl_off    = 0x160,
    .calib_off  = 0x174,
    .c1_off     = 0x164,
    .c2_off     = 0x168,
    .spec_off   = 0x208,
    .intren_off = 0x150,
    .intrst_off = 0x154,
    .intr_bit   = 26,
  };

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config));
#endif

}

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  uart_rpmsg_init(CPU_NAME_AP, "RPM", 1024, true);
}
#endif

#ifdef CONFIG_SONG_MBOX
static void up_mbox_init(void)
{
  static const struct song_mbox_config_s config[] =
  {
    {
      .index      = CPU_INDEX_AP,
      .base       = TOP_MAILBOX_BASE,
      .set_off    = 0x0,
      .en_off     = UINT32_MAX,
      .en_bit     = UINT32_MAX,
      .src_en_off = UINT32_MAX,
      .sta_off    = UINT32_MAX,
      .chnl_count = 64,
      .irq        = -1,
    },
    {
      .index      = CPU_INDEX_RPM,
      .base       = TOP_MAILBOX_BASE,
      .set_off    = UINT32_MAX,
      .en_off     = 0x34,
      .en_bit     = 16,
      .src_en_off = 0x34,
      .sta_off    = 0x38,
      .chnl_count = 16,
      .irq        = 21,
    }
  };

  song_mbox_allinitialize(config, ARRAY_SIZE(config), g_mbox);
}
#endif

#ifdef CONFIG_SONG_RPTUN
static void up_rptun_init(void)
{
  static struct rptun_rsc_s rptun_rsc_ap
    __attribute__ ((section(".resource_table"))) =
  {
    .rsc_tbl_hdr     =
    {
      .ver           = 1,
      .num           = 1,
    },
    .offset          =
    {
      offsetof(struct rptun_rsc_s, rpmsg_vdev),
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
    .cpuname = CPU_NAME_AP,
    .rsc     = &rptun_rsc_ap,
    .vringtx = 62,
    .vringrx = 15,
  };

  song_rptun_initialize(&rptun_cfg_ap, g_mbox[CPU_INDEX_AP], g_mbox[CPU_INDEX_RPM]);

#  ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize();
#  endif

#  ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#  endif

#  ifdef CONFIG_RTC_RPMSG
  up_rtc_set_lowerhalf(rpmsg_rtc_initialize(CPU_NAME_AP, 0));
#  endif

#  ifdef CONFIG_FS_HOSTFS_RPMSG
  hostfs_rpmsg_init(CPU_NAME_AP);
#  endif
}
#endif

void up_lateinitialize(void)
{

#ifdef CONFIG_SONG_MBOX
  up_mbox_init();
#endif

#ifdef CONFIG_SONG_RPTUN
  up_rptun_init();
#endif

}

#endif /* CONFIG_ARCH_CHIP_V1_RPM */
