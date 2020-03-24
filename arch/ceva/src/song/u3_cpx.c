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

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"
#include "psu.h"

/****************************************************************************
 * Pre-processor Definitions
  ****************************************************************************/

#define CPU_NAME_AP                     "ap"
#define CPU_NAME_CPR                    "cpr"
#define CPU_NAME_CPX                    "cpx"
#define CPU_INDEX_AP                    0
#define CPU_INDEX_CPR                   1
#define CPU_INDEX_CPX                   2

#define LOGBUF_BASE                     ((uintptr_t)&_slog)

#define TOP_MAILBOX_BASE                (0xb0030000)

#define TOP_PWR_BASE                    (0xb0040000)
#define TOP_PWR_SLPCTL_CP_XC5           (TOP_PWR_BASE + 0x360)

#define TOP_PWR_CP_XC5_SLP_EN           (1 << 0)    //TOP_PWR_SLPCTL_CP_XC5
#define TOP_PWR_CP_XC5_SLP_MK           (1 << 1)
#define TOP_PWR_CP_XC5_DS_SLP_EN        (1 << 2)

/****************************************************************************
 * Public Data
  ****************************************************************************/

extern uint32_t _slog;

#ifdef CONFIG_SONG_MBOX
  FAR struct mbox_dev_s *g_mbox[4] =
  {
      [3] = DEV_END,
  };
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
  static const struct simple_addrenv_s addrenv[] =
  {
    {.va = 0x40300000, .pa = 0xd0300000, .size = 0x00100000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  simple_addrenv_initialize(addrenv);

  putreg32(TOP_PWR_CP_XC5_SLP_EN << 16 |
           TOP_PWR_CP_XC5_SLP_MK << 16 |
           TOP_PWR_CP_XC5_DS_SLP_EN << 16, TOP_PWR_SLPCTL_CP_XC5);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(CPU_NAME_AP, (void *)LOGBUF_BASE, CONFIG_LOGBUF_SIZE);
#endif
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
}

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  uart_rpmsg_init(CPU_NAME_AP, "CPX", 256, true);
}
#endif

#ifdef CONFIG_SONG_RPTUN
static void up_rptun_init(void)
{
  static struct rptun_rsc_s rptun_rsc_ap
    __attribute__ ((section(".DSECT resource_table"))) =
    {
      .rsc_tbl_hdr     =
      {
        .ver           = 1,
        .num           = 1,
      },
      .offset          =
      {
        C2B(offsetof(struct rptun_rsc_s, rpmsg_vdev)),
      },
      .rpmsg_vdev      =
      {
        .type          = RSC_VDEV,
        .id            = VIRTIO_ID_RPMSG,
        .dfeatures     = 1 << VIRTIO_RPMSG_F_NS
                       | 1 << VIRTIO_RPMSG_F_BIND
                       | 1 << VIRTIO_RPMSG_F_BUFSZ,
        .num_of_vrings = 2,
      },
      .rpmsg_vring0    =
      {
        .align         = 0x8,
        .num           = 4,
      },
      .rpmsg_vring1    =
      {
        .align         = 0x8,
        .num           = 4,
      },
      .buf_size        = 0x100,
    };

  static const struct song_rptun_config_s rptun_cfg_ap =
  {
    .cpuname = CPU_NAME_AP,
    .rsc     = &rptun_rsc_ap,
    .vringtx = 15,
    .vringrx = 15,
  };

  song_rptun_initialize(&rptun_cfg_ap, g_mbox[CPU_INDEX_AP], g_mbox[CPU_INDEX_CPX]);

#  ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#  endif

#  ifdef CONFIG_FS_HOSTFS_RPMSG
  hostfs_rpmsg_init(CPU_NAME_AP);
#  endif
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
      .set_off    = 0x20,
      .en_off     = 0x24,
      .en_bit     = 16,
      .src_en_off = 0x24,
      .sta_off    = 0x28,
      .chnl_count = 16,
      .irq        = -1,
    },
    {
      .index      = CPU_INDEX_CPR,
      .base       = TOP_MAILBOX_BASE,
      .set_off    = 0x0,
      .en_off     = 0x4,
      .en_bit     = 16,
      .src_en_off = 0x4,
      .sta_off    = 0x8,
      .chnl_count = 16,
      .irq        = -1,
    },
    {
      .index      = CPU_INDEX_CPX,
      .base       = TOP_MAILBOX_BASE,
      .set_off    = 0x10,
      .en_off     = 0x14,
      .en_bit     = 16,
      .src_en_off = 0x14,
      .sta_off    = 0x18,
      .chnl_count = 16,
      .irq        = IRQ_VINT2,
    }
  };

  song_mbox_allinitialize(config, ARRAY_SIZE(config), g_mbox);
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

void up_finalinitialize(void)
{
}

static void up_cpu_lp(bool pwr_sleep, bool deep_sleep)
{
  if (pwr_sleep)
    {
      putreg32(TOP_PWR_CP_XC5_SLP_EN << 16 |
               TOP_PWR_CP_XC5_SLP_EN, TOP_PWR_SLPCTL_CP_XC5);
    }
  else
    {
      putreg32(TOP_PWR_CP_XC5_SLP_EN << 16, TOP_PWR_SLPCTL_CP_XC5);
    }

  if (deep_sleep)
    {
      putreg32(TOP_PWR_CP_XC5_DS_SLP_EN << 16 |
               TOP_PWR_CP_XC5_DS_SLP_EN, TOP_PWR_SLPCTL_CP_XC5);
    }
  else
    {
      putreg32(TOP_PWR_CP_XC5_DS_SLP_EN << 16, TOP_PWR_SLPCTL_CP_XC5);
    }
}

void up_cpu_doze(void)
{
  up_cpu_lp(false, false);
  up_psu_lp(XC5_DOZE);
}

void up_cpu_idle(void)
{
  up_cpu_lp(false, false);
  up_psu_lp(XC5_IDLE);
}

void up_cpu_standby(void)
{
  up_cpu_lp(true, false);
  up_psu_lp(XC5_IDLE);
}

void up_cpu_sleep(void)
{
  up_cpu_lp(true, true);
  up_psu_lp(XC5_IDLE);
}

#endif /* CONFIG_ARCH_CHIP_U3_CPX */
