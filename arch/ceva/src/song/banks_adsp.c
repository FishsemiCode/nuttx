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

#ifdef CONFIG_ARCH_CHIP_BANKS_ADSP

#include <nuttx/audio/audio_comp.h>
#include <nuttx/audio/audio_dma.h>
#include <nuttx/audio/audio_i2s.h>
#include <nuttx/audio/song_pcm.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/drivers/addrenv.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/rpmsg_rtc.h>
#include <nuttx/timers/song_oneshot.h>

#include <string.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                 "ap"
#define CPU_INDEX_AP                0
#define CPU_INDEX_ADSP              1

#define LOGBUF_BASE                 ((uintptr_t)&_slog)

#define TOP_MAILBOX_BASE            (0xf9000000)

#define DDR_PWR_BASE                (0xf9210000)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SONG_DMAS
static FAR struct dma_dev_s *g_dma[2] =
{
  [1] = DEV_END,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _slog;

#ifdef CONFIG_SONG_MBOX
FAR struct mbox_dev_s *g_mbox[3] =
{
  [2] = DEV_END,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
  static const struct simple_addrenv_s addrenv[] =
  {
    {.va = 0x00000000, .pa = 0xf8400000, .size = 0x00200000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  simple_addrenv_initialize(addrenv);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(CPU_NAME_AP, (void *)LOGBUF_BASE, CONFIG_LOGBUF_SIZE);
#endif
}

void up_dma_initialize(void)
{
#ifdef CONFIG_SONG_DMAS
  g_dma[0] = song_dmas_initialize(2, B2C(0xf8109000), IRQ_INT2, NULL);
#endif
}

#if defined(CONFIG_16550_UART) && defined(CONFIG_SONG_DMAS)
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
  return NULL; /* Can't use top dmas */
}
#endif

void ceva_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG
  static const struct song_oneshot_config_s config0 =
  {
    .minor      = -1,
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

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config0));

#  ifdef CONFIG_CPULOAD_ONESHOT
  static const struct song_oneshot_config_s config1 =
  {
    .minor      = -1,
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

  sched_oneshot_extclk(song_oneshot_initialize(&config1));
#  endif
#endif
}

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  uart_rpmsg_init(CPU_NAME_AP, "ADSP", B2C(256), true);
}
#endif

#ifdef CONFIG_SONG_MBOX
static void up_mbox_init(void)
{
  static const struct song_mbox_config_s config[] =
  {
    {
      .index      = CPU_INDEX_AP,
      .base       = B2C(TOP_MAILBOX_BASE),
      .set_off    = 0x40,
      .en_off     = 0x44,
      .en_bit     = 0,
      .src_en_off = 0x48,
      .sta_off    = 0x50,
      .chnl_count = 64,
      .irq        = -1,
    },
    {
      .index      = CPU_INDEX_ADSP,
      .base       = B2C(TOP_MAILBOX_BASE),
      .set_off    = 0x70,
      .en_off     = 0x74,
      .en_bit     = 16,
      .src_en_off = 0x74,
      .sta_off    = 0x78,
      .chnl_count = 16,
      .irq        = IRQ_INT1,
    }
  };

  song_mbox_allinitialize(config, ARRAY_SIZE(config), g_mbox);
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
    .cpuname = CPU_NAME_AP,
    .rsc     = &rptun_rsc_ap,
    .vringtx = 63,
    .vringrx = 15,
  };

  song_rptun_initialize(&rptun_cfg_ap, g_mbox[CPU_INDEX_AP], g_mbox[CPU_INDEX_ADSP]);

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

#ifdef CONFIG_WATCHDOG_DW
void up_wdtinit(void)
{
  static const struct dw_wdt_config_s config =
  {
    .path = CONFIG_WATCHDOG_DEVPATH,
    .base = B2C(0xf8108000),
    .irq  = IRQ_VINT0,
    .tclk = "ap/audio_wdt_tclk",
  };

  dw_wdt_initialize(&config);
}
#endif

static void up_audio_init(void)
{
#ifdef CONFIG_SONG_PCM
  struct i2s_dev_s *pcm = song_pcm_initialize(B2C(0xf8107000), "ap/audio_pcm_mclk");
  audio_comp_initialize("pcm0p",
                        audio_dma_initialize(g_dma[0], 0, true, 4, B2C(0xf8107018)),
                        audio_i2s_initialize(pcm, true), NULL);
  audio_comp_initialize("pcm0c",
                        audio_dma_initialize(g_dma[0], 8, false, 4, B2C(0xf8107014)),
                        audio_i2s_initialize(pcm, false), NULL);
#endif
}

void up_lateinitialize(void)
{
#ifdef CONFIG_SONG_MBOX
  up_mbox_init();
#endif

#ifdef CONFIG_SONG_RPTUN
  up_rptun_init();
#endif

#ifdef CONFIG_WATCHDOG_DW
  up_wdtinit();
#endif

  up_audio_init();
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

#endif /* CONFIG_ARCH_CHIP_BANKS_ADSP */
