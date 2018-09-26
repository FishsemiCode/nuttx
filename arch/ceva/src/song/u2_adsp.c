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

#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/song_oneshot.h>

#include "song_addrenv.h"
#include "up_arch.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_U2_ADSP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                 "ap"

#define LOGBUF_BASE                 ((uintptr_t)&_slog)

#define TOP_MAILBOX_BASE            (0xa0050000)

#define TOP_PWR_BASE                (0xa00e0000)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SONG_DMAS
static FAR struct dma_dev_s *g_dma[3] =
{
  [2] = DEV_END,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _slog;

#ifdef CONFIG_SONG_IOE
FAR struct ioexpander_dev_s *g_ioe[2] =
{
  [1] = DEV_END,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
  static const struct song_addrenv_s addrenv[] =
  {
    {.va = 0x00000000, .pa = 0x80000000, .size = 0x00010000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  up_addrenv_initialize(addrenv);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(CPU_NAME_AP, (void *)LOGBUF_BASE, CONFIG_LOGBUF_SIZE);
#endif
}

#ifdef CONFIG_SONG_DMAS
void up_dmainitialize(void)
{
  g_dma[0] = song_dmas_initialize(1, B2C(0xa0040000), IRQ_INT1, "ap/top_dmas_hclk");
  g_dma[1] = song_dmas_initialize(1, B2C(0xa0080000), IRQ_INT0, "ap/audio_dmas_hclk");
}
#endif

#if defined(CONFIG_16550_UART) && defined(CONFIG_SONG_DMAS)
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
  return g_dma[0] ? DMA_GET_CHAN(g_dma[0], ident) : NULL;
}
#endif

void ceva_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG
  static const struct song_oneshot_config_s config0 =
  {
    .minor      = -1,
    .base       = B2C(TOP_PWR_BASE),
    .irq        = IRQ_VINT5, /* VINT5 */
    .c1_max     = 600,
    .c1_freq    = 6000000, /* 6Mhz */
    .ctl_off    = 0x290, /* TOP_PWR_AT_CTL */
    .calib_off  = 0x2b4, /* TOP_PWR_AT_CALIB_CTL */
    .c1_off     = 0x294, /* TOP_PWR_AT_C1 */
    .c2_off     = 0x298, /* TOP_PWR_AT_C2 */
    .spec_off   = 0x2c4, /* TOP_PWR_AT_SPEC_TIM0 */
    .intren_off = 0x218, /* TOP_PWR_INTR_EN_TL421 */
    .intrst_off = 0x220, /* TOP_PWR_INTR_ST_TL421 */
    .intr_bit   = 0,     /* TOP_PWR_AT_SPEC_TIM0_BIT */
  };

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config0));

#  ifdef CONFIG_CPULOAD_ONESHOT
  static const struct song_oneshot_config_s config1 =
  {
    .minor      = -1,
    .base       = B2C(TOP_PWR_BASE),
    .irq        = IRQ_VINT5, /* VINT5 */
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

  sched_oneshot_extclk(song_oneshot_initialize(&config1));
#  endif
#endif
}

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
#  ifdef CONFIG_SERIAL_CONSOLE
  uart_rpmsg_init(CPU_NAME_AP, "ADSP", B2C(1024), false);
#  else
  uart_rpmsg_init(CPU_NAME_AP, "ADSP", B2C(1024), true);
#  endif
}
#endif

#ifdef CONFIG_SONG_RPTUN
static void up_openamp_initialize(void)
{
  struct mbox_dev_s *mbox_ap, *mbox_adsp;

  static const struct song_mbox_config_s mbox_cfg_ap =
  {
    .base       = B2C(TOP_MAILBOX_BASE),
    .set_off    = 0x0, /* MAILBOX_M4_INTR_SET */
    .en_off     = 0x4, /* MAILBOX_M4_INTR_EN */
    .en_bit     = 16,
    .src_en_off = 0x4, /* MAILBOX_M4_INTR_EN */
    .sta_off    = 0x8, /* MAILBOX_M4_INTR_STA */
    .chnl_count = 16,
    .irq        = -1,
  };

  static const struct song_mbox_config_s mbox_cfg_adsp =
  {
    .base       = B2C(TOP_MAILBOX_BASE),
    .set_off    = 0x10, /* MAILBOX_TL421_INTR_SET */
    .en_off     = 0x14, /* MAILBOX_TL421_INTR_EN */
    .en_bit     = 16,
    .src_en_off = 0x14, /* MAILBOX_TL421_INTR_EN */
    .sta_off    = 0x18, /* MAILBOX_TL421_INTR_STA */
    .chnl_count = 16,
    .irq        = IRQ_VINT12, /* VINT12 */
  };

  static const struct song_rptun_config_s rptun_cfg_ap =
  {
    .cpu_name    = CPU_NAME_AP,
    .role        = RPMSG_REMOTE,
    .ch_start_tx = 0,
    .ch_vring_tx = 1,
    .ch_start_rx = 0,
    .ch_vring_rx = 1,
    .rsc         =
    {
      .rsc_tab   = (void *)B2C(0x6001f000),
      .size      = sizeof(struct rptun_rsc_s),
    },
  };

  mbox_ap = song_mbox_initialize(&mbox_cfg_ap);
  mbox_adsp = song_mbox_initialize(&mbox_cfg_adsp);

  song_rptun_initialize(&rptun_cfg_ap, mbox_ap, mbox_adsp);

#  ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#  endif

#  ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize(false);
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
    .base = 0xa0180000,
    .irq  = IRQ_VINT13,
    .tclk = "ap/tl421_wdt_tclk",
  };

  dw_wdt_initialize(&config);
}
#endif

void up_lateinitialize(void)
{
#ifdef CONFIG_SONG_RPTUN
  up_openamp_initialize();
#endif

#ifdef CONFIG_WATCHDOG_DW
  up_wdtinit();
#endif

#ifdef CONFIG_SONG_IOE
  g_ioe[0] = song_ioe_initialize(1, B2C(0xa00f0000), IRQ_VINT6);
#endif
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

#endif /* CONFIG_ARCH_CHIP_U2_ADSP */
