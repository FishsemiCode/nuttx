/****************************************************************************
 * arch/arm/src/song/u1_cp.c
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

#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/power/regulator.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/song_oneshot.h>
#include <nuttx/timers/song_rtc.h>

#include "chip.h"
#include "nvic.h"
#include "song_addrenv.h"
#include "song_idle.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_U1_CP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_SP                 "sp"

#define LOGBUF_BASE                 ((uintptr_t)&_slog)
#define LOGBUF_SIZE                 ((uint32_t)&_logsize)

#define TOP_MAILBOX_BASE            (0xb0030000)

#define TOP_PWR_BASE                (0xb0040000)
#define TOP_PWR_CP_M4_INTR2SLP_MK0  (TOP_PWR_BASE + 0x150)
#define TOP_PWR_CP_UNIT_PD_CTL      (TOP_PWR_BASE + 0x1fc)
#define TOP_PWR_BOOT_REG            (TOP_PWR_BASE + 0x290)
#define TOP_PWR_SLPCTL_CP_M4        (TOP_PWR_BASE + 0x35c)
#define TOP_PWR_CP_M4_TCM_PD_CTL0   (TOP_PWR_BASE + 0x3e0)

#define TOP_PWR_CP_M4_PD_MK         (1 << 3)
#define TOP_PWR_CP_M4_AU_PU_MK      (1 << 6)
#define TOP_PWR_CP_M4_AU_PD_MK      (1 << 7)

#define TOP_PWR_CP_M4_COLD_BOOT     (1 << 2)

#define TOP_PWR_CP_M4_SLP_EN        (1 << 0)
#define TOP_PWR_CP_M4_DS_SLP_EN     (1 << 2)

#define TOP_PWR_CP_M4_TCM_AU_PD_MK  (1 << 7)

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
extern uint32_t _logsize;

#ifdef CONFIG_SONG_IOE
FAR struct ioexpander_dev_s *g_ioe[2] =
{
  [1] = DEV_END,
};
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_SONG
static FAR struct rtc_lowerhalf_s *g_rtc_lower;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlystart(void)
{
  if (getreg32(TOP_PWR_BOOT_REG) & TOP_PWR_CP_M4_COLD_BOOT)
    {
      /* Initial power on. Clear boot flag. */

      putreg32(TOP_PWR_CP_M4_COLD_BOOT << 16, TOP_PWR_BOOT_REG);
    }
  else
    {
      /* Power on from standby. Restore context (will not return). */

      up_cpu_restore();
    }
}

void up_earlyinitialize(void)
{
  static const struct song_addrenv_s addrenv[] =
  {
    {.va = 0x01000000, .pa = 0x00000000, .size = 0x00100000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  up_addrenv_initialize(addrenv);

  /* Enable TCM auto low power */
  putreg32(TOP_PWR_CP_M4_TCM_AU_PD_MK << 16, TOP_PWR_CP_M4_TCM_PD_CTL0);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(CPU_NAME_SP, (void *)LOGBUF_BASE, LOGBUF_SIZE);
#endif
}

#ifdef CONFIG_RTC_SONG
int up_rtc_initialize(void)
{
  static const struct song_rtc_config_s config =
  {
    .base  = 0xb2020000,
    .irq   = 16,
    .index = 0,
  };

  g_rtc_lower = song_rtc_initialize(&config);
  up_rtc_set_lowerhalf(g_rtc_lower);

  return 0;
}
#endif

void arm_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG
  static const struct song_oneshot_config_s config =
  {
    .minor      = -1,
    .base       = TOP_PWR_BASE,
    .irq        = 18,
    .c1_freq    = 8192000,
    .ctl_off    = 0x170,
    .calib_off  = 0x194,
    .c1_off     = 0x174,
    .c2_off     = 0x178,
    .spec_off   = 0x1a4,
    .intren_off = 0x12c,
    .intrst_off = 0x138,
    .intr_bit   = 0,
  };

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config));
#endif

#ifdef CONFIG_CPULOAD_PERIOD
  sched_period_extclk(systick_initialize(false, 32768, -1));
#endif
}

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
#ifdef CONFIG_SERIAL_CONSOLE
  uart_rpmsg_init(CPU_NAME_SP, "CP", 1024, false);
#else
  uart_rpmsg_init(CPU_NAME_SP, "CP", 1024, true);
#endif
  uart_rpmsg_init(CPU_NAME_SP, "AT", 1024, false);
  uart_rpmsg_init(CPU_NAME_SP, "GPS", 1024, false);
}
#endif

#ifdef CONFIG_OPENAMP
static void up_openamp_initialize(void)
{
  struct mbox_dev_s *mbox_cp, *mbox_sp;

  static const struct song_mbox_config_s mbox_cfg_cp =
  {
    .base       = TOP_MAILBOX_BASE,
    .set_off    = 0x0,
    .en_off     = 0x4,
    .en_bit     = 16,
    .src_en_off = 0x4,
    .sta_off    = 0x8,
    .chnl_count = 16,
    .irq        = 21,
  };

  static const struct song_mbox_config_s mbox_cfg_sp =
  {
    .base       = TOP_MAILBOX_BASE,
    .set_off    = 0x20,
    .en_off     = 0x24,
    .en_bit     = 16,
    .src_en_off = 0x24,
    .sta_off    = 0x28,
    .chnl_count = 16,
    .irq        = -1,
  };

  static const struct song_rptun_config_s rptun_cfg_sp =
  {
    .cpu_name    = CPU_NAME_SP,
    .role        = RPMSG_REMOTE,
    .ch_start_rx = 0,
    .ch_vring_rx = 1,
    .ch_start_tx = 0,
    .ch_vring_tx = 1,
    .rsc         =
    {
      .rsc_tab   = (void *)0xb0010000,
      .size      = sizeof(struct rptun_rsc_s),
    },
  };

  mbox_cp = song_mbox_initialize(&mbox_cfg_cp);
  mbox_sp = song_mbox_initialize(&mbox_cfg_sp);

  song_rptun_initialize(&rptun_cfg_sp, mbox_cp, mbox_sp);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#endif

#ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize(false);
#endif

#ifdef CONFIG_FS_HOSTFS_RPMSG
  hostfs_rpmsg_init(CPU_NAME_SP);
#endif
}
#endif

void up_lateinitialize(void)
{
#ifdef CONFIG_OPENAMP
  up_openamp_initialize();
#endif

#ifdef CONFIG_SONG_CLK
  up_clk_initialize();
#endif

#ifdef CONFIG_RTC_SONG
  rtc_initialize(0, g_rtc_lower);
#endif

#ifdef CONFIG_SONG_DMAS
  g_dma[0] = song_dmas_initialize(0, 0xb0020000, 28, "sp/dmas_hclk");
#endif

#ifdef CONFIG_SONG_IOE
  g_ioe[0] = song_ioe_initialize(0, 0xb0060000, 19);
#endif

#ifdef CONFIG_RPMSG_REGULATOR
  rpmsg_regulator_init(CPU_NAME_SP, 0);
#endif
}

FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
#ifdef CONFIG_SONG_DMAS
  return g_dma[0] ? DMA_GET_CHAN(g_dma[0], ident) : NULL;
#else
  return NULL;
#endif
}

void up_cpu_doze(void)
{
  /* Forbid the full chip power down */
  putreg32(TOP_PWR_CP_M4_DS_SLP_EN << 16, TOP_PWR_SLPCTL_CP_M4);

  /* Forbid the subsystem power down */
  putreg32(TOP_PWR_CP_M4_AU_PD_MK << 16 |
           TOP_PWR_CP_M4_AU_PD_MK, TOP_PWR_CP_UNIT_PD_CTL);

  /* Forbid the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) & ~NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_wfi();
}

void up_cpu_idle(void)
{
  /* Forbid the full chip power down */
  putreg32(TOP_PWR_CP_M4_DS_SLP_EN << 16, TOP_PWR_SLPCTL_CP_M4);

  /* Forbid the subsystem power down */
  putreg32(TOP_PWR_CP_M4_AU_PD_MK << 16 |
           TOP_PWR_CP_M4_AU_PD_MK, TOP_PWR_CP_UNIT_PD_CTL);

  /* Allow the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_wfi();
}

void up_cpu_standby(void)
{
  /* Forbid the full chip power down */
  putreg32(TOP_PWR_CP_M4_DS_SLP_EN << 16, TOP_PWR_SLPCTL_CP_M4);

  /* Allow the subsystem power down */
  putreg32(TOP_PWR_CP_M4_AU_PD_MK << 16, TOP_PWR_CP_UNIT_PD_CTL);

  /* Allow the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_save();
}

void up_cpu_sleep(void)
{
  /* Allow the full chip power down */
  putreg32(TOP_PWR_CP_M4_DS_SLP_EN << 16 |
           TOP_PWR_CP_M4_DS_SLP_EN, TOP_PWR_SLPCTL_CP_M4);

  /* Allow the subsystem power down */
  putreg32(TOP_PWR_CP_M4_AU_PD_MK << 16, TOP_PWR_CP_UNIT_PD_CTL);

  /* Allow the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_save();
}

void up_wic_disable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_CP_M4_INTR2SLP_MK0, 0, 1 << (irq - NVIC_IRQ_FIRST));
    }
}

void up_wic_enable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_CP_M4_INTR2SLP_MK0, 1 << (irq - NVIC_IRQ_FIRST), 0);
    }
}

void up_wic_initialize(void)
{
  putreg32(0xffffffff, TOP_PWR_CP_M4_INTR2SLP_MK0);
}

#endif /* CONFIG_ARCH_CHIP_U1_CP */
