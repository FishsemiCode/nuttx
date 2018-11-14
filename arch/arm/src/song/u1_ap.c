/****************************************************************************
 * arch/arm/src/song/u1_ap.c
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

#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/i2c/i2c_dw.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/power/regulator.h>
#include <nuttx/pwm/song_pwm.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/spi/spi_dw.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/song_oneshot.h>
#include <nuttx/timers/song_rtc.h>

#include "chip.h"
#include "song_addrenv.h"
#include "song_idle.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_U1_AP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_CP                 "cp"
#define CPU_NAME_SP                 "sp"
#define CPU_INDEX_AP                0
#define CPU_INDEX_CP                1
#define CPU_INDEX_SP                2

#define TOP_MAILBOX_BASE            (0xb0030000)

#define TOP_PWR_BASE                (0xb0040000)
#define TOP_PWR_AP_M4_RSTCTL        (TOP_PWR_BASE + 0x0e0)
#define TOP_PWR_SFRST_CTL           (TOP_PWR_BASE + 0x11c)
#define TOP_PWR_AP_M4_INTR2SLP_MK0  (TOP_PWR_BASE + 0x14c)
#define TOP_PWR_AP_UNIT_PD_CTL      (TOP_PWR_BASE + 0x204)
#define TOP_PWR_RES_REG2            (TOP_PWR_BASE + 0x260)
#define TOP_PWR_SLPCTL_AP_M4        (TOP_PWR_BASE + 0x360)
#define TOP_PWR_AP_M4_TCM_PD_CTL    (TOP_PWR_BASE + 0x3ec)

#define TOP_PWR_AP_M4_SFRST         (1 << 4)
#define TOP_PWR_AP_M4_IDLE_MK       (1 << 5)

#define TOP_PWR_SFRST_RESET         (1 << 0)

#define TOP_PWR_AP_M4_PD_MK         (1 << 3)
#define TOP_PWR_AP_M4_AU_PU_MK      (1 << 6)
#define TOP_PWR_AP_M4_AU_PD_MK      (1 << 7)

#define TOP_PWR_RESET_NORMAL        (0x00000000)
#define TOP_PWR_RESET_ROMBOOT       (0xaaaa1234)

#define TOP_PWR_AP_M4_SLP_EN        (1 << 0)
#define TOP_PWR_AP_M4_DS_SLP_EN     (1 << 2)

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

#ifdef CONFIG_SONG_MBOX
FAR struct mbox_dev_s *g_mbox[4] =
{
  [3] = DEV_END,
};
#endif

#ifdef CONFIG_SONG_IOE
FAR struct ioexpander_dev_s *g_ioe[2] =
{
  [1] = DEV_END,
};
#endif

#ifdef CONFIG_SPI_DW
FAR struct spi_dev_s *g_spi[2] =
{
  [1] = DEV_END,
};
#endif

#ifdef CONFIG_I2C_DW
FAR struct i2c_master_s *g_i2c[3] =
{
  [2] = DEV_END,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
  static const struct song_addrenv_s addrenv[] =
  {
    {.va = 0x21000000, .pa = 0xb1000000, .size = 0x00100000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  up_addrenv_initialize(addrenv);

  /* Unmask SLEEPING for reset */
  putreg32(TOP_PWR_AP_M4_IDLE_MK << 16, TOP_PWR_AP_M4_RSTCTL);

  /* Always allow enter FLASH_S */
  putreg32(TOP_PWR_AP_M4_DS_SLP_EN << 16 |
           TOP_PWR_AP_M4_DS_SLP_EN, TOP_PWR_SLPCTL_AP_M4);

#ifndef CONFIG_CPULOAD_PERIOD
  /* Allow TCM to LP, careful with it. At this time,
   * if use systick as weakup reason form DEEPSLEEP, CPU will hang.
   */
  putreg32(TOP_PWR_AP_M4_AU_PD_MK << 16, TOP_PWR_AP_M4_TCM_PD_CTL);
#endif

  /* Forbid the AP power down, AP will power down following SP */
  putreg32(TOP_PWR_AP_M4_AU_PD_MK << 16 |
           TOP_PWR_AP_M4_AU_PD_MK, TOP_PWR_AP_UNIT_PD_CTL);
}

void up_wic_initialize(void)
{
  putreg32(0xffffffff, TOP_PWR_AP_M4_INTR2SLP_MK0);
}

void up_wic_enable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_AP_M4_INTR2SLP_MK0, 1 << (irq - NVIC_IRQ_FIRST), 0);
    }
}

void up_wic_disable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_AP_M4_INTR2SLP_MK0, 0, 1 << (irq - NVIC_IRQ_FIRST));
    }
}

#ifdef CONFIG_SONG_DMAS
void up_dmainitialize(void)
{
  g_dma[0] = song_dmas_initialize(1, 0xb0020000, 28, "sp/dmas_hclk");
}
#endif

#if defined(CONFIG_16550_UART) && defined(CONFIG_SONG_DMAS)
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
  return g_dma[0] ? DMA_GET_CHAN(g_dma[0], ident) : NULL;
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
    .spec_off   = 0x1a8,
    .intren_off = 0x128,
    .intrst_off = 0x134,
    .intr_bit   = 1,
  };
#endif

#ifdef CONFIG_ONESHOT_SONG
  up_alarm_set_lowerhalf(song_oneshot_initialize(&config));
#endif

#ifdef CONFIG_CPULOAD_PERIOD
  sched_period_extclk(systick_initialize(false, 32768, -1));
#endif
}

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  uart_rpmsg_server_init("SP", 1024);
  uart_rpmsg_server_init("CP", 1024);
  uart_rpmsg_server_init("AT", 1024);
  uart_rpmsg_server_init("GPS", 1024);
}
#endif

#ifdef CONFIG_SONG_RPTUN
static void up_openamp_initialize(void)
{
  static const struct song_rptun_config_s rptun_cfg_cp =
  {
    .cpu_name    = CPU_NAME_CP,
    .role        = RPMSG_MASTER,
    .ch_start_tx = 2,
    .ch_vring_tx = 3,
    .ch_start_rx = 2,
    .ch_vring_rx = 3,
    .rsc         =
    {
      .rsc_tab   = (void *)0xb0003000,
      .size      = sizeof(struct rptun_rsc_s),
    },
  };

  static const struct song_rptun_config_s rptun_cfg_sp =
  {
    .cpu_name    = CPU_NAME_SP,
    .role        = RPMSG_REMOTE,
    .ch_start_tx = 14,
    .ch_vring_tx = 15,
    .ch_start_rx = 14,
    .ch_vring_rx = 15,
    .rsc         =
    {
      .rsc_tab   = (void *)0xb0000000,
      .size      = sizeof(struct rptun_rsc_s),
    },
  };

  song_rptun_initialize(&rptun_cfg_cp, g_mbox[CPU_INDEX_CP], g_mbox[CPU_INDEX_AP]);
  song_rptun_initialize(&rptun_cfg_sp, g_mbox[CPU_INDEX_SP], g_mbox[CPU_INDEX_AP]);

#  ifdef CONFIG_RPMSG_REGULATOR
  rpmsg_regulator_init(CPU_NAME_SP, false);
#  endif

#  ifdef CONFIG_SYSLOG_RPMSG_SERVER
  syslog_rpmsg_server_init();
#  endif

#  ifdef CONFIG_FS_HOSTFS_RPMSG
  hostfs_rpmsg_init(CPU_NAME_SP);
#  endif
}
#endif

#ifdef CONFIG_RTC_SONG
int up_rtc_initialize(void)
{
  static const struct song_rtc_config_s config =
  {
    .minor = 0,
    .base  = 0xb2020000,
    .irq   = 16,
    .index = 1,
  };

  up_rtc_set_lowerhalf(song_rtc_initialize(&config));
  return 0;
}
#endif

#ifdef CONFIG_WATCHDOG_DW
void up_wdtinit(void)
{
  static const struct dw_wdt_config_s config =
  {
    .path = CONFIG_WATCHDOG_DEVPATH,
    .base = 0xb0090000,
    .irq  = 20,
    .tclk = "apwdt_tclk",
  };

  dw_wdt_initialize(&config);
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
      .set_off    = 0x10,
      .en_off     = 0x14,
      .en_bit     = 16,
      .src_en_off = 0x14,
      .sta_off    = 0x18,
      .chnl_count = 16,
      .irq        = 21,
    },
    {
      .index      = CPU_INDEX_CP,
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
      .index      = CPU_INDEX_SP,
      .base       = TOP_MAILBOX_BASE,
      .set_off    = 0x20,
      .en_off     = 0x24,
      .en_bit     = 16,
      .src_en_off = 0x24,
      .sta_off    = 0x28,
      .chnl_count = 16,
      .irq        = -1,
    }
  };

  song_mbox_allinitialize(config, ARRAY_SIZE(config), g_mbox);
}
#endif

#ifdef CONFIG_SPI_DW
static void up_spi_init(void)
{
  static const struct dw_spi_config_s config =
  {
    .bus = 0,
    .base = 0xb0110000,
    .irq = 27,
    .cs_num = 1,
    .cs_gpio[0] = 22,
    .mclk = "spi0_mclk",
  };

  g_spi[config.bus] = dw_spi_initialize(&config, g_ioe[0]);
}
#endif

#ifdef CONFIG_I2C_DW
static void up_i2c_init(void)
{
  static const struct dw_i2c_config_s config[] =
  {
    {
      .bus        = 0,
      .base       = 0xb00e0000,
      .mclk       = "i2c0_mclk",
      .irq        = 25,
      .sda_hold   = 7,
      .fs_spklen  = 1,
      .hs_spklen  = 1,
      .ss_hcnt    = 62,
      .ss_lcnt    = 92,
      .fs_hcnt    = 14,
      .fs_lcnt    = 17,
      .hs_hcnt    = 6,
      .hs_lcnt    = 8,
    },
    {
      .bus        = 1,
      .base       = 0xb00f0000,
      .mclk       = "i2c1_mclk",
      .irq        = 26,
      .sda_hold   = 7,
      .fs_spklen  = 1,
      .hs_spklen  = 1,
      .ss_hcnt    = 56,
      .ss_lcnt    = 88,
      .fs_hcnt    = 11,
      .fs_lcnt    = 12,
      .hs_hcnt    = 6,
      .hs_lcnt    = 8,
    }
  };
  int config_num = sizeof(config) / sizeof(config[0]);

  dw_i2c_allinitialize(config, config_num, g_i2c);
}
#endif

void up_lateinitialize(void)
{
#ifdef CONFIG_SONG_MBOX
  up_mbox_init();
#endif

#ifdef CONFIG_SONG_RPTUN
  up_openamp_initialize();
#endif

#ifdef CONFIG_SONG_CLK
  up_clk_initialize();
#endif

#ifdef CONFIG_RTC_SONG
  up_rtc_initialize();
#endif

#ifdef CONFIG_WATCHDOG_DW
  up_wdtinit();
#endif

#ifdef CONFIG_SONG_IOE
  g_ioe[0] = song_ioe_initialize(1, 0xb0060000, 19);
#endif

#ifdef CONFIG_SPI_DW
  up_spi_init();
#endif

#ifdef CONFIG_I2C_DW
  up_i2c_init();
#endif

#ifdef CONFIG_PWM_SONG
  song_pwm_initialize(0, 0xb0100000, 4, "pwm_mclk");
#endif

#ifdef CONFIG_SONG_CLK
  clk_disable_unused();
#endif
}

void up_reset(int status)
{
  if (status == 1)
    {
      /* Reset board to romboot */

      putreg32(TOP_PWR_RESET_ROMBOOT, TOP_PWR_RES_REG2);
      putreg32(TOP_PWR_SFRST_RESET << 16 |
               TOP_PWR_SFRST_RESET, TOP_PWR_SFRST_CTL);
    }
  else if (status == 2)
    {
      /* Reset current core */

      putreg32(TOP_PWR_AP_M4_SFRST << 16 |
               TOP_PWR_AP_M4_SFRST, TOP_PWR_AP_M4_RSTCTL);
    }
  else
    {
      /* Reset board */

      putreg32(TOP_PWR_RESET_NORMAL, TOP_PWR_RES_REG2);
      putreg32(TOP_PWR_SFRST_RESET << 16 |
               TOP_PWR_SFRST_RESET, TOP_PWR_SFRST_CTL);
    }
}

void up_cpu_doze(void)
{
  /* Forbid the full chip power down */
  putreg32(TOP_PWR_AP_M4_SLP_EN << 16, TOP_PWR_SLPCTL_AP_M4);

  /* Forbid the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) & ~NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_wfi();
}

void up_cpu_idle(void)
{
  /* Forbid the full chip power down */
  putreg32(TOP_PWR_AP_M4_SLP_EN << 16, TOP_PWR_SLPCTL_AP_M4);

  /* Allow the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_wfi();
}

void up_cpu_standby(void)
{
  up_cpu_idle();
}

void up_cpu_sleep(void)
{
  /* Allow the full chip power down */
  putreg32(TOP_PWR_AP_M4_SLP_EN << 16 |
           TOP_PWR_AP_M4_SLP_EN, TOP_PWR_SLPCTL_AP_M4);

  /* Allow the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_wfi();
}

#endif /* CONFIG_ARCH_CHIP_U1_AP */
