/****************************************************************************
 * arch/arm/src/song/u3_ap.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Guiding Li<liguiding@pinecone.net>
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

#ifdef CONFIG_ARCH_CHIP_U3_AP

#include <nuttx/arch.h>
#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/drivers/addrenv.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/i2c/i2c_dw.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/power/consumer.h>
#include <nuttx/power/regulator.h>
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
#include <nuttx/wqueue.h>

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                     "ap"
#define CPU_NAME_CPR                    "cpr"
#define CPU_NAME_CPX                    "cpx"
#define CPU_INDEX_AP                    0
#define CPU_INDEX_CPR                   1
#define CPU_INDEX_CPX                   2

#define TOP_MAILBOX_BASE                (0xb0030000)

#define TOP_PWR_BASE                    (0xb0040000)
#define TOP_PWR_RSTCTL1                 (TOP_PWR_BASE + 0x0d4)
#define TOP_PWR_CHIPRST_CTL             (TOP_PWR_BASE + 0x114)
#define TOP_PWR_TOP_ROCKET_INTR2SLP_MK0 (TOP_PWR_BASE + 0x148)
#define TOP_PWR_CP_XC5_CTL0             (TOP_PWR_BASE + 0x210)
#define TOP_PWR_CP_XC5_BOOT_ADDR        (TOP_PWR_BASE + 0x214)
#define TOP_PWR_CP_ROCKET_CTL           (TOP_PWR_BASE + 0x21c)
#define TOP_PWR_RES_REG2                (TOP_PWR_BASE + 0x260)

#define TOP_PWR_CP_ROCKET_RSTN          (1 << 11)   //TOP_PWR_RSTCTL1
#define TOP_PWR_CP_XC5_RSTN             ((1 << 12) | (1 << 19) | (1 << 20))

#define TOP_PWR_CP_XC5_WAIT             (1 << 3)    //TOP_PWR_CP_XC5_CTL0
#define TOP_PWR_CP_XC5_CACHE_INV        (1 << 4)

#define TOP_PWR_SFRST                   (1 << 1)    //TOP_PWR_CHIPRST_CTL

#define TOP_PWR_RESET_NORMAL            (0x00000000)//TOP_PWR_RES_REG2
#define TOP_PWR_RESET_ROMBOOT           (0xaaaa1234)
#define TOP_PWR_RESET_RECOVERY          (0xbbbb1234)

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

#ifdef CONFIG_SONG_IOE
FAR struct ioexpander_dev_s *g_ioe[2] =
{
  [1] = DEV_END,
};
#endif

#ifdef CONFIG_SONG_MBOX
FAR struct mbox_dev_s *g_mbox[4] =
{
  [3] = DEV_END,
};
#endif

#ifdef CONFIG_SPI_DW
FAR struct spi_dev_s *g_spi[4] =
{
  [3] = DEV_END,
};
#endif

#ifdef CONFIG_I2C_DW
FAR struct i2c_master_s *g_i2c[3] =
{
  [2] = DEV_END,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int cpr_start(const struct song_rptun_config_s *config)
{
  putreg32(0x40100410, TOP_PWR_CP_ROCKET_CTL);
  modifyreg32(TOP_PWR_RSTCTL1, TOP_PWR_CP_ROCKET_RSTN, 0);
  return 0;
}

static int cpx_config(const struct song_rptun_config_s *config, void *data)
{
  modifyreg32(TOP_PWR_CP_XC5_CTL0, TOP_PWR_CP_XC5_WAIT, TOP_PWR_CP_XC5_WAIT);
  modifyreg32(TOP_PWR_RSTCTL1, TOP_PWR_CP_XC5_RSTN, 0);

  return 0;
}

static int cpx_start(const struct song_rptun_config_s *config)
{
  modifyreg32(TOP_PWR_CP_XC5_CTL0, TOP_PWR_CP_XC5_CACHE_INV, TOP_PWR_CP_XC5_CACHE_INV);
  putreg32(0x40200000, TOP_PWR_CP_XC5_BOOT_ADDR);
  modifyreg32(TOP_PWR_RSTCTL1, TOP_PWR_CP_XC5_RSTN, TOP_PWR_CP_XC5_RSTN);
  modifyreg32(TOP_PWR_CP_XC5_CTL0, TOP_PWR_CP_XC5_WAIT, 0);
  modifyreg32(TOP_PWR_RSTCTL1, TOP_PWR_CP_XC5_RSTN, 0);

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
  static const struct simple_addrenv_s addrenv[] =
  {
    {.va = 0x40000000, .pa = 0xd0000000, .size = 0x00100000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  simple_addrenv_initialize(addrenv);
}

void up_wic_initialize(void)
{
  putreg32(0xffffffff, TOP_PWR_TOP_ROCKET_INTR2SLP_MK0);
}

void up_wic_enable_irq(int irq)
{
  modifyreg32(TOP_PWR_TOP_ROCKET_INTR2SLP_MK0, 1 << (irq - 1), 0);
}

void up_wic_disable_irq(int irq)
{
  modifyreg32(TOP_PWR_TOP_ROCKET_INTR2SLP_MK0, 0, 1 << (irq - 1));
}

void up_dma_initialize(void)
{
#ifdef CONFIG_SONG_DMAS
  g_dma[0] = song_dmas_initialize(2, 0xb0020000, 12, "dmas_hclk");
#endif
}

#if defined(CONFIG_16550_UART) && defined(CONFIG_SONG_DMAS)
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
  return g_dma[0] ? DMA_GET_CHAN(g_dma[0], ident) : NULL;
}
#endif

void riscv_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG
  static const struct song_oneshot_config_s config =
  {
    .minor      = -1,
    .base       = TOP_PWR_BASE,
    .irq        = 3,
    .c1_max     = 480,
    .c1_freq    = 4800000,
    .ctl_off    = 0x170,
    .calib_off  = 0x194,
    .calib_inc  = 0x198,
    .c1_off     = 0x174,
    .c2_off     = 0x178,
    .spec_off   = 0x1ac,
    .intren_off = 0x124,
    .intrst_off = 0x130,
    .intr_bit   = 2,
  };

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config));
#endif

#ifdef CONFIG_SONG_CLK
  up_clk_initialize();
#endif
}

#ifdef CONFIG_RTC_SONG
int up_rtc_initialize(void)
{
  static const struct song_rtc_config_s config =
  {
    .minor = 0,
    .base  = 0xb2020000,
    .irq   = 1,
    .index = 2,
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
    .irq  = 5,
    .tclk = "swdt_tclk",
  };

  dw_wdt_initialize(&config);
}
#endif

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  uart_rpmsg_init(CPU_NAME_CPR, "CPR", 1024, false);
  uart_rpmsg_init(CPU_NAME_CPX, "CPX", 1024, false);
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
      .irq        = 6,
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
      .irq        = -1,
    }
  };

  song_mbox_allinitialize(config, ARRAY_SIZE(config), g_mbox);
}
#endif

#ifdef CONFIG_SONG_RPTUN
static void up_rptun_init(void)
{
  static const struct rptun_addrenv_s addrenv[] =
  {
    {.pa = 0xd0100000, .da = 0x40100000, .size = 0x00100000},
    {.pa = 0xd0200000, .da = 0x40200000, .size = 0x00100000},
    {.pa = 0x00000000, .da = 0x00000000, .size = 0x00000000},
  };

  static const struct song_rptun_config_s rptun_cfg_cpr =
  {
    .cpuname    = CPU_NAME_CPR,
    .firmware   = "/dev/cpr.elf",
    .addrenv    = addrenv,
    .nautostart = true,
    .master     = true,
    .vringtx    = 1,
    .vringrx    = 1,
    .start      = cpr_start,
  };

  static const struct song_rptun_config_s rptun_cfg_cpx = {
    .cpuname    = CPU_NAME_CPX,
    .firmware   = "/dev/cpx.coff",
    .addrenv    = addrenv,
    .nautostart = true,
    .master     = true,
    .vringtx    = 1,
    .vringrx    = 1,
    .config     = cpx_config,
    .start      = cpx_start,
  };

  song_rptun_initialize(&rptun_cfg_cpr, g_mbox[CPU_INDEX_CPR], g_mbox[CPU_INDEX_AP]);
  song_rptun_initialize(&rptun_cfg_cpx, g_mbox[CPU_INDEX_CPX], g_mbox[CPU_INDEX_AP]);

#  ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize();
#  endif

#  ifdef CONFIG_SYSLOG_RPMSG_SERVER
  syslog_rpmsg_server_init();
#  endif

#  ifdef CONFIG_FS_HOSTFS_RPMSG_SERVER
  hostfs_rpmsg_server_init();
#  endif
}
#endif

#ifdef CONFIG_SONG_IOE
void up_ioe_init(void)
{
  static const struct song_ioe_config_s cfg =
  {
    .cpu  = 2,
    .base = 0xb0060000,
    .irq  = 4,
    .mclk = "gpio_clk32k",
  };

  g_ioe[0] = song_ioe_initialize(&cfg);
}
#endif

#ifdef CONFIG_I2C_DW
static void up_i2c_init(void)
{
  static const struct dw_i2c_config_s config[] =
  {
    {
      .bus  = 0,
      .base = 0xb00e0000,
      .mclk = "i2c0_mclk",
      .rate = 16000000,
      .irq  = 9,
    },
    {
      .bus  = 1,
      .base = 0xb00f0000,
      .mclk = "i2c1_mclk",
      .rate = 16000000,
      .irq  = 10,
    },
    {
      .bus  = 2,
      .base = 0xc0000000,
      .mclk = "i2c2_mclk",
      .rate = 16000000,
      .irq  = 18,
    }
  };
  int config_num = sizeof(config) / sizeof(config[0]);

  dw_i2c_allinitialize(config, config_num, g_i2c);
}
#endif

#ifdef CONFIG_SPI_DW
static void up_spi_init(void)
{
  static const struct dw_spi_config_s config[] =
  {
    {
      .bus = 0,
      .base = 0xb0110000,
      .irq = 11,
      .tx_dma = 4,
      .rx_dma = 12,
      .cs_num = 1,
      .cs_gpio[0] = 20,
      .mclk = "spi0_mclk",
    },
    {
      .bus = 1,
      .base = 0xb0120000,
      .irq = 15,
      .tx_dma = 5,
      .rx_dma = 13,
      .cs_num = 1,
      .cs_gpio[0] = 24,
      .mclk = "spi1_mclk",
    },
    {
      .bus = 2,
      .base = 0xb0250000,
      .irq = 16,
      .tx_dma = 3,
      .rx_dma = 11,
      .cs_num = 1,
      .cs_gpio[0] = 28,
      .mclk = "spi2_mclk",
    },
  };
  int config_num = sizeof(config) / sizeof(config[0]);

  dw_spi_allinitialize(config, config_num, g_ioe[0], g_dma[0], g_spi);
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

#ifdef CONFIG_RTC_SONG
  up_rtc_initialize();
#endif

#ifdef CONFIG_WATCHDOG_DW
  up_wdtinit();
#endif

#ifdef CONFIG_SONG_IOE
  up_ioe_init();
#endif

#ifdef CONFIG_I2C_DW
  up_i2c_init();
#endif

#ifdef CONFIG_SPI_DW
  up_spi_init();
#endif

#ifdef CONFIG_PWM_SONG
  song_pwm_initialize(0, 0xb0100000, 4, "pwm_mclk");
#endif
}

void up_finalinitialize(void)
{
#ifdef CONFIG_SONG_CLK
  up_clk_finalinitialize();
#endif
}

void up_reset(int status)
{
  if (status == 1)
    {
      /* Reset board to romboot */

      putreg32(TOP_PWR_RESET_ROMBOOT, TOP_PWR_RES_REG2);
      putreg32(TOP_PWR_SFRST << 16 |
               TOP_PWR_SFRST, TOP_PWR_CHIPRST_CTL);
    }
  else
    {
      /* Reset board */

      putreg32(TOP_PWR_RESET_NORMAL, TOP_PWR_RES_REG2);
      putreg32(TOP_PWR_SFRST << 16 |
               TOP_PWR_SFRST, TOP_PWR_CHIPRST_CTL);
    }
}

void up_cpu_doze(void)
{
  up_cpu_wfi();
}

void up_cpu_idle(void)
{
  up_cpu_wfi();
}

void up_cpu_standby(void)
{
  up_cpu_wfi();
}

void up_cpu_sleep(void)
{
  up_cpu_wfi();
}

#endif /* CONFIG_ARCH_CHIP_U3_AP */
