/****************************************************************************
 * arch/arm/src/song/u11_ck.c
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
 *   Author: Guiding Li <liguiding@fishsemi.com>
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

#ifdef CONFIG_ARCH_CHIP_U11_CK

#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/drivers/addrenv.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/i2c/i2c_dw.h>
#include <nuttx/pinctrl/pinctrl.h>
#include <nuttx/pinctrl/song_pinctrl.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/misc/misc_rpmsg.h>
#include <nuttx/power/pm.h>
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

#include <arch/irq.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <stdlib.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_CP                 "cp"
#define CPU_INDEX_AP                0
#define CPU_INDEX_CP                1

#define TOP_MAILBOX_BASE            (0xb0030000)

#define TOP_PWR_BASE                (0xb0040000)
#define TOP_PWR_AP_M4_RSTCTL        (TOP_PWR_BASE + 0x0e0)
#define TOP_PWR_SFRST_CTL           (TOP_PWR_BASE + 0x11c)
#define TOP_PWR_INTR_EN_AP_M4       (TOP_PWR_BASE + 0x128)
#define TOP_PWR_INTR_ST_AP_M4       (TOP_PWR_BASE + 0x134)
#define TOP_PWR_AP_M4_INTR2SLP_MK0  (TOP_PWR_BASE + 0x14c)
#define TOP_PWR_AP_UNIT_PD_CTL      (TOP_PWR_BASE + 0x204)
#define TOP_PWR_CK802_CTL0          (TOP_PWR_BASE + 0x22c)
#define TOP_PWR_RES_REG2            (TOP_PWR_BASE + 0x260)
#define TOP_PWR_SLPCTL_AP_M4        (TOP_PWR_BASE + 0x360)
#define TOP_PWR_AP_M4_TCM_PD_CTL    (TOP_PWR_BASE + 0x3ec)

#define TOP_PWR_AP_M4_SFRST         (1 << 4)
#define TOP_PWR_AP_M4_IDLE_MK       (1 << 5)

#define TOP_PWR_SFRST_RESET         (1 << 0)

#define TOP_PWR_SLP_U1RXD_ACT       (1 << 21)

#define TOP_PWR_AP_M4_PD_MK         (1 << 3)
#define TOP_PWR_AP_M4_AU_PU_MK      (1 << 6)
#define TOP_PWR_AP_M4_AU_PD_MK      (1 << 7)
#define TOP_PWR_AP_M4_PD_CPU_SEL    (1 << 8)

#define TOP_PWR_SLP_LPMD_B_MK       (1 << 12)
#define TOP_PWR_CK802_SLP_EN        (1 << 14)

#define TOP_PWR_RESET_NORMAL        (0x00000000)
#define TOP_PWR_RESET_ROMBOOT       (0xaaaa1234)
#define TOP_PWR_RESET_RECOVERY      (0xbbbb1234)

#define TOP_PWR_AP_M4_SLP_EN        (1 << 0)
#define TOP_PWR_AP_M4_SLP_MK        (1 << 1)
#define TOP_PWR_AP_M4_DS_SLP_EN     (1 << 2)

#define TOP_PMICFSM_BASE                (0xb2010000)
#define TOP_PMICFSM_AP_M4_INT2SLP_MK0   (TOP_PMICFSM_BASE + 0xf4)

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

#ifdef CONFIG_SONG_MBOX
FAR struct mbox_dev_s *g_mbox[3] =
{
  [2] = DEV_END,
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

#ifdef CONFIG_SONG_PINCTRL
FAR struct pinctrl_dev_s *g_pinctrl[2] =
{
  [1] = DEV_END,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
  static const struct simple_addrenv_s addrenv[] =
  {
    {.va = 0x00000000, .pa = 0xb1000000, .size = 0x00020000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  simple_addrenv_initialize(addrenv);

  /* Mask AP M4 effect to DS */

  putreg32(TOP_PWR_AP_M4_SLP_MK << 16 |
           TOP_PWR_AP_M4_SLP_MK, TOP_PWR_SLPCTL_AP_M4);

  /* Unmask CK802 LPMD_B effect to DS */

  modifyreg32(TOP_PWR_CK802_CTL0, TOP_PWR_SLP_LPMD_B_MK, 0);

  /* Which CPU should watch when set core power down, 1 = CK802 */

  putreg32(TOP_PWR_AP_M4_PD_CPU_SEL << 16 |
           TOP_PWR_AP_M4_PD_CPU_SEL, TOP_PWR_AP_UNIT_PD_CTL);

  /* Forbid the AP power down, AP will power down following CP */

  putreg32(TOP_PWR_AP_M4_AU_PD_MK << 16 |
           TOP_PWR_AP_M4_AU_PD_MK, TOP_PWR_AP_UNIT_PD_CTL);
}

void up_wic_initialize(void)
{
  putreg32(0xffffffff, TOP_PMICFSM_AP_M4_INT2SLP_MK0);
}

void up_wic_enable_irq(int irq)
{
  modifyreg32(TOP_PMICFSM_AP_M4_INT2SLP_MK0, 1 << irq, 0);
}

void up_wic_disable_irq(int irq)
{
  modifyreg32(TOP_PMICFSM_AP_M4_INT2SLP_MK0, 0, 1 << irq);
}

void up_dma_initialize(void)
{
#ifdef CONFIG_SONG_DMAS
  g_dma[0] = song_dmas_initialize(1, 0xb0020000, 12, NULL);
#endif
}

#if defined(CONFIG_16550_UART) && defined(CONFIG_SONG_DMAS)
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
  return g_dma[0] ? DMA_GET_CHAN(g_dma[0], ident) : NULL;
}
#endif

void csky_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG
  static const struct song_oneshot_config_s config =
    {
      .minor      = -1,
      .base       = TOP_PWR_BASE,
      .irq        = 2,
      .c1_freq    = 8192000,
      .ctl_off    = 0x170,
      .calib_off  = 0x194,
      .calib_inc  = 0x198,
      .c1_off     = 0x174,
      .c2_off     = 0x178,
      .spec_off   = 0x1a8,
      .intren_off = 0x128,
      .intrst_off = 0x134,
      .intr_bit   = 1,
    };

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config));
#endif
}


#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  uart_rpmsg_init(CPU_NAME_CP, "CP", 256, false);
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
      .irq        = 5,
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
  };

  song_mbox_allinitialize(config, ARRAY_SIZE(config), g_mbox);
}
#endif

#ifdef CONFIG_SONG_RPTUN
static void up_rptun_init(void)
{
  static const struct song_rptun_config_s rptun_cfg_cp =
  {
    .cpuname = CPU_NAME_CP,
    .rsc     = (struct rptun_rsc_s *)0xb0000000,
    .vringtx = 15,
    .vringrx = 15,
  };

  song_rptun_initialize(&rptun_cfg_cp, g_mbox[CPU_INDEX_CP], g_mbox[CPU_INDEX_AP]);

#ifdef CONFIG_SYSLOG_RPMSG_SERVER
  syslog_rpmsg_server_init();
#endif
}
#endif

#ifdef CONFIG_RTC_SONG
int up_rtc_initialize(void)
{
  static const struct song_rtc_config_s config =
  {
    .minor   = 0,
    .base    = 0xb2020000,
    .irq     = 0,
    .index   = 1,
    .correct = 1,
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
    .irq  = 4,
    .tclk = "apwdt_tclk",
  };

  dw_wdt_initialize(&config);
}
#endif

#ifdef CONFIG_SONG_IOE
void up_ioe_init(void)
{
  static const struct song_ioe_config_s cfg =
  {
    .cpu  = 1,
    .base = 0xb0060000,
    .irq  = 3,
  };

  g_ioe[0] = song_ioe_initialize(&cfg);
}
#endif

#ifdef CONFIG_SPI_DW
static void up_spi_init(void)
{
  static const struct dw_spi_config_s config =
  {
    .bus = 0,
    .base = 0xb0110000,
    .irq = 11,
    .tx_dma = 4,
    .rx_dma = 12,
    .cs_num = 1,
    .cs_gpio[0] = 22,
    .hbits = true,
    .mclk = "spi0_mclk",
  };

  g_spi[config.bus] = dw_spi_initialize(&config, g_ioe[0], g_dma[0]);
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
    }
  };
  int config_num = sizeof(config) / sizeof(config[0]);

  dw_i2c_allinitialize(config, config_num, g_i2c);
}
#endif

#ifdef CONFIG_SONG_PINCTRL
static void up_pinctrl_init(void)
{
  static const struct pinctrl_mapping_s mapping[] =
  {
    { 0, 0x00}, { 1, 0x04}, { 2, 0x08}, { 3, 0x0C},
    { 4, 0x10}, { 5, 0x1C}, { 6, 0x20}, { 7, 0x2C},
    { 8, 0x30}, { 9, 0x34}, {10, 0x38}, {11, 0x3C},
    {12, 0x40}, {13, 0x44}, {14, 0x48}, {15, 0x4C},
    {16, 0x50}, {17, 0x54}, {18, 0x58}, {19, 0x5C},
    {20, 0x60}, {21, 0x6C}, {22, 0x70}, {23, 0x74},
    {24, 0x78}, {25, 0x7C}, {26, 0x80}, {27, 0x84},
    {28, 0x88}, {29, 0x8C}, {30, 0x90}, {31, 0x94},
    {32, 0x98}, {.pin = UINT32_MAX,},
  };

  g_pinctrl[0] = song_pinctrl_initialize(0xb0050000, mapping);
  pinctrl_register(g_pinctrl[0], 0);
}
#endif

static void up_extra_init(void)
{
}

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

#ifdef CONFIG_PWM_SONG
  song_pwm_initialize(0, 0xb0100000, 4, "pwm_mclk");
#endif

#ifdef CONFIG_SPI_DW
  up_spi_init();
#endif

#ifdef CONFIG_I2C_DW
  up_i2c_init();
#endif

#ifdef CONFIG_SONG_PINCTRL
  up_pinctrl_init();
#endif

  up_extra_init();
}

static int up_top_pwr_isr(int irq, FAR void *context, FAR void *arg)
{
  if (getreg32(TOP_PWR_INTR_ST_AP_M4) & TOP_PWR_SLP_U1RXD_ACT)
    {
      putreg32(TOP_PWR_SLP_U1RXD_ACT, TOP_PWR_INTR_ST_AP_M4);
#if defined(CONFIG_PM) && defined(CONFIG_SERIAL_CONSOLE)
      pm_activity(CONFIG_SERIAL_PM_ACTIVITY_DOMAIN,
                  CONFIG_SERIAL_PM_ACTIVITY_PRIORITY);
#endif
    }

  return 0;
}

void up_finalinitialize(void)
{
  /* Attach TOP_PWR intr */

  irq_attach(2, up_top_pwr_isr, NULL);
  up_enable_irq(2);

  /* Enable SLP_U1RXD_ACT intr */

  modifyreg32(TOP_PWR_INTR_EN_AP_M4, 0, TOP_PWR_SLP_U1RXD_ACT);
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
  else if (status == 3)
    {
      /* Reset board to recovery */

      putreg32(TOP_PWR_RESET_RECOVERY, TOP_PWR_RES_REG2);
      putreg32(TOP_PWR_SFRST_RESET << 16 |
               TOP_PWR_SFRST_RESET, TOP_PWR_SFRST_CTL);
    }
  else
    {
      /* Reset board */

      putreg32(TOP_PWR_RESET_NORMAL, TOP_PWR_RES_REG2);
      putreg32(TOP_PWR_SFRST_RESET << 16 |
               TOP_PWR_SFRST_RESET, TOP_PWR_SFRST_CTL);
    }
}

static void up_cpu_lp(bool pwr_sleep, bool ds_sleep)
{
  /* Allow PWR_SLEEP (VDDMAIN ON)? */

  if (pwr_sleep)
    modifyreg32(TOP_PWR_CK802_CTL0, 0, TOP_PWR_CK802_SLP_EN);
  else
    modifyreg32(TOP_PWR_CK802_CTL0, TOP_PWR_CK802_SLP_EN, 0);

  /* Allow DS_SLEEP (VDDMAIN OFF)? */

  if (ds_sleep)
    putreg32(TOP_PWR_AP_M4_DS_SLP_EN << 16 |
             TOP_PWR_AP_M4_DS_SLP_EN, TOP_PWR_SLPCTL_AP_M4);
  else
    putreg32(TOP_PWR_AP_M4_DS_SLP_EN << 16, TOP_PWR_SLPCTL_AP_M4);
}

void up_cpu_doze(void)
{
  up_cpu_lp(false, false);
  __STOP();
}

void up_cpu_idle(void)
{
  up_cpu_doze();
}

void up_cpu_standby(void)
{
  up_cpu_lp(true, false);
  __STOP();
}

void up_cpu_sleep(void)
{
  up_cpu_lp(true, true);
  __STOP();
}

#endif /* CONFIG_ARCH_CHIP_U11_CK */