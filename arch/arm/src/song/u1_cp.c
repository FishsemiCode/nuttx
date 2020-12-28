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

#ifdef CONFIG_ARCH_CHIP_U1_CP

#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/misc/misc_rpmsg.h>
#include <nuttx/power/regulator.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/song_oneshot.h>
#include <nuttx/timers/song_rtc.h>
#include <nuttx/analog/song_adc.h>
#include <nuttx/crashdump/dumpfile.h>

#include <crc32.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#include "chip.h"
#include "nvic.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"
#include "u1_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                 "ap"
#define CPU_NAME_SP                 "sp"
#define CPU_INDEX_AP                0
#define CPU_INDEX_CP                1
#define CPU_INDEX_SP                2

#define LOGBUF_BASE                 ((uintptr_t)&_slog)
#define LOGBUF_SIZE                 ((uint32_t)&_logsize)

#define RSVDMEM_MAGIC               (0x1234abcd)

#define TOP_MAILBOX_BASE            (0xb0030000)

#define TOP_PWR_BASE                (0xb0040000)
#define TOP_PWR_CP_M4_RSTCTL        (TOP_PWR_BASE + 0x0dc)
#define TOP_PWR_SFRST_CTL           (TOP_PWR_BASE + 0x11c)
#define TOP_PWR_CP_M4_INTR2SLP_MK0  (TOP_PWR_BASE + 0x150)
#define TOP_PWR_CP_UNIT_PD_CTL      (TOP_PWR_BASE + 0x1fc)
#define TOP_PWR_RES_REG2            (TOP_PWR_BASE + 0x260)
#define TOP_PWR_BOOT_REG            (TOP_PWR_BASE + 0x290)
#define TOP_PWR_SLPCTL1             (TOP_PWR_BASE + 0x354)
#define TOP_PWR_SLPCTL_CP_M4        (TOP_PWR_BASE + 0x35c)
#define TOP_PWR_CP_M4_TCM_PD_CTL0   (TOP_PWR_BASE + 0x3e0)

#define TOP_PWR_CP_M4_SFRST         (1 << 4)
#define TOP_PWR_CP_M4_IDLE_MK       (1 << 5)

#define TOP_PWR_SFRST_RESET         (1 << 0)

#define TOP_PWR_CP_M4_PD_MK         (1 << 3)
#define TOP_PWR_CP_M4_AU_PU_MK      (1 << 6)
#define TOP_PWR_CP_M4_AU_PD_MK      (1 << 7)

#define TOP_PWR_RESET_NORMAL        (0x00000000)
#define TOP_PWR_RESET_ROMBOOT       (0xaaaa1234)
#define TOP_PWR_RESET_RECOVERY      (0xbbbb1234)

#define TOP_PWR_CP_M4_COLD_BOOT     (1 << 2)

#define TOP_PWR_RF_TP_TM2_SLP_MK    (1 << 8)
#define TOP_PWR_RF_TP_CALIB_SLP_MK  (1 << 9)

#define TOP_PWR_CP_M4_SLP_EN        (1 << 0)
#define TOP_PWR_CP_M4_SLP_MK        (1 << 1)
#define TOP_PWR_CP_M4_DS_SLP_EN     (1 << 2)

#define TOP_PWR_CP_M4_TCM_AU_PD_MK  (1 << 7)

/****************************************************************************
 * Private Types
 ****************************************************************************/

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

extern uint32_t _cpram1_srsvd;
extern uint32_t _cpram1_ersvd;

extern uint32_t _ssharmv2;
extern uint32_t _esharmv2;

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

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_MISC_RPMSG
static void up_misc_init(void)
{
  uint32_t size = (uintptr_t)&_cpram1_ersvd - (uintptr_t)&_cpram1_srsvd;
  void *base = &_cpram1_srsvd;
  int fd;

  /* Clear the memory if not wakeup from DS */

  if (!up_is_warm_rstn())
    {
      memset(base, 0, size);
    }

  /* Retention init */

  misc_rpmsg_initialize(CPU_NAME_SP, true);

  /* Add ram-misc block */

  fd = open("/dev/miscsp", 0);
  if (fd >= 0)
    {
      struct misc_retent_add_s add =
        {
          .blkid = 0,
          .base  = base,
          .size  = size,
          .dma   = 1,
        };

      ioctl(fd, MISC_RETENT_ADD, (unsigned long)&add);

      /* Get board-id env from sp */

      struct misc_remote_envsync_s env =
        {
          .name = "board-id",
        };

      ioctl(fd, MISC_REMOTE_ENVSYNC, (unsigned long)&env);
      close(fd);
    }
}
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlystart(void)
{
  if (!(getreg32(TOP_PWR_BOOT_REG) & TOP_PWR_CP_M4_COLD_BOOT))
    {
      /* CP core auto power on */

      up_cpu_restore();
    }

  putreg32(TOP_PWR_CP_M4_COLD_BOOT << 16, TOP_PWR_BOOT_REG);
}

void up_earlyinitialize(void)
{
  /* Unmask SLEEPING for reset */

  putreg32(TOP_PWR_CP_M4_IDLE_MK << 16, TOP_PWR_CP_M4_RSTCTL);

  /* Set RF & SLP init value */

  putreg32(TOP_PWR_RF_TP_TM2_SLP_MK << 16 |
           TOP_PWR_RF_TP_CALIB_SLP_MK << 16, TOP_PWR_SLPCTL1);
  putreg32(TOP_PWR_CP_M4_SLP_EN << 16 |
           TOP_PWR_CP_M4_SLP_MK << 16 |
           TOP_PWR_CP_M4_DS_SLP_EN << 16, TOP_PWR_SLPCTL_CP_M4);

#ifndef CONFIG_CPULOAD_PERIOD
  /* Allow TCM to LP, careful with it. At this time,
   * if use systick as wakeup reason form DEEPSLEEP, CPU will hang.
   */

  putreg32(TOP_PWR_CP_M4_TCM_AU_PD_MK << 16, TOP_PWR_CP_M4_TCM_PD_CTL0);
#endif

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(CPU_NAME_AP, (void *)LOGBUF_BASE, LOGBUF_SIZE);
#endif
}

void up_wic_initialize(void)
{
  putreg32(0xffffffff, TOP_PMICFSM_CP_M4_INT2SLP_MK0);
}

void up_wic_enable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PMICFSM_CP_M4_INT2SLP_MK0, 1 << (irq - NVIC_IRQ_FIRST), 0);
    }
}

void up_wic_disable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PMICFSM_CP_M4_INT2SLP_MK0, 0, 1 << (irq - NVIC_IRQ_FIRST));
    }
}

void up_dma_initialize(void)
{
#ifdef CONFIG_SONG_DMAS
  g_dma[0] = song_dmas_initialize(0, 0xb0020000, 28, NULL);
#endif
}

#if defined(CONFIG_16550_UART) && defined(CONFIG_SONG_DMAS)
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
  return g_dma[0] ? DMA_GET_CHAN(g_dma[0], ident) : NULL;
}
#endif

void up_timer_initialize(void)
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
      .calib_inc  = 0x198,
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

#ifdef CONFIG_SONG_CLK
  up_clk_initialize();
#endif
}

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  uart_rpmsg_init(CPU_NAME_AP, "CP", 256, true);
  uart_rpmsg_init(CPU_NAME_AP, "AT", 1024, false);
  uart_rpmsg_init(CPU_NAME_AP, "AT1", 256, false);
  uart_rpmsg_init(CPU_NAME_AP, "GPS", 256, false);
  uart_rpmsg_init(CPU_NAME_AP, "GPS1", 256, false);

#ifdef CONFIG_NBIOT_HLS_SOFTSIM

#ifdef CONFIG_SOFTSIM_ON_CHIP_SP
  uart_rpmsg_init(CPU_NAME_SP, "AT3", 256, false);
#else
  uart_rpmsg_init(CPU_NAME_AP, "AT3", 256, false);
#endif

#endif
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
      .irq        = -1,
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
      .irq        = 21,
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

#ifdef CONFIG_SONG_RPTUN
static void up_rptun_init(void)
{
  static const struct song_rptun_config_s rptun_cfg_ap =
  {
    .cpuname = CPU_NAME_AP,
    .rsc     = (struct rptun_rsc_s *)0xb0003400,
    .vringtx = 3,
    .vringrx = 3,
  };

  static const struct song_rptun_config_s rptun_cfg_sp =
  {
    .cpuname = CPU_NAME_SP,
    .rsc     = (struct rptun_rsc_s *)0xb0010000,
    .vringtx = 1,
    .vringrx = 1,
  };

  song_rptun_initialize(&rptun_cfg_ap, g_mbox[CPU_INDEX_AP], g_mbox[CPU_INDEX_CP]);
  song_rptun_initialize(&rptun_cfg_sp, g_mbox[CPU_INDEX_SP], g_mbox[CPU_INDEX_CP]);

#  ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize();
#  endif

#  ifdef CONFIG_RPMSG_REGULATOR
  rpmsg_regulator_init(CPU_NAME_SP, false);
#  endif

#  ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#  endif

#  ifdef CONFIG_FS_HOSTFS_RPMSG
  hostfs_rpmsg_init(CPU_NAME_SP);
#  endif

#  ifdef CONFIG_MISC_RPMSG
  up_misc_init();
#  endif
}
#endif

#ifdef CONFIG_RTC_SONG
int up_rtc_initialize(void)
{
  static const struct song_rtc_config_s config =
  {
    .minor   = 0,
    .base    = 0xb2020000,
    .irq     = 16,
    .index   = 0,
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
    .base = 0xb0080000,
    .irq  = 20,
    .tclk = "cpwdt_tclk",
  };

  dw_wdt_initialize(&config);
}
#endif

#ifdef CONFIG_SONG_IOE
void up_ioe_init(void)
{
  static const struct song_ioe_config_s cfg =
  {
    .cpu  = 0,
    .base = 0xb0060000,
    .irq  = 19,
  };

  g_ioe[0] = song_ioe_initialize(&cfg);
}
#endif

#ifdef CONFIG_ADC_SONG
void up_adc_init(void)
{
  static const struct song_adc_config_s cfg =
  {
    .minor  = 0,
    .base = 0xb0180000,
    .ctl_off = 0xf0,
    .sta_off = 0xd4,
    .clk_off = 0x68,
  };

  song_adc_register(&cfg);
}
#endif

void up_extra_init(void)
{
  /* Set RFIF IRQ SUB PRIORITY */

  up_prioritize_irq(31, NVIC_SYSH_PRIORITY_DEFAULT - NVIC_SYSH_PRIORITY_SUBSTEP);

  /* Set start reason to env */

  setenv("START_REASON", up_get_wkreason_env(), 1);
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

#ifdef CONFIG_ADC_SONG
  up_adc_init();
#endif

  up_extra_init();
}

void up_finalinitialize(void)
{
#ifdef CONFIG_SONG_CLK
  up_clk_finalinitialize();
#endif

#ifdef CONFIG_CRASH_DUMPFILE
  dumpfile_initialize("cp", (char *)&_ssharmv2, \
                      ((uintptr_t)&_esharmv2) - ((uintptr_t)&_ssharmv2));
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
      /* Reset current core.
       * Set boot flag to fake up_earlystart this is a cold reset.
       */

      putreg32(TOP_PWR_CP_M4_COLD_BOOT << 16 |
               TOP_PWR_CP_M4_COLD_BOOT, TOP_PWR_BOOT_REG);

      putreg32(TOP_PWR_CP_M4_SFRST << 16 |
               TOP_PWR_CP_M4_SFRST, TOP_PWR_CP_M4_RSTCTL);
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

static void up_cpu_lp(bool deep_sleep, bool pwr_sleep)
{
  /* Allow ARM to enter deep sleep in WFI? */

  if (deep_sleep)
    putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);
  else
    putreg32(getreg32(NVIC_SYSCON) & ~NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  /* Allow PWR_SLEEP (VDDMAIN ON)?
   * CP auto power down is enabled in PWR_SLEEP.
   */

  if (pwr_sleep)
    {
      putreg32(TOP_PWR_CP_M4_SLP_EN << 16 |
               TOP_PWR_CP_M4_SLP_EN, TOP_PWR_SLPCTL_CP_M4);
      putreg32(TOP_PWR_CP_M4_AU_PD_MK << 16, TOP_PWR_CP_UNIT_PD_CTL);
    }
  else
    {
      putreg32(TOP_PWR_CP_M4_SLP_EN << 16, TOP_PWR_SLPCTL_CP_M4);
      putreg32(TOP_PWR_CP_M4_AU_PD_MK << 16 |
               TOP_PWR_CP_M4_AU_PD_MK, TOP_PWR_CP_UNIT_PD_CTL);
    }
}

static void up_cpu_ds(bool ds_sleep)
{
  /* Allow DS_SLEEP (VDDMAIN OFF)? */

  if (ds_sleep)
    {
      putreg32(TOP_PWR_CP_M4_DS_SLP_EN << 16 |
               TOP_PWR_CP_M4_DS_SLP_EN, TOP_PWR_SLPCTL_CP_M4);
      putreg32(TOP_PWR_RF_TP_TM2_SLP_MK << 16 |
               TOP_PWR_RF_TP_TM2_SLP_MK |
               TOP_PWR_RF_TP_CALIB_SLP_MK << 16 |
               TOP_PWR_RF_TP_CALIB_SLP_MK, TOP_PWR_SLPCTL1);
    }
  else
    {
      putreg32(TOP_PWR_CP_M4_DS_SLP_EN << 16, TOP_PWR_SLPCTL_CP_M4);
      putreg32(TOP_PWR_RF_TP_TM2_SLP_MK << 16 |
               TOP_PWR_RF_TP_CALIB_SLP_MK << 16, TOP_PWR_SLPCTL1);
    }
}

void up_cpu_doze(void)
{
  up_cpu_lp(false, false);
  up_cpu_wfi();
}

void up_cpu_idle(void)
{
  up_cpu_lp(true, false);
  up_cpu_wfi();
}

void up_cpu_standby(void)
{
  up_cpu_lp(true, true);
  up_cpu_save();
}

void up_cpu_sleep(void)
{
  up_cpu_ds(true);
  up_cpu_standby();
  up_cpu_ds(false);
}

#endif /* CONFIG_ARCH_CHIP_U1_CP */
