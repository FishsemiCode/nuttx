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

#include <crc32.h>

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
#include <nuttx/timers/dw_wdt.h>
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

#define CPU_NAME_AP                 "ap"
#define CPU_NAME_SP                 "sp"

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

#define TOP_PWR_CP_M4_COLD_BOOT     (1 << 2)

#define TOP_PWR_RF_TP_TM2_SLP_MK    (1 << 8)
#define TOP_PWR_RF_TP_CALIB_SLP_MK  (1 << 9)

#define TOP_PWR_CP_M4_SLP_EN        (1 << 0)
#define TOP_PWR_CP_M4_DS_SLP_EN     (1 << 2)

#define TOP_PWR_CP_M4_TCM_AU_PD_MK  (1 << 7)

#define TOP_PMICFSM_BASE            (0xb2010000)
#define TOP_PMICFSM_WAKEUP_REASON   (TOP_PMICFSM_BASE + 0x18)

#define TOP_PMICFSM_FIRST_PON       (1 << 0)
#define TOP_PMICFSM_PON             (1 << 1)
#define TOP_PMICFSM_UART            (1 << 2)
#define TOP_PMICFSM_RTC             (1 << 3)
#define TOP_PMICFSM_GPIO0           (1 << 4)
#define TOP_PMICFSM_GPIO1           (1 << 5)
#define TOP_PMICFSM_GPIO2           (1 << 6)
#define TOP_PMICFSM_GPIO3           (1 << 7)
#define TOP_PMICFSM_WDT_RSTN        (1 << 8)
#define TOP_PMICFSM_SOFT_RSTN       (1 << 9)
#define TOP_PMICFSM_BUTTON_RSTN     (1 << 10)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct rsvdmem_head_s
{
  uint32_t magic;
  uint32_t size;
  uint32_t crc;
};

enum start_reason_e
{
  START_REASON_PON_RSTN,
  START_REASON_CPU_PD,
  START_REASON_SOC_PD,
  START_REASON_MAX
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_DMA
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

extern struct rsvdmem_head_s _cpram1_srsvd;
extern uint8_t _cpram1_ersvd;

#ifdef CONFIG_SONG_IOE
FAR struct ioexpander_dev_s *g_ioe[2] =
{
  [1] = DEV_END,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static enum start_reason_e up_get_startreason(void)
{
  if (getreg32(TOP_PWR_BOOT_REG) & TOP_PWR_CP_M4_COLD_BOOT)
    {
      if ((getreg32(TOP_PMICFSM_WAKEUP_REASON) &
                    (TOP_PMICFSM_UART  |
                     TOP_PMICFSM_RTC   |
                     TOP_PMICFSM_GPIO0 |
                     TOP_PMICFSM_GPIO1 |
                     TOP_PMICFSM_GPIO2 |
                     TOP_PMICFSM_GPIO3)) != 0)
        {
          return START_REASON_SOC_PD;
        }
      else
        {
          return START_REASON_PON_RSTN;
        }
    }
  else
    {
      return START_REASON_CPU_PD;
    }
}

static void up_init_startreason(void)
{
    static const char *start_reason_env[START_REASON_MAX] =
    {
      [START_REASON_PON_RSTN] = "pon_rstn",
      [START_REASON_CPU_PD]   = "cpu_pd",
      [START_REASON_SOC_PD]   = "soc_pd",
    };

    setenv("START_REASON", start_reason_env[up_get_startreason()], 1);

    /* Clear cold boot flag. */

    putreg32(TOP_PWR_CP_M4_COLD_BOOT << 16, TOP_PWR_BOOT_REG);
}

static size_t up_rsvdmem_size(void)
{
  return &_cpram1_ersvd - (uint8_t *)(&_cpram1_srsvd + 1);
}

static uint32_t up_rsvdmem_crc(void)
{
  return crc32((uint8_t *)(&_cpram1_srsvd + 1), up_rsvdmem_size());
}

static void up_rsvdmem_init(enum start_reason_e start_reason)
{
  struct rsvdmem_head_s *head = &_cpram1_srsvd;

  /* Clean up reserve memory area in case of initial power up, or
   * corrupted memory.
   */

  if (start_reason == START_REASON_PON_RSTN ||
      head->magic  != RSVDMEM_MAGIC         ||
      head->size   != up_rsvdmem_size()     ||
      head->crc    != up_rsvdmem_crc())
    {
      memset(head + 1, 0, up_rsvdmem_size());
    }
}

static void up_rsvdmem_sleep(void)
{
  struct rsvdmem_head_s *head = &_cpram1_srsvd;

  /* Update reserve memory header before entering sleep. */

  head->magic = RSVDMEM_MAGIC;
  head->size  = up_rsvdmem_size();
  head->crc   = up_rsvdmem_crc();
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlystart(void)
{
  enum start_reason_e start_reason = up_get_startreason();

  switch (start_reason)
    {
      case START_REASON_CPU_PD:
        up_cpu_restore();
        break;

      case START_REASON_PON_RSTN:
      case START_REASON_SOC_PD:
        up_rsvdmem_init(start_reason);
        break;

      default:
        break;
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

  /* Unmask SLEEPING for reset */
  putreg32(TOP_PWR_CP_M4_IDLE_MK << 16, TOP_PWR_CP_M4_RSTCTL);

  /* Always allow enter FLASH_S */
  putreg32(TOP_PWR_CP_M4_DS_SLP_EN << 16 |
           TOP_PWR_CP_M4_DS_SLP_EN, TOP_PWR_SLPCTL_CP_M4);

  /* Enable TCM auto low power */
  putreg32(TOP_PWR_CP_M4_TCM_AU_PD_MK << 16, TOP_PWR_CP_M4_TCM_PD_CTL0);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(CPU_NAME_AP, (void *)LOGBUF_BASE, LOGBUF_SIZE);
#endif
}

#ifdef CONFIG_RTC_SONG
int up_rtc_initialize(void)
{
  static const struct song_rtc_config_s config =
  {
    .minor = 0,
    .base  = 0xb2020000,
    .irq   = 16,
    .index = 0,
  };

  up_rtc_set_lowerhalf(song_rtc_initialize(&config));
  return 0;
}
#endif

void up_wic_initialize(void)
{
  putreg32(0xffffffff, TOP_PWR_CP_M4_INTR2SLP_MK0);
}

void up_wic_enable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_CP_M4_INTR2SLP_MK0, 1 << (irq - NVIC_IRQ_FIRST), 0);
    }
}

void up_wic_disable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_CP_M4_INTR2SLP_MK0, 0, 1 << (irq - NVIC_IRQ_FIRST));
    }
}

#ifdef CONFIG_ARCH_DMA
void up_dmainitialize(void)
{
#  ifdef CONFIG_SONG_DMAS
  g_dma[0] = song_dmas_initialize(0, 0xb0020000, 28, "sp/dmas_hclk");
#  endif
}
#endif

#ifdef CONFIG_16550_UART
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
#  ifdef CONFIG_ARCH_DMA
  return g_dma[0] ? DMA_GET_CHAN(g_dma[0], ident) : NULL;
#  else
  return NULL;
#  endif
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
#  ifdef CONFIG_SERIAL_CONSOLE
  uart_rpmsg_init(CPU_NAME_AP, "CP", 1024, false);
#  else
  uart_rpmsg_init(CPU_NAME_AP, "CP", 1024, true);
#  endif
  uart_rpmsg_init(CPU_NAME_AP, "AT", 1024, false);
  uart_rpmsg_init(CPU_NAME_AP, "GPS", 1024, false);
}
#endif

#ifdef CONFIG_OPENAMP
static void up_openamp_initialize(void)
{
  struct mbox_dev_s *mbox_ap, *mbox_cp, *mbox_sp;

  static const struct song_mbox_config_s mbox_cfg_ap =
  {
    .base       = TOP_MAILBOX_BASE,
    .set_off    = 0x10,
    .en_off     = 0x14,
    .en_bit     = 16,
    .src_en_off = 0x14,
    .sta_off    = 0x18,
    .chnl_count = 16,
    .irq        = -1,
  };

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

  static const struct song_rptun_config_s rptun_cfg_ap =
  {
    .cpu_name    = CPU_NAME_AP,
    .role        = RPMSG_REMOTE,
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
    .ch_start_tx = 0,
    .ch_vring_tx = 1,
    .ch_start_rx = 0,
    .ch_vring_rx = 1,
    .rsc         =
    {
      .rsc_tab   = (void *)0xb0010000,
      .size      = sizeof(struct rptun_rsc_s),
    },
  };

  mbox_ap = song_mbox_initialize(&mbox_cfg_ap);
  mbox_cp = song_mbox_initialize(&mbox_cfg_cp);
  mbox_sp = song_mbox_initialize(&mbox_cfg_sp);

  song_rptun_initialize(&rptun_cfg_ap, mbox_ap, mbox_cp);
  song_rptun_initialize(&rptun_cfg_sp, mbox_sp, mbox_cp);

#  ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#  endif

#  ifdef CONFIG_FS_HOSTFS_RPMSG
  hostfs_rpmsg_init(CPU_NAME_SP);
#  endif
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

void up_lateinitialize(void)
{
  up_init_startreason();

#ifdef CONFIG_OPENAMP
  up_openamp_initialize();
#endif

#ifdef CONFIG_SONG_CLK
  up_clk_initialize();
#endif

#ifdef CONFIG_WATCHDOG_DW
  up_wdtinit();
#endif

#ifdef CONFIG_SONG_IOE
  g_ioe[0] = song_ioe_initialize(0, 0xb0060000, 19);
#endif

#ifdef CONFIG_RPMSG_REGULATOR
  rpmsg_regulator_init(CPU_NAME_SP, 0);
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
  putreg32(TOP_PWR_CP_M4_SLP_EN << 16, TOP_PWR_SLPCTL_CP_M4);
  putreg32(TOP_PWR_RF_TP_TM2_SLP_MK << 16 |
           TOP_PWR_RF_TP_CALIB_SLP_MK << 16, TOP_PWR_SLPCTL1);

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
  putreg32(TOP_PWR_CP_M4_SLP_EN << 16, TOP_PWR_SLPCTL_CP_M4);
  putreg32(TOP_PWR_RF_TP_TM2_SLP_MK << 16 |
           TOP_PWR_RF_TP_CALIB_SLP_MK << 16, TOP_PWR_SLPCTL1);

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
  putreg32(TOP_PWR_CP_M4_SLP_EN << 16, TOP_PWR_SLPCTL_CP_M4);
  putreg32(TOP_PWR_RF_TP_TM2_SLP_MK << 16 |
           TOP_PWR_RF_TP_CALIB_SLP_MK << 16, TOP_PWR_SLPCTL1);

  /* Allow the subsystem power down */
  putreg32(TOP_PWR_CP_M4_AU_PD_MK << 16, TOP_PWR_CP_UNIT_PD_CTL);

  /* Allow the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_save();
}

void up_cpu_sleep(void)
{
  up_rsvdmem_sleep();

  /* Allow the full chip power down */
  putreg32(TOP_PWR_CP_M4_SLP_EN << 16 |
           TOP_PWR_CP_M4_SLP_EN, TOP_PWR_SLPCTL_CP_M4);
  putreg32(TOP_PWR_RF_TP_TM2_SLP_MK << 16 |
           TOP_PWR_RF_TP_TM2_SLP_MK |
           TOP_PWR_RF_TP_CALIB_SLP_MK << 16 |
           TOP_PWR_RF_TP_CALIB_SLP_MK, TOP_PWR_SLPCTL1);

  /* Allow the subsystem power down */
  putreg32(TOP_PWR_CP_M4_AU_PD_MK << 16, TOP_PWR_CP_UNIT_PD_CTL);

  /* Allow the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_save();
}

#endif /* CONFIG_ARCH_CHIP_U1_CP */