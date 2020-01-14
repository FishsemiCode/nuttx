/****************************************************************************
 * arch/arm/src/song/u1_sp.c
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

#ifdef CONFIG_ARCH_CHIP_U1_SP

#include <nuttx/arch.h>
#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/drivers/addrenv.h>
#include <nuttx/drivers/ramdisk.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/fs/partition.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/misc/misc_rpmsg.h>
#include <nuttx/mtd/song_onchip_flash.h>
#include <nuttx/net/rpmsgdrv.h>
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
#include <nuttx/crashdump/dumpfile.h>
#include <sys/mount.h>

#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>

#include "chip.h"
#include "nvic.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"
#include "u1_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONFIG_BOARD_LOOPSPER10USEC  ((CONFIG_BOARD_LOOPSPERMSEC+50)/100)
#define CONFIG_BOARD_LOOPSPERUSEC    ((CONFIG_BOARD_LOOPSPERMSEC+500)/1000)

#ifdef CONFIG_RAMDISK
#define CP_RVSD_FILE                "/disksp/retent/cpram1.rsvd"
#else
#define CP_RVSD_FILE                "/persist/cpram1.rsvd"
#endif

#define CPU_NAME_AP                 "ap"
#define CPU_NAME_CP                 "cp"
#define CPU_INDEX_AP                0
#define CPU_INDEX_CP                1
#define CPU_INDEX_SP                2

#define LOGBUF_BASE                 ((uintptr_t)&_slog)
#define LOGBUF_SIZE                 ((uint32_t)&_logsize)

#define CPRAM1_RSVD_BASE            (0x60054000)
#define CPRAM1_RSVD_SIZE            (CPRAM1_RSVD_BASE + 0x4)

#define TOP_MAILBOX_BASE            (0xb0030000)

#define TOP_PWR_BASE                (0xb0040000)
#define TOP_PWR_TOP_PCLK1_CTL       (TOP_PWR_BASE + 0x04c)
#define TOP_PWR_AP_M4_RSTCTL        (TOP_PWR_BASE + 0x0e0)
#define TOP_PWR_SEC_M4_RSTCTL       (TOP_PWR_BASE + 0x0d8)
#define TOP_PWR_CP_M4_RSTCTL        (TOP_PWR_BASE + 0x0dc)
#define TOP_PWR_WDTRST_CTL          (TOP_PWR_BASE + 0x114)
#define TOP_PWR_SFRST_CTL           (TOP_PWR_BASE + 0x11c)
#define TOP_PWR_INTR_EN_SEC_M4_1    (TOP_PWR_BASE + 0x140)
#define TOP_PWR_INTR_ST_SEC_M4_1    (TOP_PWR_BASE + 0x144)
#define TOP_PWR_SEC_M4_INTR2SLP_MK0 (TOP_PWR_BASE + 0x148)
#define TOP_PWR_CP_UNIT_PD_CTL      (TOP_PWR_BASE + 0x1fc)
#define TOP_PWR_AP_M4_CTL0          (TOP_PWR_BASE + 0x218)
#define TOP_PWR_CK802_CTL0          (TOP_PWR_BASE + 0x22c)
#define TOP_PWR_RES_REG2            (TOP_PWR_BASE + 0x260)
#define TOP_PWR_SLPCTL0             (TOP_PWR_BASE + 0x350)
#define TOP_PWR_SLPCTL1             (TOP_PWR_BASE + 0x354)
#define TOP_PWR_SLPCTL_SEC_M4       (TOP_PWR_BASE + 0x358)
#define TOP_PWR_SLPCTL_CP_M4        (TOP_PWR_BASE + 0x35c)
#define TOP_PWR_SLPCTL_AP_M4        (TOP_PWR_BASE + 0x360)
#define TOP_PWR_PLLTIME             (TOP_PWR_BASE + 0x36c)
#define TOP_PWR_FLASH_PD_CTL        (TOP_PWR_BASE + 0x3f0)
#define TOP_PWR_CP_M4_TCM_PD_CTL0   (TOP_PWR_BASE + 0x3e0)
#define TOP_PWR_SEC_M4_TCM_PD_CTL   (TOP_PWR_BASE + 0x3e8)

#define TOP_PWR_AP_M4_PORESET       (1 << 0)

#define TOP_PWR_SEC_SFRST           (1 << 4)
#define TOP_PWR_SEC_IDLE_MK         (1 << 5)

#define TOP_PWR_CP_M4_PORESET       (1 << 0)

#define TOP_PWR_SFRST_RESET         (1 << 0)

#define TOP_PWR_SLPU_FLASH_S        (1 << 0)
#define TOP_PWR_CP_DS_WAKEUP        (1 << 4)
#define TOP_PWR_AP_DS_WAKEUP        (1 << 5)

#define TOP_PWR_CP_PD_EN            (1 << 0)
#define TOP_PWR_CP_WK_UP            (1 << 1)
#define TOP_PWR_CP_HW_IDLE_MK       (1 << 8)

#define TOP_PWR_AP_CPU_SEL          (1 << 22)

#define TOP_PWR_CPU_RSTCTL          (1 << 1)
#define TOP_PWR_CPUCLK_EN           (1 << 0)

#define TOP_PWR_RESET_NORMAL        (0x00000000)
#define TOP_PWR_RESET_ROMBOOT       (0xaaaa1234)
#define TOP_PWR_RESET_RECOVERY      (0xbbbb1234)

#define TOP_PWR_SLP_DMA_MK          (1 << 2)

#define TOP_PWR_SLP_RF_MASK         (3 << 8)

#define TOP_PWR_SEC_M4_SLP_EN       (1 << 0)
#define TOP_PWR_SEC_M4_DS_SLP_EN    (1 << 2)
#define TOP_PWR_FLASH_S_2PD_JMP     (1 << 8)

#define TOP_PWR_CP_SLP_MASK         (7 << 0)

#define TOP_PWR_AP_SLP_MASK         (7 << 0)

#define TOP_PWR_CTRL_MODE           (1 << 0)

#define TOP_PWR_CP_AU_PD_MK         (1 << 7)

#define TOP_PWR_SEC_AU_PD_MK        (1 << 7)

#define TOP_PWR_CPWDT_BITED         (1 << 10)
#define TOP_PWR_APWDT_BITED         (1 << 9)
#define TOP_PWR_SPWDT_BITED         (1 << 8)
#define TOP_PWR_WDT_BITED_MASK      (TOP_PWR_CPWDT_BITED | TOP_PWR_APWDT_BITED | TOP_PWR_SPWDT_BITED)

#define TOP_PWR_PLL_STABLE_TIME     (0xf << 8)
#define TOP_PWR_OSC_STABLE_TIME     (0x52 << 0)

#define SECURITY_BASE               (0xb0150000)
#define SECURITY_CFG_0              (SECURITY_BASE + 0x30)
#define SECURITY_CFG_0_VALUE        (0x00010000)

#define MUX_PIN09                   (0xb0050014)
#define MUX_PIN23                   (0xb005004c)
#define MUX_PIN24                   (0xb0050050)
#define MUX_PIN25                   (0xb0050054)
#define MUX_PIN26                   (0xb0050058)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_SONG_DMAS
static FAR struct dma_dev_s *g_dma[2] =
{
  [1] = DEV_END,
};
#endif

#ifdef CONFIG_MISC_RPMSG
static FAR struct misc_dev_s *g_misc[3] =
{
  [2] = DEV_END,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _slog;
extern uint32_t _logsize;

extern uint32_t _ssharmv1;
extern uint32_t _esharmv1;

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

#ifdef CONFIG_SPI_DW
FAR struct spi_dev_s *g_spi[3] =
{
  [2] = DEV_END,
};
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

static void udelay_coarse(useconds_t microseconds)
{
  volatile int i;

  /* Cause this is called very early, and no global value this time.
   * So can't use up_udelay().
   */

  while (microseconds > 10)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPER10USEC; i++)
        {
        }

      microseconds -= 10;
    }

  while (microseconds > 0)
    {
      for (i = 0; i < CONFIG_BOARD_LOOPSPERUSEC; i++)
        {
        }

      microseconds--;
    }
}

void up_earlystart(void)
{
  if (up_is_u1v1())
    {
      int i;

      /* Set freq of TOP_PCLK1 low */

      putreg32(0xf000f0, TOP_PWR_TOP_PCLK1_CTL);
      udelay_coarse(5);

      /* VDDAON(LDO0) runs at low voltage in deep sleep. Recover the voltage
       * as early as possible.
       */

      for (i = 0; i < 1000; i++)
        {
          /* WA - writing to TOP_PMICFSM on power up seems to be unstable. Read
           * the register back to make sure it's programmed correctly.
           */

          putreg32(TOP_PMICFSM_LDO0_DEFAULT, TOP_PMICFSM_LDO0);

          if (getreg32(TOP_PMICFSM_LDO0) == TOP_PMICFSM_LDO0_DEFAULT)
            {
              break;
            }
        }

      /* Restore freq of TOP_PCLK1 */

      udelay_coarse(5);
      putreg32(0xf00090, TOP_PWR_TOP_PCLK1_CTL);
    }
  else
    {
      /* Set VDDMAIN 1.1V@work+0.8V@SLEEP+off@DS */

      putreg32(0xc2401, TOP_PMICFSM_BUCK1);

      /* Set VDDAON 1.1V@work and 0.7V@SLEEP */

      putreg32(0x41403, TOP_PMICFSM_LDO0);
    }
}

void up_earlyinitialize(void)
{
  static const struct simple_addrenv_s addrenv[] =
  {
    {.va = 0x21000000, .pa = 0xc1000000, .size = 0x00100000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  simple_addrenv_initialize(addrenv);

  /* Check whether jump to recovery.bin */

  if (getreg32(TOP_PWR_RES_REG2) == TOP_PWR_RESET_RECOVERY)
    {
      up_jump_to_recovery();
    }

#ifndef CONFIG_CPULOAD_PERIOD
  /* Allow TCM to LP, careful with it. At this time,
   * if use systick as wakeup reason form DEEPSLEEP, CPU will hang.
   */

  putreg32(TOP_PWR_SEC_AU_PD_MK << 16, TOP_PWR_SEC_M4_TCM_PD_CTL);
#endif

  if (up_is_u1v1())
    {
      /* Set the DMAS no effort to power down */

      putreg32(TOP_PWR_SLP_DMA_MK << 16 |
               TOP_PWR_SLP_DMA_MK, TOP_PWR_SLPCTL0);

      /* Configure PLL stable time (~1.15ms). */

      putreg32(TOP_PWR_PLL_STABLE_TIME | TOP_PWR_OSC_STABLE_TIME, TOP_PWR_PLLTIME);
    }
  else
    {
      /* Configure PLL stable time (~1.15ms). */

      putreg32(TOP_PMICFSM_PLL_STABLE_TIME |
               TOP_PMICFSM_OSC_STABLE_TIME, TOP_PMICFSM_PLLTIME);
      modifyreg32(TOP_PMICFSM_CONFIG2, 0, TOP_PMICFSM_DS_RF_RST_MK);
    }

  /* Set flash no effort to PWR_SLEEP */

  putreg32(TOP_PWR_CTRL_MODE << 16, TOP_PWR_FLASH_PD_CTL);

  /* Set PMICFSM disable full chip to DS */

  modifyreg32(TOP_PMICFSM_CONFIG1, TOP_PMICFSM_DS_SLP_VALID, 0);

  /* Set PMICFSM WAKEUP_ENABLE, now only support UART0 RTC wakeup DS */

  putreg32(TOP_PMICFSM_UART_ENABLE |
           TOP_PMICFSM_RTC_ENABLE, TOP_PMICFSM_WAKEUP_ENABLE);

  /* Set CP & RF no effect to power */

  putreg32(TOP_PWR_SLP_RF_MASK << 16 |
           TOP_PWR_SLP_RF_MASK, TOP_PWR_SLPCTL1);
  putreg32(TOP_PWR_CP_SLP_MASK << 16 |
           TOP_PWR_CP_SLP_MASK, TOP_PWR_SLPCTL_CP_M4);

  /* Set top shram1 enabled */

  putreg32(SECURITY_CFG_0_VALUE, SECURITY_CFG_0);

  if (up_is_warm_rstn())
    {
      /* Wake up from DS, resotre UART0 interrupt setting */

      modifyreg32(0xb2000004, 0x1, 0);
    }

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(CPU_NAME_AP, (void *)LOGBUF_BASE, LOGBUF_SIZE);
#endif
}

void up_wic_initialize(void)
{
  putreg32(0xffffffff, TOP_PWR_SEC_M4_INTR2SLP_MK0);
}

void up_wic_enable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_SEC_M4_INTR2SLP_MK0, 1 << (irq - NVIC_IRQ_FIRST), 0);
    }
}

void up_wic_disable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_SEC_M4_INTR2SLP_MK0, 0, 1 << (irq - NVIC_IRQ_FIRST));
    }
}

void up_dma_initialize(void)
{
#ifdef CONFIG_SONG_DMAS
  g_dma[0] = song_dmas_initialize(2, 0xb0020000, 28, "dmas_hclk");
#endif
}

#if defined(CONFIG_16550_UART) && defined(CONFIG_SONG_DMAS)
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
  if (up_is_u1v1())
    {
      return NULL;
    }

  return g_dma[0] ? DMA_GET_CHAN(g_dma[0], ident) : NULL;
}
#endif

void arm_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG

  if (up_is_u1v1())
    {
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
          .spec_off   = 0x1ac,
          .intren_off = 0x124,
          .intrst_off = 0x130,
          .intr_bit   = 2,
          .man_calib  = true,
          .man_calibv = 0xfa0000,
        };

      up_alarm_set_lowerhalf(song_oneshot_initialize(&config));
    }
  else
    {
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
          .spec_off   = 0x1ac,
          .intren_off = 0x124,
          .intrst_off = 0x130,
          .intr_bit   = 2,
        };

      up_alarm_set_lowerhalf(song_oneshot_initialize(&config));
    }
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
  uart_rpmsg_init(CPU_NAME_AP, "SP", 1024, true);
}
#endif

#ifdef CONFIG_NET_RPMSG_DRV
void up_netinitialize(void)
{
  net_rpmsg_drv_init(CPU_NAME_CP, "sl0", NET_LL_SLIP);
  net_rpmsg_drv_init(CPU_NAME_CP, "sl1", NET_LL_SLIP);
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
      .irq        = 21,
    }
  };

  song_mbox_allinitialize(config, ARRAY_SIZE(config), g_mbox);
}
#endif

#ifdef CONFIG_SONG_RPTUN
static int ap_start(const struct song_rptun_config_s *config)
{
  /* SP <--shram0--> AP
   * shram0 default enabled
   */

  if (getreg32(TOP_PWR_AP_M4_CTL0) & TOP_PWR_AP_CPU_SEL)
    {
      /* Boot AP CK802 */

      modifyreg32(TOP_PWR_CK802_CTL0, 0, TOP_PWR_CPUCLK_EN);
      modifyreg32(TOP_PWR_CK802_CTL0, TOP_PWR_CPU_RSTCTL, 0);
    }
  else
    {
      /* Boot AP M4 */

      putreg32(TOP_PWR_AP_M4_PORESET << 16, TOP_PWR_AP_M4_RSTCTL);
    }

  return 0;
}

static int cp_start(const struct song_rptun_config_s *config)
{
#ifdef CONFIG_MISC_RPMSG
  /* Restore CP flash to ram */

  if (up_is_warm_rstn())
    {
      MISC_RETENT_RESTORE(g_misc[1], CP_RVSD_FILE);
    }
#endif

  /* Make sure LDO0 voltage is correct. */

  if (up_is_u1v1())
    {
      ASSERT(getreg32(TOP_PMICFSM_LDO0) == TOP_PMICFSM_LDO0_DEFAULT);
    }

  /* SP <--shram1--> CP */

  putreg32(TOP_PWR_CP_M4_PORESET << 16, TOP_PWR_CP_M4_RSTCTL);
  return 0;
}

static void up_rptun_init(void)
{
  static struct rptun_rsc_s rptun_rsc_ap
    __attribute__ ((section(".resource_table.ap"))) =
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
    .buf_size        = 0x640,
  };

  static const struct song_rptun_config_s rptun_cfg_ap =
  {
    .cpuname    = CPU_NAME_AP,
    .rsc        = &rptun_rsc_ap,
    .nautostart = true,
    .master     = true,
    .vringtx    = 15,
    .vringrx    = 15,
    .start      = ap_start,
  };

  static struct rptun_rsc_s rptun_rsc_cp
    __attribute__ ((section(".resource_table.cp"))) =
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
    .buf_size        = 0x640,
  };

  static const struct song_rptun_config_s rptun_cfg_cp =
  {
    .cpuname    = CPU_NAME_CP,
    .rsc        = &rptun_rsc_cp,
    .nautostart = true,
    .master     = true,
    .vringtx    = 1,
    .vringrx    = 1,
    .start      = cp_start,
  };

  static struct rptun_rsc_s rptun_rsc_acp
    __attribute__ ((section(".resource_table.acp")))
    __attribute__ ((used)) =
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
      .num_of_vrings = 2,
    },
    .rpmsg_vring0    =
    {
      .align         = 0x8,
      .num           = 8,
    },
    .rpmsg_vring1    =
    {
      .align         = 0x8,
      .num           = 8,
    },
    .buf_size        = 0x80,
  };

  song_rptun_initialize(&rptun_cfg_ap, g_mbox[CPU_INDEX_AP], g_mbox[CPU_INDEX_SP]);
  song_rptun_initialize(&rptun_cfg_cp, g_mbox[CPU_INDEX_CP], g_mbox[CPU_INDEX_SP]);

#  ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize();
#  endif

#  ifdef CONFIG_RPMSG_REGULATOR
  rpmsg_regulator_init(CPU_NAME_AP, true);
#  endif

#  ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#  endif

#  ifdef CONFIG_FS_HOSTFS_RPMSG_SERVER
  hostfs_rpmsg_server_init();
#  endif

#  ifdef CONFIG_MISC_RPMSG
  g_misc[0] = misc_rpmsg_initialize(CPU_NAME_AP, false);
  g_misc[1] = misc_rpmsg_initialize(CPU_NAME_CP, false);
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
    .index   = 2,
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
    .base = 0xb0070000,
    .irq  = 20,
    .tclk = "swdt_tclk",
  };

  dw_wdt_initialize(&config);
}
#endif

#ifdef CONFIG_SONG_IOE
void up_ioe_init(void)
{
  static const struct song_ioe_config_s cfg =
  {
    .cpu  = 2,
    .base = 0xb0060000,
    .irq  = 19,
    .mclk = "gpio_clk32k",
  };

  g_ioe[0] = song_ioe_initialize(&cfg);
}
#endif

#ifdef CONFIG_SPI_DW
static void up_spi_init(void)
{
  static const struct dw_spi_config_s config =
  {
    .bus = 1,
    .base = 0xb0120000,
    .irq = 31,
    .tx_dma = 5,
    .rx_dma = 13,
    .cs_num = 2,
    .cs_gpio[0] = 26,
    .cs_gpio[1] = 9,
    .hbits = true,
    .mclk = "spi1_mclk",
  };

  /* XXX: temporarily set the spi1 pinmux, later we will use official
   * pinmux api instead */
  putreg32(0x10, MUX_PIN23);
  putreg32(0x10, MUX_PIN24);
  putreg32(0x10, MUX_PIN25);
  putreg32(0x12, MUX_PIN26);
  putreg32(0x12, MUX_PIN09);
  g_spi[config.bus] = dw_spi_initialize(&config, g_ioe[0], g_dma[0]);
}
#endif

#ifdef CONFIG_SONG_ONCHIP_FLASH
static void up_partition_init(FAR struct partition_s *part, FAR void *arg)
{
  char path[NAME_MAX];

  snprintf(path, NAME_MAX, "/dev/%s", part->name);
  register_mtdpartition(path, 0, arg, part->firstblock, part->nblocks);
}

static void up_flash_init(void)
{
  static const struct song_onchip_flash_timing_s timing[] =
  {
    {0x00, 0x000},   /* RD_CTRL1 */
    {0x04, 0x000},   /* RD_CTRL2 */
    {0x3c, 0x346},   /* PRIORITY_CTL */
    {-1,0},
  };
  static const struct song_onchip_flash_config_s config =
  {
    .base         = 0xb0130000,
    .cpu_base     = 0x04000000,
    .xaddr_shift  = 4,
    .yaddr_shift  = 5,
    .neraseblocks = {256, 1},
    .timing       = timing,
    .mclk         = "flash_ctrl_hclk",
    .rate         = 102400000,
  };
  char *path = "/dev/onchip";
  FAR struct mtd_dev_s *mtd[2];

  song_onchip_flash_initialize(&config, mtd);
  register_mtddriver(path, mtd[0], 0, mtd[0]);
  parse_block_partition(path, up_partition_init, path);

  register_mtddriver("/dev/onchip-info", mtd[1], 0, mtd[1]);
}
#endif

static int up_pmicfsm_isr(int irq, FAR void *context, FAR void *arg)
{
  if (getreg32(TOP_PMICFSM_INT_STATUS) & TOP_PMICFSM_SLP_U0RXD_ACT)
    {
      putreg32(TOP_PMICFSM_SLP_U0RXD_ACT, TOP_PMICFSM_INT_STATUS);

      /* WORKAROUND here, for ap response uart0 slowly */

      up_mdelay(1);
    }

  return 0;
}

static void up_extra_init(void)
{
  uint32_t wdtrst;

  if (!up_is_u1v1())
    {
      /* Attach and enable PMICFSM intrrupt */

      irq_attach(32, up_pmicfsm_isr, NULL);
      up_enable_irq(32);

      /* Enable SLP_U0RXD_ACT in PMICFSM */

      modifyreg32(TOP_PMICFSM_INT_MASK, TOP_PMICFSM_SLP_U0RXD_ACT, 0);
    }

#ifdef CONFIG_RAMDISK
  /* Register a RAMDISK device: /dev/ram1 */

  ramdisk_register(1, (uint8_t *)U1_SP_RAMDISK_BASE,
                   U1_SP_RAMDISK_SECTOR, U1_RAMDISK_SECTOR_SZ,
                   RDFLAG_WRENABLED | RDFLAG_FUNLINK);
#endif

  /* Set start reason to env */

  setenv("START_REASON", up_get_wkreason_env(), 1);

  syslog(LOG_INFO, "START_REASON: %s, PIMCFSM 0x%x, WDTRST 0x%x\n",
          up_get_wkreason_env(), getreg32(TOP_PMICFSM_WAKEUP_REASON), getreg32(TOP_PWR_WDTRST_CTL));

  wdtrst = getreg32(TOP_PWR_WDTRST_CTL) & TOP_PWR_WDT_BITED_MASK;
  if (wdtrst)
    {
      modifyreg32(TOP_PWR_WDTRST_CTL, 0, (wdtrst << 16) | wdtrst);
    }
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

#ifdef CONFIG_SPI_DW
  up_spi_init();
#endif

#ifdef CONFIG_SONG_ONCHIP_FLASH
  up_flash_init();
#endif

#ifdef CONFIG_SONG_PMIC_APB
  up_pmu_initialize();
#endif

  up_extra_init();
}

#ifdef CONFIG_MISC_RPMSG
static void cp_flash_save_prepare(void)
{
  /* CP TCM, AU_PD_MK = 1 */

  putreg32(TOP_PWR_CP_AU_PD_MK << 16 |
           TOP_PWR_CP_AU_PD_MK, TOP_PWR_CP_M4_TCM_PD_CTL0);

  /* CP TCM, WK_UP = 1, trigger PU */

  putreg32(TOP_PWR_CP_WK_UP << 16 |
           TOP_PWR_CP_WK_UP, TOP_PWR_CP_M4_TCM_PD_CTL0);

  /* CP TCM, wait until PU done */

  while (getreg32(TOP_PWR_CP_M4_TCM_PD_CTL0) & TOP_PWR_CP_WK_UP);

  /* CP UNIT, sfrst of cp_m4 asserted */

  putreg32(TOP_PWR_CP_M4_PORESET << 16 |
           TOP_PWR_CP_M4_PORESET, TOP_PWR_CP_M4_RSTCTL);

  /* CP UNIT, WK_UP = 1, trigger PU */

  putreg32(TOP_PWR_CP_WK_UP << 16 |
           TOP_PWR_CP_WK_UP, TOP_PWR_CP_UNIT_PD_CTL);

  /* CP UNIT, wait until PU done */

  while (getreg32(TOP_PWR_CP_UNIT_PD_CTL) & TOP_PWR_CP_WK_UP);
}

static void cp_flash_save_finish(void)
{
  /* HW_IDLE_MK = 1 */

  putreg32(TOP_PWR_CP_HW_IDLE_MK << 16 |
           TOP_PWR_CP_HW_IDLE_MK, TOP_PWR_CP_UNIT_PD_CTL);

  /* PD_EN = 1, trigger PD */

  putreg32(TOP_PWR_CP_PD_EN << 16 |
           TOP_PWR_CP_PD_EN, TOP_PWR_CP_UNIT_PD_CTL);

  /* Wait until PD done */

  while (getreg32(TOP_PWR_CP_UNIT_PD_CTL) & TOP_PWR_CP_PD_EN);

  /* HW_IDLE_MK = 0 */

  putreg32(TOP_PWR_CP_HW_IDLE_MK << 16, TOP_PWR_CP_UNIT_PD_CTL);

  /* Sfrst of cp_m4 released */

  putreg32(TOP_PWR_CP_M4_PORESET << 16, TOP_PWR_CP_M4_RSTCTL);
}
#endif

static void up_ds_enter_final(void)
{
  if (up_is_u1v1())
    {
      struct regulator *reg;
      struct clk *clk;

      /* Decrease LDO0/VDDAON voltage to 0.625V. Before setting
       * voltage, decrease clock rate first.
       */

      clk = clk_get("top_bus_mclk");
      if (clk != NULL)
        {
          clk_set_rate(clk, 51200000);
        }

      clk = clk_get("top_pclk1");
      if (clk != NULL)
        {
          clk_set_rate(clk, 3200000);
        }

      reg = regulator_get(NULL, "ldo0");
      if (reg != NULL)
        {
          const int voltage = 700000;
          regulator_set_voltage(reg, voltage, voltage);
        }
    }

  /* Force enable UART0 interrupt for UART0 wakeup DS */

  modifyreg32(0xb2000004, 0, 0x1);

  /* Jump to flash_pd */

  putreg32(TOP_PWR_FLASH_S_2PD_JMP << 16 |
           TOP_PWR_FLASH_S_2PD_JMP, TOP_PWR_SLPCTL_SEC_M4);

  /* Set PMICFSM enable full chip to DS */

  modifyreg32(TOP_PMICFSM_CONFIG1, 0, TOP_PMICFSM_DS_SLP_VALID);
}

static void up_ds_enter_work(void)
{
#ifdef CONFIG_MISC_RPMSG
  /* Save cpram to flash */

  cp_flash_save_prepare();
  MISC_RETENT_SAVE(g_misc[1], CP_RVSD_FILE);
  cp_flash_save_finish();
#endif

  /* Final work */

  up_ds_enter_final();
}

static void up_ds_enter_exit_work(FAR void *arg)
{
  uint32_t status = (uintptr_t)arg;

  if (status & TOP_PWR_SLPU_FLASH_S)
    {
      up_ds_enter_work();
    }
  else if (status & TOP_PWR_AP_DS_WAKEUP)
    {
#ifdef CONFIG_SONG_RPTUN
      rptun_boot(CPU_NAME_AP);
#endif
    }
  else if (status & TOP_PWR_CP_DS_WAKEUP)
    {
#ifdef CONFIG_SONG_RPTUN
      rptun_boot(CPU_NAME_CP);
#endif
    }
}

static int up_ds_enter_exit_isr(int irq, FAR void *context, FAR void *arg)
{
  static struct work_s worker;
  uint32_t status = getreg32(TOP_PWR_INTR_ST_SEC_M4_1);

  if (status & TOP_PWR_SLPU_FLASH_S)
    {
      work_queue(HPWORK, &worker, up_ds_enter_exit_work, (void *)status, 0);
      putreg32(TOP_PWR_SLPU_FLASH_S, TOP_PWR_INTR_ST_SEC_M4_1);
    }
  else if (status & TOP_PWR_AP_DS_WAKEUP)
    {
      work_queue(HPWORK, &worker, up_ds_enter_exit_work, (void *)status, 0);
      putreg32(TOP_PWR_AP_DS_WAKEUP, TOP_PWR_INTR_ST_SEC_M4_1);
    }
  else if (status & TOP_PWR_CP_DS_WAKEUP)
    {
      work_queue(HPWORK, &worker, up_ds_enter_exit_work, (void *)status, 0);
      putreg32(TOP_PWR_CP_DS_WAKEUP, TOP_PWR_INTR_ST_SEC_M4_1);
    }

  return 0;
}

void up_finalinitialize(void)
{
  /* Attach and enable DS enter exit intr */

  irq_attach(18, up_ds_enter_exit_isr, NULL);
  up_enable_irq(18);
  modifyreg32(TOP_PWR_INTR_EN_SEC_M4_1, 0, TOP_PWR_SLPU_FLASH_S);
  if (!up_is_u1v1())
    {
      modifyreg32(TOP_PWR_INTR_EN_SEC_M4_1, 0, TOP_PWR_AP_DS_WAKEUP);
      modifyreg32(TOP_PWR_INTR_EN_SEC_M4_1, 0, TOP_PWR_CP_DS_WAKEUP);
    }

#ifdef CONFIG_SONG_RPTUN
  if (!up_is_warm_rstn())
    {
      rptun_boot(CPU_NAME_AP);
      rptun_boot(CPU_NAME_CP);
    }
  else
    {
      if (up_is_u1v1())
        {
          /* Force boot AP, choice boot CP */

          rptun_boot(CPU_NAME_AP);
          if (up_get_wkreason() == WAKEUP_REASON_RTC_RSTN)
            {
              if (getreg32(0xb2020040) & 0x1)
                rptun_boot(CPU_NAME_CP);
            }
          else
            {
              rptun_boot(CPU_NAME_CP);
            }
        }
      else
        {
          if (up_get_wkreason() == WAKEUP_REASON_UART_RSTN)
            {
              rptun_boot(CPU_NAME_CP);
            }
        }
    }
#endif

#ifdef CONFIG_SONG_CLK
  up_clk_finalinitialize();
#endif

#ifdef CONFIG_CRASH_DUMPFILE
  if (up_is_u1v1())
    {
      dumpfile_initialize("sp", (char *)&_ssharmv1, \
                          ((uintptr_t)&_esharmv1) - ((uintptr_t)&_ssharmv1));
    }
  else
    {
      dumpfile_initialize("sp", (char *)&_ssharmv2, \
                          ((uintptr_t)&_esharmv2) - ((uintptr_t)&_ssharmv2));
    }
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

      putreg32(TOP_PWR_SEC_IDLE_MK << 16 |
               TOP_PWR_SEC_IDLE_MK, TOP_PWR_SEC_M4_RSTCTL);
      putreg32(TOP_PWR_SEC_SFRST << 16 |
               TOP_PWR_SEC_SFRST, TOP_PWR_SEC_M4_RSTCTL);
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

  /* Allow PWR_SLEEP (VDDMAIN ON)? */

  if (pwr_sleep)
    putreg32(TOP_PWR_SEC_M4_SLP_EN << 16 |
             TOP_PWR_SEC_M4_SLP_EN, TOP_PWR_SLPCTL_SEC_M4);
  else
    putreg32(TOP_PWR_SEC_M4_SLP_EN << 16, TOP_PWR_SLPCTL_SEC_M4);
}

static void up_cpu_ds(bool ds_sleep)
{
  /* Allow DS_SLEEP (VDDMAIN OFF)? */

  if (ds_sleep)
    putreg32(TOP_PWR_SEC_M4_DS_SLP_EN << 16 |
             TOP_PWR_SEC_M4_DS_SLP_EN, TOP_PWR_SLPCTL_SEC_M4);
  else
    putreg32(TOP_PWR_SEC_M4_DS_SLP_EN << 16, TOP_PWR_SLPCTL_SEC_M4);
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
  up_cpu_wfi();
}

void up_cpu_sleep(void)
{
  up_cpu_ds(true);
  up_cpu_standby();
  up_cpu_ds(false);
}

#endif /* CONFIG_ARCH_CHIP_U1_SP */
