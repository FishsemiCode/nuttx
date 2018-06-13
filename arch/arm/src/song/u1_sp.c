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

#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/fs/partition.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/mtd/song_onchip_flash.h>
#include <nuttx/net/rpmsgdrv.h>
#include <nuttx/power/regulator.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/spi/spi_dw.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/song_oneshot.h>
#include <nuttx/timers/song_rtc.h>

#include "chip.h"
#include "song_addrenv.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_U1_SP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                 "ap"
#define CPU_NAME_CP                 "cp"

#define RSCTBL_BASE_AP              ((uintptr_t)&_srsctbl_ap)
#define RSCTBL_BASE_CP              ((uintptr_t)&_srsctbl_cp)

#define TOP_MAILBOX_BASE            (0xb0030000)

#define TOP_PWR_BASE                (0xb0040000)
#define TOP_PWR_AP_M4_RSTCTL        (TOP_PWR_BASE + 0x0e0)
#define TOP_PWR_CP_M4_RSTCTL        (TOP_PWR_BASE + 0x0dc)
#define TOP_PWR_SFRST_CTL           (TOP_PWR_BASE + 0x11c)
#define TOP_PWR_SEC_M4_INTR2SLP_MK0 (TOP_PWR_BASE + 0x148)
#define TOP_PWR_RES_REG2            (TOP_PWR_BASE + 0x260)
#define TOP_PWR_SLPST               (TOP_PWR_BASE + 0x368)

#define PMIC_FSM_BASE               (0xb2010000)
#define PMIC_FSM_CONFIG1            (PMIC_FSM_BASE + 0x0c)

#define SECURITY_BASE               (0xb0150000)
#define SECURITY_CFG_0              (SECURITY_BASE + 0x30)

#ifdef CONFIG_SONG_DMAS
static FAR struct dma_dev_s *g_dma[2] =
{
  [1] = DEV_END,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _srsctbl_ap;
extern uint32_t _srsctbl_cp;

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
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_RTC_SONG
static FAR struct rtc_lowerhalf_s *g_rtc_lower;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
  static const struct song_addrenv_s addrenv[] =
  {
    {.va = 0x21000000, .pa = 0xc1000000, .size = 0x00100000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  /* Set up addrenv */

  up_addrenv_initialize(addrenv);
}

#ifdef CONFIG_RTC_SONG
int up_rtc_initialize(void)
{
  static const struct song_rtc_config_s config =
  {
    .base  = 0xb2020000,
    .irq   = 16,
    .index = 2,
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
    .spec_off   = 0x1ac,
    .intren_off = 0x124,
    .intrst_off = 0x130,
    .intr_bit   = 2,
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
  uart_rpmsg_server_init("AP", 1024);
  uart_rpmsg_server_init("ATAP", 1024);

  uart_rpmsg_server_init("CP", 1024);
  uart_rpmsg_server_init("AT", 1024);
  uart_rpmsg_server_init("GPS", 1024);
}
#endif

#ifdef CONFIG_OPENAMP
static int ap_boot(const struct song_rptun_config_s *config)
{
  /* SP <--shram0--> AP
   * shram0 default enabled
   */

  putreg32(0x00010000, TOP_PWR_AP_M4_RSTCTL);
  return 0;
}

static int cp_boot(const struct song_rptun_config_s *config)
{
  /* SP <--shram1--> CP
   * enable shram1 for IPC
   */

  putreg32(0x00010000, SECURITY_CFG_0);
  putreg32(0x00010000, TOP_PWR_CP_M4_RSTCTL);
  return 0;
}

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
    .irq        = -1,
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
    .irq        = 21,
  };

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
    .buf_size        = 0x600,
  };

  static const struct song_rptun_config_s rptun_cfg_ap =
  {
    .cpu_name    = CPU_NAME_AP,
    .role        = RPMSG_MASTER,
    .ch_start_rx = 14,
    .ch_vring_rx = 15,
    .ch_start_tx = 14,
    .ch_vring_tx = 15,
    .rsc         =
    {
      .rsc_tab   = &rptun_rsc_ap.rsc_tbl_hdr,
      .size      = sizeof(rptun_rsc_ap),
    },
    .rsc_flash   = RSCTBL_BASE_AP,
    .boot        = ap_boot,
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
      .num           = 8,
    },
    .rpmsg_vring1    =
    {
      .align         = 0x8,
      .num           = 4,
    },
    .buf_size        = 0x600,
  };

  static const struct song_rptun_config_s rptun_cfg_cp =
  {
    .cpu_name    = CPU_NAME_CP,
    .role        = RPMSG_MASTER,
    .ch_start_rx = 0,
    .ch_vring_rx = 1,
    .ch_start_tx = 0,
    .ch_vring_tx = 1,
    .rsc         =
    {
      .rsc_tab   = &rptun_rsc_cp.rsc_tbl_hdr,
      .size      = sizeof(rptun_rsc_cp),
    },
    .rsc_flash   = RSCTBL_BASE_CP,
    .boot        = cp_boot,
  };

  mbox_ap = song_mbox_initialize(&mbox_cfg_ap);
  mbox_cp = song_mbox_initialize(&mbox_cfg_cp);
  mbox_sp = song_mbox_initialize(&mbox_cfg_sp);

  song_rptun_initialize(&rptun_cfg_ap, mbox_sp, mbox_ap);
  song_rptun_initialize(&rptun_cfg_cp, mbox_sp, mbox_cp);

#ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize(true);   /* it is server */
#endif

#ifdef CONFIG_SYSLOG_RPMSG_SERVER
  syslog_rpmsg_server_init();
#endif

#ifdef CONFIG_FS_HOSTFS_RPMSG_SERVER
  hostfs_rpmsg_server_init();
#endif
}
#endif

#ifdef CONFIG_NETDEVICES
void up_netinitialize(void)
{
  net_rpmsg_drv_init(CPU_NAME_CP, "sl0", NET_LL_SLIP);
}
#endif

#ifdef CONFIG_SONG_ONCHIP_FLASH
static void up_partition_init(FAR struct partition_s *part, FAR void *arg)
{
#ifdef CONFIG_MTD_PARTITION
  FAR struct mtd_dev_s *mtd;

  mtd = mtd_partition(arg, part->firstblock, part->nblocks);
#ifdef CONFIG_MTD_PARTITION_NAMES
  mtd_setpartitionname(mtd, part->name);
#endif
  blk_initialize_by_name(part->name, mtd);
#endif
}

static void up_flash_init(void)
{
  static const struct song_onchip_flash_config_s config =
  {
    .base = 0xb0130000,
    .cpu_base = 0x04000000,
    .xaddr_shift = 4,
    .yaddr_shift = 5,
    .neraseblocks = 256,
  };
  FAR struct mtd_dev_s *mtd;

  mtd = song_onchip_flash_initialize(&config);
  parse_mtd_partition(mtd, up_partition_init, mtd);
}
#endif

#ifdef CONFIG_SPI_DW
static void up_spi_init(void)
{
  static struct dw_spi_config_s configs[] =
  {
    {
      .base = 0xb0120000,
      .irq = 31,
      .clk_rate = 4096000,
      .bus = 1,
      .cs_num = 1,
      .cs_gpio[0] = 24,
    },
  };

  dw_spi_initialize_all(configs, sizeof(configs) / sizeof(configs[0]),
                        g_spi, sizeof(g_spi) / sizeof(g_spi[0]), g_ioe[0]);
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
  g_dma[0] = song_dmas_initialize(2, 0xb0020000, 28, "dmas_hclk");
#endif

#ifdef CONFIG_SONG_IOE
  g_ioe[0] = song_ioe_initialize(2, 0xb0060000, 19);
#endif

#ifdef CONFIG_SONG_ONCHIP_FLASH
  up_flash_init();
#endif

#ifdef CONFIG_SPI_DW
  up_spi_init();
#endif

#ifdef CONFIG_SONG_PMIC_APB
  spmu_regulator_apb_initialize(0xB2010000, 0xB0180000);
#endif

#ifdef CONFIG_RPMSG_REGULATOR
  rpmsg_regulator_init(CPU_NAME_AP, 1);
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

int board_power_off(int status)
{
  modifyreg32(PMIC_FSM_CONFIG1, 1, 1);
  putreg32(0x1, TOP_PWR_SLPST);
  return 0;
}

int board_reset(int status)
{
  if (status == 0)
    {
      /* Reset board */

      putreg32(0x0, TOP_PWR_RES_REG2);
    }
  else
    {
      /* Reset board to bootloader */

      putreg32(0xaaaa1234, TOP_PWR_RES_REG2);
    }

  putreg32(0x10001, TOP_PWR_SFRST_CTL);
  return 0;
}

void up_wic_disable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_SEC_M4_INTR2SLP_MK0, 0, 1 << (irq - NVIC_IRQ_FIRST));
    }
}

void up_wic_enable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_SEC_M4_INTR2SLP_MK0, 1 << (irq - NVIC_IRQ_FIRST), 0);
    }
}

void up_wic_initialize(void)
{
  putreg32(0xffffffff, TOP_PWR_SEC_M4_INTR2SLP_MK0);
}

#endif /* CONFIG_ARCH_CHIP_U1_SP */
