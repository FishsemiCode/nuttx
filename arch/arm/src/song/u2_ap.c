/****************************************************************************
 * arch/arm/src/song/u2_ap.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: Wang Yanjiong <wangyanjiong@pinecone.net>
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
#include <nuttx/mtd/mtd.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/spi/spi_dw.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/song_oneshot.h>

#include "chip.h"
#include "nvic.h"
#include "song_addrenv.h"
#include "song_idle.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_U2_AP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_ADSP               "adsp"

#define RSCTBL_BASE_ADSP            ((uintptr_t)&_srsctbl_adsp)

#define TOP_MAILBOX_BASE            (0xa0050000)

#define TOP_PWR_BASE                (0xa00e0000)
#define TOP_PWR_M4_INTR2SLP_MK0     (TOP_PWR_BASE + 0x224)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_DMA
static FAR struct dma_dev_s *g_dma[3] =
{
  [2] = DEV_END,
};
#endif

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _srsctbl_adsp;

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

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_wic_initialize(void)
{
  putreg32(0xffffffff, TOP_PWR_M4_INTR2SLP_MK0);
}

void up_wic_enable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_M4_INTR2SLP_MK0, 1 << (irq - NVIC_IRQ_FIRST), 0);
    }
}

void up_wic_disable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(TOP_PWR_M4_INTR2SLP_MK0, 0, 1 << (irq - NVIC_IRQ_FIRST));
    }
}

#ifdef CONFIG_ARCH_DMA
void up_dmainitialize(void)
{
#  ifdef CONFIG_SONG_DMAS
  g_dma[0] = song_dmas_initialize(0, 0xa0030000, 17, "top_dmas_hclk");
  g_dma[1] = song_dmas_initialize(0, 0xa0080000, 16, "audio_dmas_hclk");
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
    .irq        = 25,
    .c1_freq    = 19200000,
    .ctl_off    = 0x290,
    .calib_off  = 0x2b4,
    .c1_off     = 0x294,
    .c2_off     = 0x298,
    .spec_off   = 0x2cc,
    .intren_off = 0x214,
    .intrst_off = 0x21c,
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
  uart_rpmsg_server_init("ADSP", 1024);
}
#endif

#ifdef CONFIG_OPENAMP
static void up_openamp_initialize(void)
{
  struct mbox_dev_s *mbox_ap, *mbox_adsp;

  static const struct song_mbox_config_s mbox_cfg_ap =
  {
    .base       = TOP_MAILBOX_BASE,
    .set_off    = 0x0, /* MAILBOX_M4_INTR_SET */
    .en_off     = 0x4, /* MAILBOX_M4_INTR_EN */
    .en_bit     = 16,
    .src_en_off = 0x4, /* MAILBOX_M4_INTR_EN */
    .sta_off    = 0x8, /* MAILBOX_M4_INTR_STA */
    .chnl_count = 16,
    .irq        = 32,
  };

  static const struct song_mbox_config_s mbox_cfg_adsp =
  {
    .base       = TOP_MAILBOX_BASE,
    .set_off    = 0x10, /* MAILBOX_TL421_INTR_SET */
    .en_off     = 0x14, /* MAILBOX_TL421_INTR_EN */
    .en_bit     = 16,
    .src_en_off = 0x14, /* MAILBOX_TL421_INTR_EN */
    .sta_off    = 0x18, /* MAILBOX_TL421_INTR_STA */
    .chnl_count = 16,
    .irq        = -1,
  };

  static struct rptun_rsc_s rptun_rsc_adsp
    __attribute__ ((section(".resource_table.adsp"))) =
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
    .buf_size        = 0xe0,
  };

  static const struct song_rptun_config_s rptun_cfg_adsp =
  {
    .cpu_name    = CPU_NAME_ADSP,
    .role        = RPMSG_MASTER,
    .ch_start_tx = 0,
    .ch_vring_tx = 1,
    .ch_start_rx = 0,
    .ch_vring_rx = 1,
    .rsc         =
    {
      .rsc_tab   = &rptun_rsc_adsp.rsc_tbl_hdr,
      .size      = sizeof(rptun_rsc_adsp),
    },
    .rsc_flash   = RSCTBL_BASE_ADSP,
  };

  mbox_ap = song_mbox_initialize(&mbox_cfg_ap);
  mbox_adsp = song_mbox_initialize(&mbox_cfg_adsp);

  song_rptun_initialize(&rptun_cfg_adsp, mbox_adsp, mbox_ap);

#  ifdef CONFIG_SYSLOG_RPMSG_SERVER
  syslog_rpmsg_server_init();
#  endif

#  ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize(true);   /* it is server */
#  endif

#  ifdef CONFIG_FS_HOSTFS_RPMSG_SERVER
  hostfs_rpmsg_server_init();
#  endif
}
#endif

#ifdef CONFIG_WATCHDOG_DW
void up_wdtinit(void)
{
  static const struct dw_wdt_config_s config =
  {
    .path = CONFIG_WATCHDOG_DEVPATH,
    .base = 0xa0170000,
    .irq  = 37,
    .tclk = "m4_wdt_tclk",
    .pclk = "m4_wdt_pclk",
  };

  dw_wdt_initialize(&config);
}
#endif

#ifdef CONFIG_SPI_DW
static void up_spi_init(void)
{
  static struct dw_spi_config_s configs[] =
  {
    {
      .base = 0xa0130000,
      .irq = 29,
      .clk_rate = 78000000,
      .bus = 0,
      .cs_num = 1,
      .cs_gpio[0] = 28,
    },
  };

  dw_spi_initialize_all(configs, sizeof(configs) / sizeof(configs[0]),
                        g_spi, sizeof(g_spi) / sizeof(g_spi[0]), g_ioe[0]);
}
#endif

#ifdef CONFIG_MTD_GD25
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
  FAR struct mtd_dev_s *mtd;

  mtd = gd25_initialize(g_spi[0]);
  parse_mtd_partition(mtd, up_partition_init, mtd);
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

#ifdef CONFIG_WATCHDOG_DW
  up_wdtinit();
#endif

#ifdef CONFIG_SONG_IOE
  g_ioe[0] = song_ioe_initialize(0, 0xa00f0000, 26);
#endif

#ifdef CONFIG_SPI_DW
  up_spi_init();
#endif

#ifdef CONFIG_MTD_GD25
  up_flash_init();
#endif
}

#endif /* CONFIG_ARCH_CHIP_U2_AP */
