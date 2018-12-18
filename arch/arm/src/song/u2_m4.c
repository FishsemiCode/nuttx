/****************************************************************************
 * arch/arm/src/song/u2_m4.c
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
#include <nuttx/fs/fs.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/fs/partition.h>
#include <nuttx/i2c/i2c_dw.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/power/regulator.h>
#include <nuttx/pwm/song_pwm.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/spi/spi_dw.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/song_oneshot.h>

#include <stdio.h>

#include "chip.h"
#include "song_addrenv.h"
#include "song_idle.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_U2_M4

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_ADSP               "adsp"
#define CPU_INDEX_AP                0
#define CPU_INDEX_ADSP              1

#define TOP_MAILBOX_BASE            (0xa0050000)

#define TOP_PWR_BASE                (0xa00e0000)
#define TOP_PWR_SFRST_CTL           (TOP_PWR_BASE + 0x178)
#define TOP_PWR_M4_INTR2SLP_MK0     (TOP_PWR_BASE + 0x224)
#define TOP_PWR_SLPCTL_M4           (TOP_PWR_BASE + 0x404)
#define TOP_PWR_RES_REG2            (TOP_PWR_BASE + 0x4fc)

#define TOP_PWR_SFRST_RESET         (1 << 0)

#define TOP_PWR_SLPCTL_M4_SLP_EN    (1 << 0)

#define TOP_PWR_RESET_NORMAL        (0x00000000)
#define TOP_PWR_RESET_ROMBOOT       (0xaaaa1234)

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
FAR struct spi_dev_s *g_spi[3] =
{
  [2] = DEV_END,
};
#endif

#ifdef CONFIG_I2C_DW
FAR struct i2c_master_s *g_i2c[4] =
{
  [3] = DEV_END,
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

void up_dmainitialize(void)
{
#ifdef CONFIG_SONG_DMAS
  g_dma[0] = song_dmas_initialize(0, 0xa0040000, 17, "top_dmas_hclk");
  g_dma[1] = song_dmas_initialize(0, 0xa0080000, 16, "audio_dmas_hclk");
#endif
}

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
    .irq        = 25,
    .c1_max     = 600,
    .c1_freq    = 6000000,
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

#ifdef CONFIG_SONG_MBOX
static void up_mbox_init(void)
{
  static const struct song_mbox_config_s config[] =
  {
    {
      .index      = CPU_INDEX_AP,
      .base       = TOP_MAILBOX_BASE,
      .set_off    = 0x0, /* MAILBOX_M4_INTR_SET */
      .en_off     = 0x4, /* MAILBOX_M4_INTR_EN */
      .en_bit     = 16,
      .src_en_off = 0x4, /* MAILBOX_M4_INTR_EN */
      .sta_off    = 0x8, /* MAILBOX_M4_INTR_STA */
      .chnl_count = 16,
      .irq        = 32,
    },
    {
      .index      = CPU_INDEX_ADSP,
      .base       = TOP_MAILBOX_BASE,
      .set_off    = 0x10, /* MAILBOX_TL421_INTR_SET */
      .en_off     = 0x14, /* MAILBOX_TL421_INTR_EN */
      .en_bit     = 16,
      .src_en_off = 0x14, /* MAILBOX_TL421_INTR_EN */
      .sta_off    = 0x18, /* MAILBOX_TL421_INTR_STA */
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
  };

  song_rptun_initialize(&rptun_cfg_adsp, g_mbox[CPU_INDEX_ADSP], g_mbox[CPU_INDEX_AP]);

#  ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize(true);
#  endif

#  ifdef CONFIG_SYSLOG_RPMSG_SERVER
  syslog_rpmsg_server_init();
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
    .base = 0xa00f0000,
    .irq  = 26,
    .mclk = "gpio_clk32k",
  };

  g_ioe[0] = song_ioe_initialize(&cfg);
}
#endif

#ifdef CONFIG_SPI_DW
static void up_spi_init(void)
{
  static const struct dw_spi_config_s config[] =
  {
    {
      .bus = 0,
      .base = 0xa0130000,
      .irq = 29,
      .cs_num = 1,
      .cs_gpio[0] = 28,
      .mclk = "spi0_mclk",
    },
    {
      .bus = 1,
      .base = 0xa0140000,
      .irq = 30,
      .cs_num = 1,
      .cs_gpio[0] = 32,
      .mclk = "spi1_mclk",
    },
  };
  int config_num = sizeof(config) / sizeof(config[0]);

  dw_spi_allinitialize(config, config_num, g_ioe[0], g_spi);
}
#endif

#ifdef CONFIG_I2C_DW
static void up_i2c_init(void)
{
  static const struct dw_i2c_config_s config[] =
  {
    {
      .bus  = 0,
      .base = 0xa0110000,
      .mclk = "i2c0_mclk",
      .rate = 25600000,
      .irq  = 27,
    },
    {
      .bus  = 1,
      .base = 0xa0120000,
      .mclk = "i2c1_mclk",
      .rate = 25600000,
      .irq  = 28,
    },
    {
      .bus  = 2,
      .base = 0xa0190000,
      .mclk = "i2c2_mclk",
      .rate = 25600000,
      .irq  = 19,
    }
  };
  int config_num = sizeof(config) / sizeof(config[0]);

  dw_i2c_allinitialize(config, config_num, g_i2c);
}
#endif


#ifdef CONFIG_MTD_GD25
static void up_partition_init(FAR struct partition_s *part, FAR void *arg)
{
  char path[NAME_MAX];

  snprintf(path, NAME_MAX, "/dev/%s", part->name);
  register_mtdpartition(path, 0, arg, part->firstblock, part->nblocks);
}

static void up_flash_init(void)
{
  char *path = "/dev/gd25";
  FAR struct mtd_dev_s *mtd;

  mtd = gd25_initialize(g_spi[0]);
  register_mtddriver(path, mtd, 0, mtd);
  parse_block_partition(path, up_partition_init, path);
}
#endif

void up_lateinitialize(void)
{
#ifdef CONFIG_SONG_CLK
  up_clk_initialize();
#endif

#ifdef CONFIG_SONG_MBOX
  up_mbox_init();
#endif

#ifdef CONFIG_SONG_RPTUN
  up_rptun_init();
#endif

#ifdef CONFIG_WATCHDOG_DW
  up_wdtinit();
#endif

#ifdef CONFIG_SONG_IOE
  up_ioe_init();
#endif

#ifdef CONFIG_PWM_SONG
  song_pwm_initialize(0, 0xa0100000, 4, "pwm_mclk");
#endif

#ifdef CONFIG_SPI_DW
  up_spi_init();
#endif

#ifdef CONFIG_I2C_DW
  up_i2c_init();
#endif

#ifdef CONFIG_SONG_PMIC_I2C
  spmu_regulator_i2c_initialize(g_i2c[0], 0x70, 400000);
#endif

#ifdef CONFIG_MTD_GD25
  up_flash_init();
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
  if (status == 0)
    {
      /* Reset board */

      putreg32(TOP_PWR_RESET_NORMAL, TOP_PWR_RES_REG2);
    }
  else
    {
      /* Reset board to romboot */

      putreg32(TOP_PWR_RESET_ROMBOOT, TOP_PWR_RES_REG2);
    }

  putreg32(TOP_PWR_SFRST_RESET << 16 |
           TOP_PWR_SFRST_RESET, TOP_PWR_SFRST_CTL);
}

void up_cpu_doze(void)
{
  putreg32(getreg32(NVIC_SYSCON) & ~NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);
  putreg32(TOP_PWR_SLPCTL_M4_SLP_EN << 16, TOP_PWR_SLPCTL_M4);
  up_cpu_wfi();
}

void up_cpu_idle(void)
{
  putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);
  putreg32(TOP_PWR_SLPCTL_M4_SLP_EN << 16 |
           TOP_PWR_SLPCTL_M4_SLP_EN, TOP_PWR_SLPCTL_M4);
  up_cpu_wfi();
}

void up_cpu_standby(void)
{
  up_cpu_idle();
}

void up_cpu_sleep(void)
{
  up_cpu_standby();
}

#endif /* CONFIG_ARCH_CHIP_U2_M4 */
