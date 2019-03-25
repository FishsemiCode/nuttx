/****************************************************************************
 * arch/arm/src/song/banks_sensor.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
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

#ifdef CONFIG_ARCH_CHIP_BANKS_SENSOR

#include <nuttx/audio/audio_comp.h>
#include <nuttx/audio/audio_dma.h>
#include <nuttx/audio/audio_i2s.h>
#include <nuttx/audio/song_i2s.h>
#include <nuttx/audio/song_pdm.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/drivers/addrenv.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/i2c/i2c_dw.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/spi/spi_dw.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/rpmsg_rtc.h>
#include <nuttx/timers/song_oneshot.h>

#include "chip.h"
#include "nvic.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                 "ap"
#define CPU_INDEX_AP                0
#define CPU_INDEX_SENSOR            1

#define _LOGBUF_BASE                ((uintptr_t)&_slog)
#define _LOGBUF_SIZE                ((uint32_t)&_logsize)

#define SEN_PWR_BASE                (0xf8b14000)
#define SEN_PWR_SLPCTL0             (SEN_PWR_BASE + 0x000)
#define SEN_PWR_SEN_M4_INTR2SLP_MK0 (SEN_PWR_BASE + 0x160)

#define SEN_PWR_SEN_M4_SLP_EN       (1 << 0)

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
FAR struct mbox_dev_s *g_mbox[3] =
{
  [2] = DEV_END,
};
#endif

#ifdef CONFIG_SONG_IOE
FAR struct ioexpander_dev_s *g_ioe[3] =
{
  [2] = DEV_END,
};
#endif

#ifdef CONFIG_SPI_DW
FAR struct spi_dev_s *g_spi[3] =
{
  [2] = DEV_END,
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
  static const struct simple_addrenv_s addrenv[] =
  {
    {.va = 0x00000000, .pa = 0xf8800000, .size = 0x00100000},
    {.va = 0x20000000, .pa = 0xf8a00000, .size = 0x00100000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  simple_addrenv_initialize(addrenv);

  /* Always enable SLEEPDEEP and control the sleep through PWR */
  modifyreg32(NVIC_SYSCON, 0, NVIC_SYSCON_SLEEPDEEP);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(CPU_NAME_AP, (void *)_LOGBUF_BASE, _LOGBUF_SIZE);
#endif
}

void up_wic_initialize(void)
{
  putreg32(0xffffffff, SEN_PWR_SEN_M4_INTR2SLP_MK0);
}

void up_wic_enable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(SEN_PWR_SEN_M4_INTR2SLP_MK0, 1 << (irq - NVIC_IRQ_FIRST), 0);
    }
}

void up_wic_disable_irq(int irq)
{
  if (irq >= NVIC_IRQ_FIRST)
    {
      modifyreg32(SEN_PWR_SEN_M4_INTR2SLP_MK0, 0, 1 << (irq - NVIC_IRQ_FIRST));
    }
}

void up_dma_initialize(void)
{
#ifdef CONFIG_SONG_DMAS
  g_dma[0] = song_dmas_initialize(0, 0xf8b1d000, 18, "sen_dmas_hclk");
#endif
}

#if defined(CONFIG_16550_UART) && defined(CONFIG_SONG_DMAS)
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
  return NULL; /* Can't use top dmas */
}
#endif

void arm_timer_initialize(void)
{
#ifdef CONFIG_ONESHOT_SONG
  static const struct song_oneshot_config_s config =
  {
    .minor      = -1,
    .base       = SEN_PWR_BASE,
    .irq        = 21,
    .c1_max     = 2600,
    .c1_freq    = 26000000,
    .ctl_off    = 0x170,
    .calib_off  = 0x194,
    .c1_off     = 0x174,
    .c2_off     = 0x178,
    .spec_off   = 0x1a4,
    .intren_off = 0x138,
    .intrst_off = 0x13c,
    .intr_bit   = 0,
  };

  up_alarm_set_lowerhalf(song_oneshot_initialize(&config));
#endif

#ifdef CONFIG_CPULOAD_PERIOD
  sched_period_extclk(systick_initialize(false, 26000000, -1));
#endif
}

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  uart_rpmsg_init(CPU_NAME_AP, "SENSOR", 1024, true);
  uart_rpmsg_init(CPU_NAME_AP, "GPS", 1024, false);
}
#endif

#ifdef CONFIG_SONG_MBOX
static void up_mbox_init(void)
{
  static const struct song_mbox_config_s config[] =
  {
    {
      .index      = CPU_INDEX_AP,
      .base       = SEN_PWR_BASE,
      .set_off    = 0x120,
      .en_off     = UINT32_MAX,
      .en_bit     = UINT32_MAX,
      .src_en_off = UINT32_MAX,
      .sta_off    = UINT32_MAX,
      .chnl_count = 16,
      .irq        = -1,
    },
    {
      .index      = CPU_INDEX_SENSOR,
      .base       = SEN_PWR_BASE,
      .set_off    = UINT32_MAX,
      .en_off     = 0x12c,
      .en_bit     = 16,
      .src_en_off = 0x12c,
      .sta_off    = 0x130,
      .chnl_count = 16,
      .irq        = 30,
    }
  };

  song_mbox_allinitialize(config, ARRAY_SIZE(config), g_mbox);
}
#endif

#ifdef CONFIG_SONG_RPTUN
static void up_rptun_init(void)
{
  static struct rptun_rsc_s rptun_rsc_ap
    __attribute__ ((section(".resource_table"))) =
  {
    .rsc_tbl_hdr     =
    {
      .ver           = 1,
      .num           = 2,
    },
    .offset          =
    {
      offsetof(struct rptun_rsc_s, log_trace),
      offsetof(struct rptun_rsc_s, rpmsg_vdev),
    },
    .log_trace       =
    {
      .type          = RSC_TRACE,
      .da            = _LOGBUF_BASE,
      .len           = _LOGBUF_SIZE,
    },
    .rpmsg_vdev      =
    {
      .type          = RSC_VDEV,
      .id            = VIRTIO_ID_RPMSG,
      .dfeatures     = 1 << VIRTIO_RPMSG_F_NS
                     | 1 << VIRTIO_RPMSG_F_BIND
                     | 1 << VIRTIO_RPMSG_F_BUFSZ,
      .config_len    = 4,
      .num_of_vrings = 2,
    },
    .rpmsg_vring0    =
    {
      .align         = 8,
      .num           = 8,
    },
    .rpmsg_vring1    =
    {
      .align         = 8,
      .num           = 8,
    },
    .buf_size        = 0x1e0,
  };

  static const struct song_rptun_config_s rptun_cfg_ap =
  {
    .cpuname = CPU_NAME_AP,
    .rsc     = &rptun_rsc_ap,
    .vringtx = 15,
    .vringrx = 15,
  };

  song_rptun_initialize(&rptun_cfg_ap, g_mbox[CPU_INDEX_AP], g_mbox[CPU_INDEX_SENSOR]);

#  ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize();
#  endif

#  ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#  endif

#  ifdef CONFIG_RTC_RPMSG
  up_rtc_set_lowerhalf(rpmsg_rtc_initialize(CPU_NAME_AP, 0));
#  endif

#  ifdef CONFIG_FS_HOSTFS_RPMSG
  hostfs_rpmsg_init(CPU_NAME_AP);
#  endif
}
#endif

#ifdef CONFIG_WATCHDOG_DW
void up_wdtinit(void)
{
  static const struct dw_wdt_config_s config =
  {
    .path = CONFIG_WATCHDOG_DEVPATH,
    .base = 0xf8b1b000,
    .irq  = 26,
    .tclk = "sen_wdt_tclk",
  };

  dw_wdt_initialize(&config);
}
#endif

#ifdef CONFIG_SONG_IOE
void up_ioe_init(void)
{
  static const struct song_ioe_config_s cfg[] =
  {
    {
      .cpu  = 0,
      .base = 0xf8b13000,
      .irq  = 22,
      .mclk = "sen_gpio_clk32k",
    },
    {
      .cpu  = 4,
      .base = 0xf900c000,
      .irq  = 28,
      .mclk = "sen_gpio_clk32k",
    },
  };

  /* sensor gpio initialization */
  g_ioe[0] = song_ioe_initialize(&cfg[0]);

  /* general gpio initialization */
  g_ioe[1] = song_ioe_initialize(&cfg[1]);
}
#endif

#ifdef CONFIG_SPI_DW
static void up_spi_init(void)
{
  static const struct dw_spi_config_s config[] =
  {
    {
      .bus = 0,
      .base = 0xf8b10000,
      .irq = 17,
      .tx_dma = 1,
      .rx_dma = 9,
      .cs_num = 2,
      .mclk = "sen_ssi0_mclk",
    },
    {
      .bus = 1,
      .base = 0xf8b1c000,
      .irq = 25,
      .tx_dma = 2,
      .rx_dma = 10,
      .cs_num = 2,
      .mclk = "sen_ssi1_mclk",
    }
  };
  int config_num = sizeof(config) / sizeof(config[0]);

  dw_spi_allinitialize(config, config_num, NULL, g_dma[0], g_spi);
}
#endif

#ifdef CONFIG_I2C_DW
static void up_i2c_init(void)
{
  static const struct dw_i2c_config_s config[] =
  {
    {
      .bus  = 0,
      .base = 0xf8b11000,
      .mclk = "i2c0_mclk",
      .rate = 26000000,
      .irq  = 23,
    },
    {
      .bus  = 1,
      .base = 0xf8b12000,
      .mclk = "i2c1_mclk",
      .rate = 26000000,
      .irq  = 24,
    }
  };
  int config_num = sizeof(config) / sizeof(config[0]);

  dw_i2c_allinitialize(config, config_num, g_i2c);
}
#endif

static void up_audio_init(void)
{
#ifdef CONFIG_SONG_I2S
  struct i2s_dev_s *i2s = song_i2s_initialize(0xf8b15000, "sen_i2s_mclk");
  audio_comp_initialize("pcm0p",
                        audio_dma_initialize(g_dma[0], 0, true, 4, 0xf8b15018),
                        audio_i2s_initialize(i2s, true), NULL);
  audio_comp_initialize("pcm0c",
                        audio_dma_initialize(g_dma[0], 8, false, 4, 0xf8b15014),
                        audio_i2s_initialize(i2s, false), NULL);
#endif

#ifdef CONFIG_SONG_PDM
  struct i2s_dev_s *pdm0 = song_pdm_initialize(0xf8b17000, "sen_pdm0_clk", 64, 15);
  audio_comp_initialize("pcm1p",
                        audio_dma_initialize(g_dma[0], 3, true, 4, 0xf8b17008),
                        audio_i2s_initialize(pdm0, true), NULL);
  audio_comp_initialize("pcm1c",
                        audio_dma_initialize(g_dma[0], 11, false, 4, 0xf8b17008),
                        audio_i2s_initialize(pdm0, false), NULL);

  struct i2s_dev_s *pdm1 = song_pdm_initialize(0xf8b18000, "sen_pdm1_clk", 64, 15);
  audio_comp_initialize("pcm2p",
                        audio_dma_initialize(g_dma[0], 4, true, 4, 0xf8b18008),
                        audio_i2s_initialize(pdm1, true), NULL);
  audio_comp_initialize("pcm2c",
                        audio_dma_initialize(g_dma[0], 12, false, 4, 0xf8b18008),
                        audio_i2s_initialize(pdm1, false), NULL);
#endif
}

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

#ifdef CONFIG_SPI_DW
  up_spi_init();
#endif

#ifdef CONFIG_I2C_DW
  up_i2c_init();
#endif

  up_audio_init();
}

void up_finalinitialize(void)
{
#ifdef CONFIG_SONG_CLK
  up_clk_finalinitialize();
#endif
}

void up_cpu_doze(void)
{
  /* Forbid the deep sleep */
  putreg32(SEN_PWR_SEN_M4_SLP_EN << 16, SEN_PWR_SLPCTL0);

  up_cpu_wfi();
}

void up_cpu_idle(void)
{
  /* Allow the deep sleep */
  putreg32(SEN_PWR_SEN_M4_SLP_EN << 16 |
           SEN_PWR_SEN_M4_SLP_EN, SEN_PWR_SLPCTL0);

  up_cpu_wfi();
}

#endif /* CONFIG_ARCH_CHIP_BANKS_SENSOR */
