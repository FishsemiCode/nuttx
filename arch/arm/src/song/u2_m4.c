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

#ifdef CONFIG_ARCH_CHIP_U2_M4

#include <nuttx/audio/audio.h>
#include <nuttx/audio/audio_comp.h>
#include <nuttx/audio/audio_i2s.h>
#include <nuttx/audio/audio_dma.h>
#include <nuttx/audio/ak4332.h>
#include <nuttx/audio/song_audio_path.h>
#include <nuttx/audio/song_i2s.h>
#include <nuttx/audio/thinker.h>
#include <nuttx/clk/clk.h>

#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/partition.h>
#include <nuttx/i2c/i2c_dw.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/power/regulator.h>
#include <nuttx/pwm/song_pwm.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/spi/spi_dw.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/song_oneshot.h>

#include <stdio.h>

#include "chip.h"
#include "nvic.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_ADSP               "adsp"
#define CPU_INDEX_AP                0
#define CPU_INDEX_ADSP              1

#define TOP_MAILBOX_BASE            (0xa0050000)

#define MUX_PIN_BASE                (0xa00d0000)
#define MUXPIN_CLKO0_CTL            (MUX_PIN_BASE + 0x14)
#define MUXPIN_IIS0DI_CTL           (MUX_PIN_BASE + 0x7c)
#define MUXPIN_IIS0DO_CTL           (MUX_PIN_BASE + 0x80)
#define MUXPIN_IIS0CK_CTL           (MUX_PIN_BASE + 0x84)
#define MUXPIN_IIS0WS_CTL           (MUX_PIN_BASE + 0x88)
#define MUXPIN_GPIO8_CTL            (MUX_PIN_BASE + 0xe4)

#define MUXPIN_DS                   0x4
#define MUXPIN_PDU                  0x2
#define MUXPIN_FUNC_SEL             0x0

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

void up_dma_initialize(void)
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

#ifdef CONFIG_SONG_CLK
  up_clk_initialize();
#endif
}

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
      .tx_dma = 0,
      .rx_dma = 8,
      .cs_num = 1,
      .cs_gpio[0] = 28,
      .mclk = "spi0_mclk",
    },
    {
      .bus = 1,
      .base = 0xa0140000,
      .irq = 30,
      .tx_dma = 1,
      .rx_dma = 9,
      .cs_num = 1,
      .cs_gpio[0] = 32,
      .mclk = "spi1_mclk",
    },
  };
  int config_num = sizeof(config) / sizeof(config[0]);

  dw_spi_allinitialize(config, config_num, g_ioe[0], g_dma[0], g_spi);
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

  mtd = gd25_initialize(g_spi[0], 0);
  if (mtd == NULL)
    return;
  register_mtddriver(path, mtd, 0, mtd);
  parse_block_partition(path, up_partition_init, path);
}
#endif

#ifdef CONFIG_AUDIO

static void up_audio_thinker_init(struct audio_lowerhalf_s *dp_adc2)
{
#ifdef CONFIG_AUDIO_THINKER
  struct audio_lowerhalf_s *thinker;
  struct audio_lowerhalf_s *audio_dma_vt;
  struct audio_lowerhalf_s *audio_path_vt;
#ifdef CONFIG_AUDIO_VT_OUT_DMA
  struct audio_lowerhalf_s *audio_dma_vtout =
      audio_dma_initialize(g_dma[1], 8, false, 4, 0xa0070408);
#endif

  thinker      = thinker_initialize(0xa0200000, 4);
  audio_dma_vt = audio_dma_initialize(g_dma[1], 0, true, 0, 0xa007040c);

#if defined(CONFIG_AUDIO_VT_SRC_DMA)
  audio_path_vt = song_audio_path_vt_initialize(0xa0070000,
                  AUDIO_PATH_VT_SRC_DMA0,
                  false);
  audio_comp_initialize("pcm2c", thinker, audio_path_vt, NULL);
#elif defined(CONFIG_AUDIO_VT_OUT_DMA)
  audio_path_vt = song_audio_path_vt_initialize(0xa0070000,
                  dp_adc2 ? AUDIO_PATH_VT_SRC_EXTERN_ADC3 : AUDIO_PATH_VT_SRC_VOICE_ADC3,
                  true);
  audio_comp_initialize("pcm2c", thinker, audio_path_vt, audio_dma_vtout, dp_adc2, NULL);
#else
  audio_path_vt = song_audio_path_vt_initialize(0xa0070000,
                  dp_adc2 ? AUDIO_PATH_VT_SRC_EXTERN_ADC3 : AUDIO_PATH_VT_SRC_VOICE_ADC3,
                  false);
  audio_comp_initialize("pcm2c", thinker, audio_path_vt, dp_adc2, NULL);
#endif

  audio_comp_initialize("pcm2p", audio_dma_vt, NULL);
#endif
}

static void up_audio_init(void)
{
  struct audio_lowerhalf_s *ak4332_0;
  struct audio_lowerhalf_s *ak4332_1;
  struct audio_lowerhalf_s *audio_dma_in;
  struct audio_lowerhalf_s *audio_dma_voice;
  struct audio_lowerhalf_s *audio_path_anc;
  struct audio_lowerhalf_s *audio_path_in;
  struct audio_lowerhalf_s *audio_path_voice;
  struct audio_lowerhalf_s *dma_playback;
  struct audio_lowerhalf_s *dma_capture;
  struct audio_lowerhalf_s *pcm_playback;
  struct audio_lowerhalf_s *pcm_capture;
#ifdef CONFIG_AUDIO_DP_ADC
  struct audio_lowerhalf_s *dp_adc0;
  struct audio_lowerhalf_s *dp_adc1;
  struct audio_lowerhalf_s *dp_adc2;
  uint32_t voice_adcs;
#endif

  /* audio_mclk_mx mux to audio_mclk */

  clk_set_parent(clk_get("audio_mclk_mx"), clk_get("audio_mclk"));

  clk_enable(clk_get("i2c1_mclk"));
  clk_enable(clk_get("i2c2_mclk"));

  putreg32(2 << (MUXPIN_DS), MUXPIN_CLKO0_CTL);
  putreg32(1 << (MUXPIN_DS) | 2 << (MUXPIN_PDU) | 3 << (MUXPIN_FUNC_SEL), MUXPIN_IIS0DI_CTL);
  putreg32(1 << (MUXPIN_DS) | 0 << (MUXPIN_PDU) | 3 << (MUXPIN_FUNC_SEL), MUXPIN_IIS0DO_CTL);
  putreg32(1 << (MUXPIN_DS) | 2 << (MUXPIN_PDU) | 3 << (MUXPIN_FUNC_SEL), MUXPIN_IIS0CK_CTL);
  putreg32(1 << (MUXPIN_DS) | 1 << (MUXPIN_PDU) | 3 << (MUXPIN_FUNC_SEL), MUXPIN_IIS0WS_CTL);
  putreg32(1 << (MUXPIN_DS), MUXPIN_GPIO8_CTL);
  IOEXP_SETDIRECTION(g_ioe[0], 0x8, IOEXPANDER_DIRECTION_OUT);
  IOEXP_WRITEPIN(g_ioe[0], 0x8, true);

  ak4332_0 = ak4332_initialize(g_i2c[1], "audio_sys_akm_clk", 3);
  ak4332_1 = ak4332_initialize(g_i2c[2], "audio_sys_akm_clk", 3);

  pcm_playback = audio_i2s_initialize(song_i2s_initialize(0xa0060000, "pcm_mclk"), true);
  pcm_capture  = audio_i2s_initialize(song_i2s_initialize(0xa0060000, "pcm_mclk"), false);
  dma_playback = audio_dma_initialize(g_dma[0], 3, true, 4, 0xa0060018);
  dma_capture  = audio_dma_initialize(g_dma[0], 11, false, 4, 0xa0060014);

  audio_dma_in    = audio_dma_initialize(g_dma[1], 1, true, 0, 0xa0070490);
  audio_dma_voice = audio_dma_initialize(g_dma[1], 8, false, 4, 0xa0070408);

#ifdef CONFIG_AUDIO_DP_ADC
  dp_adc0 = dp_adc_initialize("dolphin_adc_mclk", 0xa0090000, 0, 0);
  dp_adc1 = dp_adc_initialize("dolphin_adc_mclk", 0xa0090000, 0, 1);
  dp_adc2 = dp_adc_initialize("dolphin_adc_mclk", 0xa0090000, 0, 2);

  voice_adcs       = 0 | (0x2 << 8) | (0xff << 16) | (0xff << 24);
  audio_path_voice = song_audio_path_voice_initialize(0xa0070000, true, voice_adcs);
  audio_path_in    = song_audio_path_in_initialize(0xa0070000, "audio_sys_in_clk", "audio_i2s_mclk");
  audio_path_anc   = song_audio_path_anc_initialize(0xa0070000, true);

  audio_comp_initialize("pcm0c", audio_dma_voice, audio_path_voice, dp_adc0, dp_adc2, NULL);
  audio_comp_initialize("pcm0p", ak4332_0, ak4332_1, audio_path_in, audio_dma_in, NULL);
  audio_comp_initialize("pcm1c", dma_capture, pcm_capture, NULL);
  audio_comp_initialize("pcm1p", dma_playback, pcm_playback, NULL);
  audio_comp_initialize("pcm3p", ak4332_0, ak4332_1, audio_path_anc, dp_adc0, dp_adc1, NULL);

  up_audio_thinker_init(dp_adc2);
#else
  audio_path_voice = song_audio_path_voice_initialize(0xa0070000, false, 0);
  audio_path_in    = song_audio_path_in_initialize(0xa0070000, "audio_sys_in_clk", "audio_i2s_mclk");
  audio_path_anc   = song_audio_path_anc_initialize(0xa0070000, false);

  audio_comp_initialize("pcm0c", audio_dma_voice, audio_path_voice, NULL);
  audio_comp_initialize("pcm0p", ak4332_0, ak4332_1, audio_path_in, audio_dma_in, NULL);
  audio_comp_initialize("pcm1c", dma_capture, pcm_capture, NULL);
  audio_comp_initialize("pcm1p", dma_playback, pcm_playback, NULL);
  audio_comp_initialize("pcm3p", ak4332_0, ak4332_1, audio_path_anc, NULL);

  up_audio_thinker_init(NULL);
#endif

#ifdef CONFIG_AUDIO_DP_VAD
  dp_vad_initialize("vad_mclk_gated", 0xa00a0000, 23);
#endif
}
#endif

void up_lateinitialize(void)
{
#ifdef CONFIG_SONG_MBOX
  up_mbox_init();
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
  up_pmu_initialize();
#endif

#ifdef CONFIG_MTD_GD25
  up_flash_init();
#endif

#ifdef CONFIG_AUDIO
  up_audio_init();
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
