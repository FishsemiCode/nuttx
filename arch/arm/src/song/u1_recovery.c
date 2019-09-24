/****************************************************************************
 * arch/arm/src/song/u1_recovery.c
 *
 *   Copyright (C) 2010 FishSemi Inc. All rights reserved.
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

#ifdef CONFIG_ARCH_CHIP_U1_RECOVERY

#include <nuttx/arch.h>
#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/drivers/addrenv.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/partition.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mtd/song_onchip_flash.h>
#include <nuttx/power/consumer.h>
#include <nuttx/power/regulator.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/spi/spi_dw.h>
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
#include "nvic.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"
#include "u1_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IDLE_STACK      ((unsigned)&_ebss+CONFIG_IDLETHREAD_STACKSIZE-4)

#define CONFIG_BOARD_LOOPSPER10USEC  ((CONFIG_BOARD_LOOPSPERMSEC+50)/100)
#define CONFIG_BOARD_LOOPSPERUSEC    ((CONFIG_BOARD_LOOPSPERMSEC+500)/1000)

#define TOP_MAILBOX_BASE            (0xb0030000)

#define TOP_PWR_BASE                (0xb0040000)
#define TOP_PWR_TOP_PCLK1_CTL       (TOP_PWR_BASE + 0x04c)
#define TOP_PWR_AP_M4_RSTCTL        (TOP_PWR_BASE + 0x0e0)
#define TOP_PWR_SEC_M4_RSTCTL       (TOP_PWR_BASE + 0x0d8)
#define TOP_PWR_CP_M4_RSTCTL        (TOP_PWR_BASE + 0x0dc)
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

#define TOP_PWR_CP_PD_EN            (1 << 0)
#define TOP_PWR_CP_WK_UP            (1 << 1)
#define TOP_PWR_CP_HW_IDLE_MK       (1 << 8)

#define TOP_PWR_AP_CPU_SEL          (1 << 22)

#define TOP_PWR_CPU_RSTCTL          (1 << 1)
#define TOP_PWR_CPUCLK_EN           (1 << 0)

#define TOP_PWR_RESET_NORMAL        (0x00000000)
#define TOP_PWR_RESET_ROMBOOT       (0xaaaa1234)

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
 * Public Function prototypes
 ****************************************************************************/

extern void __start(void);
extern void __startup(void);

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

extern uint32_t _sflashap;
extern uint32_t _eflashap;

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

unsigned start_vectors[]
    __attribute__ ((used))
    __attribute__ ((section(".start_vectors"))) =
{
  IDLE_STACK,
  (unsigned)&__startup,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

__attribute__ ((used)) __attribute__ ((section(".start"))) void __startup(void)
{
  uint32_t *dst, *src;

  for (src = &_sflashap, dst = &_stext; src < &_eflashap; src++, dst++)
    {
      *dst = *src;
    }

  __start();
}

void up_earlystart(void)
{
}

void up_earlyinitialize(void)
{
  static const struct simple_addrenv_s addrenv[] =
  {
    {.va = 0x21000000, .pa = 0xc1000000, .size = 0x00100000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  simple_addrenv_initialize(addrenv);

#ifndef CONFIG_CPULOAD_PERIOD
  /* Allow TCM to LP, careful with it. At this time,
   * if use systick as wakeup reason form DEEPSLEEP, CPU will hang.
   */

  putreg32(TOP_PWR_SEC_AU_PD_MK << 16, TOP_PWR_SEC_M4_TCM_PD_CTL);
#endif

  /* Set the DMAS no effort to power down */

  putreg32(TOP_PWR_SLP_DMA_MK << 16 |
           TOP_PWR_SLP_DMA_MK, TOP_PWR_SLPCTL0);

  /* Set flash no effort to PWR_SLEEP */

  putreg32(TOP_PWR_CTRL_MODE << 16, TOP_PWR_FLASH_PD_CTL);

  /* Configure PLL stable time (~1.15ms). */

  putreg32(TOP_PWR_PLL_STABLE_TIME | TOP_PWR_OSC_STABLE_TIME, TOP_PWR_PLLTIME);

  /* Set PMICFSM disable full chip to DS */

  modifyreg32(TOP_PMICFSM_CONFIG1, TOP_PMICFSM_DS_SLP_VALID, 0);

  /* Set PMICFSM WAKEUP_ENABLE, now only support UART0 RTC wakeup DS */

  putreg32(TOP_PMICFSM_UART_ENABLE |
           TOP_PMICFSM_RTC_ENABLE, TOP_PMICFSM_WAKEUP_ENABLE);
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
#endif

#ifdef CONFIG_CPULOAD_PERIOD
  sched_period_extclk(systick_initialize(false, 32768, -1));
#endif
}

#ifdef CONFIG_RTC_SONG
static int up_rtc_initialize(void)
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

#ifdef CONFIG_SONG_IOE
static void up_ioe_init(void)
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
    {0x00, 0x00},   /* RD_CTRL1 */
    {0x04, 0x00},   /* RD_CTRL2 */
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

static void up_extra_init(void)
{
  /* Set start reason to env */

  setenv("START_REASON", up_get_wkreason_env(), 1);

  syslog(LOG_INFO, "START_REASON: %s, PIMCFSM 0x%x\n",
          up_get_wkreason_env(), getreg32(TOP_PMICFSM_WAKEUP_REASON));
}

void up_lateinitialize(void)
{
#ifdef CONFIG_SONG_CLK
  up_clk_initialize();
#endif

#ifdef CONFIG_RTC_SONG
  up_rtc_initialize();
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
  else
    {
      /* Reset board */

      putreg32(TOP_PWR_RESET_NORMAL, TOP_PWR_RES_REG2);
      putreg32(TOP_PWR_SFRST_RESET << 16 |
               TOP_PWR_SFRST_RESET, TOP_PWR_SFRST_CTL);
    }
}

#endif /* CONFIG_ARCH_CHIP_U1_RECOVERY */
