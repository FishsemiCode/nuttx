/****************************************************************************
 * arch/arm/src/song/u11_cp.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
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

#ifdef CONFIG_ARCH_CHIP_U11_CP

#include <nuttx/arch.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/eeprom/song_efuse.h>
#include <nuttx/fs/fs.h>
#include <nuttx/fs/partition.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/ioexpander/song_ioe.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/misc/misc_rpmsg.h>
#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/song_spi_flash.h>
#include <nuttx/power/regulator.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/power/pm.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/song_oneshot.h>
#include <nuttx/timers/song_rtc.h>
#include <nuttx/wqueue.h>

#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#include "chip.h"
#include "nvic.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"
#include "u11_common.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                 "ap"
#define CPU_INDEX_AP                0
#define CPU_INDEX_CP                1

#define LOGBUF_BASE                 ((uintptr_t)&_slog)
#define LOGBUF_SIZE                 ((uint32_t)&_logsize)

#define TOP_MAILBOX_BASE            (0xb0030000)

#define TOP_PWR_BASE                (0xb0040000)
#define TOP_PWR_CP_M4_RSTCTL        (TOP_PWR_BASE + 0x0dc)
#define TOP_PWR_AP_M4_RSTCTL        (TOP_PWR_BASE + 0x0e0)
#define TOP_PWR_SFRST_CTL           (TOP_PWR_BASE + 0x11c)
#define TOP_PWR_INTR_EN_CP_M4       (TOP_PWR_BASE + 0x12c)
#define TOP_PWR_INTR_ST_CP_M4       (TOP_PWR_BASE + 0x138)
#define TOP_PWR_INTR_EN_CP_M4_1     (TOP_PWR_BASE + 0x140)
#define TOP_PWR_INTR_ST_CP_M4_1     (TOP_PWR_BASE + 0x144)
#define TOP_PWR_CP_UNIT_PD_CTL      (TOP_PWR_BASE + 0x1fc)
#define TOP_PWR_AP_M4_CTL0          (TOP_PWR_BASE + 0x218)
#define TOP_PWR_CK802_CTL0          (TOP_PWR_BASE + 0x22c)
#define TOP_PWR_RFIC_MODE_CTL       (TOP_PWR_BASE + 0x248)
#define TOP_PWR_RES_REG2            (TOP_PWR_BASE + 0x260)
#define TOP_PWR_BOOT_REG            (TOP_PWR_BASE + 0x290)
#define TOP_PWR_SLPCTL1             (TOP_PWR_BASE + 0x354)
#define TOP_PWR_SLPCTL_SEC_M4       (TOP_PWR_BASE + 0x358)
#define TOP_PWR_SLPCTL_CP_M4        (TOP_PWR_BASE + 0x35c)
#define TOP_PWR_CP_M4_TCM_PD_CTL0   (TOP_PWR_BASE + 0x3e0)

#define TOP_PWR_CP_M4_SFRST         (1 << 4)    //TOP_PWR_CP_M4_RSTCTL
#define TOP_PWR_CP_M4_IDLE_MK       (1 << 5)

#define TOP_PWR_AP_M4_PORESET       (1 << 0)    //TOP_PWR_AP_M4_RSTCTL

#define TOP_PWR_SFRST_RESET         (1 << 0)    //TOP_PWR_SFRST_CTL

#define TOP_PWR_SLPU_FLASH_S        (1 << 0)    //TOP_PWR_INTR_EN_CP_M4_1
#define TOP_PWR_AP_DS_WAKEUP        (1 << 5)    //TOP_PWR_INTR_ST_CP_M4_1

#define TOP_PWR_CP_M4_PD_MK         (1 << 3)    //TOP_PWR_CP_UNIT_PD_CTL
#define TOP_PWR_CP_M4_AU_PU_MK      (1 << 6)
#define TOP_PWR_CP_M4_AU_PD_MK      (1 << 7)

#define TOP_PWR_AP_CPU_SEL_MOD_MK   (3 << 20)   //TOP_PWR_AP_M4_CTL0
#define TOP_PWR_AP_CPU_SEL_MOD_M4   (0 << 20)   //TOP_PWR_AP_M4_CTL0
#define TOP_PWR_AP_CPU_SEL_MOD_CK   (1 << 20)   //TOP_PWR_AP_M4_CTL0
#define TOP_PWR_AP_CPU_SEL          (1 << 22)

#define TOP_PWR_CPU_RSTCTL          (1 << 1)    //TOP_PWR_CK802_CTL0
#define TOP_PWR_CPUCLK_EN           (1 << 0)

#define TOP_PWR_RESET_NORMAL        (0x00000000)//TOP_PWR_RES_REG2
#define TOP_PWR_RESET_ROMBOOT       (0xaaaa1234)
#define TOP_PWR_RESET_RECOVERY      (0xbbbb1234)

#define TOP_PWR_CP_M4_COLD_BOOT     (1 << 2)    //TOP_PWR_BOOT_REG

#define TOP_PWR_RF_TP_TM2_SLP_MK    (1 << 8)    //TOP_PWR_SLPCTL1
#define TOP_PWR_RF_TP_CALIB_SLP_MK  (1 << 9)

#define TOP_PWR_SEC_M4_SLP_EN       (1 << 0)    //TOP_PWR_SLPCTL_SEC_M4
#define TOP_PWR_SEC_M4_SLP_MK       (1 << 1)
#define TOP_PWR_SEC_M4_DS_SLP_EN    (1 << 2)

#define TOP_PWR_CP_M4_SLP_EN        (1 << 0)    //TOP_PWR_SLPCTL_CP_M4
#define TOP_PWR_CP_M4_SLP_MK        (1 << 1)
#define TOP_PWR_CP_M4_DS_SLP_EN     (1 << 2)
#define TOP_PWR_FLASH_S_2PD_JMP     (1 << 8)

#define TOP_PWR_CP_M4_TCM_AU_PD_MK  (1 << 7)    //TOP_PWR_CP_M4_TCM_PD_CTL0

#define TOP_PWR_RFIC_MODE_CTL_CLK   (1 << 15)

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

#ifdef CONFIG_SONG_IOE
FAR struct ioexpander_dev_s *g_ioe[2] =
{
  [1] = DEV_END,
};
#endif

#ifdef CONFIG_SONG_MBOX
FAR struct mbox_dev_s *g_mbox[3] =
{
  [2] = DEV_END,
};
#endif

#ifdef CONFIG_SONG_SPI_FLASH
struct song_spi_flash_config_s spiflash =
{
  .base = 0xb0130000,
  .cpu_base = 0x02000000,
};
#endif

extern uint32_t _vectors[];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

#ifdef CONFIG_SONG_WARM_START
__attribute__((section(".warm_start")))
#endif
void up_earlystart(void)
{
  if (!(getreg32(TOP_PWR_BOOT_REG) & TOP_PWR_CP_M4_COLD_BOOT))
    {
      /* Reset the NVIC vector location */

      putreg32((uint32_t)_vectors, NVIC_VECTAB);

      /* CP core auto power on */

      up_cpu_restore();
    }

  putreg32(TOP_PWR_CP_M4_COLD_BOOT << 16, TOP_PWR_BOOT_REG);
}

void up_earlyinitialize(void)
{
  /* Configure PLL stable time (~1.15ms). */

  putreg32(TOP_PMICFSM_PLL_STABLE_TIME |
           TOP_PMICFSM_OSC_STABLE_TIME, TOP_PMICFSM_PLLTIME);

  /* Mask SP effect to system */

  putreg32(TOP_PWR_SEC_M4_SLP_EN << 16 |
           TOP_PWR_SEC_M4_SLP_EN, TOP_PWR_SLPCTL_SEC_M4);

  putreg32(TOP_PWR_SEC_M4_SLP_MK << 16 |
           TOP_PWR_SEC_M4_SLP_MK, TOP_PWR_SLPCTL_SEC_M4);

  putreg32(TOP_PWR_SEC_M4_DS_SLP_EN << 16 |
           TOP_PWR_SEC_M4_DS_SLP_EN, TOP_PWR_SLPCTL_SEC_M4);

  /* Unmask SLEEPING for reset */

  putreg32(TOP_PWR_CP_M4_IDLE_MK << 16, TOP_PWR_CP_M4_RSTCTL);

  /* Set PMICFSM disable full chip to DS */

  modifyreg32(TOP_PMICFSM_CONFIG1, TOP_PMICFSM_DS_SLP_VALID, 0);

  /* Set PMICFSM WAKEUP_ENABLE, now only support UART0 RTC wakeup DS */

  putreg32(TOP_PMICFSM_UART_ENABLE |
           TOP_PMICFSM_RTC_ENABLE, TOP_PMICFSM_WAKEUP_ENABLE);

  /* Set RF & SLP init value */

  putreg32(TOP_PWR_RF_TP_TM2_SLP_MK << 16 |
           TOP_PWR_RF_TP_CALIB_SLP_MK << 16, TOP_PWR_SLPCTL1);
  putreg32(TOP_PWR_CP_M4_SLP_EN << 16 |
           TOP_PWR_CP_M4_SLP_MK << 16 |
           TOP_PWR_CP_M4_DS_SLP_EN << 16, TOP_PWR_SLPCTL_CP_M4);

  modifyreg32(TOP_PWR_RFIC_MODE_CTL,
           TOP_PWR_RFIC_MODE_CTL_CLK, TOP_PWR_RFIC_MODE_CTL_CLK);

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
  g_dma[0] = song_dmas_initialize(0, 0xb0020000, 28, "dmas_hclk");
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
      .c1_freq    = 7680000,
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
  };

  song_mbox_allinitialize(config, ARRAY_SIZE(config), g_mbox);
}
#endif

#ifdef CONFIG_SONG_RPTUN
static int ap_relocate(uint32_t ap_flash, uint32_t ap_sram_va, uint32_t ap_sram_pa)
{
  struct vector_table
  {
    uint32_t magic;
    uint32_t reserved[3];
    uint32_t warm_flash;
    uint32_t warm_size;
    uint32_t warm_sram;
  };

  struct vector_table *vector = (struct vector_table *)(ap_flash + 0x400);
  uint32_t dst;

  if (vector->magic != 0x21494350)
    {
      return -EINVAL;
    }

  dst = vector->warm_sram - ap_sram_va + ap_sram_pa;
  memcpy((void *)dst, (void *)vector->warm_flash, vector->warm_size);

  return 0;
}

static int ap_start(const struct song_rptun_config_s *config)
{
  int ret;

  /* Set ldo_ano to 1.1V for AP TCM */

  putreg32(0xa1203, TOP_PMICFSM_LDO0);
  usleep(500);

  /* Relocate for AP WARM start */

  ret = ap_relocate(0x02124000, 0x00000000, 0xb1000000);
  if (ret)
    {
      return ret;
    }

  if (getreg32(TOP_PWR_AP_M4_CTL0) & TOP_PWR_AP_CPU_SEL)
    {
      /* Boot AP CK802 */

      modifyreg32(TOP_PWR_AP_M4_CTL0, TOP_PWR_AP_CPU_SEL_MOD_MK,
                  TOP_PWR_AP_CPU_SEL_MOD_CK);
      modifyreg32(TOP_PWR_CK802_CTL0, 0, TOP_PWR_CPUCLK_EN);
      modifyreg32(TOP_PWR_CK802_CTL0, TOP_PWR_CPU_RSTCTL, 0);
    }
  else
    {
      /* Boot AP M4 */

      modifyreg32(TOP_PWR_AP_M4_CTL0, TOP_PWR_AP_CPU_SEL_MOD_MK,
                  TOP_PWR_AP_CPU_SEL_MOD_M4);
      putreg32(TOP_PWR_AP_M4_PORESET << 16, TOP_PWR_AP_M4_RSTCTL);
    }

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
      .num           = 8,
    },
    .rpmsg_vring1    =
    {
      .align         = 0x8,
      .num           = 8,
    },
    .buf_size        = 0x320,
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

  song_rptun_initialize(&rptun_cfg_ap, g_mbox[CPU_INDEX_AP], g_mbox[CPU_INDEX_CP]);

#  ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize();
#  endif

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#endif

#ifdef CONFIG_FS_HOSTFS_RPMSG_SERVER
  hostfs_rpmsg_server_init();
#endif
}
#endif

#ifdef CONFIG_RTC_SONG
static int up_rtc_initialize(void)
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
static void up_ioe_init(void)
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

#ifdef CONFIG_SONG_SPI_FLASH
static void up_partition_init(FAR struct partition_s *part, FAR void *arg)
{
  char path[NAME_MAX];

  snprintf(path, NAME_MAX, "/dev/%s", part->name);

  register_mtdpartition(path, 0, arg, part->firstblock, part->nblocks);
}

#ifdef CONFIG_SONG_EFUSE
static void up_efuse_init(void)
{
  song_efuse_initialize(0, 0xb0150000, 45);
}
#endif

static void up_flash_init(void)
{
  char *path = "/dev/spiflash";
  FAR struct mtd_dev_s *mtd;

  mtd = song_spi_flash_initialize(&spiflash);
  if (mtd == NULL)
    {
      ferr("ERROR: Spi flash initialize failed\n");
      return;
    }

  register_mtddriver(path, mtd, 0, mtd);
  parse_block_partition(path, up_partition_init, path);
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

#ifdef CONFIG_SONG_EFUSE
  up_efuse_init();
#endif

#ifdef CONFIG_SONG_SPI_FLASH
  up_flash_init();
#endif

  up_extra_init();
}

static void up_ds_enter_work(void)
{
  /* Jump to flash_pd */

  putreg32(TOP_PWR_FLASH_S_2PD_JMP << 16 |
           TOP_PWR_FLASH_S_2PD_JMP, TOP_PWR_SLPCTL_CP_M4);

  /* Set PMICFSM enable full chip to DS */

  modifyreg32(TOP_PMICFSM_CONFIG1, 0, TOP_PMICFSM_DS_SLP_VALID);
}

static void up_top_pwr_work(FAR void *arg)
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
}

static int up_top_pwr_isr(int irq, FAR void *context, FAR void *arg)
{
  static struct work_s worker;
  uint32_t status = getreg32(TOP_PWR_INTR_ST_CP_M4_1);

  if (status & TOP_PWR_SLPU_FLASH_S)
    {
      work_queue(HPWORK, &worker, up_top_pwr_work, (void *)status, 0);
      putreg32(TOP_PWR_SLPU_FLASH_S, TOP_PWR_INTR_ST_CP_M4_1);
    }
  else if (status & TOP_PWR_AP_DS_WAKEUP)
    {
      work_queue(HPWORK, &worker, up_top_pwr_work, (void *)status, 0);
      putreg32(TOP_PWR_AP_DS_WAKEUP, TOP_PWR_INTR_ST_CP_M4_1);
    }

  return 0;
}

void up_finalinitialize(void)
{
  /* Attach TOP_PWR intr */

  irq_attach(18, up_top_pwr_isr, NULL);
  up_enable_irq(18);

  /* Enable SLPU_FLASH_S & AP_DS_WAKEUP intr */

  modifyreg32(TOP_PWR_INTR_EN_CP_M4_1, 0, TOP_PWR_SLPU_FLASH_S);
  modifyreg32(TOP_PWR_INTR_EN_CP_M4_1, 0, TOP_PWR_AP_DS_WAKEUP);

#ifdef CONFIG_SONG_RPTUN
  rptun_boot(CPU_NAME_AP);
#endif

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
  up_cpu_ds(false);
  up_cpu_wfi();
}

void up_cpu_idle(void)
{
  up_cpu_lp(true, false);
  up_cpu_ds(false);
  up_cpu_wfi();
}

void up_cpu_standby(void)
{
  up_cpu_lp(true, true);
  up_cpu_ds(false);
  up_cpu_save();
}

void up_cpu_sleep(void)
{
  up_cpu_lp(true, true);
  up_cpu_ds(true);
  putreg32(TOP_PWR_CP_M4_AU_PD_MK << 16 |
           TOP_PWR_CP_M4_AU_PD_MK, TOP_PWR_CP_UNIT_PD_CTL);
  up_cpu_save();
}

#endif /* CONFIG_ARCH_CHIP_U11_CP */
