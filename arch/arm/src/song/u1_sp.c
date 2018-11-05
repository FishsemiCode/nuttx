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

#include <fcntl.h>

#include <nuttx/arch.h>
#include <nuttx/clk/clk.h>
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
#include <nuttx/spi/spi_dw.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/arch_alarm.h>
#include <nuttx/timers/arch_rtc.h>
#include <nuttx/timers/dw_wdt.h>
#include <nuttx/timers/song_oneshot.h>
#include <nuttx/timers/song_rtc.h>
#include <nuttx/wqueue.h>

#include <sys/stat.h>

#include "chip.h"
#include "song_addrenv.h"
#include "song_idle.h"
#include "systick.h"
#include "up_arch.h"
#include "up_internal.h"

#ifdef CONFIG_ARCH_CHIP_U1_SP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CPU_NAME_AP                 "ap"
#define CPU_NAME_CP                 "cp"

#define LOGBUF_BASE                 ((uintptr_t)&_slog)
#define LOGBUF_SIZE                 ((uint32_t)&_logsize)

#define CPRAM1_RSVD_BASE            (0x60054000)
#define CPRAM1_RSVD_SIZE            (CPRAM1_RSVD_BASE + 0x4)

#define TOP_MAILBOX_BASE            (0xb0030000)

#define TOP_PWR_BASE                (0xb0040000)
#define TOP_PWR_AP_M4_RSTCTL        (TOP_PWR_BASE + 0x0e0)
#define TOP_PWR_SEC_M4_RSTCTL       (TOP_PWR_BASE + 0x0d8)
#define TOP_PWR_CP_M4_RSTCTL        (TOP_PWR_BASE + 0x0dc)
#define TOP_PWR_SFRST_CTL           (TOP_PWR_BASE + 0x11c)
#define TOP_PWR_INTR_EN_SEC_M4_1    (TOP_PWR_BASE + 0x140)
#define TOP_PWR_INTR_ST_SEC_M4_1    (TOP_PWR_BASE + 0x144)
#define TOP_PWR_SEC_M4_INTR2SLP_MK0 (TOP_PWR_BASE + 0x148)
#define TOP_PWR_CP_UNIT_PD_CTL      (TOP_PWR_BASE + 0x1fc)
#define TOP_PWR_RES_REG2            (TOP_PWR_BASE + 0x260)
#define TOP_PWR_SLPCTL0             (TOP_PWR_BASE + 0x350)
#define TOP_PWR_SLPCTL_SEC_M4       (TOP_PWR_BASE + 0x358)
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

#define TOP_PWR_RESET_NORMAL        (0x00000000)
#define TOP_PWR_RESET_ROMBOOT       (0xaaaa1234)

#define TOP_PWR_SLP_DMA_MK          (1 << 2)

#define TOP_PWR_SEC_M4_SLP_EN       (1 << 0)
#define TOP_PWR_SEC_M4_DS_SLP_EN    (1 << 2)
#define TOP_PWR_FLASH_S_2PD_JMP     (1 << 8)

#define TOP_PWR_CP_AU_PD_MK         (1 << 7)

#define TOP_PWR_SEC_AU_PD_MK        (1 << 7)

#define PMIC_FSM_BASE               (0xb2010000)
#define PMIC_FSM_CONFIG1            (PMIC_FSM_BASE + 0x0c)

#define PMIC_FSM_DS_SLP_VALID       (1 << 0)

#define SECURITY_BASE               (0xb0150000)
#define SECURITY_CFG_0              (SECURITY_BASE + 0x30)
#define SECURITY_CFG_0_VALUE        (0x00010000)

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

#ifdef CONFIG_SPI_DW
FAR struct spi_dev_s *g_spi[3] =
{
  [2] = DEV_END,
};
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

  up_addrenv_initialize(addrenv);

  /* Always enable the full chip deep sleep */
  modifyreg32(PMIC_FSM_CONFIG1, 0, PMIC_FSM_DS_SLP_VALID);

  /* Always allow enter FLASH_S */
  putreg32(TOP_PWR_SEC_M4_DS_SLP_EN << 16 |
           TOP_PWR_SEC_M4_DS_SLP_EN, TOP_PWR_SLPCTL_SEC_M4);

#ifndef CONFIG_CPULOAD_PERIOD
  /* Allow TCM to LP, careful with it. At this time,
   * if use systick as weakup reason form DEEPSLEEP, CPU will hang.
   */
  putreg32(TOP_PWR_SEC_AU_PD_MK << 16, TOP_PWR_SEC_M4_TCM_PD_CTL);
#endif

  /* Set the DMAS no effort to power down */
  putreg32(TOP_PWR_SLP_DMA_MK << 16 |
           TOP_PWR_SLP_DMA_MK, TOP_PWR_SLPCTL0);

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

#ifdef CONFIG_SONG_DMAS
void up_dmainitialize(void)
{
  g_dma[0] = song_dmas_initialize(2, 0xb0020000, 28, "dmas_hclk");
}
#endif

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
  uart_rpmsg_init(CPU_NAME_AP, "SP", 1024, true);
}
#endif

#ifdef CONFIG_NET_RPMSG_DRV
void up_netinitialize(void)
{
  net_rpmsg_drv_init(CPU_NAME_CP, "sl0", NET_LL_SLIP);
}
#endif

#ifdef CONFIG_SONG_RPTUN
static int ap_boot(const struct song_rptun_config_s *config)
{
  /* SP <--shram0--> AP
   * shram0 default enabled
   */

  putreg32(TOP_PWR_AP_M4_PORESET << 16, TOP_PWR_AP_M4_RSTCTL);
  return 0;
}

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

static void cp_flash_save_data(void)
{
  int fd;
  size_t save_bytes;
  size_t bytes;

  fd = open("/data/cpram1.rsvd", O_WRONLY | O_CREAT | O_TRUNC);
  if (fd < 0)
    {
      return;
    }
    /*
     * struct cpram1.rsvd
     * {
     *   uint32_t magic;
     *   uint32_t size;
     *   uint32_t checksum;
     *   uint8_t  data[];
     * }
     */

  save_bytes = getreg32(CPRAM1_RSVD_SIZE) + 12;
  bytes = write(fd, (FAR const void *)CPRAM1_RSVD_BASE, save_bytes);
  close(fd);
  if (bytes != save_bytes)
    {
      /* Write not completely successfully, delelte file */

      unlink("/data/cpram1.rsvd");
    }
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

  /* Jump to flash_pd */

  putreg32(TOP_PWR_FLASH_S_2PD_JMP << 16 |
           TOP_PWR_FLASH_S_2PD_JMP, TOP_PWR_SLPCTL_SEC_M4);
}

static void cp_flash_save_work(FAR void *arg)
{
  /* Save cpram to flash */

  cp_flash_save_prepare();
  cp_flash_save_data();
  cp_flash_save_finish();
}

static int cp_flash_save_isr(int irq, FAR void *context, FAR void *arg)
{
  if (getreg32(TOP_PWR_INTR_ST_SEC_M4_1) & TOP_PWR_SLPU_FLASH_S)
    {
      static struct work_s worker;
      modifyreg32(TOP_PWR_INTR_ST_SEC_M4_1, 0, TOP_PWR_SLPU_FLASH_S);
      work_queue(LPWORK, &worker, cp_flash_save_work, NULL, 0);
    }

  return 0;
}

static void cp_flash_restore(void)
{
  int fd;
  struct stat f_info;

  fd = open("/data/cpram1.rsvd", O_RDONLY);
  if (fd < 0)
    {
      return;
    }

  fstat(fd, &f_info);
  read(fd, (FAR void *)CPRAM1_RSVD_BASE, f_info.st_size);
  close(fd);
}

static int cp_boot(const struct song_rptun_config_s *config)
{
  cp_flash_restore();

  /* Attach and enable flash_s intr. */
  irq_attach(18, cp_flash_save_isr, NULL);
  up_enable_irq(18);
  modifyreg32(TOP_PWR_INTR_EN_SEC_M4_1, 0, TOP_PWR_SLPU_FLASH_S);

  /* SP <--shram1--> CP
   * enable shram1 for IPC
   */

  putreg32(SECURITY_CFG_0_VALUE, SECURITY_CFG_0);
  putreg32(TOP_PWR_CP_M4_PORESET << 16, TOP_PWR_CP_M4_RSTCTL);
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
    .buf_size        = 0x5e0,
  };

  static const struct song_rptun_config_s rptun_cfg_ap =
  {
    .cpu_name    = CPU_NAME_AP,
    .role        = RPMSG_MASTER,
    .ch_start_tx = 14,
    .ch_vring_tx = 15,
    .ch_start_rx = 14,
    .ch_vring_rx = 15,
    .rsc         =
    {
      .rsc_tab   = &rptun_rsc_ap.rsc_tbl_hdr,
      .size      = sizeof(rptun_rsc_ap),
    },
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
    .buf_size        = 0x5e0,
  };

  static const struct song_rptun_config_s rptun_cfg_cp =
  {
    .cpu_name    = CPU_NAME_CP,
    .role        = RPMSG_MASTER,
    .ch_start_tx = 0,
    .ch_vring_tx = 1,
    .ch_start_rx = 0,
    .ch_vring_rx = 1,
    .rsc         =
    {
      .rsc_tab   = &rptun_rsc_cp.rsc_tbl_hdr,
      .size      = sizeof(rptun_rsc_cp),
    },
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
      .num           = 4,
    },
    .rpmsg_vring1    =
    {
      .align         = 0x8,
      .num           = 4,
    },
    .buf_size        = 0x1e0,
  };

  mbox_ap = song_mbox_initialize(&mbox_cfg_ap);
  mbox_cp = song_mbox_initialize(&mbox_cfg_cp);
  mbox_sp = song_mbox_initialize(&mbox_cfg_sp);

  song_rptun_initialize(&rptun_cfg_ap, mbox_ap, mbox_sp);
  song_rptun_initialize(&rptun_cfg_cp, mbox_cp, mbox_sp);

#  ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#  endif

#  ifdef CONFIG_FS_HOSTFS_RPMSG_SERVER
  hostfs_rpmsg_server_init();
#  endif
}
#endif

#ifdef CONFIG_RTC_SONG
int up_rtc_initialize(void)
{
  static const struct song_rtc_config_s config =
  {
    .minor = 0,
    .base  = 0xb2020000,
    .irq   = 16,
    .index = 2,
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

#ifdef CONFIG_SPI_DW
static void up_spi_init(void)
{
  static const struct dw_spi_config_s config =
  {
    .bus = 1,
    .base = 0xb0120000,
    .irq = 31,
    .cs_num = 1,
    .cs_gpio[0] = 26,
    .mclk = "spi1_mclk",
  };

  /* XXX: temporarily set the spi1 pinmux, later we will use official
   * pinmux api instead */
  putreg32(0x10, MUX_PIN23);
  putreg32(0x10, MUX_PIN24);
  putreg32(0x10, MUX_PIN25);
  putreg32(0x12, MUX_PIN26);
  g_spi[config.bus] = dw_spi_initialize(&config, g_ioe[0]);
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
    .neraseblocks = 256,
    .timing       = timing,
    .mclk         = "flash_ctrl_hclk",
    .rate         = 102400000,
  };
  char *path = "/dev/onchip";
  FAR struct mtd_dev_s *mtd;

  mtd = song_onchip_flash_initialize(&config);
  register_mtddriver(path, mtd, 0, mtd);
  parse_block_partition(path, up_partition_init, path);
}
#endif

void up_lateinitialize(void)
{
#ifdef CONFIG_SONG_RPTUN
  up_openamp_initialize();
#endif

#ifdef CONFIG_SONG_CLK
  up_clk_initialize();
#endif

#ifdef CONFIG_RTC_SONG
  up_rtc_initialize();
#endif

#ifdef CONFIG_WATCHDOG_DW
  up_wdtinit();
#endif

#ifdef CONFIG_SONG_IOE
  g_ioe[0] = song_ioe_initialize(2, 0xb0060000, 19);
#endif

#ifdef CONFIG_SPI_DW
  up_spi_init();
#endif

#ifdef CONFIG_SONG_ONCHIP_FLASH
  up_flash_init();
#endif

#ifdef CONFIG_SONG_PMIC_APB
  spmu_regulator_apb_initialize(0xb2010000, 0xb0180000);
#endif

#ifdef CONFIG_RPMSG_REGULATOR
  rpmsg_regulator_init(CPU_NAME_AP, 1);
#endif

#ifdef CONFIG_SONG_CLK
  clk_disable_unused();
#endif
}

void up_finalinitialize(void)
{
#ifdef CONFIG_SONG_RPTUN
  ap_boot(NULL);
  cp_boot(NULL);
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

void up_cpu_doze(void)
{
  /* Forbid the full chip power down */
  putreg32(TOP_PWR_SEC_M4_SLP_EN << 16, TOP_PWR_SLPCTL_SEC_M4);

  /* Forbid the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) & ~NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_wfi();
}

void up_cpu_idle(void)
{
  /* Forbid the full chip power down */
  putreg32(TOP_PWR_SEC_M4_SLP_EN << 16, TOP_PWR_SLPCTL_SEC_M4);

  /* Allow the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_wfi();
}

void up_cpu_standby(void)
{
  up_cpu_idle();
}

void up_cpu_sleep(void)
{
  /* Allow the full chip power down */
  putreg32(TOP_PWR_SEC_M4_SLP_EN << 16 |
           TOP_PWR_SEC_M4_SLP_EN, TOP_PWR_SLPCTL_SEC_M4);

  /* Allow the deep sleep */
  putreg32(getreg32(NVIC_SYSCON) | NVIC_SYSCON_SLEEPDEEP, NVIC_SYSCON);

  up_cpu_wfi();
}

#endif /* CONFIG_ARCH_CHIP_U1_SP */
