/****************************************************************************
 * arch/arm/src/song/banks_a7.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Yuan Zhang <zhangyuan7@pinecone.net>
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

#ifdef CONFIG_ARCH_CHIP_BANKS_A7

#include <nuttx/drivers/addrenv.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/dma/song_dmag.h>
#include <nuttx/fs/hostfs_rpmsg.h>
#include <nuttx/mbox/song_mbox.h>
#include <nuttx/rptun/song_rptun.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/serial/uart_rpmsg.h>
#include <nuttx/syslog/syslog_rpmsg.h>
#include <nuttx/timers/dw_timer.h>
#include <nuttx/timers/arch_timer.h>

#include "chip.h"
#include "mmu.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Public Define
 ****************************************************************************/

#define TOP_MAILBOX_BASE            (0xE1000000)

#define CPU_NAME_AP                 "ap"
#define CPU_NAME_A7                 "a7"

#define CPU_INDEX_AP                0
#define CPU_INDEX_A7                1

#define _LOGBUF_BASE                ((uintptr_t)&_slog)
#define _LOGBUF_SIZE                ((uint32_t)&_logsize)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _slog;
extern uint32_t _logsize;

/****************************************************************************
 * Private Data
 ****************************************************************************/

#if defined (CONFIG_SONG_DMAS) || defined (CONFIG_SONG_DMAG)
static FAR struct dma_dev_s *g_dma[3] =
{
  [2] = DEV_END,
};
#endif

#ifdef CONFIG_SONG_MBOX
FAR struct mbox_dev_s *g_mbox[3] =
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
    {.va = 0xe0000000, .pa = 0xf8000000, .size = 0x08000000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

#ifdef CONFIG_ARCH_USE_MMU
  const struct section_mapping_s mapping[] =
  {
    {NUTTX_DEV_PADDR, NUTTX_DEV_VADDR, MMU_IOFLAGS, NUTTX_DEV_NSECT},
  };

  mmu_l1_map_regions(mapping, ARRAY_SIZE(mapping));
#endif

  simple_addrenv_initialize(addrenv);

#ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init_early(CPU_NAME_AP, (void *)_LOGBUF_BASE, _LOGBUF_SIZE);
#endif
}

void arm_timer_initialize(void)
{
#ifdef CONFIG_TIMER_DW
  static const struct dw_timer_config_s config0 =
  {
    .minor = -1,
    .base  = 0x87020000,
    .irq   = 42,
    .freq  = 26000000,
  };

  up_timer_set_lowerhalf(dw_timer_initialize(&config0));

#ifdef CONFIG_CPULOAD_PERIOD
  static const struct dw_timer_config_s config1 =
  {
    .minor = -1,
    .base  = 0x87020014,
    .irq   = 43,
    .freq  = 26000000,
  };

  sched_period_extclk(dw_timer_initialize(&config1));
#endif

  static const struct dw_timer_config_s config2 =
  {
    .minor = 2,
    .base  = 0x87020028,
    .irq   = 44,
    .freq  = 26000000,
  };

  dw_timer_initialize(&config2);

  static const struct dw_timer_config_s config3 =
  {
    .minor = 3,
    .base  = 0x8702003c,
    .irq   = 45,
    .freq  = 26000000,
  };

  dw_timer_initialize(&config3);
#endif
}

void up_dma_initialize(void)
{
#ifdef CONFIG_SONG_DMAS
  g_dma[0] = song_dmas_initialize(CPU_INDEX_A7, 0xe1003000, 52, NULL);
#endif
#ifdef CONFIG_SONG_DMAG
  g_dma[1] = song_dmag_initialize(CPU_INDEX_A7, 0x86300000, 51, NULL);
#endif
}

#if defined(CONFIG_16550_UART) && defined(CONFIG_SONG_DMAS)
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
  return g_dma[0] ? DMA_GET_CHAN(g_dma[0], ident) : NULL;
}
#endif

#ifdef CONFIG_RPMSG_UART
void rpmsg_serialinit(void)
{
  uart_rpmsg_init(CPU_NAME_AP, "A7", 1024, true);
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
      .set_off    = 0x40,
      .en_off     = 0x44,
      .en_bit     = 0,
      .src_en_off = 0x48,
      .sta_off    = 0x50,
      .chnl_count = 64,
      .irq        = -1,
    },
    {
      .index      = CPU_INDEX_A7,
      .base       = TOP_MAILBOX_BASE,
      .set_off    = 0x0,
      .en_off     = 0x4,
      .en_bit     = 0,
      .src_en_off = 0x8,
      .sta_off    = 0x10,
      .chnl_count = 64,
      .irq        = 48,
    },
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
      .num           = 1,
    },
    .offset          =
    {
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
    .cpuname = CPU_NAME_AP,
    .rsc     = &rptun_rsc_ap,
    .vringtx = 0,
    .vringrx = 0,
  };

  song_rptun_initialize(&rptun_cfg_ap, g_mbox[CPU_INDEX_AP], g_mbox[CPU_INDEX_A7]);

#  ifdef CONFIG_CLK_RPMSG
  clk_rpmsg_initialize();
#  endif

#  ifdef CONFIG_RPMSG_REGULATOR
  rpmsg_regulator_init(CPU_NAME_AP, true);
#  endif

#  ifdef CONFIG_SYSLOG_RPMSG
  syslog_rpmsg_init();
#  endif

#  ifdef CONFIG_FS_HOSTFS_RPMSG
  hostfs_rpmsg_init(CPU_NAME_AP);
#  endif
}
#endif

void up_lateinitialize(void)
{
#ifdef CONFIG_SONG_MBOX
      up_mbox_init();
#endif

#ifdef CONFIG_SONG_RPTUN
      up_rptun_init();
#endif
}

void up_finalinitialize(void)
{
}

#endif /* CONFIG_ARCH_CHIP_BANKS_A7 */
