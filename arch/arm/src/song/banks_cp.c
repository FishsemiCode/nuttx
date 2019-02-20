/****************************************************************************
 * arch/arm/src/song/banks_cp.c
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

#ifdef CONFIG_ARCH_CHIP_BANKS_CP

#include <nuttx/drivers/addrenv.h>
#include <nuttx/dma/song_dmas.h>
#include <nuttx/serial/uart_16550.h>
#include <nuttx/timers/dw_timer.h>
#include <nuttx/timers/arch_timer.h>

#include "chip.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Public Define
 ****************************************************************************/

/****************************************************************************
 * Public Data
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
 * Public Functions
 ****************************************************************************/

void up_earlyinitialize(void)
{
  static const struct simple_addrenv_s addrenv[] =
  {
    {.va = 0xe0000000, .pa = 0xf8000000, .size = 0x08000000},
    {.va = 0x00000000, .pa = 0x00000000, .size = 0x00000000},
  };

  simple_addrenv_initialize(addrenv);
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
  g_dma[0] = song_dmas_initialize(0, 0xe1003000, 52, NULL);
#endif
}

#if defined(CONFIG_16550_UART) && defined(CONFIG_SONG_DMAS)
FAR struct dma_chan_s *uart_dmachan(uart_addrwidth_t base, unsigned int ident)
{
  return g_dma[0] ? DMA_GET_CHAN(g_dma[0], ident) : NULL;
}
#endif

void up_lateinitialize(void)
{

}

void up_finalinitialize(void)
{

}

#endif /* CONFIG_ARCH_CHIP_BANKS_CP */
