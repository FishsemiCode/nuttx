/****************************************************************************
 * arch/arm/src/song/u1_ap_clk.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: zhuyanlin <zhuyanlin@pinecone.net>
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
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

#if defined(CONFIG_ARCH_CHIP_U1_AP) && defined(CONFIG_SONG_CLK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct song_default_rate_clk def_rate[] =
{
  {
    .name = "sp/i2c0_mclk",
    .rate = 80000000,
  },
  {
    .name = "sp/i2c1_mclk",
    .rate = 80000000,
  },
#ifdef CONFIG_16550_UART2
  {
    .name = "sp/uart2_clk",
    .rate = CONFIG_16550_UART2_CLOCK,
  },
#endif
#ifdef CONFIG_16550_UART3
  {
    .name = "sp/uart3_clk",
    .rate = CONFIG_16550_UART3_CLOCK,
  },
#endif
  {}
};

static const struct song_clk_table clk_tbl =
{
  .rpmsg_server      = false,
  .def_rate          = def_rate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xb0040000, &clk_tbl);
}
#endif /* CONFIG_ARCH_CHIP_U1_CP */
