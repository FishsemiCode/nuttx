/****************************************************************************
 * arch/risc-v/src/song/u3_cpx_clk.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: zhuyanlin <zhuyanlin@fishsemi.com>
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

#if defined(CONFIG_ARCH_CHIP_U3_CPX) && defined(CONFIG_SONG_CLK)

#include <nuttx/clk/clk.h>
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct song_sdiv_gr_clk sdiv_gr[] =
{
  {
    .name = "ceva_free_clk",
    .parent_name = "ap/pll1",
    .div_offset = 0x2e4,
    .div_width = 4,
  },
  {},
};

static const struct song_div_clk div[] =
{
  {
    .name = "cp_ecs",
    .parent_name = "ap/pll1",
    .en_offset = 0x05c,
    .en_shift = 0,
    .div_offset = 0x05c,
    .div_shift = 4,
    .div_width = 4,
  },
  {},
};

static const struct song_gate_clk gate[] =
{
  {
    .name = "viterbi_clk",
    .parent_name = "cpr/cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 0,
  },
  {
    .name = "dlharq_clk",
    .parent_name = "cpr/cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 1,
  },
  {
    .name = "dlsp_clk",
    .parent_name = "cpr/cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 2,
  },
  {
    .name = "ulbsp_clk",
    .parent_name = "cpr/cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 3,
  },
  {
    .name = "pdcp_clk",
    .parent_name = "cpr/cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 4,
  },
  {
    .name = "wdt2_tclk",
    .parent_name = "ap/clk32k",
    .en_offset = 0x94,
    .en_shift = 9,
  },
  {
    .name = "xc5_ictl_clk",
    .parent_name = "cpr/cp_bus_mclk",
    .en_offset = 0x98,
    .en_shift = 9,
    .clk_flags = CLK_IS_CRITICAL,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "wdt2_pclk",
    .parent_name = "ap/top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 7,
  },
#endif
  {},
};

static const struct song_clk_table clk_tbl =
{
  .sdiv_gr_clks      = sdiv_gr,
  .div_clks          = div,
  .gate_clks         = gate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xb0040000, &clk_tbl);
}

void up_clk_finalinitialize(void)
{
  clk_disable_unused();
}
#endif /* (CONFIG_ARCH_CHIP_U3_CPX) && (CONFIG_SONG_CLK) */
