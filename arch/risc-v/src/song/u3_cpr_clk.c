/****************************************************************************
 * arch/risc-v/src/song/u3_cpr_clk.c
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

#if defined(CONFIG_ARCH_CHIP_U3_CPR) && defined(CONFIG_SONG_CLK)

#include <nuttx/clk/clk.h>
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *rf_port_src[] =
{
  "ap/sys_clk",
  "ap/pll0_mclk",
};

static const struct song_mux_div_clk mux_div[] =
{
  {
    .name = "rf_port_clk",
    .parent_names = rf_port_src,
    .num_parents = ARRAY_SIZE(rf_port_src),
    .en_offset = 0x2c4,
    .en_shift = 0,
    .mux_offset = 0x2c4,
    .mux_shift = 11,
    .mux_width = 1,
    .div_offset = 0x2c4,
    .div_shift = 4,
    .div_width = 5,
  },
  {},
};

static const struct song_div_clk div[] =
{
  {
    .name = "cp_bus_mclk",
    .parent_name = "ap/pll1",
    .en_offset = 0x058,
    .en_shift = 0,
    .div_offset = 0x058,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IS_CRITICAL,
  },
  {
    .name = "sim_clk",
    .parent_name = "ap/sys_clk",
    .en_offset = 0x064,
    .en_shift = 0,
    .div_offset = 0x064,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "sim_hi_clk",
    .parent_name = "ap/pll0_mclk",
    .en_offset = 0x068,
    .en_shift = 0,
    .div_offset = 0x068,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "rfif_spi_clk",
    .parent_name = "ap/sys_clk",
    .en_offset = 0x2e0,
    .en_shift = 0,
    .div_offset = 0x2e0,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rf_tm2_calib_clk",
    .parent_name = "ap/pll0_mclk",
    .en_offset = 0x02c8,
    .en_shift = 0,
    .div_offset = 0x02c8,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rf_ltesp_clk",
    .parent_name = "ap/pll0_mclk",
    .en_offset = 0x2cc,
    .en_shift = 0,
    .div_offset = 0x2cc,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rfif_rffe_clk",
    .parent_name = "ap/pll0_mclk",
    .en_offset = 0x2d0,
    .en_shift = 0,
    .div_offset = 0x2d0,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rfif_match_rx_clk",
    .parent_name = "ap/sys_clk",
    .en_offset = 0x2e8,
    .en_shift = 0,
    .div_offset = 0x2e8,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rfif_match_tx_clk",
    .parent_name = "ap/sys_clk",
    .en_offset = 0x2ec,
    .en_shift = 0,
    .div_offset = 0x2ec,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "cp_rocket",
    .parent_name = "ap/pll1",
    .en_offset = 0x0c0,
    .en_shift = 0,
    .div_offset = 0x0c0,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IS_CRITICAL,
  },
  {
    .name = "gnss_clk",
    .parent_name = "ap/pll1",
    .en_offset = 0x078,
    .en_shift = 0,
    .div_offset = 0x078,
    .div_shift = 4,
    .div_width = 4,
  },
  {},
};

static const struct song_gate_clk gate[] =
{
  {
    .name = "rf_tp_clk32k",
    .parent_name = "ap/clk32k",
    .en_offset = 0x90,
    .en_shift = 13,
  },
  {
    .name = "rfif_bus_clk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x90,
    .en_shift = 14,
  },
  {
    .name = "pdcp_clk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x094,
    .en_shift = 4,
  },
  {
    .name = "dmag_clk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 5,
  },
  {
    .name = "cp_bus_shram_mclk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 6,
    .clk_flags = CLK_IS_CRITICAL,
  },
  {
    .name = "wdt1_tclk",
    .parent_name = "ap/clk32k",
    .en_offset = 0x94,
    .en_shift = 8,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "sim_pclk",
    .parent_name = "ap/top_peribus_mclk",
    .en_offset = 0x98,
    .en_shift = 7,
  },
  {
    .name = "wdt1_pclk",
    .parent_name = "ap/top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 6,
  },
#endif
  {
    .name = "gnss_hclk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 0,
  },
  {
    .name = "gnss_pp_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 1,
  },
  {
    .name = "gnss_ae_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 2,
  },
  {
    .name = "gnss_ae_fifo_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 3,
  },
  {
    .name = "gnss_conf_mem_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 4,
  },
  {
    .name = "gnss_te_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 5,
  },
  {
    .name = "gnss_te_mem_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 6,
  },
  {
    .name = "gnss_te_fifo_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 7,
  },
  {
    .name = "gnss_i_rf0_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 8,
  },
  {},
};

static const struct song_clk_table clk_tbl =
{
  .mux_div_clks      = mux_div,
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
#endif /* (CONFIG_ARCH_CHIP_U3_CPR) && (CONFIG_SONG_CLK) */
