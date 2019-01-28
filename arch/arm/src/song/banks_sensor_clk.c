/****************************************************************************
 * arch/arm/src/song/banks_sensor_clk.c
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

#include <nuttx/clk/clk.h>
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

#if defined(CONFIG_ARCH_CHIP_BANKS_SENSOR) && defined(CONFIG_SONG_CLK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct song_fixed_rate_clk fixed_rate[] =
{
  {
    .name = "sen_clk32k",
    .fixed_rate = 32768,
  },
  {
    .name = "sen_clk26m",
    .fixed_rate = 24576000,
  },
  {},
};

static const struct song_fixed_factor_clk fixed_factor[] =
{
  {
    .name = "sen_wdt_tclk_en",
    .parent_name = "sen_clk26m",
    .fixed_mult = 1,
    .fixed_div = 2,
  },
  {},
};

static const struct song_sdiv_gr_clk sdiv_gr[] =
{
  {
    .name = "sen_bus_mclk",
    .parent_name = "sen_pll_out",
    .div_offset = 0x044,
    .div_width = 3,
  },
  {},
};

static const struct song_sdiv_gr_clk gr_sdiv[] =
{
  {
    .name = "sen_ssi0_mclk",
    .parent_name = "sen_pll_mclk",
    .div_offset = 0x070,
    .div_width = 4,
  },
  {
    .name = "sen_ssi1_mclk",
    .parent_name = "sen_pll_mclk",
    .div_offset = 0x074,
    .div_width = 4,
  },
  {},
};

static const struct song_gr_fdiv_clk gr_fdiv[] =
{
  {
    .name = "sen_i2s_mclk",
    .parent_name = "sen_pll_mclk",
    .en_offset = 0x58,
    .en_shift = 0,
    .gr_offset = 0x58,
    .div_offset = 0x5c,
  },
  {},
};

static const struct song_gr_clk gr[] =
{
  {
    .name = "gnss_clk",
    .parent_name = "sen_bus_mclk",
    .en_offset = 0x78,
    .en_shift = 0,
    .mult_offset = 0x78,
    .mult_shift = 4,
    .mult_width = 3,
  },
  {},
};

static const struct song_div_clk div[] =
{
  {
    .name = "sen_pll_mclk",
    .parent_name = "sen_pll_out",
    .en_offset = 0x040,
    .en_shift = 0,
    .div_offset = 0x040,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "sen_i2c0_mclk",
    .parent_name = "sen_pll_mclk",
    .en_offset = 0x050,
    .en_shift = 0,
    .div_offset = 0x050,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "sen_i2c1_mclk",
    .parent_name = "sen_pll_mclk",
    .en_offset = 0x054,
    .en_shift = 0,
    .div_offset = 0x054,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "at_clk_src_pll",
    .parent_name = "sen_pll_mclk",
    .en_offset = 0x064,
    .en_shift = 0,
    .div_offset = 0x064,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "sen_pdm0_clk",
    .parent_name = "sen_clk26m",
    .en_offset = 0x060,
    .en_shift = 0,
    .div_offset = 0x060,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "sen_pdm1_clk",
    .parent_name = "sen_clk26m",
    .en_offset = 0x068,
    .en_shift = 0,
    .div_offset = 0x068,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "bus_lp_clk",
    .parent_name = "sen_clk26m",
    .en_offset = 0x04c,
    .en_shift = 0,
    .div_offset = 0x04c,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "pll_adj_clk",
    .parent_name = "sen_clk26m",
    .en_offset = 0x0a0,
    .en_shift = 0,
    .div_offset = 0x0a0,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "sen_pclk",
    .parent_name = "sen_bus_mclk",
    .en_offset = 0x048,
    .en_shift = 0,
    .div_offset = 0x048,
    .div_shift = 4,
    .div_width = 4,
  },
  {},
};

static const struct song_gate_clk gate[] =
{
  {
    .name = "gnss_hclk",
    .parent_name = "gnss_clk",
    .en_offset = 0x090,
    .en_shift = 0,
  },
  {
    .name = "gnss_pp_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x090,
    .en_shift = 1,
  },
  {
    .name = "gnss_ae_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x090,
    .en_shift = 2,
  },
  {
    .name = "gnss_ae_fifo_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x090,
    .en_shift = 3,
  },
  {
    .name = "gnss_conf_mem_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x090,
    .en_shift = 4,
  },
  {
    .name = "gnss_te_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x090,
    .en_shift = 5,
  },
  {
    .name = "gnss_te_mem_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x090,
    .en_shift = 6,
  },
  {
    .name = "gnss_te_fifo_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x090,
    .en_shift = 7,
  },
  {
    .name = "gnss_viterbi_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x090,
    .en_shift = 8,
  },
  {
    .name = "gnss_pps_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x090,
    .en_shift = 9,
  },
  {
    .name = "gnss_i_clk32k",
    .parent_name = "sen_clk32k",
    .en_offset = 0x090,
    .en_shift = 10,
  },
  {
    .name = "iram_icm_hclk",
    .parent_name = "sen_bus_mclk",
    .en_offset = 0x090,
    .en_shift = 11,
  },
  {
    .name = "dram_icm_hclk",
    .parent_name = "sen_bus_mclk",
    .en_offset = 0x090,
    .en_shift = 12,
  },
  {
    .name = "sen_m4_clk",
    .parent_name = "sen_bus_mclk",
    .en_offset = 0x094,
    .en_shift = 0,
  },
  {
    .name = "sen_m4_wic_clk",
    .parent_name = "sen_bus_mclk",
    .en_offset = 0x094,
    .en_shift = 0,
  },
  {
    .name = "sen_m4_stclk",
    .parent_name = "sen_clk26m",
    .en_offset = 0x094,
    .en_shift = 2,
  },
  {
    .name = "sen_dmas_hclk",
    .parent_name = "sen_bus_mclk",
    .en_offset = 0x094,
    .en_shift = 3,
  },
  {
    .name = "sen_wdt_tclk",
    .parent_name = "sen_wdt_tclk_en",
    .en_offset = 0x094,
    .en_shift = 5,
  },
  {
    .name = "sen_gpio_clk32k",
    .parent_name = "sen_clk32k",
    .en_offset = 0x094,
    .en_shift = 7,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "sensorbus_gpsclk",
    .parent_name = "gnss_clk",
    .en_offset = 0x090,
    .en_shift = 13,
  },
  {
    .name = "sen_wdt_pclk",
    .parent_name = "sen_pclk",
    .en_offset = 0x094,
    .en_shift = 4,
  },
  {
    .name = "sen_gpio_pclk",
    .parent_name = "sen_pclk",
    .en_offset = 0x094,
    .en_shift = 6,
  },
  {
    .name = "sen_pdm0_pclk",
    .parent_name = "sen_pclk",
    .en_offset = 0x094,
    .en_shift = 8,
  },
  {
    .name = "sen_pdm1_pclk",
    .parent_name = "sen_pclk",
    .en_offset = 0x094,
    .en_shift = 9,
  },
  {
    .name = "sen_i2s_pclk",
    .parent_name = "sen_pclk",
    .en_offset = 0x094,
    .en_shift = 10,
  },
  {
    .name = "sen_i2c0_pclk",
    .parent_name = "sen_pclk",
    .en_offset = 0x094,
    .en_shift = 11,
  },
  {
    .name = "sen_i2s1_pclk",
    .parent_name = "sen_pclk",
    .en_offset = 0x094,
    .en_shift = 12,
  },
  {
    .name = "sen_ssi0_pclk",
    .parent_name = "sen_pclk",
    .en_offset = 0x094,
    .en_shift = 13,
  },
  {
    .name = "sen_ssi1_pclk",
    .parent_name = "sen_pclk",
    .en_offset = 0x094,
    .en_shift = 14,
  },
#endif
  {},
};

static const char * const at_clk_parents[] =
{
  "sen_clk26m",
  "at_clk_src_pll",
};

static const struct song_mux_clk mux[] =
{
  {
    .name = "at_clk",
    .parent_names = at_clk_parents,
    .num_parents = ARRAY_SIZE(at_clk_parents),
    .en_offset = 0x06c,
    .en_shift = 0,
    .mux_offset = 0x06c,
    .mux_shift = 4,
    .mux_width = 1,
    .mux_flags = CLK_MUX_HIWORD_MASK,
  },
  {},
};

static const struct song_pll_clk pll[] =
{
  {
    .name = "sen_pll_out",
    .parent_name = "sen_clk26m",
    .cfg0_offset = 0x30,
    .cfg1_offset = 0x34,
    .ctl_offset = 0x38,
    .ctl_shift = 0,
  },
  {},
};

static const struct song_lp_reg_clk lp_reg[] =
{
  /* LP_EN0 */
  {
    .offset = 0x1c4,
    .value = 0xffffffff,
  },
  {}
};

static const struct song_clk_table banks_sen_clk_tbl =
{
  .fixed_rate_clks   = fixed_rate,
  .fixed_factor_clks = fixed_factor,
  .sdiv_gr_clks      = sdiv_gr,
  .gr_sdiv_clks      = gr_sdiv,
  .gr_fdiv_clks      = gr_fdiv,
  .gr_clks           = gr,
  .div_clks          = div,
  .gate_clks         = gate,
  .mux_clks          = mux,
  .pll_clks          = pll,
  .lp_reg            = lp_reg,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xf8b14000, &banks_sen_clk_tbl);
}

void up_clk_finalinitialize(void)
{
}
#endif /* (CONFIG_ARCH_CHIP_BANKS_SENSOR) && (CONFIG_SONG_CLK) */
