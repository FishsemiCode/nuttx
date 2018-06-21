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

#include <nuttx/clk/clk-provider.h>
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
    .flags = CLK_IS_ROOT,
    .fixed_rate = 32768,
  },
  {
    .name = "sen_clk26m",
    .flags = CLK_IS_ROOT,
    .fixed_rate = 26000000,
  },
  {},
};

static const struct song_fixed_factor_clk fixed_factor[] =
{
  {
    .name = "sen_wdt_tclk_en",
    .parent_name = "sen_clk26m",
    .flags = 0,
    .fixed_mul = 1,
    .fixed_div = 2,
  },
  {},
};

static const struct song_sdiv_gr_clk sdiv_gr[] =
{
  {
    .name = "sen_bus_mclk",
    .parent_name = "sen_pll_out",
    .flags = 0,
    .div_offset = 0x044,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT) |
                      (CLK_MULT_HIWORD_MASK << SONG_CLK_MULT_FLAG_SHIFT) |
                      (SONG_CLK_GR_DIV_16 << SONG_CLK_PRIVATE_FLAG_SHIFT)),
  },
  {},
};

static const struct song_sdiv_gr_clk gr_sdiv[] =
{
  {
    .name = "sen_ssi0_mclk",
    .parent_name = "sen_pll_mclk",
    .flags = 0,
    .div_offset = 0x070,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                    ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT) |
                    (CLK_MULT_HIWORD_MASK << SONG_CLK_MULT_FLAG_SHIFT)),
  },
  {
    .name = "sen_ssi1_mclk",
    .parent_name = "sen_pll_mclk",
    .flags = 0,
    .div_offset = 0x074,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                    ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT) |
                    (CLK_MULT_HIWORD_MASK << SONG_CLK_MULT_FLAG_SHIFT)),
  },
  {},
};

static const struct song_gr_fdiv_clk gr_fdiv[] =
{
  {
  .name = "sen_i2s_mclk",
  .parent_name = "sen_pll_mclk",
  .flags = 0,
  .en_offset = 0x58,
  .en_shift = 0,
  .gr_offset = 0x58,
  .div_offset = 0x5c,
  .fixed_gr = 0,
  .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                   ((CLK_FRAC_MUL_NEED_EVEN) << SONG_CLK_FRAC_FLAG_SHIFT)),
  },
  {},
};

static const struct song_gr_clk gr[] =
{
  {
    .name = "gnss_clk",
    .parent_name = "sen_bus_mclk",
    .flags = 0,
    .en_offset = 0x78,
    .en_shift = 0,
    .mul_offset = 0x78,
    .mul_shift = 4,
    .mul_width = 3,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     (CLK_MULT_HIWORD_MASK << SONG_CLK_MULT_FLAG_SHIFT)),
  },
  {},
};

static const struct song_sdiv_clk sdiv[] =
{
  {
    .name = "sen_pll_mclk",
    .parent_name = "sen_pll_out",
    .flags = 0,
    .en_offset = 0x040,
    .en_shift = 0,
    .div_offset = 0x040,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "sen_i2c0_mclk",
    .parent_name = "sen_pll_mclk",
    .flags = 0,
    .en_offset = 0x050,
    .en_shift = 0,
    .div_offset = 0x050,
    .div_shift = 4,
    .div_width = 5,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "sen_i2c1_mclk",
    .parent_name = "sen_pll_mclk",
    .flags = 0,
    .en_offset = 0x054,
    .en_shift = 0,
    .div_offset = 0x054,
    .div_shift = 4,
    .div_width = 5,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "at_clk_src_pll",
    .parent_name = "sen_pll_mclk",
    .flags = 0,
    .en_offset = 0x064,
    .en_shift = 0,
    .div_offset = 0x064,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "sen_pdm0_clk",
    .parent_name = "sen_clk26m",
    .flags = 0,
    .en_offset = 0x060,
    .en_shift = 0,
    .div_offset = 0x060,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "sen_pdm1_clk",
    .parent_name = "sen_clk26m",
    .flags = 0,
    .en_offset = 0x068,
    .en_shift = 0,
    .div_offset = 0x068,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "bus_lp_clk",
    .parent_name = "sen_clk26m",
    .flags = 0,
    .en_offset = 0x04c,
    .en_shift = 0,
    .div_offset = 0x04c,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "pll_adj_clk",
    .parent_name = "sen_clk26m",
    .flags = 0,
    .en_offset = 0x0a0,
    .en_shift = 0,
    .div_offset = 0x0a0,
    .div_shift = 4,
    .div_width = 5,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "sen_pclk",
    .parent_name = "sen_bus_mclk",
    .flags = 0,
    .en_offset = 0x048,
    .en_shift = 0,
    .div_offset = 0x048,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {},
};

static const struct song_gate_clk gate[] =
{
  {
    .name = "gnss_hclk",
    .parent_name = "gnss_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 0,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gnss_pp_clk",
    .parent_name = "gnss_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 1,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gnss_ae_clk",
    .parent_name = "gnss_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 2,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gnss_ae_fifo_clk",
    .parent_name = "gnss_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 3,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gnss_conf_mem_clk",
    .parent_name = "gnss_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 4,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gnss_te_clk",
    .parent_name = "gnss_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 5,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gnss_te_mem_clk",
    .parent_name = "gnss_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 6,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gnss_te_fifo_clk",
    .parent_name = "gnss_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 7,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gnss_viterbi_clk",
    .parent_name = "gnss_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 8,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gnss_pps_clk",
    .parent_name = "gnss_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 9,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gnss_i_clk32k",
    .parent_name = "sen_clk32k",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 10,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "iram_icm_hclk",
    .parent_name = "sen_bus_mclk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 11,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "dram_icm_hclk",
    .parent_name = "sen_bus_mclk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 12,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sensorbus_gpsclk",
    .parent_name = "gnss_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 13,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_m4_clk",
    .parent_name = "sen_bus_mclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 0,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_m4_wic_clk",
    .parent_name = "sen_bus_mclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 0,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_m4_stclk",
    .parent_name = "sen_clk26m",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 2,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_dmas_hclk",
    .parent_name = "sen_bus_mclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 3,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_wdt_pclk",
    .parent_name = "sen_pclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 4,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_wdt_tclk",
    .parent_name = "sen_wdt_tclk_en",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 5,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_gpio_pclk",
    .parent_name = "sen_pclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 6,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_gpio_clk32k",
    .parent_name = "sen_clk32k",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 7,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_pdm0_pclk",
    .parent_name = "sen_pclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 8,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_pdm1_pclk",
    .parent_name = "sen_pclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 9,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_i2s_pclk",
    .parent_name = "sen_pclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 10,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_i2c0_pclk",
    .parent_name = "sen_pclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 11,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_i2s1_pclk",
    .parent_name = "sen_pclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 12,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_ssi0_pclk",
    .parent_name = "sen_pclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 13,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sen_ssi1_pclk",
    .parent_name = "sen_pclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 14,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {},
};

static const char * const at_clk_parents[] =
{
  "sen_clk26m",
  "at_clk_src_pll",
};

static const struct song_mux_gate_clk mux_gate[] =
{
  {
    .name = "at_clk",
    .parent_name = at_clk_parents,
    .num_parents = ARRAY_SIZE(at_clk_parents),
    .flags = 0,
    .en_offset = 0x06c,
    .en_shift = 0,
    .mux_offset = 0x06c,
    .mux_shift = 4,
    .mux_width = 1,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_MUX_HIWORD_MASK << SONG_CLK_MUX_FLAG_SHIFT)),
  },
  {},
};

static const struct song_pll_clk pll[] =
{
  {
    .name = "sen_pll_out",
    .parent_name = "sen_clk26m",
    .flags = 0,
    .cfg_reg0_offset = 0x30,
    .cfg_reg1_offset = 0x34,
    .ctl_reg_offset = 0x38,
    .ctl_shift = 0,
  },
  {},
};

static const struct song_clk_table banks_sen_clk_tbl =
{
  .fixed_rate_clks   = fixed_rate,
  .fixed_factor_clks = fixed_factor,
  .sdiv_gr_clks      = sdiv_gr,
  .gr_sdiv_clks      = gr_sdiv,
  .gr_fdiv_clks      = gr_fdiv,
  .gr_clks           = gr,
  .sdiv_clks         = sdiv,
  .gate_clks         = gate,
  .mux_gate_clks     = mux_gate,
  .pll_clks          = pll,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xf8b14000, &banks_sen_clk_tbl);
}
#endif  /* CONFIG_ARCH_CHIP_BANKS_SENSOR */
