/****************************************************************************
 * arch/arm/src/song/u1_cp_clk.c
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

#if defined(CONFIG_ARCH_CHIP_U1_CP) && defined(CONFIG_SONG_CLK)

#include <nuttx/clk/clk.h>
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This describes mux clk parent source */
static const char * const rfif_spi_src[] =
{
  "rfphy_clk38p4m",
  "rfphy_lte_clk",
};

static const char * const at_calib_src[] =
{
  "sp/pll0_out",
  "rfphy_clk38p4m",
  "gnss_i_rf0_clk",
};

static const char * const gnss_i_rf0_src[] =
{
  "chipio_gnss_rf0_clk",
  "rfphy_gnss_rf0_clk",
};

static const char * const timer_clk_src[] =
{
  "gnss_i_rf0_clk",
  "sp/top_bus_mclk0",
  "sp/pll1_mclk",
  "sp/pll0_out",
  "sp/pll0_out",
  "clk32k",
};

/* This describes unicorn cp clk input */
static const struct song_fixed_rate_clk fixed_rate[] =
{
  /* input from inchip RFPHY */
  {
    .name = "rfphy_clk38p4m",
    .fixed_rate = 38400000,
  },
  {
    .name = "rfphy_lte_clk",
    .fixed_rate = 30720000,
  },
  {
    .name = "rfphy_agc_adcclk",
    .fixed_rate = 15360000,
  },
  {
    .name = "rfphy_clk_dfe_lpf",
    .fixed_rate = 15360000,
  },
  {
    .name = "rfphy_gnss_rf0_clk",
    .fixed_rate = 26000000,
  },

  /* input from outchip io pin */
  {
    .name = "chipio_gnss_rf0_clk",
    .fixed_rate = 26000000,
  },
  /* input from outside osc in */
  {
    .name = "clk32k",
    .fixed_rate = 32768,
  },
  {},
};

static const struct song_fixed_factor_clk fixed_factor[] =
{
  {
    .name = "rfphy_lte_clk_div2",
    .parent_name = "rfphy_lte_clk",
    .fixed_mult = 1,
    .fixed_div = 2,
  },
  {},
};

static const struct song_timer_clk timer[] =
{
  {
    .name = "timer0",
    .parent_names = timer_clk_src,
    .num_parents = ARRAY_SIZE(timer_clk_src),
    .ctl_offset = 0x2b0,
  },
  {
    .name = "timer1",
    .parent_names = timer_clk_src,
    .num_parents = ARRAY_SIZE(timer_clk_src),
    .ctl_offset = 0x2b4,
  },
  {},
};

static const struct song_gr_clk gr[] =
{
  {
    .name = "cp_m4_cti_clk",
    .parent_name = "cp_m4_clk",
    .en_offset = 0xc0,
    .en_shift = 0,
    .mult_offset = 0xc0,
    .mult_shift = 4,
    .mult_width = 3,
  },
  {},
};

static const struct song_div_clk div[] =
{
  {
    .name = "cp_bus_mclk0",
    .parent_name = "sp/pll1_out",
    .en_offset = 0x058,
    .en_shift = 0,
    .div_offset = 0x058,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "cp_bus_mclk1",
    .parent_name = "sp/pll1_out",
    .en_offset = 0x05c,
    .en_shift = 0,
    .div_offset = 0x05c,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "cp_pclk",
    .parent_name = "cp_bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 0,
    .div_offset = 0x060,
    .div_shift = 4,
    .div_width = 3,
    .clk_flags = CLK_IS_CRITICAL,
  },
  {
    .name = "sim_clk",
    .parent_name = "sp/pll0_out",
    .en_offset = 0x064,
    .en_shift = 0,
    .div_offset = 0x064,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "sim_hi_clk",
    .parent_name = "sp/pll0_out",
    .en_offset = 0x068,
    .en_shift = 0,
    .div_offset = 0x068,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "gnss_clk",
    .parent_name = "sp/pll1_out",
    .en_offset = 0x078,
    .en_shift = 0,
    .div_offset = 0x078,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "rfif_match_rx_clk",
    .parent_name = "rfphy_lte_clk_div2",
    .en_offset = 0x2e0,
    .en_shift = 0,
    .div_offset = 0x2e0,
    .div_shift = 4,
    .div_width = 3,
    .div_flags = CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_MAX_HALF,
  },
  {
    .name = "rfif_match_tx_clk",
    .parent_name = "rfphy_lte_clk_div2",
    .en_offset = 0x2e4,
    .en_shift = 0,
    .div_offset = 0x2e4,
    .div_shift = 4,
    .div_width = 3,
    .div_flags = CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_MAX_HALF,
  },
  {
    .name = "rf_tp_tm2_calib_mclk",
    .parent_name = "rfphy_clk38p4m",
    .en_offset = 0x2e8,
    .en_shift = 0,
    .div_offset = 0x2e8,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rf_tp_tdltesp_mclk",
    .parent_name = "rfphy_clk38p4m",
    .en_offset = 0x2ec,
    .en_shift = 0,
    .div_offset = 0x2ec,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rfif_rffe_mclk",
    .parent_name = "sp/pll1_out",
    .en_offset = 0x2f0,
    .en_shift = 0,
    .div_offset = 0x2f0,
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
    .name = "rfphy_lte_clk_gated0",
    .parent_name = "rfphy_lte_clk",
    .en_offset = 0x090,
    .en_shift = 9,
  },
  {
    .name = "rfphv_clk38p4m_gated",
    .parent_name = "rfphy_clk38p4m",
    .en_offset = 0x090,
    .en_shift = 10,
  },
  {
    .name = "rf_tp_clk32k",
    .parent_name = "clk32k",
    .en_offset = 0x090,
    .en_shift = 11,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "rfphy_pclk",
    .parent_name = "sp/top_pclk2",
    .en_offset = 0x090,
    .en_shift = 12,
  },
  {
    .name = "cpwdt_pclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x090,
    .en_shift = 13,
  },
#endif
  {
    .name = "cpwdt_tclk",
    .parent_name = "clk32k",
    .en_offset = 0x090,
    .en_shift = 14,
  },
  {
    .name = "cp_shram_icm_hclk",
    .parent_name = "cp_m4_clk",
    .en_offset = 0x090,
    .en_shift = 15,
    .clk_flags = CLK_IS_CRITICAL,
  },
  {
    .name = "cp_m4_clk",
    .parent_name = "cp_bus_mclk0",
    .en_offset = 0x094,
    .en_shift = 0,
    .clk_flags = CLK_IS_CRITICAL,
  },
  {
    .name = "cp_m4_wic_clk",
    .parent_name = "cp_m4_clk",
    .en_offset = 0x094,
    .en_shift = 1,
    .clk_flags = CLK_IS_CRITICAL,
  },
  {
    .name = "cp_m4_stclk",
    .parent_name = "clk32k",
    .en_offset = 0x094,
    .en_shift = 2,
    .clk_flags = CLK_IS_CRITICAL,
  },
  {
    .name = "rfif_bus_clk",
    .parent_name = "cp_bus_mclk1",
    .en_offset = 0x094,
    .en_shift = 3,
    .clk_flags = CLK_IS_CRITICAL,
  },
  {
    .name = "viterbi_clk",
    .parent_name = "cp_bus_mclk1",
    .en_offset = 0x094,
    .en_shift = 4,
  },
  {
    .name = "cp_cipherhwa_clk",
    .parent_name = "cp_bus_mclk1",
    .en_offset = 0x094,
    .en_shift = 5,
  },
  {
    .name = "nb_cor_clk",
    .parent_name = "cp_bus_mclk1",
    .en_offset = 0x094,
    .en_shift = 6,
  },
  {
    .name = "np_sp_clk",
    .parent_name = "cp_bus_mclk1",
    .en_offset = 0x094,
    .en_shift = 7,
  },
  {
    .name = "cp_dmag_clk",
    .parent_name = "cp_bus_mclk1",
    .en_offset = 0x094,
    .en_shift = 8,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "sim_pclk",
    .parent_name = "cp_pclk",
    .en_offset = 0x094,
    .en_shift = 9,
  },
  {
    .name = "cp_m4_icache_clk",
    .parent_name = "cp_m4_clk",
    .en_offset = 0x094,
    .en_shift = 11,
    .clk_flags = CLK_IS_CRITICAL,
  },
#endif
  {
    .name = "cp_rfif_adc_clk",
    .parent_name = "rfphy_agc_adcclk",
    .en_offset = 0x094,
    .en_shift = 10,
  },
  {
    .name = "rfphy_lte_clk_gated1",
    .parent_name = "rfphy_lte_clk",
    .en_offset = 0x094,
    .en_shift = 12,
  },
  {
    .name = "nbsp_adc_clk",
    .parent_name = "rfphy_clk_dfe_lpf",
    .en_offset = 0x094,
    .en_shift = 13,
  },
  {
    .name = "dfe_clk_1920k",
    .parent_name = "rfphy_lte_clk_gated0",
    .en_offset = 0x2d0,
    .en_shift = 0,
  },
  {
    .name = "dfe_clk_3840k",
    .parent_name = "rfphy_lte_clk_gated0",
    .en_offset = 0x2d0,
    .en_shift = 1,
  },
  {
    .name = "dfe_clk_7680k",
    .parent_name = "rfphy_lte_clk_gated0",
    .en_offset = 0x2d0,
    .en_shift = 2,
  },
  {
    .name = "dfe_clk_30720k",
    .parent_name = "rfphy_lte_clk_gated0",
    .en_offset = 0x2d0,
    .en_shift = 3,
  },
  {
    .name = "cp_m4_dapclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x0b8,
    .en_shift = 12,
  },
  {},
};

static const struct song_mux_clk mux[] =
{
  {
    .name = "rfif_spi_mclk",
    .parent_names = rfif_spi_src,
    .num_parents = ARRAY_SIZE(rfif_spi_src),
    .en_offset = 0x2f4,
    .en_shift = 0,
    .mux_offset = 0x2f4,
    .mux_shift = 8,
    .mux_width = 1,
    .mux_flags = CLK_MUX_HIWORD_MASK,
  },
  {
    .name = "at_calib_clk",
    .parent_names = at_calib_src,
    .num_parents = ARRAY_SIZE(at_calib_src),
    .en_offset = 0x2fc,
    .en_shift = 0,
    .mux_offset = 0x2fc,
    .mux_shift = 4,
    .mux_width = 2,
    .mux_flags = CLK_MUX_HIWORD_MASK,
    .clk_flags = CLK_IS_CRITICAL,
  },
  {
    .name = "gnss_i_rf0_clk",
    .parent_names = gnss_i_rf0_src,
    .num_parents = ARRAY_SIZE(gnss_i_rf0_src),
    .en_offset = 0x090,
    .en_shift = 8,
    .mux_offset = 0x248,
    .mux_shift = 0,
    .mux_width = 1,
    .mux_flags = CLK_MUX_HIWORD_MASK,
  },
  {},
};

static const struct clk_rate def_rates[] =
{
  {
    .name = "cp_bus_mclk0",
    .rate = 204800000,
  },
  {
    .name = "cp_bus_mclk1",
    .rate = 102400000,
  },
  {
    .name = "cp_pclk",
    .rate = 51200000,
  },
  {}
};

static const struct song_clk_table u1_cp_clk_tbl =
{
  .fixed_rate_clks   = fixed_rate,
  .fixed_factor_clks = fixed_factor,
  .gr_clks           = gr,
  .div_clks          = div,
  .gate_clks         = gate,
  .mux_clks          = mux,
  .timer_clks        = timer,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xb0040000, &u1_cp_clk_tbl);
}

void up_clk_finalinitialize(void)
{
  clk_set_rates(def_rates);
  clk_disable_unused();
}
#endif /* (CONFIG_ARCH_CHIP_U1_CP) && (CONFIG_SONG_CLK) */
