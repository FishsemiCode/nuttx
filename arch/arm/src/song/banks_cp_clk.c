/****************************************************************************
 * arch/arm/src/song/banks_cp_clk.c
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

#if defined(CONFIG_ARCH_CHIP_BANKS_CP) && defined(CONFIG_SONG_CLK)

#include <nuttx/clk/clk.h>
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This describes mux clk parent source */
static const char * const timer_clk_src[] =
{
  "ctl_pclk",
  "top/com_pre_clk",
  "rf_pll_div16",
  "clk26m",
  "clk26m",
  "clk32k",
};

static const char * const out_clk_src[] =
{
  "ctl_pclk",
  "top/com_pre_clk",
  "rf_pll_div16",
  "clk26m",
  "clk32k",
  "clk32k",
};

static const char * const x1643_4210_clk_src[] =
{
  "top/top_pll1",
  "top/top_pll2",
  "top/top_pll3",
  "top/top_pll1",
};

static const char * const a7_clk_src[] =
{
  "top/top_pll1_out_div2",
  "top/top_pll2",
  "top/top_pll3",
  "top/top_pll1_out_div2",
};

static const char * const l12acc_rfsys_mclk0_src[] =
{
  "top/top_pll2",
  "top/top_pll3",
};

static const struct song_fixed_rate_clk fixed_rate[] =
{
  {
    .name = "clk32k",
    .fixed_rate = 32768,
  },
  {
    .name = "clk26m",
    .fixed_rate = 26000000,
  },
  {},
};

static const struct song_pll_clk pll[] =
{
  {
    .name = "rf_pll",
    .parent_name = "clk26m",
    .cfg0_offset = 0x38,
    .cfg1_offset = 0x48,
    .ctl_offset  = 0x34,
  },
  {},
};

static const struct song_fixed_factor_clk fixed_factor[] =
{
  {
    .name = "rf_plldiv16",
    .parent_name = "rf_pll",
    .fixed_mult = 1,
    .fixed_div  = 16,
  },
  {},
};

static const struct song_timer_clk timer[] =
{
  {
    .name = "timer0",
    .parent_names = timer_clk_src,
    .num_parents = ARRAY_SIZE(timer_clk_src),
    .ctl_offset = 0x120,
  },
  {
    .name = "timer1",
    .parent_names = timer_clk_src,
    .num_parents = ARRAY_SIZE(timer_clk_src),
    .ctl_offset = 0x124,
  },
  {
    .name = "timer3",
    .parent_names = timer_clk_src,
    .num_parents = ARRAY_SIZE(timer_clk_src),
    .ctl_offset = 0x128,
  },
  {
    .name = "timer4",
    .parent_names = timer_clk_src,
    .num_parents = ARRAY_SIZE(timer_clk_src),
    .ctl_offset = 0x12c,
  },
  {},
};

static const struct song_out_clk out[] =
{
  {
    .name = "out0",
    .parent_names = out_clk_src,
    .num_parents = ARRAY_SIZE(out_clk_src),
    .mux_offset = 0x140,
    .ctl_offset = 0x144,
    .mux_shift = 0,
  },
  {},
};

static const struct song_mux_div_clk mux_div[] =
{
  {
    .name = "l12acc_mclk0",
    .parent_names = l12acc_rfsys_mclk0_src,
    .en_offset  = 0x160,
    .mux_offset = 0x160,
    .div_offset = 0x160,
    .en_shift  = 0,
    .mux_shift = 12,
    .mux_width = 1,
    .div_shift = 4,
    .div_width = 4,
    .num_parents = ARRAY_SIZE(l12acc_rfsys_mclk0_src),
  },
  {
    .name = "rfsys_mclk0",
    .parent_names = l12acc_rfsys_mclk0_src,
    .en_offset  = 0x168,
    .mux_offset = 0x168,
    .div_offset = 0x168,
    .en_shift  = 0,
    .mux_shift = 12,
    .mux_width = 1,
    .div_shift = 4,
    .div_width = 4,
    .num_parents = ARRAY_SIZE(l12acc_rfsys_mclk0_src),
  },
  {},
};

static const struct song_mux_clk mux[] =
{
  {
    .name = "a7_clk_src",
    .parent_names = a7_clk_src,
    .num_parents = ARRAY_SIZE(a7_clk_src),
    .en_offset = 0x064,
    .en_shift = 4,
    .mux_offset = 0x158,
    .mux_shift = 0,
    .mux_width = 2,
  },
};

static const struct song_mux_sdiv_gr_clk mux_sdiv_gr[] =
{
  {
    .name = "x1643_clk_src",
    .parent_names = x1643_4210_clk_src,
    .num_parents = ARRAY_SIZE(x1643_4210_clk_src),
    .div_offset = 0x150,
    .div_width  = 4,
    .mux_width  = 2,
  },
  {
    .name = "xc4210_clk_src",
    .parent_names = x1643_4210_clk_src,
    .num_parents = ARRAY_SIZE(x1643_4210_clk_src),
    .div_offset = 0x154,
    .div_width = 4,
    .mux_width = 2,
  },
};

static const struct song_gr_clk gr[] =
{
  {
    .name = "digrfv4_pclk",
    .parent_name = "rfsys_mclk0",
    .en_offset = 0x064,
    .en_shift = 5,
    .mult_offset = 0x94,
    .mult_shift = 0,
    .mult_width = 3,
  },
  {
    .name = "a7_clk",
    .parent_name = "a7_clk_src",
    .en_offset = 0x064,
    .en_shift = 4,
    .mult_offset = 0x44,
    .mult_shift = 0,
    .mult_width = 4,
    .gr_flags = CLK_GR_DIV_16,
  },
  {
    .name = "l12acc_mclk1",
    .parent_name = "l12acc_mclk0",
    .en_offset = 0x164,
    .en_shift = 0,
    .mult_offset = 0x164,
    .mult_shift = 4,
    .mult_width = 3,
  },
  {
    .name = "rfsys_mclk1",
    .parent_name = "rfsys_mclk0",
    .en_offset = 0x16c,
    .en_shift = 0,
    .mult_offset = 0x16c,
    .mult_shift = 4,
    .mult_width = 3,
  },
  {},
};

static const struct song_sdiv_sdiv_clk sdiv_sdiv[] =
{
  {
    .name = "td_sp",
    .parent_name = "rf_pll",
    .div_offset = 0xd0,
  },
  {
    .name = "td_cal",
    .parent_name = "rf_pll",
    .div_offset = 0xd4,
  },
  {
    .name = "wcdma_sp",
    .parent_name = "rf_pll",
    .div_offset = 0xd8,
  },
  {
    .name = "wcdma_cal",
    .parent_name = "rf_pll",
    .div_offset = 0xdc,
  },
  {
    .name = "rfif_match",
    .parent_name = "rf_pll",
    .div_offset = 0xe0,
  },
  {
    .name = "rfif_digrfv4_rx",
    .parent_name = "rf_pll",
    .div_offset = 0xe4,
  },
  {
    .name = "rfif_m7_calib_mclk",
    .parent_name = "rf_pll",
    .div_offset = 0x110,
  },
  {},
};

static const struct song_div_clk div[] =
{
  {
    .name = "pll_adj",
    .parent_name = "clk26m",
    .en_offset = 0x3c,
    .en_shift = 0,
    .div_offset = 0x03c,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "stat_monitor",
    .parent_name = "top/com_pre_clk",
    .en_offset = 0x054,
    .en_shift =  0,
    .div_offset = 0x054,
    .div_shift = 4,
    .div_width = 4,
  },
  /* only for ctl pclk (work) div */
  {
    .name = "ctl_pclk",
    .parent_name = "top/top_bus_mclk1",
    .en_offset = 0x058,
    .en_shift =  0,
    .div_offset = 0x58,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "gsmsp_clk",
    .parent_name = "clk26m",
    .en_offset = 0x0a4,
    .en_shift = 0,
    .div_offset = 0x0a4,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "gsm_calclk",
    .parent_name = "clk26m",
    .en_offset = 0x0a8,
    .en_shift = 0,
    .div_offset = 0x0a8,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "sim_hi_clk",
    .parent_name = "top/com_pre_clk",
    .en_offset = 0x0ac,
    .en_shift = 2,
    .div_offset = 0x0ac,
    .div_shift = 8,
    .div_width = 3,
  },
  {
    .name = "sim0_mclk",
    .parent_name = "clk26m",
    .en_offset = 0x0ac,
    .en_shift = 0,
    .div_offset = 0x0ac,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "sim1_mclk",
    .parent_name = "clk26m",
    .en_offset = 0x0ac,
    .en_shift = 1,
    .div_offset = 0x0ac,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "a5_mclk",
    .parent_name = "top/com_pre_clk",
    .en_offset = 0x0b8,
    .en_shift = 0,
    .div_offset = 0xb8,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "gea_mclk",
    .parent_name = "top/com_pre_clk",
    .en_offset = 0x0bc,
    .en_shift = 0,
    .div_offset = 0xbc,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "rfif_spi_clk",
    .parent_name = "top/com_pre_clk",
    .en_offset = 0x0c0,
    .en_shift = 0,
    .div_offset = 0xc0,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "rfif_rffe_clk",
    .parent_name = "top/com_pre_clk",
    .en_offset = 0x0c4,
    .en_shift = 0,
    .div_offset = 0xc4,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "rfif_rffe_1_mclk",
    .parent_name = "top/com_pre_clk",
    .en_offset = 0x0c8,
    .en_shift = 0,
    .div_offset = 0xc8,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "ltetdd_sp",
    .parent_name = "rf_pll",
    .en_offset = 0x0f0,
    .en_shift = 0,
    .div_offset = 0xf0,
    .div_shift = 4,
    .div_width = 6,
  },
  {
    .name = "ltetdd_cal",
    .parent_name = "rf_pll",
    .en_offset = 0x0f4,
    .en_shift = 0,
    .div_offset = 0xf4,
    .div_shift = 4,
    .div_width = 6,
  },
  {
    .name = "ltefdd_sp",
    .parent_name = "rf_pll",
    .en_offset = 0x0f8,
    .en_shift = 0,
    .div_offset = 0xf8,
    .div_shift = 4,
    .div_width = 6,
  },
  {
    .name = "ltefdd_cal",
    .parent_name = "rf_pll",
    .en_offset = 0x0fc,
    .en_shift = 0,
    .div_offset = 0xfc,
    .div_shift = 4,
    .div_width = 6,
  },
  {
    .name = "rfif_tx_fir",
    .parent_name = "rf_pll",
    .en_offset = 0x100,
    .en_shift = 0,
    .div_offset = 0x100,
    .div_shift = 4,
    .div_width = 6,
  },
  {
    .name = "rfif_digrfv4_tx",
    .parent_name = "rfif_tx_fir",
    .en_offset = 0x114,
    .en_shift = 0,
    .div_offset = 0x114,
    .div_shift = 4,
    .div_width = 6,
  },
  {
    .name = "sw_a7_clk",
    .parent_name = "a7_clk",
    .div_offset = 0x44,
    .div_shift = 12,
    .div_width = 1,
  },
  {
    .name = "a7_atbbrg_sclk",
    .parent_name = "a7_clk",
    .en_offset = 0x70,
    .en_shift = 13,
    .div_offset = 0x44,
    .div_shift = 8,
    .div_width = 3,
  },
  {
    .name = "a7_atbbrg_mclk",
    .parent_name = "a7_clk",
    .en_offset = 0x70,
    .en_shift = 12,
    .div_offset = 0x44,
    .div_shift = 4,
    .div_width = 3,
  },
  {},
};

static const struct song_gate_clk gate[] =
{
  {
    .name = "ictl_xc4210_clk",
    .parent_name = "top/top_bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 1,
  },
  {
    .name = "ictl_x1643_clk",
    .parent_name = "top/top_bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 2,
  },
  {
    .name = "rfif_clk",
    //.parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 3,
  },
  {
    .name = "l12acc_gea_hclk",
    //.parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 7,
  },
  {
    .name = "l12acc_a5_hclk",
    //.parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 8,
  },
  {
    .name = "l12acc_cihperhwa_clk",
    //.parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 9,
  },
  {
    .name = "mailbox_clk",
    .parent_name = "top/top_bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 10,
  },
  {
    .name = "l12acc_iphwa_clk",
    //.parent_name = "bus_mclk0",
    .en_offset = 0x060,
    .en_shift = 11,
  },
  {
    .name = "noc2l12acc_niu_clk",
    //.parent_name = "bus_mclk0",
    .en_offset = 0x060,
    .en_shift = 12,
  },
  {
    .name = "l12acc_dmag_clk",
    //.parent_name = "bus_mclk0",
    .en_offset = 0x060,
    .en_shift = 13,
  },
  {
    .name = "rf_dmad_clk",
    //.parent_name = "bus_mclk0",
    .en_offset = 0x060,
    .en_shift = 14,
  },
  {
    .name = "rf_pll_div16",
    .parent_name = "rf_plldiv16",
    .en_offset = 0x060,
    .en_shift = 15,
  },
  {
    .name = "noc2rfsys_niu",
    //.parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 1,
  },
  {
    .name = "rfsys_dcpp",
    //.parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 2,
  },
  {
    .name = "rfsys_dcpp_main",
    //.parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 3,
  },
  {
    .name = "rfsys_digrf4_pclk",
    //.parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 5,
  },
  {
    .name = "shram0_clk",
    .parent_name = "top/top_bus_mclk0",
    .en_offset = 0x068,
    .en_shift = 0,
  },
  {
    .name = "shram2_clk",
    .parent_name = "top/top_bus_mclk0",
    .en_offset = 0x068,
    .en_shift = 1,
  },
  {
    .name = "digrfv4_refclk",
    //.parent_name = "clk26m0",
    .en_offset = 0x068,
    .en_shift = 2,
  },
  {
    .name = "shram1_clk",
    .parent_name = "top/top_bus_mclk0",
    .en_offset = 0x068,
    .en_shift = 6,
  },
  {
    .name = "a7_pclkdbg",
    .parent_name = "a7_clk",
    .en_offset = 0x070,
    .en_shift = 12,
  },
  {
    .name = "a7_atclk",
    .parent_name = "a7_clk",
    .en_offset = 0x070,
    .en_shift = 13,
  },
  {
    .name = "rtc_pclk",
    .parent_name = "ctl_pclk",
    .en_offset = 0x074,
    .en_shift = 0,
  },
  {
    .name = "timer_pclk",
    .parent_name = "ctl_pclk",
    .en_offset = 0x074,
    .en_shift = 1,
  },
  {
    .name = "wdt0_pclk",
    .parent_name = "ctl_pclk",
    .en_offset = 0x074,
    .en_shift = 2,
  },
  {
    .name = "sim1_pclk",
    .parent_name = "ctl_pclk",
    .en_offset = 0x074,
    .en_shift = 3,
  },
  {
    .name = "sim0_pclk",
    .parent_name = "ctl_pclk",
    .en_offset = 0x074,
    .en_shift = 4,
  },
  {
    .name = "a7_dbgclk",
    .parent_name = "a7_clk",
    .en_offset = 0x074,
    .en_shift = 5,
  },
  {
    .name = "wdt0_tclk",
    .parent_name = "top/wdt_clk",
    .en_offset = 0x074,
    .en_shift = 6,
  },
  {
    .name = "wdt1_pclk",
    .parent_name = "ctl_pclk",
    .en_offset = 0x074,
    .en_shift = 7,
  },
  {
    .name = "wdt1_tclk",
    .parent_name = "top/wdt_clk",
    .en_offset = 0x074,
    .en_shift = 8,
  },
  {
    .name = "wdt2_pclk",
    .parent_name = "ctl_pclk",
    .en_offset = 0x074,
    .en_shift = 9,
  },
  {
    .name = "wdt2_tclk",
    .parent_name = "top/wdt_clk",
    .en_offset = 0x074,
    .en_shift = 10,
  },
  {
    .name = "cp_smp_pclk",
    .parent_name = "ctl_pclk",
    .en_offset = 0x074,
    .en_shift = 13,
  },
  {
    .name = "cp_smp_dmag_clk",
    .parent_name = "l12acc_mclk0",
    .en_offset = 0x074,
    .en_shift = 13,
  },
  {
    .name = "cp_tm32k",
    .parent_name = "clk32k",
    .en_offset = 0x0a0,
    .en_shift = 2,
  },
  {
    .name = "rtc_clk32k",
    .parent_name = "clk32k",
    .en_offset = 0x0a0,
    .en_shift = 1,
  },
  {
    .name = "rfif_clk32k",
    .parent_name = "clk32k",
    .en_offset = 0x0a0,
    .en_shift = 0,
  },
  {},
};

static const struct song_lp_reg_clk lp_reg[] =
{
  /* LP_EN0 */
  {
    .offset = 0x360,
    .value = 0xffffffff,
  },
  /* LP_EN1 */
  {
    .offset = 0x364,
    .value = 0xf,
  },
  {}
};

static const struct song_clk_table banks_cp_clk_tbl =
{
  .fixed_rate_clks   = fixed_rate,
  .fixed_factor_clks = fixed_factor,
  .gr_clks           = gr,
  .mux_clks          = mux,
  .mux_div_clks      = mux_div,
  .mux_sdiv_gr_clks  = mux_sdiv_gr,
  .div_clks          = div,
  .sdiv_sdiv_clks    = sdiv_sdiv,
  .gate_clks         = gate,
  .pll_clks          = pll,
  .timer_clks        = timer,
  .out_clks          = out,
  .lp_reg            = lp_reg,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0x87040000, &banks_cp_clk_tbl);
}

void up_clk_finalinitialize(void)
{
}
#endif /* (CONFIG_ARCH_CHIP_BANKS_CP) && (CONFIG_SONG_CLK) */
