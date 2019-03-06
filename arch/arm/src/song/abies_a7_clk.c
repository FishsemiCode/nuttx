/****************************************************************************
 * arch/arm/src/song/abies_a7_clk.c
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

#if defined(CONFIG_ARCH_CHIP_ABIES_A7) && defined(CONFIG_SONG_CLK)

#include <nuttx/clk/clk.h>
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This describes mux clk parent source */
static const char * const bus_mclk0_src[] =
{
  "top/pll1_div2",
  "top/pll3",
};

static const char * const rfif_cal_src[] =
{
  "cdma_cal_clk",
  "gsm1_cal_clk",
};

static const char * const rfif_sp_src[] =
{
  "cdmasp_clk",
  "gsm1sp_clk",
};

static const char * const timer_clk_src[] =
{
  "a7_clk_div13",
  "top/pll3_mclk",
  "pll_mclk",
  "clk26m",
  "clk26m",
  "clk32k",
};

static const char * const out_clk_src[] =
{
  "a7_clk_div13",
  "top/pll3_mclk",
  "pll_mclk",
  "clk26m",
  "clk32k",
  "clk32k",
};

static const char * const a7_clk_src[] =
{
  "pll0_div2",
  "top/pll1_div2",
  "top/pll3",
  "pll0_div2",
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
  {
    .name = "clk26m0",
    .fixed_rate = 26000000,
  },
  {
    .name = "clk26m1",
    .fixed_rate = 26000000,
  },
  {},
};

static const struct song_pll_clk pll[] =
{
  {
    .name = "cp_pll",
    .parent_name = "cp_pll_in",
    .cfg0_offset = 0x38,
    .cfg1_offset = 0x48,
    .pll_flags   = CLK_PLL_READ_ONLY,
  },
  {},
};

static const struct song_fixed_factor_clk fixed_factor[] =
{
  {
    .name = "rfif_tx_fir_div10",
    .parent_name = "rfif_tx_fir",
    .fixed_mult = 1,
    .fixed_div = 10,
  },
  {
    .name = "a7_clkdiv13",
    .parent_name = "a7_clk",
    .fixed_mult = 1,
    .fixed_div = 13,
  },
  {
    .name = "pll0_div2",
    .parent_name = "top/pll0",
    .fixed_mult = 1,
    .fixed_div  = 2,
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
    .name = "bus_mclk0",
    .parent_names = bus_mclk0_src,
    .mux_offset = 0x50,
    .div_offset = 0x50,
    .mux_shift = 4,
    .mux_width = 1,
    .div_shift = 0,
    .div_width = 4,
    .num_parents = ARRAY_SIZE(bus_mclk0_src),
    .div_flags   = 1 << CLK_DIVIDER_MINDIV_OFF,
  },
  {},
};

static const struct song_mux_clk mux[] =
{
  {
    .name = "rfif_cal_clk",
    .parent_names = rfif_cal_src,
    .num_parents = ARRAY_SIZE(rfif_cal_src),
    .mux_offset = 0xb4,
    .mux_shift = 9,
    .mux_width = 1,
    .mux_flags = CLK_MUX_HIWORD_MASK,
  },
  {
    .name = "rfif_sp_clk",
    .parent_names = rfif_sp_src,
    .num_parents = ARRAY_SIZE(rfif_sp_src),
    .mux_offset = 0xb4,
    .mux_shift = 8,
    .mux_width = 1,
    .mux_flags = CLK_MUX_HIWORD_MASK,
  },
  {
    .name = "a7_clk_src",
    .parent_names = a7_clk_src,
    .num_parents = ARRAY_SIZE(a7_clk_src),
    .mux_offset = 0x158,
    .mux_shift = 0,
    .mux_width = 2,
  },
};

static const struct song_gr_clk gr[] =
{
  {
    .name = "bus_mclk1",
    .parent_name = "bus_mclk0",
    .en_offset = 0x54,
    .en_shift = 0,
    .mult_offset = 0x54,
    .mult_shift = 4,
    .mult_width = 3,
  },
  {
    .name = "ctl_apb_idle",
    .parent_name = "bus_mclk0",
    .mult_offset = 0x58,
    .mult_shift = 4,
    .mult_width = 3,
  },
  {
    .name = "ctl_apb_work",
    .parent_name = "bus_mclk0",
    .mult_offset = 0x58,
    .mult_shift = 0,
    .mult_width = 3,
  },
  {
    .name = "digrfv4_pclk",
    .parent_name = "bus_mclk0",
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

  {},
};

static const struct song_sdiv_sdiv_clk sdiv_sdiv[] =
{
  {
    .name = "td_sp",
    .parent_name = "cp_pll",
    .div_offset = 0xd0,
  },
  {
    .name = "td_cal",
    .parent_name = "cp_pll",
    .div_offset = 0xd4,
  },
  {
    .name = "wcdma_sp",
    .parent_name = "cp_pll",
    .div_offset = 0xd8,
  },
  {
    .name = "wcdma_cal",
    .parent_name = "cp_pll",
    .div_offset = 0xdc,
  },
  {
    .name = "rfif_match",
    .parent_name = "cp_pll",
    .div_offset = 0xe0,
  },
  {
    .name = "rfif_digrfv4_rx",
    .parent_name = "cp_pll",
    .div_offset = 0xe4,
  },
  /* TODO: For cdma_sp/cmda_cal must modify sdiv_sdiv bit width */
  {
    .name = "cdmasp_clk",
    .parent_name = "cp_pll",
    .div_offset = 0xe8,
  },
  {
    .name = "cdma_cal_clk",
    .parent_name = "cp_pll",
    .div_offset = 0xe8,
  },
  {},
};

static const struct song_gr_fdiv_clk gr_fdiv[] =
{
  {
    .name = "pcm_mclk",
    .parent_name = "top/pll3_mclk",
    .gr_offset = 0x150,
    .div_offset = 0x154,
  },
  {},
};

static const struct song_div_clk div[] =
{
  {
    .name = "cp_pll_in",
    .parent_name = "top/pll3",
    .en_offset = 0x64,
    .en_shift = 3,
    .div_offset = 0x088,
    .div_shift = 0,
    .div_width = 7,
  },
  {
    .name = "pll3_mclk",
    .parent_name = "top/pll3",
    .en_offset = 0x08c,
    .en_shift = 0,
    .div_offset = 0x08c,
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
    .parent_name = "pll3_mclk",
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
    .name = "gsm1_calclk",
    .parent_name = "clk26m1",
    .en_offset = 0x74,
    .en_shift = 8,
    .div_offset = 0xb0,
    .div_shift = 8,
    .div_width = 5,
  },
  {
    .name = "gsm1sp_clk",
    .parent_name = "clk26m1",
    .en_offset = 0x74,
    .en_shift = 10,
    .div_offset = 0xb0,
    .div_shift = 0,
    .div_width = 5,
  },
  {
    .name = "gsm1_spi_clk",
    .parent_name = "pll3_mclk",
    .en_offset = 0x074,
    .en_shift = 10,
    .div_offset = 0xb4,
    .div_shift = 0,
    .div_width = 6,
  },
  {
    .name = "a5_mclk",
    .parent_name = "pll3_mclk",
    .en_offset = 0x0b8,
    .en_shift = 0,
    .div_offset = 0xb8,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "gea_mclk",
    .parent_name = "pll3_mclk",
    .en_offset = 0x0bc,
    .en_shift = 0,
    .div_offset = 0xbc,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "rfif_spi_clk",
    .parent_name = "pll3_mclk",
    .en_offset = 0x0c0,
    .en_shift = 0,
    .div_offset = 0xc0,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "rfif_rffe_clk",
    .parent_name = "pll3_mclk",
    .en_offset = 0x0c4,
    .en_shift = 0,
    .div_offset = 0xc4,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "ltetdd_sp",
    .parent_name = "cp_pll",
    .en_offset = 0x0f0,
    .en_shift = 0,
    .div_offset = 0xf0,
    .div_shift = 4,
    .div_width = 6,
  },
  {
    .name = "ltetdd_cal",
    .parent_name = "cp_pll",
    .en_offset = 0x0f4,
    .en_shift = 0,
    .div_offset = 0xf4,
    .div_shift = 4,
    .div_width = 6,
  },
  {
    .name = "ltefdd_sp",
    .parent_name = "cp_pll",
    .en_offset = 0x0f8,
    .en_shift = 0,
    .div_offset = 0xf8,
    .div_shift = 4,
    .div_width = 6,
  },
  {
    .name = "ltefdd_cal",
    .parent_name = "cp_pll",
    .en_offset = 0x0fc,
    .en_shift = 0,
    .div_offset = 0xfc,
    .div_shift = 4,
    .div_width = 6,
  },
  {
    .name = "rfif_tx_fir",
    .parent_name = "cp_pll",
    .en_offset = 0x100,
    .en_shift = 0,
    .div_offset = 0x100,
    .div_shift = 4,
    .div_width = 6,
  },
  {
    .name = "pll_mclk",
    .parent_name = "cp_pll",
    .en_offset = 0x104,
    .en_shift = 0,
    .div_offset = 0x104,
    .div_shift = 4,
    .div_width = 4,
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
    .parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 1,
  },
  {
    .name = "ictl_x1643_clk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 2,
  },
  {
    .name = "rfif_hclk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 3,
  },
  {
    .name = "tft_clk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 4,
  },
  {
    .name = "tdul_clk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 5,
  },
  {
    .name = "lteul_clk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 6,
  },
  {
    .name = "gea_hclk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 7,
  },
  {
    .name = "a5_hclk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 8,
  },
  {
    .name = "cihperhwa_clk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x060,
    .en_shift = 9,
  },
  {
    .name = "cp_mailbox_clk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x060,
    .en_shift = 10,
  },
  {
    .name = "iphwa_clk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x060,
    .en_shift = 11,
  },
  {
    .name = "xc_dma_clk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x060,
    .en_shift = 12,
  },
  {
    .name = "cp_dmag_clk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x060,
    .en_shift = 13,
  },
  {
    .name = "rf_dmad_clk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x060,
    .en_shift = 14,
  },
  {
    .name = "cp_dmad_clk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x060,
    .en_shift = 15,
  },
  {
    .name = "xc4210_mld_clk",
    .parent_name = "xc4210_clk",
    .en_offset = 0x064,
    .en_shift = 0,
  },
  {
    .name = "xc4210_clk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 1,
  },
  {
    .name = "x1643_clk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 2,
  },
  {
    .name = "a7sw_ddr_clk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 6,
  },
  {
    .name = "cp_sw1_mainclk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 7,
  },
  {
    .name = "ctl_bus2_dmag",
    .parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 9,
  },
  {
    .name = "cp_sw1_gea",
    .parent_name = "bus_mclk1",
    .en_offset = 0x064,
    .en_shift = 10,
  },
  {
    .name = "a7sw_a7_clk",
    .parent_name = "a7_clk",
    .en_offset = 0x064,
    .en_shift = 11,
  },
  {
    .name = "a7sw_mainclk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 12,
  },
  {
    .name = "msw_4210_dma",
    .parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 13,
  },
  {
    .name = "sw1_a5_clk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x064,
    .en_shift = 14,
  },
  {
    .name = "sw1_cp_dmad",
    .parent_name = "bus_mclk0",
    .en_offset = 0x064,
    .en_shift = 15,
  },
  {
    .name = "shram0_clk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x068,
    .en_shift = 0,
  },
  {
    .name = "msw_x1643_i",
    .parent_name = "bus_mclk0",
    .en_offset = 0x068,
    .en_shift = 1,
  },
  {
    .name = "digrfv4_refclk",
    .parent_name = "clk26m0",
    .en_offset = 0x068,
    .en_shift = 2,
  },
  {
    .name = "a7sw_cpmsw",
    .parent_name = "bus_mclk0",
    .en_offset = 0x068,
    .en_shift = 3,
  },
  {
    .name = "ctl_bus2_tdul",
    .parent_name = "bus_mclk1",
    .en_offset = 0x068,
    .en_shift = 4,
  },
  {
    .name = "ctl_bus0_ictl1643",
    .parent_name = "bus_mclk1",
    .en_offset = 0x068,
    .en_shift = 5,
  },
  {
    .name = "shram1_clk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x068,
    .en_shift = 6,
  },
  {
    .name = "sw1_iphwa",
    .parent_name = "bus_mclk0",
    .en_offset = 0x068,
    .en_shift = 7,
  },
  {
    .name = "ctl_bus0_ictlsdr",
    .parent_name = "bus_mclk1",
    .en_offset = 0x068,
    .en_shift = 8,
  },
  {
    .name = "msw_xc4210_d",
    .parent_name = "bus_mclk0",
    .en_offset = 0x068,
    .en_shift = 9,
  },
  {
    .name = "ctrl_bus1_lteul",
    .parent_name = "bus_mclk1",
    .en_offset = 0x068,
    .en_shift = 10,
  },
  {
    .name = "ctrl_bus1_tft",
    .parent_name = "bus_mclk1",
    .en_offset = 0x068,
    .en_shift = 11,
  },
  {
    .name = "ctl_bus1_rf_dmad",
    .parent_name = "bus_mclk1",
    .en_offset = 0x068,
    .en_shift = 12,
  },
  {
    .name = "msw_rf_dmad",
    .parent_name = "bus_mclk0",
    .en_offset = 0x068,
    .en_shift = 13,
  },
  {
    .name = "ctl_bus0_ctlbus1",
    .parent_name = "bus_mclk0",
    .en_offset = 0x068,
    .en_shift = 14,
  },
  {
    .name = "ctl_bus2_dmad",
    .parent_name = "bus_mclk0",
    .en_offset = 0x068,
    .en_shift = 15,
  },
  {
    .name = "ctl_bus1_rfif",
    .parent_name = "bus_mclk1",
    .en_offset = 0x06c,
    .en_shift = 0,
  },
  {
    .name = "ctl_bus0_ctlbus2",
    .parent_name = "bus_mclk0",
    .en_offset = 0x06c,
    .en_shift = 1,
  },
  {
    .name = "msw_ddr_clk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x06c,
    .en_shift = 3,
  },
  {
    .name = "ctl_bus2_gea",
    .parent_name = "bus_mclk1",
    .en_offset = 0x06c,
    .en_shift = 4,
  },
  {
    .name = "ctl_bus2_a5",
    .parent_name = "bus_mclk1",
    .en_offset = 0x06c,
    .en_shift = 5,
  },
  {
    .name = "ctl_bus2_iphwa",
    .parent_name = "bus_mclk0",
    .en_offset = 0x06c,
    .en_shift = 6,
  },
  {
    .name = "sw1_xc_dma0",
    .parent_name = "bus_mclk0",
    .en_offset = 0x06c,
    .en_shift = 8,
  },
  {
    .name = "sw2_rfif_clk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x06c,
    .en_shift = 9,
  },
  {
    .name = "sw1_cipherhwa_clk",
    .parent_name = "bus_mclk1",
    .en_offset = 0x06c,
    .en_shift = 10,
  },
  {
    .name = "sw2_topdsp_clk",
    .parent_name = "top2cp_topbus_2cp",
    .en_offset = 0x06c,
    .en_shift = 11,
  },
  {
    .name = "msw_x1643_d",
    .parent_name = "bus_mclk0",
    .en_offset = 0x06c,
    .en_shift = 12,
  },
  {
    .name = "ctl_bus0_top",
    .parent_name = "bus_mclk0",
    .en_offset = 0x06c,
    .en_shift = 13,
  },
  {
    .name = "sw1_xc_dma1",
    .parent_name = "bus_mclk0",
    .en_offset = 0x06c,
    .en_shift = 14,
  },
  {
    .name = "ctl_bus2_cihperhwa",
    .parent_name = "bus_mclk1",
    .en_offset = 0x06c,
    .en_shift = 15,
  },
  {
    .name = "sw1_dmag_clk",
    .parent_name = "bus_mclk0",
    .en_offset = 0x070,
    .en_shift = 0,
  },
  {
    .name = "msw_xc4210_i",
    .parent_name = "bus_mclk0",
    .en_offset = 0x070,
    .en_shift = 1,
  },
  {
    .name = "ctl_bus0_cp_mailbox",
    .parent_name = "bus_mclk0",
    .en_offset = 0x070,
    .en_shift = 2,
  },
  {
    .name = "cp2top_cpctrlbus0_top",
    .parent_name = "bus_mclk0",
    .en_offset = 0x070,
    .en_shift = 6,
  },
  {
    .name = "cp2top_ddrbus_cpddr",
    .parent_name = "bus_mclk0",
    .en_offset = 0x070,
    .en_shift = 7,
  },
  {
    .name = "top2cp_topbus_2cp_clk_gated",
    .parent_name = "top2cp_topbus_2cp",
    .en_offset = 0x070,
    .en_shift = 8,
  },
  {
    .name = "cp_msw_ddr1",
    .parent_name = "bus_mclk0",
    .en_offset = 0x070,
    .en_shift = 10,
  },
  {
    .name = "a7_sw_ddr1",
    .parent_name = "bus_mclk0",
    .en_offset = 0x070,
    .en_shift = 11,
  },
  {
    .name = "cp_msw_1643ram",
    .parent_name = "bus_mclk0",
    .en_offset = 0x070,
    .en_shift = 14,
  },
  {
    .name = "cp_msw_4210ram",
    .parent_name = "bus_mclk0",
    .en_offset = 0x070,
    .en_shift = 15,
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
    .name = "wdt_pclk",
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
    .name = "a7_dbg_clk",
    .parent_name = "a7_clk",
    .en_offset = 0x074,
    .en_shift = 5,
  },
  {
    .name = "a7_clk_div13",
    .parent_name = "a7_clkdiv13",
    .en_offset = 0x074,
    .en_shift = 6,
  },
  {
    .name = "gsm1_26m",
    .parent_name = "clk26m1",
    .en_offset = 0x074,
    .en_shift = 7,
  },
  {
    .name = "gsm1sp_clk",
    .parent_name = "clk26m1",
    .en_offset = 0x074,
    .en_shift = 8,
  },
  {
    .name = "ctl_bus2_mainclk",
    .parent_name = "ctl_bus0_ctlbus2",
    .en_offset = 0x074,
    .en_shift = 11,
  },
  {
    .name = "ctl_bus1_mainclk",
    .parent_name = "ctl_bus0_ctlbus1",
    .en_offset = 0x074,
    .en_shift = 12,
  },
  {
    .name = "cp_msw_shram2",
    .parent_name = "bus_mclk0",
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
  {
    .name = "rfif_digrfv4_tx",
    .parent_name = "rfif_tx_fir_div10",
    .en_offset = 0x100,
    .en_shift = 1,
  },
  {},
};

static const struct song_clk_table abies_cp_clk_tbl =
{
  .fixed_rate_clks   = fixed_rate,
  .fixed_factor_clks = fixed_factor,
  .gr_clks           = gr,
  .gr_fdiv_clks      = gr_fdiv,
  .mux_clks          = mux,
  .mux_div_clks      = mux_div,
  .div_clks          = div,
  .sdiv_sdiv_clks    = sdiv_sdiv,
  .gate_clks         = gate,
  .pll_clks          = pll,
  .timer_clks        = timer,
  .out_clks          = out,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0x87040000, &abies_cp_clk_tbl);
}

void up_clk_finalinitialize(void)
{
}
#endif /* (CONFIG_ARCH_CHIP_ABIES_A7) && (CONFIG_SONG_CLK) */

