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

/* This describes mux clk parent source */
static const char * const out_clk_src[] =
{
  "cp/gnss_i_rf0_clk",
  "sp/top_bus_mclk0",
  "sp/pll1_mclk",
  "sp/pll0_out",
  "sp/clk32k",
  "sp/pll0_out",
};

static const struct song_out_clk out[] =
{
  {
    .name = "out0",
    .parent_names = out_clk_src,
    .num_parents = ARRAY_SIZE(out_clk_src),
    .mux_offset = 0x2b8,
    .ctl_offset = 0x2bc,
    .mux_shift = 0,
  },
  {},
};

static const struct song_gr_clk gr[] =
{
  {
    .name = "ap_m4_cti_clk",
    .parent_name = "ap_m4_clk",
    .en_offset = 0xbc,
    .en_shift = 0,
    .mult_offset = 0xbc,
    .mult_shift = 4,
    .mult_width = 3,
  },
  {},
};

static const struct song_sdiv_sdiv_clk sdiv_sdiv[] =
{
  {
    .name = "spi0_mclk",
    .parent_name = "sp/pll1_mclk",
    .div_offset = 0x70,
  },
  {
    .name = "spi2_mclk",
    .parent_name = "sp/pll1_mclk",
    .div_offset = 0xc8,
  },
  {},
};

static const struct song_gr_fdiv_clk gr_fdiv[] =
{
  {
    .name = "uart0_tx_clk",
    .parent_name = "sp/pll0_out",
    .en_offset = 0x9c,
    .en_shift = 3,
    .gr_offset = 0x0,
    .div_offset = 0x80,
  },
  {
    .name = "uart1_clk",
    .parent_name = "sp/pll1_mclk",
    .en_offset = 0x9c,
    .en_shift = 11,
    .gr_offset = 0x0,
    .div_offset = 0x84,
  },
  {
    .name = "uart2_clk",
    .parent_name = "sp/pll1_mclk",
    .en_offset = 0x9c,
    .en_shift = 13,
    .gr_offset = 0x0,
    .div_offset = 0x88,
  },
  {
    .name = "uart3_clk",
    .parent_name = "sp/pll1_mclk",
    .en_offset = 0x9c,
    .en_shift = 15,
    .gr_offset = 0x0,
    .div_offset = 0x8c,
  },
};

static const struct song_div_clk div[] =
{
  {
    .name = "i2c0_mclk",
    .parent_name = "sp/pll0_out",
    .en_offset = 0x0b0,
    .en_shift = 0,
    .div_offset = 0x0b0,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "i2c1_mclk",
    .parent_name = "sp/pll0_out",
    .en_offset = 0x0b4,
    .en_shift = 0,
    .div_offset = 0x0b4,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "ap_m4_bus_clk",
    .parent_name = "sp/pll1_out",
    .en_offset = 0x2cc,
    .en_shift = 0,
    .div_offset = 0x2cc,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "pwm_mclk",
    .parent_name = "sp/pll0_out_div2",
    .en_offset = 0x304,
    .en_shift = 0,
    .div_offset = 0x304,
    .div_shift = 4,
    .div_width = 3,
    .div_flags = CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_MAX_HALF,
  },
};

static const struct song_gate_clk gate[] =
{
  {
    .name = "pwm_pclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x098,
    .en_shift = 9,
  },
  {
    .name = "i2c0_pclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x098,
    .en_shift = 10,
  },
  {
    .name = "i2c1_pclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x098,
    .en_shift = 11,
  },
  {
    .name = "spi0_pclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x098,
    .en_shift = 12,
  },
  {
    .name = "timer_pclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x098,
    .en_shift = 15,
  },
  {
    .name = "uart0_pclk",
    .parent_name = "sp/top_pclk1",
    .en_offset = 0x09c,
    .en_shift = 2,
  },
  {
    .name = "spi2_pclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x09c,
    .en_shift = 9,
  },
  {
    .name = "uart1_pclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x09c,
    .en_shift = 10,
  },
  {
    .name = "uart2_pclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x09c,
    .en_shift = 12,
  },
  {
    .name = "uart3_pclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x09c,
    .en_shift = 14,
  },
  {
    .name = "topbus_aptcmclk",
    .parent_name = "ap_m4_bus_clk",
    .en_offset = 0x0b8,
    .en_shift = 3,
  },
  {
    .name = "i2c_ahb_hclk",
    .parent_name = "sp/top_bus_mclk",
    .en_offset = 0x0b8,
    .en_shift = 13,
  },
  {
    .name = "ap_m4_dapclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x0b8,
    .en_shift = 11,
  },
  {
    .name = "ap_m4_clk",
    .parent_name = "ap_m4_bus_clk",
    .en_offset = 0x0c4,
    .en_shift = 0,
  },
  {
    .name = "ap_m4_wic_clk",
    .parent_name = "ap_m4_clk",
    .en_offset = 0x0c4,
    .en_shift = 1,
  },
  {
    .name = "ap_m4_stclk",
    .parent_name = "sp/clk32k",
    .en_offset = 0x0c4,
    .en_shift = 2,
  },
  {
    .name = "apwdt_pclk",
    .parent_name = "sp/top_pclk0",
    .en_offset = 0x0c4,
    .en_shift = 3,
  },
  {
    .name = "apwdt_tclk",
    .parent_name = "sp/clk32k",
    .en_offset = 0x0c4,
    .en_shift = 4,
  },
  {
    .name = "ap_tcm_icm_hclk",
    .parent_name = "ap_m4_clk",
    .en_offset = 0x0c4,
    .en_shift = 5,
  },
  {
    .name = "ap_m4_icache_clk",
    .parent_name = "ap_m4_clk",
    .en_offset = 0x0c4,
    .en_shift = 6,
  },
  {},
};

static const struct song_default_rate_clk def_rate[] =
{
  {
    .name = "i2c0_mclk",
    .rate = 80000000,
  },
  {
    .name = "i2c1_mclk",
    .rate = 80000000,
  },
#ifdef CONFIG_16550_UART2
  {
    .name = "uart2_clk",
    .rate = CONFIG_16550_UART2_CLOCK,
  },
#endif
#ifdef CONFIG_16550_UART3
  {
    .name = "uart3_clk",
    .rate = CONFIG_16550_UART3_CLOCK,
  },
#endif
  {}
};

static const struct song_clk_table clk_tbl =
{
  .gr_clks           = gr,
  .gr_fdiv_clks      = gr_fdiv,
  .div_clks          = div,
  .sdiv_sdiv_clks    = sdiv_sdiv,
  .gate_clks         = gate,
  .out_clks          = out,
  .def_rate          = def_rate,
  .rpmsg_server      = false,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xb0040000, &clk_tbl);
}
#endif /* CONFIG_ARCH_CHIP_U1_AP */
