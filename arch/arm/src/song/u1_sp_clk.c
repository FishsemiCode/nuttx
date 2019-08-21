/****************************************************************************
 * arch/arm/src/song/u1_sp_clk.c
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

#if defined(CONFIG_ARCH_CHIP_U1_SP) && defined(CONFIG_SONG_CLK)

#include <nuttx/clk/clk.h>
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This describes unicorn 32k osc clk */
static const struct song_fixed_rate_clk fixed_rate[] =
{
  {
    .name = "clk32k",
    .fixed_rate = 32768,
  },
  {},
};

/* This describes unicorn two pll */
static const struct song_pll_lf_clk pll_lf[] =
{
  {
    .name = "pll0_out",
    .parent_name = "clk32k",
    .cfg0_offset = 0x30,
    .cfg1_offset = 0x34,
  },
  {},
};

static const struct song_pll_clk pll[] =
{
  {
    .name = "pll1_out_ori",
    .parent_name = "pll0_out",
    .cfg0_offset = 0x38,
    .cfg1_offset = 0x3c,
    .ctl_offset = 0x40,
    .ctl_shift = 0,
  },
  {},
};

static const struct song_fixed_factor_clk fixed_factor[] =
{
  {
    .name = "pll0_out_div2",
    .parent_name = "pll0_out",
    .fixed_mult = 1,
    .fixed_div = 2,
  },
  {
    .name = "pll0_out_mult",
    .parent_name = "pll0_out",
    .fixed_mult = 16,
    .fixed_div = 1,
  },
  {},
};

static const struct song_gr_clk gr[] =
{
  {
    .name = "sec_m4_cti_clk",
    .parent_name = "sec_m4_clk",
    .en_offset = 0x7c,
    .en_shift = 0,
    .mult_offset = 0x7c,
    .mult_shift = 4,
    .mult_width = 3,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {},
};

static const struct song_sdiv_sdiv_clk sdiv_sdiv[] =
{
  {
    .name = "spi1_mclk",
    .parent_name = "pll1_mclk",
    .div_offset = 0x74,
    .div1_flags = 1 << CLK_DIVIDER_MINDIV_OFF,
    .div2_flags = 3 << CLK_DIVIDER_MINDIV_OFF,
  },
  {},
};

static const struct song_gr_fdiv_clk gr_fdiv[] =
{
  {
    .name = "uart0_tx_clk",
    .parent_name = "pll0_out_mult",
    .en_offset = 0x9c,
    .en_shift = 3,
    .gr_offset = 0x0,
    .div_offset = 0x80,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "uart1_clk",
    .parent_name = "pll1_mclk",
    .en_offset = 0x9c,
    .en_shift = 11,
    .gr_offset = 0x0,
    .div_offset = 0x84,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "uart2_clk",
    .parent_name = "pll1_mclk",
    .en_offset = 0x9c,
    .en_shift = 13,
    .gr_offset = 0x0,
    .div_offset = 0x88,
  },
  {
    .name = "uart3_clk",
    .parent_name = "pll1_mclk",
    .en_offset = 0x9c,
    .en_shift = 15,
    .gr_offset = 0x0,
    .div_offset = 0x8c,
  },
  {},
};

static const struct song_div_clk div[] =
{
  {
    .name = "pll_adj_clk",
    .parent_name = "pll0_out_div2",
    .en_offset = 0x0a0,
    .en_shift = 0,
    .div_offset = 0x0a0,
    .div_shift = 4,
    .div_width = 3,
    .div_flags = CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_MAX_HALF,
  },
  {
    .name = "top_bus_mclk",
    .parent_name = "pll1_out",
    .en_offset = 0x044,
    .en_shift = 0,
    .div_offset = 0x044,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "top_pclk0",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x048,
    .en_shift = 0,
    .div_offset = 0x048,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "top_pclk1",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x04c,
    .en_shift = 0,
    .div_offset = 0x04c,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "top_pclk2",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x054,
    .en_shift = 0,
    .div_offset = 0x054,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "pll1_mclk",
    .parent_name = "pll1_out",
    .en_offset = 0x050,
    .en_shift = 0,
    .div_offset = 0x050,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "at_clk",
    .parent_name = "pll0_out",
    .en_offset = 0x06c,
    .en_shift = 0,
    .div_offset = 0x06c,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "bus_lp_clk",
    .parent_name = "pll0_out",
    .en_offset = 0x0cc,
    .en_shift = 0,
    .div_offset = 0x0cc,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "flash_ctrl_clk",
    .parent_name = "pll1_out",
    .en_offset = 0x2c4,
    .en_shift = 0,
    .div_offset = 0x2c4,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "sec_m4_bus_clk",
    .parent_name = "pll1_out",
    .en_offset = 0x2c8,
    .en_shift = 0,
    .div_offset = 0x2c8,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "wdt_dly_cnt_clk",
    .parent_name = "clk32k",
    .en_offset = 0x2f8,
    .en_shift = 0,
    .div_offset = 0x2f8,
    .div_shift = 4,
    .div_width = 3,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {},
};

static const struct song_gate_clk gate[] =
{
  {
    .name = "sec_m4_clk",
    .parent_name = "sec_m4_bus_clk",
    .en_offset = 0x098,
    .en_shift = 0,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "sec_m4_wic_clk",
    .parent_name = "sec_m4_clk",
    .en_offset = 0x098,
    .en_shift = 1,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "swdt_tclk",
    .parent_name = "clk32k",
    .en_offset = 0x098,
    .en_shift = 3,
  },
  {
    .name = "sec_tcm_icm_hclk",
    .parent_name = "sec_m4_clk",
    .en_offset = 0x098,
    .en_shift = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "dmas_hclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x098,
    .en_shift = 5,
  },
  {
    .name = "pll1_out",
    .parent_name = "pll1_out_ori",
    .en_offset = 0x098,
    .en_shift = 6,
  },
  {
    .name = "gpio_clk32k",
    .parent_name = "clk32k",
    .en_offset = 0x098,
    .en_shift = 8,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "swdt_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x098,
    .en_shift = 2,
  },
  {
    .name = "gpio_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x098,
    .en_shift = 7,
  },
  {
    .name = "spi1_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x098,
    .en_shift = 13,
  },
  {
    .name = "muxpin_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x098,
    .en_shift = 14,
  },
  {
    .name = "pmicfsm_pclk",
    .parent_name = "top_pclk1",
    .en_offset = 0x09c,
    .en_shift = 0,
  },
  {
    .name = "rtc_pclk",
    .parent_name = "top_pclk1",
    .en_offset = 0x09c,
    .en_shift = 1,
  },
  {
    .name = "uart0_pclk",
    .parent_name = "top_pclk1",
    .en_offset = 0x09c,
    .en_shift = 2,
  },
  {
    .name = "uart1_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x09c,
    .en_shift = 10,
  },
  {
    .name = "uart2_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x09c,
    .en_shift = 12,
  },
  {
    .name = "uart3_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x09c,
    .en_shift = 14,
  },
#endif
  {
    .name = "flash_ctrl_hclk",
    .parent_name = "flash_ctrl_clk",
    .en_offset = 0x09c,
    .en_shift = 4,
  },
  {
    .name = "top_cipher_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x09c,
    .en_shift = 5,
  },
  {
    .name = "mailbox_hclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x09c,
    .en_shift = 6,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "security_clk32k",
    .parent_name = "clk32k",
    .en_offset = 0x09c,
    .en_shift = 8,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "security_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x09c,
    .en_shift = 7,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "topbus_shram0clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0b8,
    .en_shift = 0,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "topbus_shram1clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0b8,
    .en_shift = 1,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "topbus_sectcmclk",
    .parent_name = "sec_m4_bus_clk",
    .en_offset = 0x0b8,
    .en_shift = 2,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "sec_m4_icache_clk",
    .parent_name = "sec_m4_clk",
    .en_offset = 0x0b8,
    .en_shift = 14,
    .clk_flags = CLK_IGNORE_UNUSED,
  }
#endif
  {
    .name = "topbus_debugclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x0b8,
    .en_shift = 4,
  },
  {
    .name = "cs_dap_clk",
    .parent_name = "top_pclk0",
    .en_offset = 0x0b8,
    .en_shift = 8,
  },
  {
    .name = "sec_m4_dapclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x0b8,
    .en_shift = 9,
  },
  {
    .name = "sec_m4_stclk",
    .parent_name = "clk32k",
    .en_offset = 0x0b8,
    .en_shift = 10,
    .clk_flags = CLK_IGNORE_UNUSED,
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
  /* LP_EN1 */
  {
    .offset = 0x1c8,
    .value = 0xffff,
  },
  {}
};

static const struct clk_rate def_rates[] =
{
  {
    .name = "top_bus_mclk",
    .rate = 81920000,
  },
  {},
};

static const struct song_clk_table u1_sp_clk_tbl =
{
  .fixed_rate_clks   = fixed_rate,
  .fixed_factor_clks = fixed_factor,
  .gr_clks           = gr,
  .gr_fdiv_clks      = gr_fdiv,
  .div_clks          = div,
  .sdiv_sdiv_clks    = sdiv_sdiv,
  .gate_clks         = gate,
  .pll_clks          = pll,
  .pll_lf_clks       = pll_lf,
  .lp_reg            = lp_reg,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xb0040000, &u1_sp_clk_tbl);
  clk_set_rates(def_rates);
}

void up_clk_finalinitialize(void)
{
  clk_disable_unused();
}
#endif /* (CONFIG_ARCH_CHIP_U1_SP) && (CONFIG_SONG_CLK) */
