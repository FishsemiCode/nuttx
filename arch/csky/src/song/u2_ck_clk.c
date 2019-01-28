/****************************************************************************
 * arch/csky/src/song/u2_ck_clk.c
 *
 *   Copyright (C) 2019 Pinecone Inc. All rights reserved.
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

#if defined(CONFIG_ARCH_CHIP_U2_CK) && defined(CONFIG_SONG_CLK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *out_clk_src[] =
{
  "pll0",         /* TODO:it is a fake clk as ic bug */
  "top_bus_mclk",
  "pll1_mclk",
  "rfphy_pll0_clk",
  "clk32k",
  "codec_ref_clk",
};

static const char *rfphy_pll0_clk_src[] =
{
  "pll0",
  "rfphy_sys_clk",
};

static const char *pll1_pll0_src[] =
{
  "pll0",
  "pll1_occ",
};

static const struct song_fixed_rate_clk fixed_rate[] =
{
  {
    .name = "clk32k",
    .fixed_rate = 32768,
  },
  {
    .name = "btrf_pclk",
    .fixed_rate = 32000000,
  },
  {
    .name = "rfphy_sys_clk",
    .fixed_rate = 24000000,
  },
  {},
};

static const struct song_pll_lf_clk pll_lf[] =
{
  {
    .name = "pll0",
    .parent_name = "clk32k",
    .cfg0_offset = 0x30,
    .cfg1_offset = 0x34,
  },
  {},
};

static const struct song_pll_clk pll[] =
{
  {
    .name = "pll1_ori",
    .parent_name = "pll1_ref_clk",
    .cfg0_offset = 0x38,
    .cfg1_offset = 0x3c,
    .ctl_offset = 0x40,
    .ctl_shift = 0,
  },
  {},
};

static const struct song_out_clk out[] =
{
  {
    .name = "out0",
    .parent_names = out_clk_src,
    .num_parents = ARRAY_SIZE(out_clk_src),
    .mux_offset = 0xc8,
    .ctl_offset = 0xcc,
    .mux_shift = 0,
  },
  {},
};

static const struct song_mux_clk mux[] =
{
  {
    .name = "rfphy_pll0_clk",
    .parent_names = rfphy_pll0_clk_src,
    .num_parents = ARRAY_SIZE(rfphy_pll0_clk_src),
    .mux_offset = 0xdc,
    .mux_shift = 2,
    .mux_width = 1,
    .mux_flags = CLK_MUX_HIWORD_MASK,
  },
  {
    .name = "pll1",
    .parent_names = pll1_pll0_src,
    .num_parents = ARRAY_SIZE(pll1_pll0_src),
    .mux_offset = 0xdc,
    .mux_shift = 1,
    .mux_width = 1,
    .mux_flags = CLK_MUX_HIWORD_MASK,
  },
  {
    .name = "pll1_ref_clk",
    .parent_names = rfphy_pll0_clk_src,
    .num_parents = ARRAY_SIZE(rfphy_pll0_clk_src),
    .mux_offset = 0xdc,
    .mux_shift = 0,
    .mux_width = 1,
    .mux_flags = CLK_MUX_HIWORD_MASK,
  },
  {},
};

static const struct song_sdiv_fdiv_clk sdiv_fdiv[] =
{
  {
    .name = "audio_sys_in_clk",
    .parent_name = "audio_mclk",
    .sdiv_offset = 0xe0,
    .fdiv_offset = 0xe4,
  },
  {},
};

static const struct song_gr_fdiv_clk gr_fdiv[] =
{
  {
    .name = "uart0_clk",
    .parent_name = "pll1_mclk",
    .en_offset = 0x9c,
    .en_shift = 2,
    .gr_offset = 0x0,
    .div_offset = 0x7c,
  },
  {
    .name = "uart1_clk",
    .parent_name = "pll1_mclk",
    .en_offset = 0x9c,
    .en_shift = 4,
    .gr_offset = 0x0,
    .div_offset = 0x80,
  },
  {
    .name = "audio_i2s_mclk",
    .parent_name = "pll1_mclk",
    .en_offset = 0x90,
    .en_shift = 10,
    .gr_offset = 0x0,
    .div_offset = 0x84,
  },
  {
    .name = "pcm_mclk",
    .parent_name = "pll1_mclk",
    .en_offset = 0x90,
    .en_shift = 11,
    .gr_offset = 0x0,
    .div_offset = 0x88,
  },
  {},
};

static const struct song_gr_clk gr[] =
{
  {
    .name = "m4_cti_clk",
    .parent_name = "m4_clk",
    .en_offset = 0x78,
    .en_shift = 0,
    .mult_offset = 0x78,
    .mult_shift = 4,
    .mult_width = 3,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {},
};

static const struct song_sdiv_sdiv_clk sdiv_sdiv[] =
{
  {
    .name = "spi0_mclk",
    .parent_name = "pll1_mclk",
    .div_offset = 0x70,
    .div1_flags = 1 << CLK_DIVIDER_MINDIV_OFF,
    .div2_flags = 2 << CLK_DIVIDER_MINDIV_OFF,
  },
  {
    .name = "spi1_mclk",
    .parent_name = "pll1_mclk",
    .div_offset = 0x74,
    .div1_flags = 1 << CLK_DIVIDER_MINDIV_OFF,
    .div2_flags = 3 << CLK_DIVIDER_MINDIV_OFF,
  },
  {},
};

static const struct song_div_clk div[] =
{
  {
    .name = "wdt_dly_cnt_clk",
    .parent_name = "clk32k",
    .en_offset = 0xd0,
    .en_shift = 0,
    .div_offset = 0xd0,
    .div_shift = 4,
    .div_width = 3,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "pwm_mclk",
    .parent_name = "rfphy_pll0_clk",
    .en_offset = 0xd8,
    .en_shift = 0,
    .div_offset = 0xd8,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "i2c0_mclk",
    .parent_name = "pll1_mclk",
    .en_offset = 0x0b8,
    .en_shift = 0,
    .div_offset = 0x0b8,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "i2c1_mclk",
    .parent_name = "pll1_mclk",
    .en_offset = 0x0bc,
    .en_shift = 0,
    .div_offset = 0x0bc,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "i2c2_mclk",
    .parent_name = "pll1_mclk",
    .en_offset = 0x0c0,
    .en_shift = 0,
    .div_offset = 0x0c0,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "bus_lp_clk",
    .parent_name = "rfphy_pll0_clk",
    .en_offset = 0x0c4,
    .en_shift = 0,
    .div_offset = 0x0c4,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "pll_adj_clk",
    .parent_name = "pll0",
    .en_offset = 0x0a8,
    .en_shift = 0,
    .div_offset = 0x0a8,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "top_bus_mclk",
    .parent_name = "pll1",
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
    .name = "pll1_mclk",
    .parent_name = "pll1",
    .en_offset = 0x050,
    .en_shift = 0,
    .div_offset = 0x050,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "tl421_bus_clk",
    .parent_name = "pll1",
    .en_offset = 0x054,
    .en_shift = 0,
    .div_offset = 0x054,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "thinkers_mclk",
    .parent_name = "audio_mclk",
    .en_offset = 0x058,
    .en_shift = 0,
    .div_offset = 0x058,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "btdm_bb_sys_clk",
    .parent_name = "rfphy_pll0_clk",
    .en_offset = 0x5c,
    .en_shift = 0,
    .div_offset = 0x5c,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "audio_mclk",
    .parent_name = "pll1",
    .en_offset = 0x060,
    .en_shift = 0,
    .div_offset = 0x060,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "audio_clk_3072k",
    .parent_name = "pll1",
    .en_offset = 0x064,
    .en_shift = 0,
    .div_offset = 0x064,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "at_clk",
    .parent_name = "rfphy_pll0_clk",
    .en_offset = 0x06c,
    .en_shift = 0,
    .div_offset = 0x06c,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "codec_ref_clk",
    .parent_name = "pll1",
    .en_offset = 0x08c,
    .en_shift = 0,
    .div_offset = 0x08c,
    .div_shift = 4,
    .div_width = 6,
  },
  {},
};

static const struct song_gate_clk gate[] =
{
  {
    .name = "top_dmas_hclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x090,
    .en_shift = 0,
  },
  {
    .name = "audio_dmas_hclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x090,
    .en_shift = 1,
  },
  {
    .name = "tl421_ictl_hclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x090,
    .en_shift = 2,
  },
  {
    .name = "tl421_wdt_tclk",
    .parent_name = "clk32k",
    .en_offset = 0x090,
    .en_shift = 3,
  },
  {
    .name = "at_calib_clk",
    .parent_name = "rfphy_pll0_clk",
    .en_offset = 0x090,
    .en_shift = 5,
  },
  {
    .name = "audio_sys_akm_clk",
    .parent_name = "codec_ref_clk",
    .en_offset = 0x090,
    .en_shift = 6,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "tl421_wdt_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x090,
    .en_shift = 4,
  },
  {
    .name = "pcm_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x090,
    .en_shift = 7,
  },
  {
    .name = "thinkers_pclk",
    .parent_name = "audio_mclk",
    .en_offset = 0x090,
    .en_shift = 8,
  },
  {
    .name = "audio_sys_pclk",
    .parent_name = "tl421_bus_clk",
    .en_offset = 0x090,
    .en_shift = 9,
  },
#endif
  {
    .name = "audio_sys_hclk",
    .parent_name = "audio_mclk",
    .en_offset = 0x090,
    .en_shift = 12,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "audio_sys_clk_30720k",
    .parent_name = "audio_mclk",
    .en_offset = 0x090,
    .en_shift = 13,
  },
  {
    .name = "btdm_bb_hclk",
    .parent_name = "rfphy_pll0_clk",
    .en_offset = 0x094,
    .en_shift = 0,
  },
  {
    .name = "btdm_i_lp_clk",
    .parent_name = "clk32k",
    .en_offset = 0x094,
    .en_shift = 1,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "m4_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x098,
    .en_shift = 0,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "m4_wic_clk",
    .parent_name = "m4_clk",
    .en_offset = 0x098,
    .en_shift = 1,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "m4_wdt_tclk",
    .parent_name = "clk32k",
    .en_offset = 0x098,
    .en_shift = 3,
  },
  {
    .name = "pll1_occ",
    .parent_name = "pll1_ori",
    .en_offset = 0x098,
    .en_shift = 4,
  },
  {
    .name = "gpio_clk32k",
    .parent_name = "clk32k",
    .en_offset = 0x098,
    .en_shift = 6,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "topbus_rfphyclk",
    .parent_name = "btrf_pclk",
    .en_offset = 0x094,
    .en_shift = 2,
  },
  {
    .name = "m4_wdt_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x098,
    .en_shift = 2,
  },
  {
    .name = "gpio_pclk",
    .parent_name = "top_pclk1",
    .en_offset = 0x098,
    .en_shift = 5,
  },
  {
    .name = "pwm_pclk",
    .parent_name = "top_pclk1",
    .en_offset = 0x098,
    .en_shift = 7,
  },
  {
    .name = "i2c0_pclk",
    .parent_name = "top_pclk1",
    .en_offset = 0x098,
    .en_shift = 8,
  },
  {
    .name = "i2c1_pclk",
    .parent_name = "top_pclk1",
    .en_offset = 0x098,
    .en_shift = 9,
  },
  {
    .name = "i2c2_pclk",
    .parent_name = "top_pclk1",
    .en_offset = 0x098,
    .en_shift = 10,
  },
  {
    .name = "spi0_pclk",
    .parent_name = "top_pclk1",
    .en_offset = 0x098,
    .en_shift = 11,
  },
  {
    .name = "spi1_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x098,
    .en_shift = 12,
  },
  {
    .name = "muxpin_pclk",
    .parent_name = "top_pclk1",
    .en_offset = 0x098,
    .en_shift = 13,
  },
  {
    .name = "uart0_pclk",
    .parent_name = "top_pclk1",
    .en_offset = 0x09c,
    .en_shift = 1,
  },
  {
    .name = "uart1_pclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x09c,
    .en_shift = 3,
  },
#endif
  {
    .name = "cs_dap_clk",
    .parent_name = "top_pclk0",
    .en_offset = 0x0a0,
    .en_shift = 8,
  },
  {
    .name = "m4_dapclk",
    .parent_name = "top_pclk0",
    .en_offset = 0x0a0,
    .en_shift = 9,
  },
  {
    .name = "mailbox_hclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x09c,
    .en_shift = 0,
  },
  {
    .name = "m4_stclk",
    .parent_name = "clk32k",
    .en_offset = 0x0a0,
    .en_shift = 10,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {},
};

static const struct song_lp_reg_clk lp_reg[] =
{
  /* LP_EN0 */
  {
    .offset = 0x314,
    .value = 0xffffffff,
  },
  {}
};

static const struct song_clk_table clk_tbl =
{
  .fixed_rate_clks   = fixed_rate,
  .gr_clks           = gr,
  .sdiv_fdiv_clks    = sdiv_fdiv,
  .gr_fdiv_clks      = gr_fdiv,
  .div_clks          = div,
  .sdiv_sdiv_clks    = sdiv_sdiv,
  .gate_clks         = gate,
  .mux_clks          = mux,
  .pll_clks          = pll,
  .pll_lf_clks       = pll_lf,
  .out_clks          = out,
  .lp_reg            = lp_reg,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xa00e0000, &clk_tbl);
}

void up_clk_finalinitialize(void)
{
  clk_disable_unused();
}
#endif /* (CONFIG_ARCH_CHIP_U2_CK) && (CONFIG_SONG_CLK) */
