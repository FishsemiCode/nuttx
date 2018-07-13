/****************************************************************************
 * arch/arm/src/song/u2_ap_clk.c
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

#if defined(CONFIG_ARCH_CHIP_U2_AP) && defined(CONFIG_SONG_CLK)

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
    .flags = 0,
    .fixed_rate = 32768,
  },
  {
    .name = "btrf_pclk",
    .flags = 0,
    .fixed_rate = 32000000,
  },
  {
    .name = "rfphy_sys_clk",
    .flags = 0,
    .fixed_rate = 24000000,
  },
  {},
};

static const struct song_pll_lf_clk pll_lf[] =
{
  {
    .name = "pll0",
    .parent_name = "clk32k",
    .flags = 0,
    .cfg_reg0_offset = 0x30,
    .cfg_reg1_offset = 0x34,
  },
  {},
};

static const struct song_pll_clk pll[] =
{
  {
    .name = "pll1_ori",
    .parent_name = "pll1_ref_clk",
    .flags = 0,
    .cfg_reg0_offset = 0x38,
    .cfg_reg1_offset = 0x3c,
    .ctl_reg_offset = 0x40,
    .ctl_shift = 0,
  },
  {},
};

static const struct song_out_clk out[] =
{
  {
    .name = "out0",
    .parent_name = out_clk_src,
    .num_parents = ARRAY_SIZE(out_clk_src),
    .mux_reg_offset = 0xc8,
    .ctl_reg_offset = 0xcc,
    .mux_shift = 0,
    .mux_width = 3,
  },
  {},
};

static const struct song_mux_gate_clk mux_gate[] =
{
  {
    .name = "rfphy_pll0_clk",
    .parent_name = rfphy_pll0_clk_src,
    .num_parents = ARRAY_SIZE(rfphy_pll0_clk_src),
    .flags = 0,
    .en_offset = 0,
    .mux_offset = 0xdc,
    .mux_shift = 2,
    .mux_width = 1,
  },
  {
    .name = "pll1",
    .parent_name = pll1_pll0_src,
    .num_parents = ARRAY_SIZE(pll1_pll0_src),
    .flags = 0,
    .en_offset = 0,
    .mux_offset = 0xdc,
    .mux_shift = 1,
    .mux_width = 1,
  },
  {
    .name = "pll1_ref_clk",
    .parent_name = rfphy_pll0_clk_src,
    .num_parents = ARRAY_SIZE(rfphy_pll0_clk_src),
    .flags = 0,
    .en_offset = 0,
    .mux_offset = 0xdc,
    .mux_shift = 0,
    .mux_width = 1,
  },
  {},
};

static const struct song_sdiv_fdiv_clk sdiv_fdiv[] =
{
  {
    .name = "audio_sys_in_clk",
    .parent_name = "audio_mclk",
    .flags = 0,
    .en_offset = 0xe0,
    .en_shift = 0,
    .sdiv_offset = 0xe0,
    .sdiv_shift = 4,
    .sdiv_width = 4,
    .fdiv_offset = 0xe4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT) |
                     ((CLK_FRAC_MUL_NEED_EVEN | CLK_FRAC_DIV_DOUBLE)
                      << SONG_CLK_FRAC_FLAG_SHIFT)),
  },
  {},
};

static const struct song_gr_fdiv_clk gr_fdiv[] =
{
  {
    .name = "uart0_clk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x9c,
    .en_shift = 2,
    .gr_offset = 0x0,
    .div_offset = 0x7c,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     (CLK_FRAC_MUL_NEED_EVEN << SONG_CLK_FRAC_FLAG_SHIFT)),
  },
  {
    .name = "uart1_clk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x9c,
    .en_shift = 4,
    .gr_offset = 0x0,
    .div_offset = 0x80,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     (CLK_FRAC_MUL_NEED_EVEN << SONG_CLK_FRAC_FLAG_SHIFT)),
  },
  {
    .name = "audio_i2s_mclk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x90,
    .en_shift = 10,
    .gr_offset = 0x0,
    .div_offset = 0x84,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     (CLK_FRAC_MUL_NEED_EVEN << SONG_CLK_FRAC_FLAG_SHIFT)),
  },
  {
    .name = "pcm_mclk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x90,
    .en_shift = 11,
    .gr_offset = 0x0,
    .div_offset = 0x88,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     (CLK_FRAC_MUL_NEED_EVEN << SONG_CLK_FRAC_FLAG_SHIFT)),
  },
  {},
};

static const struct song_gr_clk gr[] =
{
  {
    .name = "m4_cti_clk",
    .parent_name = "m4_clk",
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

static const struct song_sdiv_sdiv_clk sdiv_sdiv[] =
{
  {
    .name = "spi0_mclk",
    .parent_name = "pll1_mclk",
    .div_offset = 0x70,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "spi1_mclk",
    .parent_name = "pll1_mclk",
    .div_offset = 0x74,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "btdm_bb_sys_clk",
    .parent_name = "rfphy_pll0_clk",
    .div_offset = 0x5c,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {},
};

static const struct song_sdiv_clk sdiv[] =
{
  {
    .name = "wdt_dly_cnt_clk",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0xd0,
    .en_shift = 0,
    .div_offset = 0xd0,
    .div_shift = 4,
    .div_width = 3,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "pwm_mclk",
    .parent_name = "rfphy_pll0_clk",
    .flags = 0,
    .en_offset = 0xd8,
    .en_shift = 0,
    .div_offset = 0xd8,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "i2c0_mclk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 0,
    .div_offset = 0x0b8,
    .div_shift = 4,
    .div_width = 5,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "i2c1_mclk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x0bc,
    .en_shift = 0,
    .div_offset = 0x0bc,
    .div_shift = 4,
    .div_width = 5,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "i2c2_mclk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x0c0,
    .en_shift = 0,
    .div_offset = 0x0c0,
    .div_shift = 4,
    .div_width = 5,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "bus_lp_clk",
    .parent_name = "rfphy_pll0_clk",
    .flags = 0,
    .en_offset = 0x0c4,
    .en_shift = 0,
    .div_offset = 0x0c4,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "pll_adj_clk",
    .parent_name = "pll0",
    .flags = 0,
    .en_offset = 0x0a8,
    .en_shift = 0,
    .div_offset = 0x0a8,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "top_bus_mclk",
    .parent_name = "pll1",
    .flags = 0,
    .en_offset = 0x044,
    .en_shift = 0,
    .div_offset = 0x044,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "top_pclk0",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x048,
    .en_shift = 0,
    .div_offset = 0x048,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "top_pclk1",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x04c,
    .en_shift = 0,
    .div_offset = 0x04c,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "pll1_mclk",
    .parent_name = "pll1",
    .flags = 0,
    .en_offset = 0x050,
    .en_shift = 0,
    .div_offset = 0x050,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "tl421_bus_clk",
    .parent_name = "pll1",
    .flags = 0,
    .en_offset = 0x054,
    .en_shift = 0,
    .div_offset = 0x054,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "thinkers_mclk",
    .parent_name = "audio_mclk",
    .flags = 0,
    .en_offset = 0x058,
    .en_shift = 0,
    .div_offset = 0x058,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "audio_mclk",
    .parent_name = "pll1",
    .flags = 0,
    .en_offset = 0x060,
    .en_shift = 0,
    .div_offset = 0x060,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "audio_clk_3072k",
    .parent_name = "pll1",
    .flags = 0,
    .en_offset = 0x064,
    .en_shift = 0,
    .div_offset = 0x064,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "at_clk",
    .parent_name = "rfphy_pll0_clk",
    .flags = 0,
    .en_offset = 0x06c,
    .en_shift = 0,
    .div_offset = 0x06c,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "codec_ref_clk",
    .parent_name = "pll1",
    .flags = 0,
    .en_offset = 0x08c,
    .en_shift = 0,
    .div_offset = 0x08c,
    .div_shift = 4,
    .div_width = 6,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {},
};

static const struct song_gate_clk gate[] =
{
  {
    .name = "top_dmas_hclk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 0,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "audio_dmas_hclk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 1,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "tl421_ictl_hclk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 2,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "tl421_wdt_tclk",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 3,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "tl421_wdt_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 4,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "at_calib_clk",
    .parent_name = "rfphy_pll0_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 5,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "audio_sys_akm_clk",
    .parent_name = "codec_ref_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 6,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "pcm_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 7,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "thinkers_pclk",
    .parent_name = "audio_mclk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 8,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "audio_sys_pclk",
    .parent_name = "tl421_bus_clk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 9,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "audio_sys_hclk",
    .parent_name = "audio_mclk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 12,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "audio_sys_clk_30720k",
    .parent_name = "audio_mclk",
    .flags = 0,
    .en_offset = 0x090,
    .en_shift = 13,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "btdm_bb_hclk",
    .parent_name = "rfphy_pll0_clk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 0,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "btdm_i_lp_clk",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 1,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "topbus_rfphyclk",
    .parent_name = "btrf_pclk",
    .flags = 0,
    .en_offset = 0x094,
    .en_shift = 2,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "m4_clk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 0,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "m4_wic_clk",
    .parent_name = "m4_clk",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 1,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "m4_wdt_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 2,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "m4_wdt_tclk",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 3,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "pll1_occ",
    .parent_name = "pll1_ori",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 4,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gpio_pclk",
    .parent_name = "top_pclk1",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 5,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gpio_clk32k",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 6,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "pwm_pclk",
    .parent_name = "top_pclk1",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 7,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "i2c0_pclk",
    .parent_name = "top_pclk1",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 8,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "i2c1_pclk",
    .parent_name = "top_pclk1",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 9,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "i2c2_pclk",
    .parent_name = "top_pclk1",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 10,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "spi0_pclk",
    .parent_name = "top_pclk1",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 11,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "spi1_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 12,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "muxpin_pclk",
    .parent_name = "top_pclk1",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 13,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "mailbox_hclk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 0,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "uart0_pclk",
    .parent_name = "top_pclk1",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 1,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "uart1_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 3,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "cs_dap_clk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x0a0,
    .en_shift = 8,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "m4_dapclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x0a0,
    .en_shift = 9,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "m4_stclk",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x0a0,
    .en_shift = 10,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {},
};

static const struct song_clk_table u2_ap_clk_tbl =
{
  .fixed_rate_clks   = fixed_rate,
  .gr_clks           = gr,
  .sdiv_fdiv_clks    = sdiv_fdiv,
  .gr_fdiv_clks      = gr_fdiv,
  .sdiv_clks         = sdiv,
  .sdiv_sdiv_clks    = sdiv_sdiv,
  .gate_clks         = gate,
  .mux_gate_clks     = mux_gate,
  .pll_clks          = pll,
  .pll_lf_clks       = pll_lf,
  .out_clks          = out,
  .rpmsg_server      = true,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xa00e0000, &u2_ap_clk_tbl);
}

#endif /* CONFIG_ARCH_CHIP_U2_AP */
