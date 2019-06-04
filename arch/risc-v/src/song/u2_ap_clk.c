/****************************************************************************
 * arch/risc-v/src/song/u2_ap_clk.c
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

#if defined(CONFIG_ARCH_CHIP_U2_AP) && defined(CONFIG_SONG_CLK)

#include <nuttx/clk/clk.h>
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *out_clk_src[] =
{
  "top_bus_mclk",
  "rfphy_sys_clk",
  "pll1_mclk",
  "pll0",
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

static const char *audio_mclk_mx_src[] =
{
  "pll0",
  "audio_mclk",
};

static const char *vad_mclk_src[] =
{
  "vad_mclk_pll0",
  "clk32k",
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
  {
    .name = "audio_mclk_mx",
    .parent_names = audio_mclk_mx_src,
    .num_parents = ARRAY_SIZE(audio_mclk_mx_src),
    .en_offset = 0x68,
    .en_shift  = 0,
    .mux_offset = 0x68,
    .mux_shift = 4,
    .mux_width = 1,
    .mux_flags = CLK_MUX_HIWORD_MASK,
  },
  {
    .name = "vad_mclk",
    .parent_names = vad_mclk_src,
    .num_parents = ARRAY_SIZE(vad_mclk_src),
    .en_offset = 0xf0,
    .en_shift  = 0,
    .mux_offset = 0xf0,
    .mux_shift = 4,
    .mux_width = 1,
    .mux_flags = CLK_MUX_HIWORD_MASK,
  },
};

static const struct song_sdiv_fdiv_clk sdiv_fdiv[] =
{
  {
    .name = "audio_sys_in_clk",
    .parent_name = "audio_mclk_mx",
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
  {
    .name = "vad_mclk_pll0_div",
    .parent_name = "pll0",
    .div_offset = 0xec,
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
    .name = "spi_pclk",
    .parent_name = "pll1",
    .en_offset = 0x054,
    .en_shift = 0,
    .div_offset = 0x054,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
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
    .parent_name = "btdm_bb_hclk",
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
    .name = "thinkers_pclk",
    .parent_name = "audio_mclk_mx",
    .en_offset = 0x078,
    .en_shift = 8,
    .div_offset = 0x078,
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
  {
    .name = "vad_bus_clk",
    .parent_name = "pll0",
    .en_offset = 0x0f4,
    .en_shift = 0,
    .div_offset = 0x0f4,
    .div_shift = 4,
    .div_width = 5,
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
    .name = "ictl0_hclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x090,
    .en_shift = 2,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "rcpu1_wdt_tclk",
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
    .name = "rcpu1_wdt_pclk",
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
#endif
  {
    .name = "pd_fsm_clk",
    .parent_name = "top_pclk0",
    .en_offset = 0x090,
    .en_shift = 8,
  },
  {
    .name = "audio_sys_hclk",
    .parent_name = "audio_mclk_mx",
    .en_offset = 0x090,
    .en_shift = 12,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "audio_sys_clk_30720k",
    .parent_name = "audio_mclk_mx",
    .en_offset = 0x090,
    .en_shift = 13,
  },
  {
    .name = "topbus_ck803sclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x090,
    .en_shift = 14,
  },
  {
    .name = "ictl1_hclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x090,
    .en_shift = 15,
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
    .name = "rcpu0_wdt_tclk",
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
    .name = "rcpu0_wdt_pclk",
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
    .parent_name = "spi_pclk",
    .en_offset = 0x098,
    .en_shift = 11,
  },
  {
    .name = "spi1_pclk",
    .parent_name = "spi_pclk",
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
  {
    .name = "dolphin_adc_pclk",
    .parent_name = "codec_ref_clk",
    .en_offset = 0x0a0,
    .en_shift = 4,
  },
  {
    .name = "dolphin_vad_pclk",
    .parent_name = "vad_bus_clk",
    .en_offset = 0x0a0,
    .en_shift = 2,
  },
#endif
  {
    .name = "mailbox_hclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x09c,
    .en_shift = 0,
  },
  {
    .name = "vad_mclk_pll0",
    .parent_name = "vad_mclk_pll0_div",
    .en_offset = 0x0a0,
    .en_shift = 0,
  },
  {
    .name = "dolphin_adc_mclk",
    .parent_name = "codec_ref_clk",
    .en_offset = 0x0a0,
    .en_shift = 3,
  },
  {
    .name = "cpu_sys_icm_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a0,
    .en_shift = 8,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "bootrom_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a0,
    .en_shift = 9,
  },
  {
    .name = "cpu_sys_stclk",
    .parent_name = "clk32k",
    .en_offset = 0x0a0,
    .en_shift = 10,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm16_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a0,
    .en_shift = 11,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm17_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a0,
    .en_shift = 12,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm18_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a0,
    .en_shift = 13,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm19_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a0,
    .en_shift = 14,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "clk_cpu_sys",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a0,
    .en_shift = 15,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm0_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 0,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm1_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 1,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm2_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 2,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm3_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 3,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm4_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm5_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 5,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm6_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 6,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm7_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 7,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm8_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 8,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm9_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 9,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm10_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 10,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm11_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 11,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm12_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 12,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm13_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 13,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm14_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 14,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "tcm15_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x0a4,
    .en_shift = 15,
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
#endif /* (CONFIG_ARCH_CHIP_U2_AP) && (CONFIG_SONG_CLK) */
