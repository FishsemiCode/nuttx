/****************************************************************************
 * arch/arm/src/song/u11_ck_clk.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
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

#if defined(CONFIG_ARCH_CHIP_U11_CK) && defined(CONFIG_SONG_CLK)

#include <nuttx/clk/clk.h>
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct song_fixed_rate_clk fixed_rate[] =
{
  {
    .name = "clk32k",
    .fixed_rate = 32768,
  },
  {},
};

static const struct song_sdiv_sdiv_clk sdiv_sdiv[] =
{
  {
    .name = "spi0_mclk",
    .parent_name = "cp/pll1_mclk",
    .div_offset = 0x70,
    .div1_flags = 1 << CLK_DIVIDER_MINDIV_OFF,
    .div2_flags = 3 << CLK_DIVIDER_MINDIV_OFF,
  },
  {
    .name = "spi2_mclk",
    .parent_name = "cp/pll1_mclk",
    .div_offset = 0xc8,
    .div1_flags = 1 << CLK_DIVIDER_MINDIV_OFF,
    .div2_flags = 3 << CLK_DIVIDER_MINDIV_OFF,
  },
  {},
};

static const struct song_gr_fdiv_clk gr_fdiv[] =
{
  {
    .name = "i2s_mclk",
    .parent_name = "cp/pll1_mclk",
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
    .name = "i2c0_mclk",
    .parent_name = "cp/rfphy_clk38p4m_gated",
    .en_offset = 0x0b0,
    .en_shift = 0,
    .div_offset = 0x0b0,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "i2c1_mclk",
    .parent_name = "cp/rfphy_clk38p4m_gated",
    .en_offset = 0x0b4,
    .en_shift = 0,
    .div_offset = 0x0b4,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "pwm_mclk",
    .parent_name = "cp/rfphy_clk38p4m_gated",
    .en_offset = 0x304,
    .en_shift = 0,
    .div_offset = 0x304,
    .div_shift = 4,
    .div_width = 3,
    .div_flags = CLK_DIVIDER_ONE_BASED | CLK_DIVIDER_MAX_HALF,
  },
  {},
};

static const struct song_gate_clk gate[] =
{
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "pwm_pclk",
    .parent_name = "cp/top_pclk0",
    .en_offset = 0x098,
    .en_shift = 9,
  },
  {
    .name = "i2c0_pclk",
    .parent_name = "cp/top_pclk0",
    .en_offset = 0x098,
    .en_shift = 10,
  },
  {
    .name = "i2c1_pclk",
    .parent_name = "cp/top_pclk0",
    .en_offset = 0x098,
    .en_shift = 11,
  },
  {
    .name = "spi0_pclk",
    .parent_name = "cp/top_pclk0",
    .en_offset = 0x098,
    .en_shift = 12,
  },
  {
    .name = "spi2_pclk",
    .parent_name = "cp/top_pclk0",
    .en_offset = 0x09c,
    .en_shift = 9,
  },
  {
    .name = "i2s_pclk",
    .parent_name = "cp/top_pclk0",
    .en_offset = 0x09c,
    .en_shift = 14,
  },
#endif
  {},
};

static const struct song_clk_table u11_ck_clk_tbl =
{
  .fixed_rate_clks   = fixed_rate,
  .gr_fdiv_clks      = gr_fdiv,
  .div_clks          = div,
  .sdiv_sdiv_clks    = sdiv_sdiv,
  .gate_clks         = gate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xb0040000, &u11_ck_clk_tbl);
}

void up_clk_finalinitialize(void)
{
  clk_disable_unused();
}
#endif /* defined(CONFIG_ARCH_CHIP_U11_CK) && defined(CONFIG_SONG_CLK) */
