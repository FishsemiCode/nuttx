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

#include <nuttx/clk/clk-provider.h>
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

#if defined(CONFIG_ARCH_CHIP_U1_SP) && defined(CONFIG_SONG_CLK)

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This descibes mux clk parent source */
static const char * const timer_clk_src[] =
{
  "cp/gnss_i_rf0_clk",
  "pll0_out",
  "pll0_out",
  "clk32k",
};

static const char * const out_clk_src[] =
{
  "cp/gnss_i_rf0_clk",
  "top_bus_mclk0",
  "pll1_mclk",
  "pll0_out",
  "clk32k",
  "pll0_out",
};

/* This descibes unicorn 32k osc clk */
static const struct song_fixed_rate_clk fixed_rate[] =
{
  {
    .name = "clk32k",
    .flags = CLK_IS_ROOT,
    .fixed_rate = 32768,
  },
  {},
};

/* This descibes unicorn two pll */
static const struct song_pll_lf_clk pll_lf[] =
{
  {
    .name = "pll0_out",
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
    .name = "pll1_out_ori",
    .parent_name = "pll0_out",
    .flags = 0,
    .cfg_reg0_offset = 0x38,
    .cfg_reg1_offset = 0x3c,
    .ctl_reg_offset = 0x40,
    .ctl_shift = 0,
  },
  {},
};

static const struct song_fixed_factor_clk fixed_factor[] =
{
  {
    .name = "pll0_out_div2",
    .parent_name = "pll0_out",
    .flags = 0,
    .fixed_mul = 1,
    .fixed_div = 2,
  },
  {},
};

static const struct song_timer_clk timer[] =
{
  {
    .name = "timer0",
    .parent_name = timer_clk_src,
    .num_parents = ARRAY_SIZE(timer_clk_src),
    .ctl_reg = 0x2b0,
    .mux_shift = 28,
    .mux_width = 3,
  },
  {
    .name = "timer1",
    .parent_name = timer_clk_src,
    .num_parents = ARRAY_SIZE(timer_clk_src),
    .ctl_reg = 0x2b4,
    .mux_shift = 28,
    .mux_width = 3,
  },
  {},
};

static const struct song_out_clk out[] =
{
  {
    .name = "out0",
    .parent_name = out_clk_src,
    .num_parents = ARRAY_SIZE(out_clk_src),
    .mux_reg = 0x2b8,
    .ctl_reg = 0x2bc,
    .mux_shift = 0,
    .mux_width = 3,
  },
  {},
};

static const struct song_gr_fdiv_clk gr_fdiv[] =
{
  {
    .name = "uart0_tx_clk",
    .parent_name = "pll0_out",
    .flags = 0,
    .en_offset = 0x9c,
    .en_shift = 3,
    .gr_offset = 0x0,
    .div_offset = 0x80,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     (CLK_FRAC_MUL_NEED_EVEN << SONG_CLK_FRAC_FLAG_SHIFT)),
  },
  {
    .name = "uart1_clk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x9c,
    .en_shift = 11,
    .gr_offset = 0x0,
    .div_offset = 0x84,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     (CLK_FRAC_MUL_NEED_EVEN << SONG_CLK_FRAC_FLAG_SHIFT)),
  },
  {
    .name = "uart2_clk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x9c,
    .en_shift = 13,
    .gr_offset = 0x0,
    .div_offset = 0x88,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     (CLK_FRAC_MUL_NEED_EVEN << SONG_CLK_FRAC_FLAG_SHIFT)),
  },
  {
    .name = "uart3_clk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x9c,
    .en_shift = 15,
    .gr_offset = 0x0,
    .div_offset = 0x8c,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     (CLK_FRAC_MUL_NEED_EVEN << SONG_CLK_FRAC_FLAG_SHIFT)),
  },
  {},
};

static const struct song_gr_clk gr[] =
{
  {
    .name = "sec_m4_cti_clk",
    .parent_name = "sec_m4_clk",
    .flags = 0,
    .en_offset = 0x7c,
    .en_shift = 0,
    .mul_offset = 0x7c,
    .mul_shift = 4,
    .mul_width = 3,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     (CLK_MULT_HIWORD_MASK << SONG_CLK_MULT_FLAG_SHIFT)),
  },
  {
    .name = "ap_m4_cti_clk",
    .parent_name = "ap_m4_clk",
    .flags = 0,
    .en_offset = 0xbc,
    .en_shift = 0,
    .mul_offset = 0xbc,
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
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "spi1_mclk",
    .parent_name = "pll1_mclk",
    .div_offset = 0x74,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "spi2_mclk",
    .parent_name = "pll1_mclk",
    .div_offset = 0xc8,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                     ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {},
};

static const struct song_sdiv_clk sdiv[] =
{
  {
    .name = "pll_adj_clk",
    .parent_name = "pll0_out_div2",
    .flags = 0,
    .en_offset = 0x0a0,
    .en_shift = 0,
    .div_offset = 0x0a0,
    .div_shift = 4,
    .div_width = 3,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (((uint64_t)CLK_DIVIDER_HIWORD_MASK | CLK_DIVIDER_ONE_BASED |
                       CLK_DIVIDER_MAX_HALF) << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "top_bus_mclk",
    .parent_name = "pll1_out",
    .flags = 0,
    .en_offset = 0x044,
    .en_shift = 0,
    .div_offset = 0x044,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
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
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
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
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "pll1_mclk",
    .parent_name = "pll1_out",
    .flags = 0,
    .en_offset = 0x050,
    .en_shift = 0,
    .div_offset = 0x050,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "top_pclk2",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x054,
    .en_shift = 0,
    .div_offset = 0x054,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "at_clk",
    .parent_name = "pll0_out",
    .flags = 0,
    .en_offset = 0x06c,
    .en_shift = 0,
    .div_offset = 0x06c,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "gnss_clk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x078,
    .en_shift = 0,
    .div_offset = 0x078,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "i2c0_mclk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x0b0,
    .en_shift = 0,
    .div_offset = 0x0b0,
    .div_shift = 4,
    .div_width = 5,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "i2c1_mclk",
    .parent_name = "pll1_mclk",
    .flags = 0,
    .en_offset = 0x0b4,
    .en_shift = 0,
    .div_offset = 0x0b4,
    .div_shift = 4,
    .div_width = 5,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "bus_lp_clk",
    .parent_name = "pll0_out",
    .flags = 0,
    .en_offset = 0x0cc,
    .en_shift = 0,
    .div_offset = 0x0cc,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "flash_ctrl_clk",
    .parent_name = "pll1_out",
    .flags = 0,
    .en_offset = 0x2c4,
    .en_shift = 0,
    .div_offset = 0x2c4,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "sec_m4_bus_clk",
    .parent_name = "pll1_out",
    .flags = 0,
    .en_offset = 0x2c8,
    .en_shift = 0,
    .div_offset = 0x2c8,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "ap_m4_bus_clk",
    .parent_name = "pll1_out",
    .flags = 0,
    .en_offset = 0x2cc,
    .en_shift = 0,
    .div_offset = 0x2cc,
    .div_shift = 4,
    .div_width = 4,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "pwm_mclk",
    .parent_name = "pll0_out_div2",
    .flags = 0,
    .en_offset = 0x304,
    .en_shift = 0,
    .div_offset = 0x304,
    .div_shift = 4,
    .div_width = 3,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      (((uint64_t)CLK_DIVIDER_HIWORD_MASK | CLK_DIVIDER_ONE_BASED |
                       CLK_DIVIDER_MAX_HALF) << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {
    .name = "wdt_dly_cnt_clk",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x2f8,
    .en_shift = 0,
    .div_offset = 0x2f8,
    .div_shift = 4,
    .div_width = 3,
    .private_flags = ((CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT) |
                      ((uint64_t)CLK_DIVIDER_HIWORD_MASK << SONG_CLK_DIV_FLAG_SHIFT)),
  },
  {},
};

static const struct song_gate_clk gate[] =
{
  {
    .name = "sec_m4_clk",
    .parent_name = "sec_m4_bus_clk",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 0,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sec_m4_wic_clk",
    .parent_name = "sec_m4_clk",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 1,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "swdt_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 2,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "swdt_tclk",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 3,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sec_tcm_icm_hclk",
    .parent_name = "sec_m4_clk",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 4,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "dmas_hclk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 5,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "pll1_out",
    .parent_name = "pll1_out_ori",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 6,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gpio_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 7,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "gpio_clk32k",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 8,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "pwm_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 9,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "i2c0_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 10,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "i2c1_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 11,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "spi0_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 12,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "spi1_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 13,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "muxpin_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 14,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "timer_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x098,
    .en_shift = 15,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "pmicfsm_pclk",
    .parent_name = "top_pclk1",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 0,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "rtc_pclk",
    .parent_name = "top_pclk1",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 1,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "uart0_pclk",
    .parent_name = "top_pclk1",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 2,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "flash_ctrl_hclk",
    .parent_name = "flash_ctrl_clk",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 4,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "top_cipher_clk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 5,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "mailbox_hclk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 6,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "security_clk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 7,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "security_clk32k",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 8,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "spi2_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 9,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "uart1_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 10,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "uart2_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 12,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "uart3_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x09c,
    .en_shift = 14,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "topbus_shram0clk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 0,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "topbus_shram1clk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 1,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "topbus_sectcmclk",
    .parent_name = "sec_m4_bus_clk",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 2,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "topbus_aptcmclk",
    .parent_name = "ap_m4_bus_clk",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 3,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "topbus_debugclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 4,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "cs_dsp_clk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 8,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sec_m4_dapclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 9,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sec_m4_stclk",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 10,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "ap_m4_dapclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 11,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "cp_m4_dapclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 12,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "i2c_ahb_hclk",
    .parent_name = "top_bus_mclk",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 13,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "sec_m4_icache_clk",
    .parent_name = "sec_m4_clk",
    .flags = 0,
    .en_offset = 0x0b8,
    .en_shift = 14,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "ap_m4_clk",
    .parent_name = "ap_m4_bus_clk",
    .flags = 0,
    .en_offset = 0x0c4,
    .en_shift = 0,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "ap_m4_wic_clk",
    .parent_name = "ap_m4_clk",
    .flags = 0,
    .en_offset = 0x0c4,
    .en_shift = 1,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "ap_m4_stclk",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x0c4,
    .en_shift = 2,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "apwdt_pclk",
    .parent_name = "top_pclk0",
    .flags = 0,
    .en_offset = 0x0c4,
    .en_shift = 3,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "apwdt_tclk",
    .parent_name = "clk32k",
    .flags = 0,
    .en_offset = 0x0c4,
    .en_shift = 4,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "ap_tcm_icm_hclk",
    .parent_name = "ap_m4_clk",
    .flags = 0,
    .en_offset = 0x0c4,
    .en_shift = 5,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {
    .name = "ap_m4_icache_clk",
    .parent_name = "ap_m4_clk",
    .flags = 0,
    .en_offset = 0x0c4,
    .en_shift = 6,
    .private_flags = (CLK_GATE_HIWORD_MASK << SONG_CLK_GATE_FLAG_SHIFT),
  },
  {},
};

static const struct song_clk_table u1_sp_clk_tbl =
{
  .fixed_rate_clks   = fixed_rate,
  .fixed_factor_clks = fixed_factor,
  .gr_clks           = gr,
  .gr_fdiv_clks      = gr_fdiv,
  .sdiv_clks         = sdiv,
  .sdiv_sdiv_clks    = sdiv_sdiv,
  .gate_clks         = gate,
  .pll_clks          = pll,
  .pll_lf_clks       = pll_lf,
  .out_clks          = out,
  .timer_clks        = timer,
  .rpmsg_server      = true,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xb0040000, &u1_sp_clk_tbl);
}

#endif /* CONFIG_ARCH_CHIP_U1_SP */
