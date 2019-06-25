/****************************************************************************
 * arch/risc-v/src/song/u3_ap_clk.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: zhuyanlin <zhuyanlin@fishsemi.com>
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

#if defined(CONFIG_ARCH_CHIP_U3_AP) && defined(CONFIG_SONG_CLK)

#include <nuttx/clk/clk.h>
#include <nuttx/clk/song/song-clk.h>

#include "chip.h"

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const char *timer_out_clk_src[] =
{
  "sys_clk",
  "top_bus_mclk",
  "pll0_mclk",
  "usb_clk",
  "pll0_mclk",
  "clk32k",
};

static const char *rf_port_src[] =
{
  "sys_clk",
  "pll0_mclk",
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
    .name = "sys_clk",
    .fixed_rate = 38400000,
  },
  {},
};

static const struct song_fixed_factor_clk fixed_factor[] =
{
  {
    .name = "usb_clk_ori",
    .parent_name = "pll1",
    .fixed_mult = 1,
    .fixed_div = 16,
  },
  {
    .name = "psram_clk_2x",
    .parent_name = "psram_clk",
    .fixed_mult = 2,
    .fixed_div = 1,
  },
  {},
}

static const struct song_sdiv_gr_clk sdiv_gr[] =
{
  {
    .name = "ceva_free_clk",
    .parent_name = "pll1",
    .div_offset = 0x2e4,
    .div_width = 4,
  },
  {},
};

static const struct song_pll_clk pll[] =
{
  {
    .name = "pll0",
    .parent_name = "sys_clk",
    .cfg0_offset = 0x30,
    .cfg1_offset = 0x34,
    .ctl_offset = 0xbc,
    .ctl_shift = 0,
  },
  {
    .name = "pll1_ori",
    .parent_name = "sys_clk",
    .cfg0_offset = 0x38,
    .cfg1_offset = 0x3c,
    .ctl_offset = 0x40,
    .ctl_shift = 0,
  },
  {},
};

static const struct song_timer_clk timer[] =
{
  {
    .name = "timer0",
    .parent_names = timer_out_clk_src,
    .num_parents = ARRAY_SIZE(timer_out_clk_src),
    .ctl_offset = 0x2b0,
  },
  {
    .name = "timer1",
    .parent_names = timer_out_clk_src,
    .num_parents = ARRAY_SIZE(timer_out_clk_src),
    .ctl_offset = 0x2b4,
  },
  {},
};

static const struct song_out_clk out[] =
{
  {
    .name = "out0",
    .parent_names = timer_out_clk_src,
    .num_parents = ARRAY_SIZE(timer_out_clk_src),
    .mux_offset = 0x2b8,
    .ctl_offset = 0x2bc,
    .mux_shift = 0,
  },
  {
    .name = "out1",
    .parent_names = timer_out_clk_src,
    .num_parents = ARRAY_SIZE(timer_out_clk_src),
    .mux_offset = 0x2b8,
    .ctl_offset = 0x1cc,
    .mux_shift = 4,
  },
  {},
};

static const struct song_mux_clk mux_div[] =
{
  {
    .name = "rf_port",
    .parent_names = rf_port_src,
    .num_parents = ARRAY_SIZE(rf_port_src),
    .en_offset = 0x2c4,
    .en_shift = 0,
    .mux_offset = 0x2c4,
    .mux_shift = 11,
    .mux_width = 1,
    .div_offset = 0x2c4,
    .div_shift = 4,
    .div_width = 5,
  },
  {},
};

static const struct song_gr_fdiv_clk gr_fdiv[] =
{
  {
    .name = "uart0_tx_clk",
    .parent_name = "pll0_mclk",
    .en_offset = 0x98,
    .en_shift = 4,
    .div_offset = 0x80,
  },
  {
    .name = "uart1_clk",
    .parent_name = "pll0_mclk",
    .en_offset = 0x9c,
    .en_shift = 13,
    .gr_offset = 0x0,
    .div_offset = 0x84,
  },
  {
    .name = "uart2_clk",
    .parent_name = "pll0_mclk",
    .en_offset = 0x9c,
    .en_shift = 15,
    .div_offset = 0x88,
  },
  {
    .name = "i2s0_mclk",
    .parent_name = "pll0_mclk",
    .en_offset = 0x90,
    .en_shift = 0,
    .div_offset = 0xb8,
  },
  {
    .name = "i2s1_mclk",
    .parent_name = "pll0_mclk",
    .en_offset = 0x90,
    .en_shift = 1,
    .div_offset = 0xc4,
  },
  {},
};

static const struct song_sdiv_sdiv_clk sdiv_sdiv[] =
{
  {
    .name = "spi0_mclk",
    .parent_name = "pll0_mclk",
    .div_offset = 0x70,
  },
  {
    .name = "spi1_mclk",
    .parent_name = "pll0_mclk",
    .div_offset = 0x74,
  },
  {
    .name = "spi2_mclk",
    .parent_name = "pll0_mclk",
    .div_offset = 0xc8,
  },
  {
    .name = "spi3_mclk",
    .parent_name = "pll0_mclk",
    .div_offset = 0x308,
  },
  {},
};

static const struct song_div_clk div[] =
{
  {
    .name = "pll0_mclk",
    .parent_name = "pll0",
    .div_offset = 0x50,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "top_bus_mclk",
    .parent_name = "pll1",
    .div_offset = 0x44,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "top_peribus_mclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x098,
    .en_shift = 5,
    .div_offset = 0x048,
    .div_shift = 4,
    .div_width = 3,
  },
  {
    .name = "cp_bus_mclk",
    .parent_name = "pll1",
    .en_offset = 0x058,
    .en_shift = 0,
    .div_offset = 0x058,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "sim_clk",
    .parent_name = "sys_clk",
    .en_offset = 0x064,
    .en_shift = 0,
    .div_offset = 0x064,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "sim_hi_clk",
    .parent_name = "pll0_mclk",
    .en_offset = 0x068,
    .en_shift = 0,
    .div_offset = 0x068,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "at_clk",
    .parent_name = "sys_clk",
    .en_offset = 0x06c,
    .en_shift = 0,
    .div_offset = 0x06c,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "pwm_mclk",
    .parent_name = "sys_clk",
    .en_offset = 0x304,
    .en_shift = 0,
    .div_offset = 0x304,
    .div_shift = 4,
    .div_width = 3,
    .div_flags = CLK_DIVIDER_POWER_OF_TWO,
  },
  {
    .name = "bus_lp_pulse",
    .parent_name = "sys_clk",
    .en_offset = 0x0cc,
    .en_shift = 0,
    .div_offset = 0x0cc,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "pll_adj_clk",
    .parent_name = "sys_clk",
    .en_offset = 0xa0,
    .en_shift = 0,
    .div_offset = 0xa0,
    .div_shift = 4,
    .div_width = 3,
    .div_flags = CLK_DIVIDER_POWER_OF_TWO,
  },
  {
    .name = "i2c0_mclk",
    .parent_name = "pll0_mclk",
    .en_offset = 0x0b0,
    .en_shift = 0,
    .div_offset = 0x0b0,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "i2c1_mclk",
    .parent_name = "pll0_mclk",
    .en_offset = 0x0b4,
    .en_shift = 0,
    .div_offset = 0x0b4,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "i2c2_mclk",
    .parent_name = "pll0_mclk",
    .en_offset = 0x2f4,
    .en_shift = 0,
    .div_offset = 0x2f4,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rfif_spi_clk",
    .parent_name = "sys_clk",
    .en_offset = 0x2e0,
    .en_shift = 0,
    .div_offset = 0x2e0,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rf_tm2_calib",
    .parent_name = "pll0_mclk",
    .en_offset = 0x02c8,
    .en_shift = 0,
    .div_offset = 0x02c8,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rf_ltesp",
    .parent_name = "pll0_mclk",
    .en_offset = 0x2cc,
    .en_shift = 0,
    .div_offset = 0x2cc,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rfif_rffe",
    .parent_name = "pll0_mclk",
    .en_offset = 0x2d0,
    .en_shift = 0,
    .div_offset = 0x2d0,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rfif_match_rx",
    .parent_name = "sys_clk",
    .en_offset = 0x2e8,
    .en_shift = 0,
    .div_offset = 0x2e8,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "rfif_match_tx",
    .parent_name = "sys_clk",
    .en_offset = 0x2ec,
    .en_shift = 0,
    .div_offset = 0x2ec,
    .div_shift = 4,
    .div_width = 5,
  },
  {
    .name = "cp_rocket",
    .parent_name = "pll1",
    .en_offset = 0x0c0,
    .en_shift = 0,
    .div_offset = 0x0c0,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "top_rocket",
    .parent_name = "pll1",
    .en_offset = 0x04c,
    .en_shift = 8,
    .div_offset = 0x04c,
    .div_shift = 4,
    .div_width = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "gnss_clk",
    .parent_name = "pll1",
    .en_offset = 0x078,
    .en_shift = 0,
    .div_offset = 0x078,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "cp_ecs",
    .parent_name = "pll1",
    .en_offset = 0x05c,
    .en_shift = 0,
    .div_offset = 0x05c,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "wdt_dly_cnt",
    .parent_name = "clk32k",
    .en_offset = 0x2f8,
    .en_shift = 0,
    .div_offset = 0x2f8,
    .div_shift = 4,
    .div_width = 3,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "top_bus_apb_top",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x60,
    .en_shift = 0,
    .div_offset = 0x60,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "top_bus_apb_aon",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x7c,
    .en_shift = 0,
    .div_offset = 0x7c,
    .div_shift = 4,
    .div_width = 4,
  },
  {
    .name = "psram_clk",
    .parent_name = "pll1",
    .en_offset = 0x16c,
    .en_shift = 0,
    .div_offset = 0x16c,
    .div_shift = 4,
    .div_width = 5,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {},
};

static const struct song_gate_clk gate[] =
{
  {
    .name = "security_32k",
    .parent_name = "clk32k",
    .en_offset = 0x90,
    .en_shift = 2,
  },
  {
    .name = "usb_clk",
    .parent_name = "usb_clk_ori",
    .en_offset = 0x90,
    .en_shift = 3,
  },
  {
    .name = "mailbox_hclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x90,
    .en_shift = 4,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "dmas_hclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x90,
    .en_shift = 5,
  },
  {
    .name = "top_cipher_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x90,
    .en_shift = 6,
  },
  {
    .name = "security_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x90,
    .en_shift = 7,
  },
  {
    .name = "usb_bus_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x90,
    .en_shift = 8,
  },
  {
    .name = "lcdc_clk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x90,
    .en_shift = 9,
  },
  {
    .name = "top_bus_shram0_mclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x90,
    .en_shift = 10,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "top_bus_shram1_mclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x90,
    .en_shift = 11,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
  {
    .name = "gpio_clk32k",
    .parent_name = "clk32k",
    .en_offset = 0x90,
    .en_shift = 12,
  },
  {
    .name = "rf_tp_clk32k",
    .parent_name = "clk32k",
    .en_offset = 0x90,
    .en_shift = 13,
  },
  {
    .name = "rfif_bus_clk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x90,
    .en_shift = 14,
  },
  {
    .name = "viterbi_clk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 0,
  },
  {
    .name = "dlharq_clk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 1,
  },
  {
    .name = "dlsp_clk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 2,
  },
  {
    .name = "ulbsp_clk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 3,
  },
  {
    .name = "pdcp_clk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 4,
  },
  {
    .name = "dmag_clk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 5,
  },
  {
    .name = "cp_bus_shram_mclk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 6,
  },
  {
    .name = "wdt0_tclk",
    .parent_name = "clk32k",
    .en_offset = 0x94,
    .en_shift = 7,
  },
  {
    .name = "wdt1_tclk",
    .parent_name = "clk32k",
    .en_offset = 0x94,
    .en_shift = 8,
  },
  {
    .name = "wdt2_tclk",
    .parent_name = "clk32k",
    .en_offset = 0x94,
    .en_shift = 9,
  },
  {
    .name = "psram_aclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 10,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "psram_pclk",
    .parent_name = "top_bus_mclk",
    .en_offset = 0x94,
    .en_shift = 11,
  },
  {
    .name = "spi0_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x98,
    .en_shift = 0,
  },
  {
    .name = "spi1_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x98,
    .en_shift = 1,
  },
  {
    .name = "i2s0_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x98,
    .en_shift = 2,
  },
  {
    .name = "i2s1_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x98,
    .en_shift = 3,
  },
#endif
  {
    .name = "pll1",
    .parent_name = "pll1_ori",
    .en_offset = 0x98,
    .en_shift = 6,
  },
  {
    .name = "xc5_ictl_clk",
    .parent_name = "cp_bus_mclk",
    .en_offset = 0x98,
    .en_shift = 9,
    .clk_flags = CLK_IGNORE_UNUSED,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "sim_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x98,
    .en_shift = 7,
  },
  {
    .name = "i2c2_pclk",
    .parent_name = "top_bus_apb_top",
    .en_offset = 0x98,
    .en_shift = 10,
  },
  {
    .name = "spi2_pclk",
    .parent_name = "top_bus_apb_top",
    .en_offset = 0x98,
    .en_shift = 12,
  },
  {
    .name = "spi3_pclk",
    .parent_name = "top_bus_apb_top",
    .en_offset = 0x98,
    .en_shift = 13,
  },
  {
    .name = "pmicfsm_pclk",
    .parent_name = "top_bus_apb_aon",
    .en_offset = 0x98,
    .en_shift = 14,
  },
  {
    .name = "uart0_pclk",
    .parent_name = "top_bus_apb_aon",
    .en_offset = 0x9c,
    .en_shift = 0,
  },
  {
    .name = "rtc_pclk",
    .parent_name = "top_bus_apb_aon",
    .en_offset = 0x9c,
    .en_shift = 1,
  },
#endif
  {
    .name = "i2c_ahb_hclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 2,
  },
#ifdef CONFIG_DEBUG_SONG_CLK
  {
    .name = "muxpin_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 3,
  },
  {
    .name = "gpio_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 4,
  },
  {
    .name = "wdt0_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 5,
  },
  {
    .name = "wdt1_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 6,
  },
  {
    .name = "wdt2_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 7,
  },
  {
    .name = "timer_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 8,
  },
  {
    .name = "i2c0_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 9,
  },
  {
    .name = "i2c1_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 10,
  },
  {
    .name = "pwm_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 11,
  },
  {
    .name = "uart1_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 12,
  },
  {
    .name = "uart2_pclk",
    .parent_name = "top_peribus_mclk",
    .en_offset = 0x9c,
    .en_shift = 14,
  },
#endif
  {
    .name = "gnss_hclk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 0,
  },
  {
    .name = "gnss_pp_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 1,
  },
  {
    .name = "gnss_ae_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 2,
  },
  {
    .name = "gnss_ae_fifo_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 3,
  },
  {
    .name = "gnss_conf_mem_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 4,
  },
  {
    .name = "gnss_te_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 5,
  },
  {
    .name = "gnss_te_mem_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 6,
  },
  {
    .name = "gnss_te_fifo_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 7,
  },
  {
    .name = "gnss_i_rf0_clk",
    .parent_name = "gnss_clk",
    .en_offset = 0x2f0,
    .en_shift = 8,
  },
  {},
};

static const struct song_clk_table clk_tbl =
{
  .fixed_rate_clks   = fixed_rate,
  .fixed_factor_clks = fixed_factor,
  .sdiv_gr_clks      = sdiv_gr,
  .pll_clks          = pll,
  .out_clks          = out,
  .timer_clks        = timer,
  .mux_div_clks      = mux_div,
  .gr_fdiv_clks      = gr_fdiv,
  .sdiv_sdiv_clks    = sdiv_sdiv,
  .div_clks          = div,
  .gate_clks         = gate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_clk_initialize(void)
{
  song_clk_initialize(0xb0040000, &clk_tbl);
}

void up_clk_finalinitialize(void)
{
  clk_disable_unused();
}
#endif /* (CONFIG_ARCH_CHIP_U3_AP) && (CONFIG_SONG_CLK) */
