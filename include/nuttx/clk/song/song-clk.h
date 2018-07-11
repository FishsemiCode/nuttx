/****************************************************************************
 * include/nuttx/clk/song/song-clk.h
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
 *   Author: zhuyanlin<zhuyanlin@pinecone.net>
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

#ifndef __INCLUDE_NUTTX_CLK_SONG_SONG_CLK_H
#define __INCLUDE_NUTTX_CLK_SONG_SONG_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#ifdef CONFIG_SONG_CLK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* This descibes clk flags used arcoss song-clk wrapper */
#define SONG_CLK_GR_DIV_16              0x01

/* This descibes individual clk flag shift among song-clk wrapper */
#define SONG_CLK_GATE_FLAG_SHIFT        0
#define SONG_CLK_DIV_FLAG_SHIFT         8
#define SONG_CLK_MULT_FLAG_SHIFT        16
#define SONG_CLK_FRAC_FLAG_SHIFT        24
#define SONG_CLK_MUX_FLAG_SHIFT         32
#define SONG_CLK_PHASE_FLAG_SHIFT       40
#define SONG_CLK_PRIVATE_FLAG_SHIFT     48

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/* This descibes clk struct defined by song-clk wrapper */
struct song_fixed_rate_clk
{
  const char         *name;
  uint32_t           fixed_rate;
  uint16_t           flags;
};

struct song_fixed_factor_clk
{
  const char         *name;
  const char         *parent_name;
  uint8_t            fixed_mul;
  uint8_t            fixed_div;
  uint16_t           flags;
};

struct song_gate_clk
{
  const char         *name;
  const char         *parent_name;
  uint16_t           en_offset;
  uint16_t           flags;
  uint8_t            en_shift;
  uint64_t           private_flags;
};

struct song_sdiv_clk
{
  const char         *name;
  const char         *parent_name;
  uint16_t           en_offset;
  uint16_t           div_offset;
  uint8_t            en_shift;
  uint8_t            div_shift;
  uint8_t            div_width;
  uint16_t           flags;
  uint64_t           private_flags;
};

struct song_gr_clk
{
  const char         *name;
  const char         *parent_name;
  uint16_t           en_offset;
  uint16_t           mul_offset;
  uint8_t            en_shift;
  uint8_t            mul_shift;
  uint8_t            mul_width;
  uint16_t           flags;
  uint64_t           private_flags;
};

struct song_sdiv_sdiv_clk
{
  const char         *name;
  const char         *parent_name;
  uint16_t           flags;
  uint16_t           div_offset;
  uint64_t           private_flags;
};

struct song_sdiv_fdiv_clk
{
  const char         *name;
  const char         *parent_name;
  uint16_t           en_offset;
  uint16_t           sdiv_offset;
  uint16_t           fdiv_offset;
  uint16_t           flags;
  uint8_t            en_shift;
  uint8_t            sdiv_shift;
  uint8_t            sdiv_width;
  uint64_t           private_flags;
};

struct song_gr_fdiv_clk
{
  const char         *name;
  const char         *parent_name;
  uint16_t           en_offset;
  uint16_t           gr_offset;
  uint16_t           div_offset;
  uint8_t            en_shift;
  uint8_t            fixed_gr;
  uint16_t           flags;
  uint64_t           private_flags;
};

struct song_sdiv_gr_clk
{
  const char         *name;
  const char         *parent_name;
  uint16_t           flags;
  uint16_t           div_offset;
  uint64_t           private_flags;
};

struct song_mux_sdiv_clk
{
  const char         *name;
  const char * const *parent_name;
  uint16_t           mux_offset;
  uint16_t           div_offset;
  uint8_t            mux_shift;
  uint8_t            mux_width;
  uint8_t            div_shift;
  uint8_t            div_width;
  uint8_t            num_parents;
  uint16_t           flags;
  uint64_t           private_flags;
};

struct song_mux_gate_clk
{
  const char         *name;
  const char * const *parent_name;
  uint16_t           en_offset;
  uint16_t           mux_offset;
  uint8_t            en_shift;
  uint8_t            mux_shift;
  uint8_t            mux_width;
  uint8_t            num_parents;
  uint16_t           flags;
  uint64_t           private_flags;
};

struct song_phase_clk
{
  const char         *name;
  const char         *parent_name;
  uint16_t           reg_offset;
  uint16_t           flags;
  uint8_t            phase_shift;
  uint8_t            phase_width;
  uint8_t            phase_flags;
};

struct song_mux_sdiv_gr_clk
{
  const char         *name;
  const char * const *parent_name;
  uint16_t           div_offset;
  uint8_t            div_width;
  uint8_t            mux_width;
  uint8_t            num_parents;
  uint16_t           flags;
  uint64_t           private_flags;
};

struct song_pll_clk
{
  const char         *name;
  const char         *parent_name;
  uint16_t           cfg_reg0_offset;
  uint16_t           cfg_reg1_offset;
  uint16_t           ctl_reg_offset;
  uint8_t            ctl_shift;
  uint16_t           flags;
};

struct song_pll_lf_clk
{
  const char         *name;
  const char         *parent_name;
  uint16_t           cfg_reg0_offset;
  uint16_t           cfg_reg1_offset;
  uint16_t           flags;
};

struct song_out_clk
{
  const char         *name;
  const char * const *parent_name;
  uint16_t           mux_reg_offset;
  uint16_t           ctl_reg_offset;
  uint8_t            mux_shift;
  uint8_t            mux_width;
  uint8_t            num_parents;
};

struct song_timer_clk
{
  const char         *name;
  const char * const *parent_name;
  uint16_t           ctl_reg_offset;
  uint8_t            mux_shift;
  uint8_t            mux_width;
  uint8_t            num_parents;
};

struct song_default_rate_clk
{
  const char         *name;
  uint32_t           rate;
};

struct song_clk_table
{
  const struct song_fixed_rate_clk *fixed_rate_clks;
  const struct song_gate_clk *gate_clks;
  const struct song_sdiv_clk *sdiv_clks;
  const struct song_gr_clk *gr_clks;
  const struct song_fixed_factor_clk *fixed_factor_clks;
  const struct song_sdiv_sdiv_clk *sdiv_sdiv_clks;
  const struct song_sdiv_fdiv_clk *sdiv_fdiv_clks;
  const struct song_gr_fdiv_clk *gr_fdiv_clks;
  const struct song_sdiv_gr_clk *sdiv_gr_clks;
  const struct song_sdiv_gr_clk *gr_sdiv_clks;
  const struct song_mux_sdiv_clk *mux_sdiv_clks;
  const struct song_mux_sdiv_gr_clk *mux_sdiv_gr_clks;
  const struct song_mux_gate_clk *mux_gate_clks;
  const struct song_phase_clk *phase_clks;
  const struct song_pll_clk *pll_clks;
  const struct song_pll_lf_clk *pll_lf_clks;
  const struct song_out_clk *out_clks;
  const struct song_timer_clk *timer_clks;
  const struct song_default_rate_clk *def_rate;
  bool rpmsg_server;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: song_clk_initialize
 *
 * Description:
 *   Create a set of clk driver instances for the song platform.
 *   This will initialize the clk follow the configuration table.
 *
 * Input Parameters:
 *   base      - the clk controller base address.
 *   table     - the song platform clk table that to be configurated.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int song_clk_initialize(uint32_t base, const struct song_clk_table *table);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_SONG_CLK */
#endif /* __INCLUDE_NUTTX_CLK_SONG_SONG_CLK_H */

