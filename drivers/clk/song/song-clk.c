/****************************************************************************
 * drivers/clk/song/song-clk.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/clk/clk.h>
#include <nuttx/clk/song/song-clk.h>
#include <stdio.h>

#include "song-clk.h"

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static struct clk *song_clk_register_mux_div(const char *name,
            const char * const *parent_names, uint8_t num_parents,
            uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
            uint16_t mux_offset, uint8_t mux_shift, uint8_t mux_width,
            uint16_t div_offset, uint8_t div_shift, uint8_t div_width,
            uint16_t div_flags);
static struct clk *song_clk_register_mux(const char *name,
            const char * const *parent_names, uint8_t num_parents,
            uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
            uint16_t mux_offset, uint8_t mux_shift, uint8_t mux_width,
            uint8_t mux_flags);
static struct clk *song_clk_register_gr(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
            uint16_t mult_offset, uint8_t mult_shift, uint8_t mult_width,
            uint8_t clk_flags, uint8_t gr_flags);
static struct clk *song_clk_register_sdiv_fdiv(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t sdiv_offset, uint16_t fdiv_offset);
static struct clk *song_clk_register_gr_fdiv(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
            uint16_t gr_offset, uint16_t div_offset, uint8_t clk_flags);
static struct clk *song_clk_register_div(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
            uint16_t div_offset, uint8_t div_shift, uint8_t div_width,
            uint16_t div_flags, uint8_t clk_flags);
static struct clk *song_clk_register_sdiv_gr(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t div_offset, uint8_t div_width,
            uint16_t div_flags);
static struct clk *song_clk_register_sdiv_sdiv(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t div_offset,
            uint16_t div1_flags, uint16_t div2_flags);
static struct clk *song_clk_register_gr_sdiv(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t div_offset, uint8_t div_width,
            uint16_t div_flags);
static struct clk *song_clk_register_mux_sdiv_gr(const char *name,
            const char * const *parent_names, uint8_t num_parents,
            uint32_t reg_base, uint16_t div_offset, uint8_t div_width,
            uint8_t mux_width);

static int song_register_fixed_rate_clks(
            const struct song_fixed_rate_clk *fixed_rate_clks);
static int song_register_fixed_factor_clks(
            const struct song_fixed_factor_clk *fixed_factor_clks);
static int song_register_gate_clks(uint32_t reg_base,
            const struct song_gate_clk *gate_clks);
static int song_register_div_clks(uint32_t reg_base,
            const struct song_div_clk *div_clks);
static int song_register_gr_clks(uint32_t reg_base,
            const struct song_gr_clk *gr_clks);
static int song_register_sdiv_sdiv_clks(uint32_t reg_base,
            const struct song_sdiv_sdiv_clk *sdiv_sdiv_clks);
static int song_register_sdiv_fdiv_clks(uint32_t reg_base,
            const struct song_sdiv_fdiv_clk *sdiv_fdiv_clks);
static int song_register_gr_fdiv_clks(uint32_t reg_base,
            const struct song_gr_fdiv_clk *gr_fdiv_clks);
static int song_register_sdiv_gr_clks(uint32_t reg_base,
            const struct song_sdiv_gr_clk *sdiv_gr_clks);
static int song_register_gr_sdiv_clks(uint32_t reg_base,
            const struct song_sdiv_gr_clk *gr_sdiv_clks);
static int song_register_mux_div_clks(uint32_t reg_base,
            const struct song_mux_div_clk *mux_div_clks);
static int song_register_mux_clks(uint32_t reg_base,
            const struct song_mux_clk *mux_clks);
static int song_register_phase_clks(uint32_t reg_base,
            const struct song_phase_clk *phase_clks);
static int song_register_mux_sdiv_gr_clks(uint32_t reg_base,
            const struct song_mux_sdiv_gr_clk *mux_sdiv_gr_clks);
static int song_register_pll_clks(uint32_t reg_base,
            const struct song_pll_clk *pll_clks);
static int song_register_pll_lf_clks(uint32_t reg_base,
            const struct song_pll_lf_clk *pll_lf_clks);
static int song_register_out_clks(uint32_t reg_base,
            const struct song_out_clk *out_clks);
static int song_register_timer_clks(uint32_t reg_base,
            const struct song_timer_clk *timer_clks);
static int song_enable_lp(uint32_t reg_base,
            const struct song_lp_reg_clk *lp_reg);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static struct clk *song_clk_register_mux_div(const char *name,
            const char * const *parent_names, uint8_t num_parents,
            uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
            uint16_t mux_offset, uint8_t mux_shift, uint8_t mux_width,
            uint16_t div_offset, uint8_t div_shift, uint8_t div_width,
            uint16_t div_flags)
{
  struct clk *clk;
  char mux_name[SONG_CLK_NAME_MAX];

  snprintf(mux_name, SONG_CLK_NAME_MAX, "%s_mux", name);
  clk = clk_register_mux(mux_name, parent_names, num_parents,
    CLK_PARENT_NAME_IS_STATIC, reg_base + mux_offset, mux_shift,
    mux_width, CLK_MUX_ROUND_CLOSEST | CLK_MUX_HIWORD_MASK);
  if (!clk)
    return clk;

  if (en_offset)
    {
      char gate_name[SONG_CLK_NAME_MAX];

      snprintf(gate_name, SONG_CLK_NAME_MAX, "%s_gate", name);
      clk = clk_register_gate(gate_name, clk->name,
        CLK_SET_RATE_PARENT | CLK_PARENT_NAME_IS_STATIC,
        reg_base + en_offset, en_shift, CLK_GATE_HIWORD_MASK);
      if (!clk)
        return clk;
    }

  return clk_register_divider(name, clk->name,
    CLK_SET_RATE_PARENT | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, div_shift, div_width,
    div_flags | CLK_DIVIDER_ROUND_CLOSEST | CLK_DIVIDER_HIWORD_MASK);
}

static struct clk *song_clk_register_mux(const char *name,
            const char * const *parent_names, uint8_t num_parents,
            uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
            uint16_t mux_offset, uint8_t mux_shift, uint8_t mux_width,
            uint8_t mux_flags)
{
  if (en_offset)
    {
      struct clk *clk;
      char mux_name[SONG_CLK_NAME_MAX];

      snprintf(mux_name, SONG_CLK_NAME_MAX, "%s_mux", name);
      clk = clk_register_mux(mux_name, parent_names, num_parents,
        CLK_PARENT_NAME_IS_STATIC, reg_base + mux_offset, mux_shift,
        mux_width, CLK_MUX_ROUND_CLOSEST | CLK_MUX_HIWORD_MASK);
      if (!clk)
        return clk;

      return clk_register_gate(name, clk->name,
        CLK_SET_RATE_PARENT | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
        reg_base + en_offset, en_shift, CLK_GATE_HIWORD_MASK);
    }

  return clk_register_mux(name, parent_names, num_parents,
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC, reg_base + mux_offset,
    mux_shift, mux_width, CLK_MUX_ROUND_CLOSEST | mux_flags);
}

static struct clk *song_clk_register_gr(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
            uint16_t mult_offset, uint8_t mult_shift, uint8_t mult_width,
            uint8_t clk_flags, uint8_t gr_flags)
{
  struct clk *clk;
  char mult_name[SONG_CLK_NAME_MAX];
  uint8_t fixed_div = 8;

  if (gr_flags & CLK_GR_DIV_16)
    fixed_div = 16;

  if (en_offset)
    {
      char gate_name[SONG_CLK_NAME_MAX];

      snprintf(gate_name, SONG_CLK_NAME_MAX, "%s_gate", name);
      clk = clk_register_gate(gate_name, parent_name, clk_flags | CLK_PARENT_NAME_IS_STATIC,
        reg_base + en_offset, en_shift, CLK_GATE_HIWORD_MASK);
      if (!clk)
        return clk;
      parent_name = clk->name;
    }

  snprintf(mult_name, SONG_CLK_NAME_MAX, "%s_mult", name);
  clk = clk_register_multiplier(mult_name, parent_name,
    clk_flags | CLK_PARENT_NAME_IS_STATIC, reg_base + mult_offset, mult_shift,
    mult_width, CLK_MULT_ROUND_CLOSEST | CLK_MULT_HIWORD_MASK);
  if (!clk)
    return clk;

  return clk_register_fixed_factor(name, clk->name,
    clk_flags | CLK_SET_RATE_PARENT | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC, 1, fixed_div);
}

static struct clk *song_clk_register_sdiv_fdiv(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t sdiv_offset, uint16_t fdiv_offset)
{
  struct clk *clk;
  char gate_name[SONG_CLK_NAME_MAX];
  char fdiv_name[SONG_CLK_NAME_MAX];

  snprintf(gate_name, SONG_CLK_NAME_MAX, "%s_gate", name);
  clk = clk_register_gate(gate_name, parent_name, CLK_PARENT_NAME_IS_STATIC,
    reg_base + sdiv_offset, SONG_CLK_SDIV_FDIV_EN_SHIFT, CLK_GATE_HIWORD_MASK);
  if (!clk)
    return clk;

  snprintf(fdiv_name, SONG_CLK_NAME_MAX, "%s_fdiv", name);
  clk = clk_register_fractional_divider(fdiv_name, clk->name,
    CLK_PARENT_NAME_IS_STATIC,
    reg_base + fdiv_offset, SONG_CLK_SDIV_FDIV_MULT_SHIFT, SONG_CLK_SDIV_FDIV_MULT_WIDTH,
    SONG_CLK_SDIV_FDIV_FDIV_SHIFT, SONG_CLK_SDIV_FDIV_FDIV_WIDTH,
    CLK_FRAC_DIV_DOUBLE | CLK_FRAC_MUL_NEED_EVEN);
  if (!clk)
    return clk;

  return clk_register_divider(name, clk->name,
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC | CLK_SET_RATE_PARENT,
    reg_base + sdiv_offset, SONG_CLK_SDIV_FDIV_SDIV_SHIFT, SONG_CLK_SDIV_FDIV_SDIV_WIDTH,
    CLK_DIVIDER_ROUND_CLOSEST | CLK_DIVIDER_HIWORD_MASK);
}

static struct clk *song_clk_register_gr_fdiv(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
            uint16_t gr_offset, uint16_t div_offset, uint8_t clk_flags)
{
  struct clk *clk;
  char fixed_name[SONG_CLK_NAME_MAX];
  const char *pname = parent_name;
  uint8_t fixed_div = 2;

  if (en_offset)
    {
      char gate_name[SONG_CLK_NAME_MAX];

      snprintf(gate_name, SONG_CLK_NAME_MAX, "%s_gate", name);
      clk = clk_register_gate(gate_name, pname, clk_flags | CLK_PARENT_NAME_IS_STATIC,
        reg_base + en_offset, en_shift, CLK_GATE_HIWORD_MASK);
      if (!clk)
        return clk;

      pname = gate_name;
    }

  if (gr_offset)
    {
      char mult_name[SONG_CLK_NAME_MAX];

      snprintf(mult_name, SONG_CLK_NAME_MAX, "%s_mult", name);
      clk = clk_register_multiplier(mult_name, pname,
        clk_flags | CLK_PARENT_NAME_IS_STATIC, reg_base + gr_offset,
        SONG_CLK_GR_FDIV_GR_SHIFT, SONG_CLK_GR_FDIV_GR_WIDTH,
        CLK_MULT_ROUND_CLOSEST | CLK_MULT_HIWORD_MASK);
      if (!clk)
        return clk;
      fixed_div *= 8;

      pname = mult_name;
    }

  snprintf(fixed_name, SONG_CLK_NAME_MAX, "%s_fixed", name);
  clk = clk_register_fixed_factor(fixed_name, pname,
    clk_flags | CLK_SET_RATE_PARENT | CLK_PARENT_NAME_IS_STATIC, 1, fixed_div);
  if (!clk)
    return clk;

  return clk_register_fractional_divider(name, clk->name,
    clk_flags | CLK_SET_RATE_PARENT | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, SONG_CLK_GR_FDIV_MULT_SHIFT, SONG_CLK_GR_FDIV_MULT_WIDTH,
    SONG_CLK_GR_FDIV_DIV_SHIFT, SONG_CLK_GR_FDIV_DIV_WIDTH, CLK_FRAC_MUL_NEED_EVEN);
}

static struct clk *song_clk_register_div(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
            uint16_t div_offset, uint8_t div_shift, uint8_t div_width,
            uint16_t div_flags, uint8_t clk_flags)
{
  if (en_offset)
    {
      struct clk *clk;
      char gate_name[SONG_CLK_NAME_MAX];

      snprintf(gate_name, SONG_CLK_NAME_MAX, "%s_gate", name);
      clk = clk_register_gate(gate_name, parent_name, clk_flags | CLK_PARENT_NAME_IS_STATIC,
        reg_base + en_offset, en_shift, CLK_GATE_HIWORD_MASK);
      if (!clk)
        return clk;
      parent_name = clk->name;
    }

  return clk_register_divider(name, parent_name,
    clk_flags | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, div_shift, div_width,
    div_flags | CLK_DIVIDER_ROUND_CLOSEST | CLK_DIVIDER_HIWORD_MASK);
}

static struct clk *song_clk_register_sdiv_gr(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t div_offset, uint8_t div_width,
            uint16_t div_flags)
{
  struct clk *clk;
  char gate_name[SONG_CLK_NAME_MAX];
  char div_name[SONG_CLK_NAME_MAX];
  char mult_name[SONG_CLK_NAME_MAX];

  snprintf(gate_name, SONG_CLK_NAME_MAX, "%s_gate", name);
  clk = clk_register_gate(gate_name, parent_name, CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, SONG_CLK_SDIV_GR_EN_SHIFT, CLK_GATE_HIWORD_MASK);
  if (!clk)
    return clk;

  snprintf(div_name, SONG_CLK_NAME_MAX, "%s_div", name);
  clk = clk_register_divider(div_name, clk->name, CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, SONG_CLK_SDIV_GR_DIV_SHIFT, div_width,
    div_flags | CLK_DIVIDER_ROUND_CLOSEST | CLK_DIVIDER_HIWORD_MASK);
  if (!clk)
    return clk;

  snprintf(mult_name, SONG_CLK_NAME_MAX, "%s_mult", name);
  clk = clk_register_multiplier(mult_name, clk->name,
    CLK_SET_RATE_PARENT | CLK_PARENT_NAME_IS_STATIC, reg_base + div_offset,
    SONG_CLK_SDIV_GR_GR_SHIFT, SONG_CLK_SDIV_GR_GR_WIDTH,
    CLK_MULT_ROUND_CLOSEST | CLK_MULT_HIWORD_MASK);
  if (!clk)
    return clk;

  return clk_register_fixed_factor(name, clk->name,
    CLK_SET_RATE_PARENT | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC, 1, 16);
}

static struct clk *song_clk_register_sdiv_sdiv(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t div_offset,
            uint16_t div1_flags, uint16_t div2_flags)
{
  struct clk *clk;
  char gate_name[SONG_CLK_NAME_MAX];
  char div_name[SONG_CLK_NAME_MAX];

  snprintf(gate_name, SONG_CLK_NAME_MAX, "%s_gate", name);
  clk = clk_register_gate(gate_name, parent_name, CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, SONG_CLK_SDIV_SDIV_EN_SHIFT, CLK_GATE_HIWORD_MASK);
  if (!clk)
    return clk;

  snprintf(div_name, SONG_CLK_NAME_MAX, "%s_div", name);
  clk = clk_register_divider(div_name, clk->name,
    CLK_PARENT_NAME_IS_STATIC, reg_base + div_offset,
    SONG_CLK_SDIV_SDIV_DIV0_SHIFT, SONG_CLK_SDIV_SDIV_DIV0_WIDTH,
    CLK_DIVIDER_ROUND_CLOSEST | CLK_DIVIDER_HIWORD_MASK | div1_flags);
  if (!clk)
    return clk;

  return clk_register_divider(name, clk->name, CLK_SET_RATE_PARENT |
    CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC, reg_base + div_offset,
    SONG_CLK_SDIV_SDIV_DIV1_SHIFT, SONG_CLK_SDIV_SDIV_DIV1_WIDTH,
    CLK_DIVIDER_ROUND_CLOSEST | CLK_DIVIDER_HIWORD_MASK | div2_flags);
}

static struct clk *song_clk_register_gr_sdiv(
            const char *name, const char *parent_name,
            uint32_t reg_base, uint16_t div_offset, uint8_t div_width,
            uint16_t div_flags)
{
  struct clk *clk;
  char gate_name[SONG_CLK_NAME_MAX];
  char div_name[SONG_CLK_NAME_MAX];
  char mult_name[SONG_CLK_NAME_MAX];

  snprintf(gate_name, SONG_CLK_NAME_MAX, "%s_gate", name);
  clk = clk_register_gate(gate_name, parent_name, CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, SONG_CLK_GR_SDIV_EN_SHIFT, CLK_GATE_HIWORD_MASK);
  if (!clk)
    return clk;

  snprintf(div_name, SONG_CLK_NAME_MAX, "%s_div", name);
  clk = clk_register_divider(div_name, clk->name, CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, SONG_CLK_GR_SDIV_DIV_SHIFT, div_width,
    div_flags | CLK_DIVIDER_ROUND_CLOSEST | CLK_DIVIDER_HIWORD_MASK);
  if (!clk)
    return clk;

  snprintf(mult_name, SONG_CLK_NAME_MAX, "%s_mult", name);
  clk = clk_register_multiplier(mult_name, clk->name,
    CLK_SET_RATE_PARENT | CLK_PARENT_NAME_IS_STATIC, reg_base + div_offset,
    SONG_CLK_GR_SDIV_GR_SHIFT, SONG_CLK_GR_SDIV_GR_WIDTH,
    CLK_MULT_ROUND_CLOSEST | CLK_MULT_HIWORD_MASK);
  if (!clk)
    return clk;

  return clk_register_fixed_factor(name, clk->name,
    CLK_SET_RATE_PARENT | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC, 1, 8);
}

static struct clk *song_clk_register_mux_sdiv_gr(const char *name,
            const char * const *parent_names, uint8_t num_parents,
            uint32_t reg_base, uint16_t div_offset, uint8_t div_width,
            uint8_t mux_width)
{
  struct clk *clk;
  char mux_name[SONG_CLK_NAME_MAX];
  char gate_name[SONG_CLK_NAME_MAX];
  char div_name[SONG_CLK_NAME_MAX];
  char mult_name[SONG_CLK_NAME_MAX];

  snprintf(mux_name, SONG_CLK_NAME_MAX, "%s_mux", name);
  clk = clk_register_mux(mux_name, parent_names, num_parents,
    CLK_PARENT_NAME_IS_STATIC, reg_base + div_offset,
    SONG_CLK_MUX_SDIV_GR_MUX_SHIFT, mux_width,
    CLK_MUX_ROUND_CLOSEST | CLK_MUX_HIWORD_MASK);
  if (!clk)
    return clk;

  snprintf(gate_name, SONG_CLK_NAME_MAX, "%s_gate", name);
  clk = clk_register_gate(gate_name, clk->name,
    CLK_SET_RATE_PARENT | CLK_PARENT_NAME_IS_STATIC, reg_base + div_offset,
    SONG_CLK_MUX_SDIV_GR_EN_SHIFT, CLK_GATE_HIWORD_MASK);
  if (!clk)
    return clk;

  snprintf(div_name, SONG_CLK_NAME_MAX, "%s_div", name);
  clk = clk_register_divider(div_name, clk->name,
    CLK_SET_RATE_PARENT | CLK_PARENT_NAME_IS_STATIC, reg_base + div_offset,
    SONG_CLK_MUX_SDIV_GR_DIV_SHIFT, div_width,
    CLK_DIVIDER_ROUND_CLOSEST | CLK_DIVIDER_HIWORD_MASK);
  if (!clk)
    return clk;

  snprintf(mult_name, SONG_CLK_NAME_MAX, "%s_mult", name);
  clk = clk_register_multiplier(mult_name, clk->name,
    CLK_SET_RATE_PARENT | CLK_PARENT_NAME_IS_STATIC, reg_base + div_offset,
    SONG_CLK_MUX_SDIV_GR_GR_SHIFT, SONG_CLK_MUX_SDIV_GR_GR_WIDTH,
    CLK_MULT_ROUND_CLOSEST | CLK_MULT_HIWORD_MASK);
  if (!clk)
    return clk;

  return clk_register_fixed_factor(name, clk->name,
    CLK_SET_RATE_PARENT | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC, 1, 16);
}

static int song_register_fixed_rate_clks(
            const struct song_fixed_rate_clk *fixed_rate_clks)
{
  struct clk *clk;

  while (fixed_rate_clks->name)
    {
      clk = clk_register_fixed_rate(
              fixed_rate_clks->name,
              NULL,
              CLK_NAME_IS_STATIC,
              fixed_rate_clks->fixed_rate);
      if (!clk)
        {
          return -EINVAL;
        }
      fixed_rate_clks++;
    }

  return 0;
}

static int song_register_fixed_factor_clks(
            const struct song_fixed_factor_clk *fixed_factor_clks)
{
  struct clk *clk;

  while (fixed_factor_clks->name)
    {
      clk = clk_register_fixed_factor(
              fixed_factor_clks->name,
              fixed_factor_clks->parent_name,
              CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              fixed_factor_clks->fixed_mult,
              fixed_factor_clks->fixed_div);
      if (!clk)
        {
          return -EINVAL;
        }

      fixed_factor_clks++;
    }

  return 0;
}

static int song_register_gate_clks(uint32_t reg_base,
            const struct song_gate_clk *gate_clks)
{
  struct clk *clk;

  while (gate_clks->name)
    {
      clk = clk_register_gate(
              gate_clks->name,
              gate_clks->parent_name,
              gate_clks->clk_flags |
              CLK_SET_RATE_PARENT | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              reg_base + gate_clks->en_offset,
              gate_clks->en_shift,
              CLK_GATE_HIWORD_MASK);
      if (!clk)
        {
          return -EINVAL;
        }

      gate_clks++;
    }

  return 0;
}

static int song_register_div_clks(uint32_t reg_base,
            const struct song_div_clk *div_clks)
{
  struct clk *clk;

  while (div_clks->name)
    {
      clk = song_clk_register_div(
              div_clks->name,
              div_clks->parent_name,
              reg_base,
              div_clks->en_offset,
              div_clks->en_shift,
              div_clks->div_offset,
              div_clks->div_shift,
              div_clks->div_width,
              div_clks->div_flags,
              div_clks->clk_flags);
      if (!clk)
        {
          return -EINVAL;
        }

      div_clks++;
    }

  return 0;
}

static int song_register_gr_clks(uint32_t reg_base,
            const struct song_gr_clk *gr_clks)
{
  struct clk *clk;

  while (gr_clks->name)
    {
      clk = song_clk_register_gr(
              gr_clks->name,
              gr_clks->parent_name,
              reg_base,
              gr_clks->en_offset,
              gr_clks->en_shift,
              gr_clks->mult_offset,
              gr_clks->mult_shift,
              gr_clks->mult_width,
              gr_clks->clk_flags,
              gr_clks->gr_flags);
      if (!clk)
        {
          return -EINVAL;
        }

        gr_clks++;
      }

  return 0;
}

static int song_register_sdiv_sdiv_clks(uint32_t reg_base,
            const struct song_sdiv_sdiv_clk *sdiv_sdiv_clks)
{
  struct clk *clk;

  while (sdiv_sdiv_clks->name)
    {
      clk = song_clk_register_sdiv_sdiv(
              sdiv_sdiv_clks->name,
              sdiv_sdiv_clks->parent_name,
              reg_base,
              sdiv_sdiv_clks->div_offset,
              sdiv_sdiv_clks->div1_flags,
              sdiv_sdiv_clks->div2_flags);
      if (!clk)
        {
          return -EINVAL;
        }

      sdiv_sdiv_clks++;
    }

  return 0;
}

static int song_register_sdiv_fdiv_clks(uint32_t reg_base,
            const struct song_sdiv_fdiv_clk *sdiv_fdiv_clks)
{
  struct clk *clk;

  while (sdiv_fdiv_clks->name)
    {
      clk = song_clk_register_sdiv_fdiv(
              sdiv_fdiv_clks->name,
              sdiv_fdiv_clks->parent_name,
              reg_base,
              sdiv_fdiv_clks->sdiv_offset,
              sdiv_fdiv_clks->fdiv_offset);
      if (!clk)
        {
          return -EINVAL;
        }

      sdiv_fdiv_clks++;
    }

  return 0;
}

static int song_register_gr_fdiv_clks(uint32_t reg_base,
            const struct song_gr_fdiv_clk *gr_fdiv_clks)
{
  struct clk *clk;

  while (gr_fdiv_clks->name)
    {
      clk = song_clk_register_gr_fdiv(
              gr_fdiv_clks->name,
              gr_fdiv_clks->parent_name,
              reg_base,
              gr_fdiv_clks->en_offset,
              gr_fdiv_clks->en_shift,
              gr_fdiv_clks->gr_offset,
              gr_fdiv_clks->div_offset,
              gr_fdiv_clks->clk_flags);
      if (!clk)
        {
          return -EINVAL;
        }

      gr_fdiv_clks++;
    }

  return 0;
}

static int song_register_sdiv_gr_clks(uint32_t reg_base,
            const struct song_sdiv_gr_clk *sdiv_gr_clks)
{
  struct clk *clk;

  while (sdiv_gr_clks->name)
    {
      clk = song_clk_register_sdiv_gr(
              sdiv_gr_clks->name,
              sdiv_gr_clks->parent_name,
              reg_base,
              sdiv_gr_clks->div_offset,
              sdiv_gr_clks->div_width,
              sdiv_gr_clks->div_flags);
      if (!clk)
        {
          return -EINVAL;
        }

      sdiv_gr_clks++;
    }

  return 0;
}

static int song_register_gr_sdiv_clks(uint32_t reg_base,
            const struct song_sdiv_gr_clk *gr_sdiv_clks)
{
  struct clk *clk;

  while (gr_sdiv_clks->name)
    {
      clk = song_clk_register_gr_sdiv(
              gr_sdiv_clks->name,
              gr_sdiv_clks->parent_name,
              reg_base,
              gr_sdiv_clks->div_offset,
              gr_sdiv_clks->div_width,
              gr_sdiv_clks->div_flags);
      if (!clk)
        {
          return -EINVAL;
        }

      gr_sdiv_clks++;
    }

  return 0;
}

static int song_register_mux_div_clks(uint32_t reg_base,
            const struct song_mux_div_clk *mux_div_clks)
{
  struct clk *clk;

  while (mux_div_clks->name)
    {
      clk = song_clk_register_mux_div(
              mux_div_clks->name,
              mux_div_clks->parent_names,
              mux_div_clks->num_parents,
              reg_base,
              mux_div_clks->en_offset,
              mux_div_clks->en_shift,
              mux_div_clks->mux_offset,
              mux_div_clks->mux_shift,
              mux_div_clks->mux_width,
              mux_div_clks->div_offset,
              mux_div_clks->div_shift,
              mux_div_clks->div_width,
              mux_div_clks->div_flags);
      if (!clk)
        {
          return -EINVAL;
        }

      mux_div_clks++;
    }

  return 0;
}

static int song_register_mux_clks(uint32_t reg_base,
            const struct song_mux_clk *mux_clks)
{
  struct clk *clk;

  while (mux_clks->name)
    {
      clk = song_clk_register_mux(
              mux_clks->name,
              mux_clks->parent_names,
              mux_clks->num_parents,
              reg_base,
              mux_clks->en_offset,
              mux_clks->en_shift,
              mux_clks->mux_offset,
              mux_clks->mux_shift,
              mux_clks->mux_width,
              mux_clks->mux_flags);
      if (!clk)
        {
          return -EINVAL;
        }

      mux_clks++;
    }

  return 0;
}

static int song_register_phase_clks(uint32_t reg_base,
            const struct song_phase_clk *phase_clks)
{
  struct clk *clk;

  while (phase_clks->name)
    {
      clk = clk_register_phase(
              phase_clks->name,
              phase_clks->parent_name,
              CLK_SET_RATE_PARENT | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              reg_base + phase_clks->reg_offset,
              phase_clks->phase_shift,
              phase_clks->phase_width,
              CLK_PHASE_HIWORD_MASK);
      if (!clk)
        {
          return -EINVAL;
        }

      phase_clks++;
    }

  return 0;
}

static int song_register_mux_sdiv_gr_clks(uint32_t reg_base,
            const struct song_mux_sdiv_gr_clk *mux_sdiv_gr_clks)
{
  struct clk *clk;

  while (mux_sdiv_gr_clks->name)
    {
      clk = song_clk_register_mux_sdiv_gr(
              mux_sdiv_gr_clks->name,
              mux_sdiv_gr_clks->parent_names,
              mux_sdiv_gr_clks->num_parents,
              reg_base,
              mux_sdiv_gr_clks->div_offset,
              mux_sdiv_gr_clks->div_width,
              mux_sdiv_gr_clks->mux_width);
      if (!clk)
        {
          return -EINVAL;
        }

      mux_sdiv_gr_clks++;
    }

  return 0;
}

static int song_register_pll_clks(uint32_t reg_base,
            const struct song_pll_clk *pll_clks)
{
  struct clk *clk;

  while (pll_clks->name)
    {
      clk = clk_register_pll(
              pll_clks->name,
              pll_clks->parent_name,
              CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              reg_base + pll_clks->cfg0_offset,
              reg_base + pll_clks->cfg1_offset,
              reg_base + pll_clks->ctl_offset,
              pll_clks->ctl_shift,
              pll_clks->pll_flags);
      if (!clk)
        {
          return -EINVAL;
        }

      pll_clks++;
    }

  return 0;
}

static int song_register_pll_lf_clks(uint32_t reg_base,
            const struct song_pll_lf_clk *pll_lf_clks)
{
  struct clk *clk;

  while (pll_lf_clks->name)
    {
      clk = clk_register_pll_lf(
              pll_lf_clks->name,
              pll_lf_clks->parent_name,
              CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              reg_base + pll_lf_clks->cfg0_offset,
              reg_base + pll_lf_clks->cfg1_offset);
      if (!clk)
        {
          return -EINVAL;
        }

      pll_lf_clks++;
    }

  return 0;
}

static int song_register_out_clks(uint32_t reg_base,
            const struct song_out_clk *out_clks)
{
  struct clk *clk;

  while (out_clks->name)
    {
      clk = clk_register_out(
              out_clks->name,
              out_clks->parent_names,
              out_clks->num_parents,
              CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              reg_base + out_clks->mux_offset,
              out_clks->mux_shift,
              SONG_CLK_CLKOUT_MUX_WIDTH,
              reg_base + out_clks->ctl_offset);
      if (!clk)
        {
          return -EINVAL;
        }

      out_clks++;
    }

  return 0;
}

static int song_register_timer_clks(uint32_t reg_base,
            const struct song_timer_clk *timer_clks)
{
  struct clk *clk;

  while (timer_clks->name)
    {
      clk = clk_register_timer(
              timer_clks->name,
              timer_clks->parent_names,
              timer_clks->num_parents,
              CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              reg_base + timer_clks->ctl_offset,
              SONG_CLK_TIMER_MUX_SHIFT,
              SONG_CLK_TIMER_MUX_WIDTH);
      if (!clk)
        {
          return -EINVAL;
        }

      timer_clks++;
    }

  return 0;
}

static int song_enable_lp(uint32_t reg_base,
            const struct song_lp_reg_clk *lp_reg)
{
  while (lp_reg->offset)
    {
      clk_write(reg_base + lp_reg->offset, lp_reg->value);
      lp_reg++;
    }

  return 0;
}

/****************************************************************************
   * Public Functions
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
 *   table     - the song platform clk table that to be configured.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int song_clk_initialize(uint32_t base, const struct song_clk_table *table)
{
  int ret = 0;

  if (table->lp_reg)
    {
      ret = song_enable_lp(base, table->lp_reg);
      if (ret)
        return ret;
    }

  if (table->fixed_rate_clks)
    {
      ret = song_register_fixed_rate_clks(table->fixed_rate_clks);
      if (ret)
        return ret;
    }

  if (table->fixed_factor_clks)
    {
      ret = song_register_fixed_factor_clks(table->fixed_factor_clks);
      if (ret)
        return ret;
    }

  if (table->pll_clks)
    {
      ret = song_register_pll_clks(base, table->pll_clks);
      if (ret)
        return ret;
    }

  if (table->pll_lf_clks)
    {
      ret = song_register_pll_lf_clks(base, table->pll_lf_clks);
      if (ret)
        return ret;
    }

  if (table->out_clks)
    {
      ret = song_register_out_clks(base, table->out_clks);
      if (ret)
        return ret;
    }

  if (table->timer_clks)
    {
      ret = song_register_timer_clks(base, table->timer_clks);
      if (ret)
        return ret;
    }

  if (table->phase_clks)
    {
      ret = song_register_phase_clks(base, table->phase_clks);
      if (ret)
        return ret;
    }

  if (table->div_clks)
    {
      ret = song_register_div_clks(base, table->div_clks);
      if (ret)
        return ret;
    }

  if (table->gate_clks)
    {
      ret = song_register_gate_clks(base, table->gate_clks);
      if (ret)
        return ret;
    }

  if (table->gr_clks)
    {
      ret = song_register_gr_clks(base, table->gr_clks);
      if (ret)
        return ret;
    }

  if (table->sdiv_sdiv_clks)
    {
      ret = song_register_sdiv_sdiv_clks(base, table->sdiv_sdiv_clks);
      if (ret)
        return ret;
    }

  if (table->sdiv_fdiv_clks)
    {
      ret = song_register_sdiv_fdiv_clks(base, table->sdiv_fdiv_clks);
      if (ret)
        return ret;
    }

  if (table->gr_fdiv_clks)
    {
      ret = song_register_gr_fdiv_clks(base, table->gr_fdiv_clks);
      if (ret)
        return ret;
    }

  if (table->mux_div_clks)
    {
      ret = song_register_mux_div_clks(base, table->mux_div_clks);
      if (ret)
        return ret;
    }

  if (table->mux_clks)
    {
      ret = song_register_mux_clks(base, table->mux_clks);
      if (ret)
        return ret;
    }

  if (table->sdiv_gr_clks)
    {
      ret = song_register_sdiv_gr_clks(base, table->sdiv_gr_clks);
      if (ret)
        return ret;
    }

  if (table->mux_sdiv_gr_clks)
    {
      ret = song_register_mux_sdiv_gr_clks(base, table->mux_sdiv_gr_clks);
      if (ret)
        return ret;
    }

  if (table->gr_sdiv_clks)
    {
      ret = song_register_gr_sdiv_clks(base, table->gr_sdiv_clks);
      if (ret)
        return ret;
    }

  return 0;
}
