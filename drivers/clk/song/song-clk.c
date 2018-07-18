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
#include <nuttx/clk/clk-provider.h>
#include <nuttx/clk/song/song-clk.h>
#include <stdio.h>

#include "song-clk.h"

#ifdef CONFIG_SONG_CLK

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static struct clk *song_clk_register_gate(const char *name,
                    const char *parent_name, uint16_t flags,
                    uint32_t reg_base, uint16_t en_offset,
                    uint8_t bit_idx, uint64_t private_flags);
static struct clk *song_clk_register_mux_sdiv(const char *name,
                    const char * const *parent_name, uint8_t num_parents,
                    uint16_t flags, uint32_t reg_base,
                    uint16_t en_offset, uint8_t en_shift,
                    uint16_t mux_offset, uint8_t mux_shift, uint8_t mux_width,
                    uint16_t div_offset, uint8_t div_shift, uint8_t div_width,
                    uint64_t private_flags);
static struct clk *song_clk_register_gr(const char *name,
                    const char *parent_name, uint16_t flags,
                    uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
                    uint16_t mul_offset, uint8_t mul_shift, uint8_t mul_width,
                    uint64_t private_flags);
static struct clk *song_clk_register_sdiv_fdiv(const char *name,
                    const char *parent_name, uint16_t flags,
                    uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
                    uint16_t sdiv_offset, uint8_t sdiv_shift, uint8_t sdiv_width,
                    uint16_t fdiv_offset, uint64_t private_flags);
static struct clk *song_clk_register_gr_fdiv(const char *name,
                    const char *parent_name, uint16_t flags,
                    uint32_t reg_base, uint16_t en_offset,
                    uint8_t en_shift, uint16_t gr_offset,
                    uint16_t div_offset, uint32_t fixed_gr,
                    uint64_t private_flags);
static struct clk *song_clk_register_sdiv(const char *name,
                    const char *parent_name, uint16_t flags,
                    uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
                    uint16_t div_offset, uint8_t div_shift, uint8_t div_width,
                    uint64_t private_flags);
static struct clk *song_clk_register_sdiv_gr(const char *name,
                    const char *parent_name, uint16_t flags,
                    uint32_t reg_base, uint16_t div_offset,
                    uint64_t private_flags);
static struct clk *song_clk_register_sdiv_sdiv(const char *name,
                    const char *parent_name, uint16_t flags,
                    uint32_t reg_base, uint16_t div_offset,
                    uint64_t private_flags);
static struct clk *song_clk_register_gr_sdiv(const char *name,
                    const char *parent_name, uint16_t flags,
                    uint32_t reg_base, uint16_t div_offset,
                    uint64_t private_flags);
static struct clk *song_clk_register_mux_sdiv_gr(const char *name,
                    const char * const *parent_name, uint8_t num_parents,
                    uint16_t flags, uint32_t reg_base,
                    uint16_t div_offset, uint8_t div_width, uint8_t mux_width,
                    uint64_t private_flags);

static int song_register_fixed_rate_clks(
                    const struct song_fixed_rate_clk *fixed_rate_clks);
static int song_register_fixed_factor_clks(
                    const struct song_fixed_factor_clk *fixed_factor_clks);
static int song_register_gate_clks(uint32_t reg_base,
                    const struct song_gate_clk *gate_clks);
static int song_register_sdiv_clks(uint32_t reg_base,
                    const struct song_sdiv_clk *sdiv_clks);
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
static int song_register_mux_sdiv_clks(uint32_t reg_base,
                    const struct song_mux_sdiv_clk *mux_sdiv_clks);
static int song_register_mux_gate_clks(uint32_t reg_base,
                    const struct song_mux_gate_clk *mux_gate_clks);
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

static int song_set_default_rate(const struct song_default_rate_clk *def_rate);

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static struct clk *song_clk_register_gate(const char *name,
    const char *parent_name, uint16_t flags,
    uint32_t reg_base, uint16_t en_offset,
    uint8_t bit_idx, uint64_t private_flags)
{
  uint8_t clk_gate_flags = (private_flags >> SONG_CLK_GATE_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  return clk_register_gate(name, parent_name,
      flags | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC ,
      reg_base + en_offset, bit_idx, clk_gate_flags);
}

static struct clk *song_clk_register_mux_sdiv(const char *name,
    const char * const *parent_name, uint8_t num_parents,
    uint16_t flags, uint32_t reg_base,
    uint16_t en_offset, uint8_t en_shift,
    uint16_t mux_offset, uint8_t mux_shift, uint8_t mux_width,
    uint16_t div_offset, uint8_t div_shift, uint8_t div_width,
    uint64_t private_flags)
{
  struct clk *clk = NULL;
  char mux_clk[SONG_CLK_NAME_MAX], gate_clk[SONG_CLK_NAME_MAX];
  uint8_t gate_flags, mux_flags;
  uint16_t div_flags;
  const char *pname = *parent_name, *cname;

  snprintf(gate_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "gate");
  snprintf(mux_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "mux");

  flags |= CLK_PARENT_NAME_IS_STATIC;

  if (mux_offset)
    {
      cname = mux_clk;

      if (!en_offset && !div_offset)
        {
          cname = name;
          flags |= CLK_NAME_IS_STATIC;
        }

      mux_flags = (private_flags >> SONG_CLK_MUX_FLAG_SHIFT) &
        SONG_CLK_FLAG_MASK;
      mux_flags |= CLK_MUX_ROUND_CLOSEST;

      clk = clk_register_mux(cname, parent_name, num_parents, flags,
        reg_base + mux_offset, mux_shift, mux_width, mux_flags);
      if (!clk)
        return clk;

      pname = cname;
      flags &= ~CLK_PARENT_NAME_IS_STATIC;
    }

  if (en_offset)
    {
      cname = gate_clk;
      if (!div_offset)
        {
          cname = name;
          flags |= CLK_NAME_IS_STATIC;
        }

      gate_flags = (private_flags >> SONG_CLK_GATE_FLAG_SHIFT) &
        SONG_CLK_FLAG_MASK;

      clk = clk_register_gate(cname, pname, flags,
        reg_base + en_offset, en_shift, gate_flags);
      if (!clk)
        return clk;

      pname = cname;
      flags &= ~CLK_PARENT_NAME_IS_STATIC;
    }

  if (div_offset)
    {
      flags |= CLK_NAME_IS_STATIC;
      div_flags = (private_flags >> SONG_CLK_DIV_FLAG_SHIFT) &
        SONG_CLK_DIV_MASK;
      div_flags |= CLK_DIVIDER_ROUND_CLOSEST;

      return clk_register_divider(name, pname, flags,
        reg_base + div_offset, div_shift, div_width,
        div_flags);
    }

  return clk;
}

static struct clk *song_clk_register_gr(const char *name,
    const char *parent_name, uint16_t flags,
    uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
    uint16_t mul_offset, uint8_t mul_shift, uint8_t mul_width,
    uint64_t private_flags)
{
  struct clk *clk;
  uint8_t mult_flags, song_flags, gate_flags;
  char gate_clk[SONG_CLK_NAME_MAX], mult_clk[SONG_CLK_NAME_MAX];
  uint8_t fixed_div = 8;
  const char *pname = parent_name;
  flags |= CLK_SET_RATE_PARENT | CLK_PARENT_NAME_IS_STATIC;

  snprintf(gate_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "gate");
  snprintf(mult_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "mult");

  mult_flags = (private_flags >> SONG_CLK_MULT_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  song_flags = (private_flags >> SONG_CLK_PRIVATE_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  if (song_flags & SONG_CLK_GR_DIV_16)
    fixed_div = 16;

  if (en_offset)
    {
      gate_flags = (private_flags >> SONG_CLK_GATE_FLAG_SHIFT) &
       SONG_CLK_FLAG_MASK;

      clk = clk_register_gate(gate_clk, pname, flags,
        reg_base + en_offset, en_shift, gate_flags);
      if (!clk)
        return clk;

      pname = gate_clk;
      flags &= ~CLK_PARENT_NAME_IS_STATIC;
    }

  mult_flags |= CLK_MULT_ROUND_CLOSEST;
  clk = clk_register_multiplier(mult_clk, pname,
    flags, reg_base + mul_offset, mul_shift,
    mul_width, mult_flags);
  if (!clk)
    return clk;

  flags &= ~CLK_PARENT_NAME_IS_STATIC;
  return clk_register_fixed_factor(name, mult_clk, flags | CLK_NAME_IS_STATIC,
    1, fixed_div);
}

static struct clk *song_clk_register_sdiv_fdiv(const char *name,
    const char *parent_name, uint16_t flags,
    uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
    uint16_t sdiv_offset, uint8_t sdiv_shift, uint8_t sdiv_width,
    uint16_t fdiv_offset, uint64_t private_flags)
{
  struct clk *clk;
  char sdiv_clk[SONG_CLK_NAME_MAX];
  uint8_t sdiv_flags, frac_flags;
  const char *pname = parent_name;

  flags |= CLK_PARENT_NAME_IS_STATIC;

  snprintf(sdiv_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "sdiv");
  sdiv_flags = (private_flags >> SONG_CLK_DIV_FLAG_SHIFT) &
    SONG_CLK_DIV_MASK;

  frac_flags = (private_flags >> SONG_CLK_FRAC_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  if (en_offset)
    {
      char gate_clk[SONG_CLK_NAME_MAX];
      uint8_t gate_flags;

      snprintf(gate_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "gate");
      gate_flags = (private_flags >> SONG_CLK_GATE_FLAG_SHIFT) &
        SONG_CLK_FLAG_MASK;

      clk = clk_register_gate(gate_clk, parent_name, flags, reg_base + en_offset,
        en_shift, gate_flags);

      if (!clk)
        return clk;
      pname = gate_clk;
      flags &= ~CLK_PARENT_NAME_IS_STATIC;
    }

  clk = clk_register_divider(sdiv_clk, pname, flags,
    reg_base + sdiv_offset, sdiv_shift, sdiv_width, sdiv_flags);
  if (!clk)
    return clk;

  flags &= ~CLK_PARENT_NAME_IS_STATIC;
  flags |= CLK_NAME_IS_STATIC;

  return clk_register_fractional_divider(name, sdiv_clk, flags,
    reg_base + fdiv_offset, SONG_CLK_SDIV_FDIV_MUL_SHIFT,
    SONG_CLK_SDIV_FDIV_MUL_WIDTH, SONG_CLK_SDIV_FDIV_DIV_SHIFT,
    SONG_CLK_SDIV_FDIV_DIV_WIDTH, frac_flags);
}

static struct clk *song_clk_register_gr_fdiv(const char *name,
    const char *parent_name, uint16_t flags,
    uint32_t reg_base, uint16_t en_offset,
    uint8_t en_shift, uint16_t gr_offset,
    uint16_t div_offset, uint32_t fixed_gr,
    uint64_t private_flags)
{
  struct clk *clk;
  char gr_clk[SONG_CLK_NAME_MAX], fixed_clk[SONG_CLK_NAME_MAX],
       gate_clk[SONG_CLK_NAME_MAX];
  uint8_t frac_flags, gate_flags;
  const char *pname = gate_clk;

  snprintf(gate_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "gate");
  snprintf(gr_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "gr_fdiv");
  snprintf(fixed_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "fixed");

  frac_flags = (private_flags >> SONG_CLK_FRAC_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  gate_flags = (private_flags >> SONG_CLK_GATE_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  clk = clk_register_gate(gate_clk, parent_name, flags | CLK_PARENT_NAME_IS_STATIC,
      reg_base + en_offset, en_shift, gate_flags);
  if (!clk)
    return clk;

  if (gr_offset)
    {
      private_flags = (private_flags >> SONG_CLK_MULT_FLAG_SHIFT) &
      SONG_CLK_FLAG_MASK;

      clk = song_clk_register_gr(gr_clk, pname, flags, reg_base, 0, 0,
        gr_offset, SONG_CLK_GR_FDIV_GR_SHIFT,
        SONG_CLK_GR_FDIV_GR_WIDTH, private_flags);
      if (!clk)
        return clk;

      pname = gr_clk;
    }

  clk = clk_register_fixed_factor(fixed_clk, pname, flags, 1, 2);
  if (!clk)
    return clk;

  return clk_register_fractional_divider(name, fixed_clk, flags | CLK_NAME_IS_STATIC,
    reg_base + div_offset, SONG_CLK_GR_FDIV_MUL_SHIFT,
    SONG_CLK_GR_FDIV_MUL_WIDTH, SONG_CLK_GR_FDIV_DIV_SHIFT,
    SONG_CLK_GR_FDIV_DIV_WIDTH, frac_flags);
}

static struct clk *song_clk_register_sdiv(const char *name,
    const char *parent_name, uint16_t flags,
    uint32_t reg_base, uint16_t en_offset, uint8_t en_shift,
    uint16_t div_offset, uint8_t div_shift, uint8_t div_width,
    uint64_t private_flags)
{
  return song_clk_register_mux_sdiv(name, &parent_name, 1, flags,
    reg_base, en_offset, en_shift, 0, 0, 0, div_offset,
    div_shift, div_width, private_flags);
}

static struct clk *song_clk_register_sdiv_gr(const char *name,
    const char *parent_name, uint16_t flags,
    uint32_t reg_base, uint16_t div_offset,
    uint64_t private_flags)
{
  struct clk *clk;
  char gate_clk[SONG_CLK_NAME_MAX], div_clk[SONG_CLK_NAME_MAX],
       mult_clk[SONG_CLK_NAME_MAX];
  uint8_t mult_flags, song_flags, gate_flags, div_flags;
  uint8_t fixed_div = 8;

  snprintf(gate_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "gate");
  snprintf(div_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "div");
  snprintf(mult_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "mult");

  gate_flags = (private_flags >> SONG_CLK_GATE_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  song_flags = (private_flags >> SONG_CLK_PRIVATE_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  div_flags = (private_flags >> SONG_CLK_DIV_FLAG_SHIFT) &
    SONG_CLK_DIV_MASK;

  mult_flags = (private_flags >> SONG_CLK_MULT_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  if (song_flags & SONG_CLK_GR_DIV_16)
    fixed_div = 16;

  clk = clk_register_gate(gate_clk, parent_name, flags | CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, SONG_CLK_SDIV_GR_EN_SHIFT, gate_flags);
  if (!clk)
    return clk;

  div_flags |= CLK_DIVIDER_ROUND_CLOSEST;
  clk = clk_register_divider(div_clk, gate_clk, flags,
    reg_base + div_offset, SONG_CLK_SDIV_GR_DIV_SHIFT, SONG_CLK_SDIV_GR_DIV_WIDTH,
    div_flags);

  if (!clk)
    return clk;

  mult_flags |= CLK_MULT_ROUND_CLOSEST;
  clk = clk_register_multiplier(mult_clk, div_clk,
    flags, reg_base + div_offset, SONG_CLK_SDIV_GR_GR_SHIFT,
    SONG_CLK_SDIV_GR_GR_WIDTH, mult_flags);
  if (!clk)
    return clk;

  return clk_register_fixed_factor(name, mult_clk, flags | CLK_NAME_IS_STATIC,
    1, fixed_div);
}

static struct clk *song_clk_register_sdiv_sdiv(const char *name,
    const char *parent_name, uint16_t flags,
    uint32_t reg_base, uint16_t div_offset,
    uint64_t private_flags)
{
  struct clk *clk;
  char gate_clk[SONG_CLK_NAME_MAX], div_clk[SONG_CLK_NAME_MAX];
  uint8_t gate_flags, div_flags;

  snprintf(gate_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "gate");
  snprintf(div_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "div");

  gate_flags = (private_flags >> SONG_CLK_GATE_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  div_flags = (private_flags >> SONG_CLK_DIV_FLAG_SHIFT) &
    SONG_CLK_DIV_MASK;

  clk = clk_register_gate(gate_clk, parent_name, flags | CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, SONG_CLK_SDIV_SDIV_EN_SHIFT, gate_flags);
  if (!clk)
    return clk;

  div_flags |= CLK_DIVIDER_ROUND_CLOSEST;
  clk = clk_register_divider(div_clk, gate_clk, flags,
    reg_base + div_offset, SONG_CLK_SDIV_SDIV_DIV0_SHIFT,
    SONG_CLK_SDIV_SDIV_DIV0_WIDTH, div_flags);
  if (!clk)
    return clk;

  flags |= CLK_SET_RATE_PARENT | CLK_NAME_IS_STATIC;
  clk = clk_register_divider(name, div_clk, flags,
    reg_base + div_offset, SONG_CLK_SDIV_SDIV_DIV1_SHIFT,
    SONG_CLK_SDIV_SDIV_DIV1_WIDTH, div_flags);

  return clk;
}

static struct clk *song_clk_register_gr_sdiv(const char *name,
    const char *parent_name, uint16_t flags,
    uint32_t reg_base, uint16_t div_offset,
    uint64_t private_flags)
{
  struct clk *clk;
  char gate_clk[SONG_CLK_NAME_MAX], div_clk[SONG_CLK_NAME_MAX],
       mult_clk[SONG_CLK_NAME_MAX];
  uint8_t mult_flags, song_flags, gate_flags, div_flags;
  uint8_t fixed_div = 8;

  snprintf(gate_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "gate");
  snprintf(div_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "div");
  snprintf(mult_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "mult");

  gate_flags = (private_flags >> SONG_CLK_GATE_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  song_flags = (private_flags >> SONG_CLK_PRIVATE_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  div_flags = (private_flags >> SONG_CLK_DIV_FLAG_SHIFT) &
    SONG_CLK_DIV_MASK;

  mult_flags = (private_flags >> SONG_CLK_MULT_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  if (song_flags & SONG_CLK_GR_DIV_16)
    fixed_div = 16;

  clk = clk_register_gate(gate_clk, parent_name, flags | CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, SONG_CLK_GR_SDIV_EN_SHIFT, gate_flags);
  if (!clk)
    return clk;

  div_flags |= CLK_DIVIDER_ROUND_CLOSEST;
  clk = clk_register_divider(div_clk, gate_clk, flags,
    reg_base + div_offset, SONG_CLK_GR_SDIV_DIV_SHIFT, SONG_CLK_GR_SDIV_DIV_WIDTH,
    div_flags);

  if (!clk)
    return clk;

  mult_flags |= CLK_MULT_ROUND_CLOSEST;
  clk = clk_register_multiplier(mult_clk, div_clk,
    flags, reg_base + div_offset, SONG_CLK_GR_SDIV_GR_SHIFT,
    SONG_CLK_GR_SDIV_GR_WIDTH, mult_flags);
  if (!clk)
    return clk;

  return clk_register_fixed_factor(name, mult_clk, flags | CLK_NAME_IS_STATIC,
    1, fixed_div);
}

static struct clk *song_clk_register_mux_sdiv_gr(const char *name,
    const char * const *parent_name, uint8_t num_parents,
    uint16_t flags, uint32_t reg_base,
    uint16_t div_offset, uint8_t div_width, uint8_t mux_width,
    uint64_t private_flags)
{
  struct clk *clk;
  char gate_clk[SONG_CLK_NAME_MAX], div_clk[SONG_CLK_NAME_MAX],
       mult_clk[SONG_CLK_NAME_MAX], mux_clk[SONG_CLK_NAME_MAX];
  uint8_t mux_flags, mult_flags, song_flags, gate_flags, div_flags;
  uint8_t fixed_div = 8;

  snprintf(mux_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "mux");
  snprintf(gate_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "gate");
  snprintf(div_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "div");
  snprintf(mult_clk, SONG_CLK_NAME_MAX, "%s_%s", name, "mult");

  mux_flags = (private_flags >> SONG_CLK_MUX_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;
  mux_flags |= CLK_MUX_ROUND_CLOSEST;

  gate_flags = (private_flags >> SONG_CLK_GATE_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  song_flags = (private_flags >> SONG_CLK_PRIVATE_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  div_flags = (private_flags >> SONG_CLK_DIV_FLAG_SHIFT) &
    SONG_CLK_DIV_MASK;

  mult_flags = (private_flags >> SONG_CLK_MULT_FLAG_SHIFT) &
    SONG_CLK_FLAG_MASK;

  if (song_flags & SONG_CLK_GR_DIV_16)
    fixed_div = 16;

  clk = clk_register_mux(mux_clk, parent_name, num_parents, flags | CLK_PARENT_NAME_IS_STATIC,
    reg_base + div_offset, SONG_CLK_MUX_SDIV_GR_MUX_SHIFT, MASK(mux_width), mux_flags);
  if (!clk)
    return clk;

  clk = clk_register_gate(gate_clk, mux_clk, flags,
    reg_base + div_offset, SONG_CLK_MUX_SDIV_GR_EN_SHIFT, gate_flags);
  if (!clk)
    return clk;

  div_flags |= CLK_DIVIDER_ROUND_CLOSEST;
  clk = clk_register_divider(div_clk, gate_clk, flags,
    reg_base + div_offset, SONG_CLK_MUX_SDIV_GR_DIV_SHIFT, div_width, div_flags);
  if (!clk)
    return clk;

  flags |= CLK_SET_RATE_PARENT;

  mult_flags |= CLK_MULT_ROUND_CLOSEST;
  clk = clk_register_multiplier(mult_clk, div_clk,
    flags, reg_base + div_offset, SONG_CLK_MUX_SDIV_GR_GR_SHIFT,
    SONG_CLK_MUX_SDIV_GR_GR_WIDTH, mult_flags);
  if (!clk)
    return clk;

  return clk_register_fixed_factor(name, mult_clk, flags | CLK_NAME_IS_STATIC,
    1, fixed_div);
}

static int song_register_fixed_rate_clks(
              const struct song_fixed_rate_clk *fixed_rate_clks)
{
  struct clk *clk = NULL;

  while (fixed_rate_clks->name)
    {
      clk = clk_register_fixed_rate(
              fixed_rate_clks->name,
              NULL,
              fixed_rate_clks->flags | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
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
  struct clk *clk = NULL;

  while (fixed_factor_clks->name)
    {
      clk = clk_register_fixed_factor(
              fixed_factor_clks->name,
              fixed_factor_clks->parent_name,
              fixed_factor_clks->flags | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              fixed_factor_clks->fixed_mul,
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
  struct clk *clk = NULL;

  while (gate_clks->name)
    {
      clk = song_clk_register_gate(
            gate_clks->name,
            gate_clks->parent_name,
            gate_clks->flags,
            reg_base,
            gate_clks->en_offset,
            gate_clks->en_shift,
            gate_clks->private_flags);

      if (!clk)
        {
          return -EINVAL;
        }

      gate_clks++;
    }

  return 0;
}

static int song_register_sdiv_clks(uint32_t reg_base,
            const struct song_sdiv_clk *sdiv_clks)
{
  struct clk *clk = NULL;

  while (sdiv_clks->name)
    {
      clk = song_clk_register_sdiv(sdiv_clks->name,
              sdiv_clks->parent_name,
              sdiv_clks->flags,
              reg_base,
              sdiv_clks->en_offset,
              sdiv_clks->en_shift,
              sdiv_clks->div_offset,
              sdiv_clks->div_shift,
              sdiv_clks->div_width,
              sdiv_clks->private_flags);

      if (!clk)
        {
          return -EINVAL;
        }

      sdiv_clks++;
    }

  return 0;
}

static int song_register_gr_clks(uint32_t reg_base,
            const struct song_gr_clk *gr_clks)
{
  struct clk *clk = NULL;

  while (gr_clks->name)
    {
      clk = song_clk_register_gr(gr_clks->name,
              gr_clks->parent_name,
              gr_clks->flags,
              reg_base,
              gr_clks->en_offset,
              gr_clks->en_shift,
              gr_clks->mul_offset,
              gr_clks->mul_shift,
              gr_clks->mul_width,
              gr_clks->private_flags);
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
  struct clk *clk = NULL;

  while (sdiv_sdiv_clks->name)
    {
      clk = song_clk_register_sdiv_sdiv(
              sdiv_sdiv_clks->name,
              sdiv_sdiv_clks->parent_name,
              sdiv_sdiv_clks->flags,
              reg_base,
              sdiv_sdiv_clks->div_offset,
              sdiv_sdiv_clks->private_flags);
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
  struct clk *clk = NULL;

  while (sdiv_fdiv_clks->name)
    {
      clk = song_clk_register_sdiv_fdiv(
              sdiv_fdiv_clks->name,
              sdiv_fdiv_clks->parent_name,
              sdiv_fdiv_clks->flags,
              reg_base,
              sdiv_fdiv_clks->en_offset,
              sdiv_fdiv_clks->en_shift,
              sdiv_fdiv_clks->sdiv_offset,
              sdiv_fdiv_clks->sdiv_shift,
              sdiv_fdiv_clks->sdiv_width,
              sdiv_fdiv_clks->fdiv_offset,
              sdiv_fdiv_clks->private_flags);
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
  struct clk *clk = NULL;

  while (gr_fdiv_clks->name)
    {
      clk = song_clk_register_gr_fdiv(
              gr_fdiv_clks->name,
              gr_fdiv_clks->parent_name,
              gr_fdiv_clks->flags,
              reg_base,
              gr_fdiv_clks->en_offset,
              gr_fdiv_clks->en_shift,
              gr_fdiv_clks->gr_offset,
              gr_fdiv_clks->div_offset,
              gr_fdiv_clks->fixed_gr,
              gr_fdiv_clks->private_flags);
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
  struct clk *clk = NULL;

  while (sdiv_gr_clks->name)
    {
      clk = song_clk_register_sdiv_gr(sdiv_gr_clks->name,
              sdiv_gr_clks->parent_name,
              sdiv_gr_clks->flags,
              reg_base,
              sdiv_gr_clks->div_offset,
              sdiv_gr_clks->private_flags);
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
  struct clk *clk = NULL;

  while (gr_sdiv_clks->name)
    {
      clk = song_clk_register_gr_sdiv(
              gr_sdiv_clks->name,
              gr_sdiv_clks->parent_name,
              gr_sdiv_clks->flags,
              reg_base,
              gr_sdiv_clks->div_offset,
              gr_sdiv_clks->private_flags);
      if (!clk)
        {
          return -EINVAL;
        }

      gr_sdiv_clks++;
    }

  return 0;
}

static int song_register_mux_sdiv_clks(uint32_t reg_base,
            const struct song_mux_sdiv_clk *mux_sdiv_clks)
{
  struct clk *clk = NULL;

  while (mux_sdiv_clks->name)
    {
      clk = song_clk_register_mux_sdiv(
              mux_sdiv_clks->name,
              mux_sdiv_clks->parent_name,
              mux_sdiv_clks->num_parents,
              mux_sdiv_clks->flags,
              reg_base,
              0,
              0,
              mux_sdiv_clks->mux_offset,
              mux_sdiv_clks->mux_shift,
              mux_sdiv_clks->mux_width,
              mux_sdiv_clks->div_offset,
              mux_sdiv_clks->div_shift,
              mux_sdiv_clks->div_width,
              mux_sdiv_clks->private_flags);
      if (!clk)
        {
          return -EINVAL;
        }

      mux_sdiv_clks++;
    }

  return 0;
}

static int song_register_mux_gate_clks(uint32_t reg_base,
            const struct song_mux_gate_clk *mux_gate_clks)
{
  struct clk *clk = NULL;

  while (mux_gate_clks->name)
    {
      clk = song_clk_register_mux_sdiv(
              mux_gate_clks->name,
              mux_gate_clks->parent_name,
              mux_gate_clks->num_parents,
              mux_gate_clks->flags,
              reg_base,
              mux_gate_clks->en_offset,
              mux_gate_clks->en_shift,
              mux_gate_clks->mux_offset,
              mux_gate_clks->mux_shift,
              mux_gate_clks->mux_width,
              0,
              0,
              0,
              mux_gate_clks->private_flags);
      if (!clk)
        {
          return -EINVAL;
        }

      mux_gate_clks++;
    }

  return 0;
}

static int song_register_phase_clks(uint32_t reg_base,
            const struct song_phase_clk *phase_clks)
{
  struct clk *clk = NULL;

  while (phase_clks->name)
    {
      clk = clk_register_phase(phase_clks->name,
              phase_clks->parent_name,
              phase_clks->flags | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              reg_base + phase_clks->reg_offset,
              phase_clks->phase_shift,
              phase_clks->phase_width,
              phase_clks->phase_flags);
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
  struct clk *clk = NULL;

  while (mux_sdiv_gr_clks->name)
    {
      clk = song_clk_register_mux_sdiv_gr(
              mux_sdiv_gr_clks->name,
              mux_sdiv_gr_clks->parent_name,
              mux_sdiv_gr_clks->num_parents,
              mux_sdiv_gr_clks->flags,
              reg_base,
              mux_sdiv_gr_clks->div_offset,
              mux_sdiv_gr_clks->div_width,
              mux_sdiv_gr_clks->mux_width,
              mux_sdiv_gr_clks->private_flags);
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
  struct clk *clk = NULL;

  while (pll_clks->name)
    {
      clk = clk_register_pll(
              pll_clks->name,
              pll_clks->parent_name,
              pll_clks->flags | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              reg_base + pll_clks->cfg_reg0_offset,
              reg_base + pll_clks->cfg_reg1_offset,
              reg_base + pll_clks->ctl_reg_offset,
              pll_clks->ctl_shift);
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
  struct clk *clk = NULL;

  while (pll_lf_clks->name)
    {
      clk = clk_register_pll_lf(
              pll_lf_clks->name,
              pll_lf_clks->parent_name,
              pll_lf_clks->flags | CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              reg_base + pll_lf_clks->cfg_reg0_offset,
              reg_base + pll_lf_clks->cfg_reg1_offset);
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
  struct clk *clk = NULL;

  while (out_clks->name)
    {
      clk = clk_register_out(
              out_clks->name,
              out_clks->parent_name,
              out_clks->num_parents,
              CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              reg_base + out_clks->mux_reg_offset,
              out_clks->mux_shift,
              out_clks->mux_width,
              reg_base + out_clks->ctl_reg_offset);
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
  struct clk *clk = NULL;

  while (timer_clks->name)
    {
      clk = clk_register_timer(
              timer_clks->name,
              timer_clks->parent_name,
              timer_clks->num_parents,
              CLK_NAME_IS_STATIC | CLK_PARENT_NAME_IS_STATIC,
              reg_base + timer_clks->ctl_reg_offset,
              timer_clks->mux_shift,
              timer_clks->mux_width);
      if (!clk)
        {
          return -EINVAL;
        }

      timer_clks++;
    }

  return 0;
}

static int song_set_default_rate(const struct song_default_rate_clk *def_rate)
{
  struct clk *clk = NULL;
  int ret = 0;

  while (def_rate->name)
    {
      clk = clk_get(def_rate->name);
      if (!clk)
        return -EINVAL;
      ret = clk_set_rate(clk, def_rate->rate);
      if (ret)
        return ret;

      def_rate++;
    }

  return 0;
}

static int song_apply_patch(uint32_t reg_base,
            const struct song_clk_patch *patch)
{
  while (patch->value)
    {
      clk_write(reg_base + patch->offset, patch->value);
      patch++;
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
 *   table     - the song platform clk table that to be configurated.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int song_clk_initialize(uint32_t base, const struct song_clk_table *table)
{
  int ret = 0;

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

  if (table->sdiv_clks)
    {
      ret = song_register_sdiv_clks(base, table->sdiv_clks);
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

  if (table->mux_sdiv_clks)
    {
      ret = song_register_mux_sdiv_clks(base, table->mux_sdiv_clks);
      if (ret)
        return ret;
    }

  if (table->mux_gate_clks)
    {
      ret = song_register_mux_gate_clks(base, table->mux_gate_clks);
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

#ifdef CONFIG_CLK_RPMSG
  ret = clk_rpmsg_initialize(table->rpmsg_server);
  if (ret)
    return ret;
#endif

  if (table->def_rate)
    {
      ret = song_set_default_rate(table->def_rate);
      if (ret)
        return ret;
    }

  if (table->patch_table)
    {
      ret = song_apply_patch(base, table->patch_table);
      if (ret)
        return ret;
    }

  return 0;
}

#endif /* CONFIG_SONG_CLK */
