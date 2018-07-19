/****************************************************************************
 * drivers/clk/song/clk-out.c
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

#include <nuttx/clk/clk-provider.h>

#include "song-clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_out(_clk) (struct clk_out *)(_clk->private_data)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct div_args_s
{
  uint8_t shift;
  uint8_t mask;
};

/****************************************************************************
 * Prviate Data
 ****************************************************************************/

static const struct div_args_s g_div_para[] =
{
  {0, 0xff},
  {8, 0xff},
  {16, 0xff},
  {24, 0xf},
  {28, 0xf},
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int __clk_out_set_parent(struct clk *clk, uint8_t index)
{
  struct clk_out *out = to_clk_out(clk);
  uint32_t val;

  val = clk_read(out->mux_reg);
  val &= ~(MASK(out->mux_width) << out->mux_shift);
  val |= (index & MASK(out->mux_width)) << out->mux_shift;

  clk_write(out->mux_reg, val);
  return 0;
}

static int clk_out_enable(struct clk *clk)
{
  struct clk_out *out = to_clk_out(clk);

  return __clk_out_set_parent(clk, out->parent_index);
}

static void clk_out_disable(struct clk *clk)
{
  __clk_out_set_parent(clk, 6);
}

static int clk_out_is_enabled(struct clk *clk)
{
  struct clk_out *out = to_clk_out(clk);
  uint32_t val;

  val   = clk_read(out->mux_reg);
  val >>= out->mux_shift;
  val  &= MASK(out->mux_width);

  return val != 6;
}

static uint32_t clk_out_recalc_rate(struct clk *clk, uint32_t parent_rate)
{
  struct clk_out *out = to_clk_out(clk);
  uint8_t src_sel;
  uint16_t div;

  src_sel = out->parent_index;

  if (src_sel == 5)
    return parent_rate;

  div = clk_read(out->ctl_reg) >> g_div_para[src_sel].shift & g_div_para[src_sel].mask;

  if (src_sel == 1)
    return parent_rate / (div + 1);
  else
    return parent_rate / (2 * (div + 1));
}

static uint32_t clk_out_round_rate(struct clk *clk, uint32_t rate,
                                   uint32_t *best_parent_rate)
{
  struct clk_out *out = to_clk_out(clk);
  uint8_t src_sel;
  uint16_t div, max_div;

  src_sel = out->parent_index;

  if ((src_sel == 4 || src_sel == 5) && rate == *best_parent_rate)
    return *best_parent_rate;

  max_div = g_div_para[src_sel].mask + 1;

  if (src_sel == 1)
  {
    div = DIV_ROUND_CLOSEST(*best_parent_rate, rate);
    div = div > max_div ? max_div : div;
    return DIV_ROUND_CLOSEST(*best_parent_rate, div);
  } else {
    div = DIV_ROUND_CLOSEST(*best_parent_rate, 2 * rate);
    div = div > max_div ? max_div : div;
    return DIV_ROUND_CLOSEST(*best_parent_rate, 2 * div);
  }
}

static int clk_out_set_parent(struct clk *clk, uint8_t index)
{
  struct clk_out *out = to_clk_out(clk);

  out->parent_index = index;
  return __clk_out_set_parent(clk, index);
}

static uint8_t clk_out_get_parent(struct clk *clk)
{
  struct clk_out *out = to_clk_out(clk);

  return out->parent_index;
}

static int clk_out_set_rate(struct clk *clk, uint32_t rate, uint32_t parent_rate)
{
  struct clk_out *out = to_clk_out(clk);
  uint8_t src_sel;
  uint16_t div;
  uint32_t val;

  src_sel = out->parent_index;

  if ((src_sel == 4 || src_sel == 5) && rate == parent_rate)
    return clk_out_set_parent(clk, 5);

  if (src_sel == 5 && rate != parent_rate)
    clk_out_set_parent(clk, 4);

  if (src_sel == 1)
    div = DIV_ROUND_CLOSEST(parent_rate, rate) - 1;
  else
    div = DIV_ROUND_CLOSEST(parent_rate, 2 * rate) - 1;

  val = clk_read(out->ctl_reg);
  val &= ~(g_div_para[src_sel].mask << g_div_para[src_sel].shift);
  val |= div << g_div_para[src_sel].shift;

  clk_write(out->ctl_reg, val);

  return 0;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops clk_out_ops =
{
  .enable = clk_out_enable,
  .disable = clk_out_disable,
  .is_enabled = clk_out_is_enabled,
  .recalc_rate = clk_out_recalc_rate,
  .round_rate = clk_out_round_rate,
  .set_parent = clk_out_set_parent,
  .get_parent = clk_out_get_parent,
  .set_rate = clk_out_set_rate,
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

struct clk *clk_register_out(const char *name, const char * const *parent_names,
                             uint8_t num_parents, uint8_t flags, uint32_t mux_reg,
                             uint8_t mux_shift, uint8_t mux_width, uint32_t ctl_reg)
{
  struct clk_out out;

  out.mux_reg = mux_reg;
  out.mux_shift = mux_shift;
  out.mux_width = mux_width;

  out.ctl_reg = ctl_reg;
  out.parent_index = clk_read(mux_reg) >> mux_shift & MASK(mux_width);

  return clk_register(name, parent_names, num_parents, flags,
                      &clk_out_ops, &out, sizeof(out));
}
