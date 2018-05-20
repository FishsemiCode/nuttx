/****************************************************************************
 * drivers/clk/song/clk-timer.c
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

#include <nuttx/clk/clk.h>
#include <nuttx/clk/clk-provider.h>
#include <nuttx/clk/song/song-clk.h>
#include <nuttx/kmalloc.h>

#include "song-clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_timer(_clk) (struct clk_timer *)(_clk->private_data)

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct div_args_s
{
  uint8_t shift;
  uint8_t mask;
};

/************************************************************************************
 * Private Data
 ************************************************************************************/

static const struct div_args_s g_div_para[] =
{
  {0, 0xff},
  {8, 0xff},
  {16, 0xff},
  {24, 0xf},
};

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int __clk_timer_set_parent(struct clk *clk, uint8_t index)
{
  struct clk_timer *timer = to_clk_timer(clk);
  unsigned int val;

  val = clk_read(timer->ctl_reg);
  val &= ~(MASK(timer->mux_width) << timer->mux_shift);
  val |= (index & MASK(timer->mux_width)) << timer->mux_shift;

  clk_write(val, timer->ctl_reg);
  return 0;
}

static int clk_timer_enable(struct clk *clk)
{
  struct clk_timer *timer = to_clk_timer(clk);

  return __clk_timer_set_parent(clk, timer->parent_index);
}

static void clk_timer_disable(struct clk *clk)
{
  __clk_timer_set_parent(clk, 6);
}

static uint64_t clk_timer_recalc_rate(struct clk *clk,
    uint64_t parent_rate)
{
  struct clk_timer *timer = to_clk_timer(clk);
  uint8_t src_sel, div;

  src_sel = timer->parent_index;

  if (src_sel == 4 || src_sel == 5)
    return parent_rate;

  div = clk_read(timer->ctl_reg) >> g_div_para[src_sel].shift & g_div_para[src_sel].mask;

  return parent_rate / (2 * (div + 1));
}

static int64_t clk_timer_round_rate(struct clk *clk, uint64_t rate,
    uint64_t *best_parent_rate)
{
  struct clk_timer *timer = to_clk_timer(clk);
  uint8_t src_sel, div, max_div;

  src_sel = timer->parent_index;

  if ((src_sel == 3 || src_sel == 4 || src_sel == 5) && rate == *best_parent_rate)
    return *best_parent_rate;

  max_div = g_div_para[src_sel].mask + 1;

  div = DIV_ROUND_CLOSEST(*best_parent_rate, 2 * rate);
  div = div > max_div ? max_div : div;
  return DIV_ROUND_CLOSEST(*best_parent_rate, 2 * div);
}

static int clk_timer_set_parent(struct clk *clk, uint8_t index)
{
  struct clk_timer *timer = to_clk_timer(clk);

  timer->parent_index = index;
  return __clk_timer_set_parent(clk, index);
}

static uint8_t clk_timer_get_parent(struct clk *clk)
{
  struct clk_timer *timer = to_clk_timer(clk);

  return timer->parent_index;
}

static int clk_timer_set_rate(struct clk *clk, uint64_t rate,
    uint64_t parent_rate)
{
  struct clk_timer *timer = to_clk_timer(clk);
  uint8_t src_sel, div;
  unsigned int val;

  src_sel = timer->parent_index;

  if (src_sel == 3 && rate == parent_rate)
    return clk_timer_set_parent(clk, 4);

  if ((src_sel == 4 && rate == parent_rate) || src_sel == 5)
    return 0;

  if (src_sel == 4 && rate != parent_rate)
    clk_timer_set_parent(clk, 3);

  div = DIV_ROUND_CLOSEST(parent_rate, 2 * rate) - 1;

  val = clk_read(timer->ctl_reg);
  val &= ~(g_div_para[src_sel].mask << g_div_para[src_sel].shift);
  val |= div << g_div_para[src_sel].shift;

  clk_write(val, timer->ctl_reg);

  return 0;
}

/************************************************************************************
 * Public Data
 ************************************************************************************/

const struct clk_ops clk_timer_ops =
{
  .enable = clk_timer_enable,
  .disable = clk_timer_disable,
  .recalc_rate = clk_timer_recalc_rate,
  .round_rate = clk_timer_round_rate,
  .set_parent = clk_timer_set_parent,
  .get_parent = clk_timer_get_parent,
  .set_rate = clk_timer_set_rate,
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

struct clk *clk_register_timer(const char *name, const char **parent_names,
    uint8_t num_parents, uint8_t ctl_reg, uint8_t mux_shift, uint8_t mux_width)
{
  struct clk_timer *timer;
  struct clk *clk;

  timer = kmm_zalloc(sizeof(struct clk_timer));
  if (!timer)
    return NULL;

  timer->ctl_reg = ctl_reg;
  timer->mux_shift = mux_shift;
  timer->mux_width = mux_width;
  timer->parent_index = clk_read(ctl_reg) >> mux_shift & MASK(mux_width);

  clk = clk_register(name, num_parents, parent_names, CLK_IS_BASIC, &clk_timer_ops, timer);
  if (!clk)
  {
    kmm_free(timer);
  }

  return clk;
}
