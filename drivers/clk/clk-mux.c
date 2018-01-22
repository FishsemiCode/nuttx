/****************************************************************************
 * drivers/clk/clk-mux.c
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
#include <nuttx/kmalloc.h>

#include <debug.h>

#include "clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_mux(_clk) \
    (struct clk_mux *)(_clk->private_data)

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static bool mux_is_better_rate(uint64_t rate, uint64_t now,
        uint64_t best, uint64_t flags)
{
  if (flags & CLK_MUX_ROUND_CLOSEST)
    return abs(now - rate) < abs(best - rate);

  return now <= rate && now > best;
}

static uint8_t clk_mux_get_parent(struct clk *clk)
{
  struct clk_mux *mux = to_clk_mux(clk);
  uint32_t val;

  val = clk_read(mux->reg) >> mux->shift;
  val &= mux->mask;

  return val;
}

static int clk_mux_set_parent(struct clk *clk, uint8_t index)
{
  struct clk_mux *mux = to_clk_mux(clk);
  uint32_t val;

  if (mux->flags & CLK_MUX_HIWORD_MASK)
    {
      val = mux->mask << (mux->shift + 16);
    }
  else
    {
      val = clk_read(mux->reg);
      val &= ~(mux->mask << mux->shift);
    }
  val |= index << mux->shift;
  clk_write(mux->reg, val);

  return 0;
}

static int64_t
clk_mux_determine_rate(struct clk *clk, uint64_t rate,
      uint64_t *best_parent_rate,
      struct clk **best_parent_p)
{
  struct clk_mux *mux = to_clk_mux(clk);
  struct clk *parent, *best_parent = NULL;
  int32_t i, num_parents;
  uint64_t parent_rate, best = 0;

  if (clk->flags & CLK_SET_RATE_NO_REPARENT)
    {
      parent = clk->parent;
      if (clk->flags & CLK_SET_RATE_PARENT)
        best = clk_round_rate(parent, rate);
      else if (parent)
        best = clk_get_rate(parent);
      else
        best = clk_get_rate(clk);
      goto out;
    }

  num_parents = clk->num_parents;
  for (i = 0; i < num_parents; i++)
    {
      parent = clk_get_parent_by_index(clk, i);
      if (!parent)
        continue;
      if (clk->flags & CLK_SET_RATE_PARENT)
        parent_rate = clk_round_rate(parent, rate);
      else
        parent_rate = clk_get_rate(parent);
      if (mux_is_better_rate(rate, parent_rate, best, mux->flags))
        {
          best_parent = parent;
          best = parent_rate;
        }
    }

out:
  if (best_parent)
    *best_parent_p = best_parent;
  *best_parent_rate = best;

  return best;
}

/************************************************************************************
 * Public Data
 ************************************************************************************/

const struct clk_ops clk_mux_ops =
{
  .get_parent = clk_mux_get_parent,
  .set_parent = clk_mux_set_parent,
  .determine_rate = clk_mux_determine_rate,
};

const struct clk_ops clk_mux_ro_ops =
{
  .get_parent = clk_mux_get_parent,
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

struct clk *clk_register_mux(const char *name, const char **parent_names,
    uint8_t  num_parents, uint64_t flags,
    uint32_t reg, uint8_t shift, uint8_t mask,
    uint8_t clk_mux_flags)
{
  struct clk_mux *mux;
  struct clk *clk;

  mux = kmm_malloc(sizeof(struct clk_mux));
  if (!mux)
    {
      clkerr("could not allocate mux clk\n");
      return NULL;
    }

  mux->reg = reg;
  mux->shift = shift;
  mux->mask = mask;
  mux->flags = clk_mux_flags;

  if (clk_mux_flags & CLK_MUX_READ_ONLY)
    clk = clk_register(name, num_parents, parent_names, flags | CLK_IS_BASIC,
        &clk_mux_ro_ops, mux);
  else
    clk = clk_register(name, num_parents, parent_names, flags | CLK_IS_BASIC,
          &clk_mux_ops, mux);

  if (!clk)
    {
      kmm_free(mux);
    }

  return clk;
}
