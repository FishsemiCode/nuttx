/****************************************************************************
 * drivers/clk/clk-multiplier.c
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

#define to_clk_multiplier(_clk) \
    (struct clk_multiplier *)(_clk->private_data)

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static uint32_t _get_maxmult(struct clk_multiplier *multiplier)
{
  if (multiplier->flags & CLK_MULT_ONE_BASED)
    return MASK(multiplier->width);
  if (multiplier->flags & CLK_MULT_MAX_HALF)
    return 1 << (multiplier->width - 1);
  return MASK(multiplier->width) + 1;
}

static uint32_t _get_mult(struct clk_multiplier *multiplier, uint32_t val)
{
  if (multiplier->flags & CLK_MULT_ONE_BASED)
    return val;
  return val + 1;
}

static uint32_t _get_val(struct clk_multiplier *multiplier, uint32_t mult)
{
  if (multiplier->flags & CLK_MULT_ONE_BASED)
    return mult;
  return mult - 1;
}

static uint32_t clk_multiplier_recalc_rate(struct clk *clk,
    uint32_t parent_rate)
{
  struct clk_multiplier *multiplier = to_clk_multiplier(clk);
  uint32_t mult, val;

  val = clk_read(multiplier->reg) >> multiplier->shift;
  val &= MASK(multiplier->width);

  mult = _get_mult(multiplier, val);
  if (!mult)
    {
    if (!(multiplier->flags & CLK_MULT_ALLOW_ZERO))
        clkerr("CLK_MULT_ALLOW_ZERO not set\n", clk->name);
    return parent_rate;
    }

  return parent_rate * mult;
}

static bool __is_best_rate(uint32_t rate, uint32_t new,
    uint32_t best, uint16_t flags)
{
  if (flags & CLK_MULT_ROUND_CLOSEST)
    return abs(rate - new) < abs(rate - best);

  return new >= rate && new < best;
}

static uint32_t clk_multiplier_bestmult(struct clk *clk, uint32_t rate,
    uint32_t *best_parent_rate)
{
  struct clk_multiplier *multiplier = to_clk_multiplier(clk);
  uint32_t i, bestmult = 0;
  uint32_t parent_rate, best = 0, now, maxmult;
  uint32_t parent_rate_saved = *best_parent_rate;

  if (!rate)
    rate = 1;

  maxmult = _get_maxmult(multiplier);

  if (!(clk->flags & CLK_SET_RATE_PARENT))
    {
      parent_rate = *best_parent_rate;
      bestmult = rate / parent_rate;
      bestmult = bestmult == 0 ? 1 : bestmult;
      bestmult = bestmult > maxmult ? maxmult : bestmult;
      return bestmult;
    }

  for (i = maxmult; i >= 1; i--)
    {
      if (rate == parent_rate_saved * i)
        {
          *best_parent_rate = parent_rate_saved;
          return i;
        }
      parent_rate = clk_round_rate(clk_get_parent(clk),
          rate / i);
      now = parent_rate * i;
      if (__is_best_rate(rate, now, best, multiplier->flags))
        {
          bestmult = i;
          best = now;
          *best_parent_rate = parent_rate;
        }
    }

  if (!bestmult)
    {
      bestmult = 1;
      *best_parent_rate = clk_round_rate(clk_get_parent(clk), 1);
    }

  return bestmult;
}

static uint32_t clk_multiplier_round_rate(struct clk *clk, uint32_t rate,
        uint32_t *prate)
{
  uint32_t mult;
  mult = clk_multiplier_bestmult(clk, rate, prate);

  return *prate * mult;
}

static int clk_multiplier_set_rate(struct clk *clk, uint32_t rate,
        uint32_t parent_rate)
{
  struct clk_multiplier *multiplier = to_clk_multiplier(clk);
  uint32_t mult, value;
  uint32_t val;

  mult = rate / parent_rate;
  value = _get_val(multiplier, mult);

  if (value > MASK(multiplier->width))
    value = MASK(multiplier->width);

  if (multiplier->flags & CLK_MULT_HIWORD_MASK)
    {
      val = MASK(multiplier->width) << (multiplier->shift + 16);
    }
  else
    {
      val = clk_read(multiplier->reg);
      val &= ~(MASK(multiplier->width) << multiplier->shift);
    }
  val |= value << multiplier->shift;
  clk_write(multiplier->reg, val);

  return 0;
}

/************************************************************************************
 * Public Data
 ************************************************************************************/

const struct clk_ops clk_multiplier_ops =
{
  .recalc_rate = clk_multiplier_recalc_rate,
  .round_rate = clk_multiplier_round_rate,
  .set_rate = clk_multiplier_set_rate,
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

struct clk *clk_register_multiplier(const char *name,
    const char *parent_name, uint8_t flags,
    uint32_t reg, uint8_t shift, uint8_t width,
    uint8_t clk_multiplier_flags)
{
  struct clk_multiplier *mult;
  struct clk *clk;
  uint8_t num_parents;
  const char **parent_names;

  if (clk_multiplier_flags & CLK_MULT_HIWORD_MASK)
    {
      if (width + shift > 16)
      {
          clkerr("multiplier value exceeds LOWORD field\n");
          return NULL;
      }
    }

  mult = kmm_malloc(sizeof(struct clk_multiplier));
  if (!mult)
    {
      clkerr("could not allocate multiplier clk\n");
      return NULL;
    }

  parent_names = (parent_name ? &parent_name : NULL);
  num_parents = (parent_name ? 1 : 0);

  mult->reg = reg;
  mult->shift = shift;
  mult->width = width;
  mult->flags = clk_multiplier_flags;

  clk = clk_register(name, parent_names, num_parents, flags, &clk_multiplier_ops, mult);
  if (!clk)
    {
      kmm_free(mult);
    }

  return clk;
}

