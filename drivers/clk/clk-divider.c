/****************************************************************************
 * drivers/clk/clk-divider.c
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

#include <debug.h>
#include <stdlib.h>

#include "clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_divider(_clk) (struct clk_divider *)(_clk->private_data)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t _get_maxdiv(uint8_t width, uint16_t flags)
{
  if (flags & CLK_DIVIDER_MAX_HALF)
    return MASK(width - 1) + 1;
  if (flags & CLK_DIVIDER_ONE_BASED)
    return MASK(width);
  if (flags & CLK_DIVIDER_DIV_NEED_EVEN)
    return MASK(width) - 1;
  return MASK(width) + 1;
}

static uint32_t _get_div(uint32_t val, uint16_t flags)
{
  if (flags & CLK_DIVIDER_ONE_BASED)
    return val;
  return val + 1;
}

static uint32_t _get_val(uint32_t div, uint16_t flags)
{
  if (flags & CLK_DIVIDER_ONE_BASED)
    return div;
  return div - 1;
}

static uint32_t divider_recalc_rate(uint32_t parent_rate,
                                    uint32_t val, uint16_t flags)
{
  uint32_t div;

  div = _get_div(val, flags);
  if (!div)
    {
      return parent_rate;
    }

  return DIV_ROUND_UP(parent_rate, div);
}

static uint32_t clk_divider_recalc_rate(struct clk *clk, uint32_t parent_rate)
{
  struct clk_divider *divider = to_clk_divider(clk);
  uint32_t val;

  val = clk_read(divider->reg) >> divider->shift;
  val &= MASK(divider->width);

  return divider_recalc_rate(parent_rate, val, divider->flags);
}

static uint32_t _div_round_closest(uint32_t parent_rate, uint32_t rate)
{
  uint32_t up, down;
  uint32_t up_rate, down_rate;

  up = DIV_ROUND_UP(parent_rate, rate);
  down = parent_rate / rate;

  up_rate = DIV_ROUND_UP(parent_rate, up);
  down_rate = DIV_ROUND_UP(parent_rate, down);

  return (rate - up_rate) <= (down_rate - rate) ? up : down;
}

static uint32_t _div_round(uint32_t parent_rate, uint32_t rate,
                           uint16_t flags)
{
  if (flags & CLK_DIVIDER_ROUND_CLOSEST)
    return _div_round_closest(parent_rate, rate);

  return DIV_ROUND_UP(parent_rate, rate);
}

static bool _is_best_div(uint32_t rate, uint32_t now,
                         uint32_t best, uint16_t flags)
{
  if (flags & CLK_DIVIDER_ROUND_CLOSEST)
    return abs(rate - now) < abs(rate - best);

  return now <= rate && now > best;
}

static uint32_t clk_divider_bestdiv(struct clk *clk, uint32_t rate,
                                    uint32_t *best_parent_rate,
                                    uint8_t width)
{
  uint32_t i, bestdiv = 0, maxdiv, mindiv, step;
  uint32_t parent_rate, best = 0, now;
  uint32_t parent_rate_saved = *best_parent_rate;
  struct clk_divider *divider = to_clk_divider(clk);

  if (!rate)
    rate = 1;

  if (divider->flags & CLK_DIVIDER_READ_ONLY)
    {
      bestdiv = clk_read(divider->reg) >> divider->shift;
      bestdiv &= MASK(divider->width);
      bestdiv = _get_div(bestdiv, divider->flags);
      return bestdiv;
    }

  maxdiv = _get_maxdiv(width, divider->flags);

  step = divider->flags & CLK_DIVIDER_DIV_NEED_EVEN ? 2 : 1;
  mindiv = (divider->flags >> CLK_DIVIDER_MINDIV_OFF) & CLK_DIVIDER_MINDIV_MSK;
  if (mindiv == 0)
    mindiv = step;

  if (!(clk->flags & CLK_SET_RATE_PARENT))
    {
      parent_rate = *best_parent_rate;
      bestdiv = _div_round(parent_rate, rate, divider->flags);
      bestdiv = bestdiv == 0 ? mindiv : bestdiv;
      bestdiv = bestdiv > maxdiv ? maxdiv : bestdiv;
      return bestdiv;
    }

  maxdiv = MIN(UINT32_MAX / rate, maxdiv);
  for (i = mindiv; i <= maxdiv; i += step)
    {
      parent_rate = clk_round_rate(clk_get_parent(clk),
          MULT_ROUND_UP(rate, i));
      now = DIV_ROUND_UP(parent_rate, i);
      if (_is_best_div(rate, now, best, divider->flags))
        {
          bestdiv = i;
          best = now;
          *best_parent_rate = parent_rate;
        }
    }

  if (!bestdiv)
    {
      bestdiv = _get_maxdiv(width, divider->flags);
      *best_parent_rate = clk_round_rate(clk_get_parent(clk), 1);
    }

  return bestdiv;
}

static uint32_t divider_round_rate(struct clk *clk, uint32_t rate,
                                   uint32_t *prate, uint8_t width)
{
  uint32_t div;

  div = clk_divider_bestdiv(clk, rate, prate, width);

  return DIV_ROUND_UP(*prate, div);
}

static uint32_t clk_divider_round_rate(struct clk *clk, uint32_t rate,
                                       uint32_t *prate)
{
  struct clk_divider *divider = to_clk_divider(clk);

  return divider_round_rate(clk, rate, prate, divider->width);
}

static uint32_t divider_get_val(uint32_t rate, uint32_t parent_rate,
                                uint8_t width, uint16_t flags)
{
  uint32_t div, value;

  div = DIV_ROUND_UP(parent_rate, rate);

  value = _get_val(div, flags);

  return MIN(value, MASK(width));
}

static int clk_divider_set_rate(struct clk *clk, uint32_t rate,
                                uint32_t parent_rate)
{
  struct clk_divider *divider = to_clk_divider(clk);
  uint32_t value;
  uint32_t val;

  value = divider_get_val(rate, parent_rate, divider->width, divider->flags);

  if (divider->flags & CLK_DIVIDER_HIWORD_MASK)
    {
      val = MASK(divider->width) << (divider->shift + 16);
    }
  else
    {
      val = clk_read(divider->reg);
      val &= ~(MASK(divider->width) << divider->shift);
    }
  val |= value << divider->shift;
  clk_write(divider->reg, val);

  return 0;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops clk_divider_ops =
{
  .recalc_rate = clk_divider_recalc_rate,
  .round_rate = clk_divider_round_rate,
  .set_rate = clk_divider_set_rate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct clk *clk_register_divider(const char *name, const char *parent_name,
                                 uint8_t flags, uint32_t reg, uint8_t shift,
                                 uint8_t width, uint16_t clk_divider_flags)
{
  struct clk_divider div;
  const char **parent_names;
  uint8_t num_parents;

  if (clk_divider_flags & CLK_DIVIDER_HIWORD_MASK)
    {
      if (width + shift > 16)
      {
        clkerr("divider value exceeds LOWORD field\n");
        return NULL;
      }
    }

  parent_names = parent_name ? &parent_name: NULL;
  num_parents = parent_name ? 1 : 0;

  div.reg = reg;
  div.shift = shift;
  div.width = width;
  div.flags = clk_divider_flags;

  return clk_register(name, parent_names, num_parents, flags,
                      &clk_divider_ops, &div, sizeof(div));
}
