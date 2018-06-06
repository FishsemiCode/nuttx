/****************************************************************************
 * drivers/clk/clk-gate.c
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

#define to_clk_gate(_clk) \
    (struct clk_gate *)(_clk->private_data)

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static void clk_gate_endisable(struct clk *clk, int32_t enable)
{
  struct clk_gate *gate = to_clk_gate(clk);
  int32_t set = gate->flags & CLK_GATE_SET_TO_DISABLE ? 1 : 0;
  uint32_t val;

  set ^= enable;

  if (gate->flags & CLK_GATE_HIWORD_MASK)
    {
      val = BIT(gate->bit_idx + 16);
      if (set)
      val |= BIT(gate->bit_idx);
    }
  else
    {
      val = clk_read(gate->reg);

      if (set)
        val |= BIT(gate->bit_idx);
      else
        val &= ~BIT(gate->bit_idx);
    }

  clk_write(gate->reg, val);
}

static int clk_gate_enable(struct clk *clk)
{
  clk_gate_endisable(clk, 1);

  return 0;
}

static void clk_gate_disable(struct clk *clk)
{
  clk_gate_endisable(clk, 0);
}

static int clk_gate_is_enabled(struct clk *clk)
{
  struct clk_gate *gate = to_clk_gate(clk);
  uint32_t val;

  val = clk_read(gate->reg);

  /* if a set bit disables this clk, flip it before masking */
  if (gate->flags & CLK_GATE_SET_TO_DISABLE)
    val ^= BIT(gate->bit_idx);

  val &= BIT(gate->bit_idx);

  return val ? 1 : 0;
}

/************************************************************************************
 * Public Data
 ************************************************************************************/

const struct clk_ops clk_gate_ops =
{
  .enable = clk_gate_enable,
  .disable = clk_gate_disable,
  .is_enabled = clk_gate_is_enabled,
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

struct clk *clk_register_gate(const char *name,
    const char *parent_name, uint64_t flags,
    uint32_t reg, uint8_t bit_idx,
    uint8_t clk_gate_flags)
{
  struct clk_gate *gate;
  struct clk *clk;
  int32_t num_parents;
  const char **parent_names;

  if (clk_gate_flags & CLK_GATE_HIWORD_MASK)
    {
      if (bit_idx > 16)
        {
          clkerr("gate bit exceeds LOWORD field\n");
          return NULL;
        }
    }

  gate = kmm_malloc(sizeof(struct clk_gate));
  if (!gate)
    {
      clkerr("could not allocate gated clk\n");
      return NULL;
    }

  num_parents = (parent_name ? 1 : 0);
  parent_names = parent_name ? &parent_name : NULL;

  gate->reg = reg;
  gate->bit_idx = bit_idx;
  gate->flags = clk_gate_flags;

  clk = clk_register(name, num_parents, parent_names, flags, &clk_gate_ops, gate);
  if (!clk)
    {
      kmm_free(gate);
      return NULL;
    }

  return clk;
}
