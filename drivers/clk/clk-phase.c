/****************************************************************************
 * drivers/clk/clk-phase.c
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

#define to_clk_phase(_clk) (struct clk_phase *)(_clk->private_data)

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static int clk_phase_get_phase(struct clk *clk)
{
  struct clk_phase *phase = to_clk_phase(clk);
  uint32_t val;

  val = (clk_read(phase->reg) >> phase->shift) &
    MASK(phase->width);

  return DIV_ROUND_CLOSEST(360 * val, MASK(phase->width) + 1);
}

static int clk_phase_set_phase(struct clk *clk, int degrees)
{
  struct clk_phase *phase = to_clk_phase(clk);
  uint32_t pha, val;

  pha = DIV_ROUND_CLOSEST((MASK(phase->width) + 1) * degrees, 360);

  if (pha > MASK(phase->width))
    pha = MASK(phase->width);

  if (phase->flags & CLK_PHASE_HIWORD_MASK)
    {
      val = MASK(phase->width) << (phase->shift + 16);
    }
  else
    {
      val = clk_read(phase->reg);
      val &= ~(MASK(phase->width) << phase->shift);
    }

  val |= pha << phase->shift;
  clk_write(phase->reg, val);

  return 0;
}

/************************************************************************************
 * Public Data
 ************************************************************************************/

const struct clk_ops clk_phase_ops =
{
  .get_phase = clk_phase_get_phase,
  .set_phase = clk_phase_set_phase,
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

struct clk *clk_register_phase(const char *name, const char *parent_name,
    uint64_t flags, uint32_t reg, uint32_t shift,
    uint8_t width, uint8_t clk_phase_flags)
{
  struct clk_phase *phase;
  struct clk *clk;
  uint8_t num_parents;
  const char **parent_names;

  phase = kmm_malloc(sizeof(struct clk_phase));
  if (!phase)
    {
      return NULL;
    }

  parent_names = (parent_name ? &parent_name : NULL);
  num_parents = (parent_name ? 1 : 0);

  phase->reg = reg;
  phase->shift = shift;
  phase->width = width;
  phase->flags = clk_phase_flags;

  clk = clk_register(name, parent_names, num_parents, flags, &clk_phase_ops, phase);
  if (!clk)
    {
      kmm_free(phase);
    }

  return clk;
}
