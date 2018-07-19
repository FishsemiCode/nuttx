/****************************************************************************
 * drivers/clk/clk-fixed-rate.c
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

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define to_clk_fixed_rate(_clk) (struct clk_fixed_rate *)(_clk->private_data)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t clk_fixed_rate_recalc_rate(struct clk *clk, uint32_t parent_rate)
{
  struct clk_fixed_rate *fixed = to_clk_fixed_rate(clk);
  return fixed->fixed_rate;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops clk_fixed_rate_ops =
{
  .recalc_rate = clk_fixed_rate_recalc_rate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct clk *clk_register_fixed_rate(const char *name, const char *parent_name,
                                    uint8_t flags, uint32_t fixed_rate)
{
  struct clk_fixed_rate fixed;
  const char **parent_names;
  uint8_t num_parents;

  parent_names = parent_name ? &parent_name: NULL;
  num_parents = parent_name ? 1 : 0;

  fixed.fixed_rate = fixed_rate;

  return clk_register(name, parent_names, num_parents, flags,
                      &clk_fixed_rate_ops, &fixed, sizeof(fixed));
}
