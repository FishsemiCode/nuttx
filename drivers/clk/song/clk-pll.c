/****************************************************************************
 * drivers/clk/song/clk-pll.c
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

#include <debug.h>

#include "song-clk.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PLL_REFDIV_SHIFT    0
#define PLL_REFDIV_MASK     0x3F
#define PLL_FBDIV_SHIFT     8
#define PLL_FBDIV_MASK      0xFFF
#define PLL_POSTDIV1_SHIFT  20
#define PLL_POSTDIV1_MASK   0x7
#define PLL_POSTDIV2_SHIFT  24
#define PLL_POSTDIV2_MASK   0x7
#define PLL_DSMPD_SHIFT     28
#define PLL_DSMPD_MASK      0x1

#define PLL_FRAC_SHIFT      0
#define PLL_FRAC_MASK       0xFFFFFF

#define to_clk_pll(_clk) (struct clk_pll *)(_clk->private_data)

/************************************************************************************
 * Private Functions
 ************************************************************************************/

static uint64_t
clk_pll_recalc_rate(struct clk *clk, uint64_t parent_rate)
{
  uint32_t val;
  uint64_t rate;
  uint64_t dsmpd, refdiv, fbdiv, postdiv1, postdiv2, frac, div;
  struct clk_pll *pll = to_clk_pll(clk);

  val = clk_read(pll->cfg_reg0);
  dsmpd = (val >> PLL_DSMPD_SHIFT) & PLL_DSMPD_MASK;
  refdiv = (val >> PLL_REFDIV_SHIFT) & PLL_REFDIV_MASK;
  fbdiv = (val >> PLL_FBDIV_SHIFT) & PLL_FBDIV_MASK;
  postdiv1 = (val >> PLL_POSTDIV1_SHIFT) & PLL_POSTDIV1_MASK;
  postdiv2 = (val >> PLL_POSTDIV2_SHIFT) & PLL_POSTDIV2_MASK;

  div = refdiv * postdiv1 * postdiv2;

  if (dsmpd)
    rate = (parent_rate * fbdiv) / div;
  else
    {
      val = clk_read(pll->cfg_reg1);
      frac = (val >> PLL_FRAC_SHIFT) & PLL_FRAC_MASK;
      rate = (parent_rate * (fbdiv * (1 << 24) + frac)) / (div * (1 << 24));
    }

  return rate;
}

static int64_t
clk_pll_round_rate(struct clk *clk, uint64_t rate,
    uint64_t *best_parent_rate)
{
  uint32_t val;
  uint64_t dsmpd, refdiv, fbdiv, postdiv1, postdiv2, frac, div;
  struct clk_pll *pll = to_clk_pll(clk);

  val = clk_read(pll->cfg_reg0);
  dsmpd = (val >> PLL_DSMPD_SHIFT) & PLL_DSMPD_MASK;
  refdiv = (val >> PLL_REFDIV_SHIFT) & PLL_REFDIV_MASK;
  fbdiv = (val >> PLL_FBDIV_SHIFT) & PLL_FBDIV_MASK;
  postdiv1 = (val >> PLL_POSTDIV1_SHIFT) & PLL_POSTDIV1_MASK;
  postdiv2 = (val >> PLL_POSTDIV2_SHIFT) & PLL_POSTDIV2_MASK;

  div = refdiv * postdiv1 * postdiv2;

  if (dsmpd)
    {
      fbdiv = (rate * div) / (*best_parent_rate);
      rate = (*best_parent_rate * fbdiv) / div;
    }
  else
    {
      val = clk_read(pll->cfg_reg1);
      frac = (val >> PLL_FRAC_SHIFT) & PLL_FRAC_MASK;
      rate = (*best_parent_rate * (fbdiv * (1 << 24) + frac)) / (div * (1 << 24));
    }

  return rate;
}

static int clk_pll_set_rate(struct clk *clk, uint64_t rate,
    uint64_t parent_rate)
{
  uint32_t val;
  uint64_t dsmpd, refdiv, fbdiv, postdiv1, postdiv2, div;
  struct clk_pll *pll = to_clk_pll(clk);

  val = clk_read(pll->cfg_reg0);
  dsmpd = (val >> PLL_DSMPD_SHIFT) & PLL_DSMPD_MASK;
  refdiv = (val >> PLL_REFDIV_SHIFT) & PLL_REFDIV_MASK;
  fbdiv = (val >> PLL_FBDIV_SHIFT) & PLL_FBDIV_MASK;
  postdiv1 = (val >> PLL_POSTDIV1_SHIFT) & PLL_POSTDIV1_MASK;
  postdiv2 = (val >> PLL_POSTDIV2_SHIFT) & PLL_POSTDIV2_MASK;

  div = refdiv * postdiv1 * postdiv2;

  if (dsmpd)
    {
      clkerr("Start to adjust pll freq to %llu\n", rate);
      fbdiv = (rate * div) / parent_rate;
      val &= ~(PLL_FBDIV_MASK << PLL_FBDIV_SHIFT);
      val |= (fbdiv << PLL_FBDIV_SHIFT);
      clk_write(val, pll->cfg_reg0);
      val = (1 << (pll->ctl_shift + 16)) | (1 << pll->ctl_shift);
      clk_write(val, pll->ctl_reg);
      while (clk_read(pll->ctl_reg) & (1 << pll->ctl_shift));
      clkerr("Adjust pll freq done.\n");
    }

  return 0;
}

/************************************************************************************
 * Public Data
 ************************************************************************************/

const struct clk_ops clk_pll_ops =
{
  .recalc_rate = clk_pll_recalc_rate,
  .round_rate = clk_pll_round_rate,
  .set_rate = clk_pll_set_rate,
};

/************************************************************************************
 * Public Functions
 ************************************************************************************/

struct clk *clk_register_pll(const char *name, const char *parent_name, uint64_t flags,
    uint32_t cfg_reg0, uint32_t cfg_reg1, uint32_t ctl_reg, uint32_t ctl_shift)
{
  struct clk_pll *pll;
  struct clk *clk;
  int32_t num_parents;
  const char **parent_names;

  pll = kmm_malloc(sizeof(struct clk_pll));
  if (!pll)
    {
      return NULL;
    }

  parent_names = (parent_name ? &parent_name : NULL);
  num_parents = (parent_name ? 1 : 0);

  pll->cfg_reg0 = cfg_reg0;
  pll->cfg_reg1 = cfg_reg1;
  pll->ctl_reg = ctl_reg;
  pll->ctl_shift = ctl_shift;

  clk = clk_register(name, num_parents, parent_names, flags, &clk_pll_ops, pll);
  if (!clk)
    {
      kmm_free(pll);
    }

  return clk;
}
