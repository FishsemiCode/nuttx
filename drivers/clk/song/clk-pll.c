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

#include <nuttx/clk/clk-provider.h>

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
#define PLL_PLLPD_SHIFT     29
#define PLL_DSMPD_MASK      0x1

#define PLL_CTLST_SHIFT     4
#define PLL_CTLST_MASK      0xf
#define PLL_CTLST_PD        2
#define PLL_CTLST_WORK      0

#define PLL_FRAC_SHIFT      0
#define PLL_FRAC_MASK       0xFFFFFF

#define to_clk_pll(_clk) (struct clk_pll *)(_clk->private_data)

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static uint32_t
clk_pll_recalc_rate(struct clk *clk, uint32_t parent_rate)
{
  uint32_t val, rate, div, dsmpd, refdiv, fbdiv, postdiv1, postdiv2;
  struct clk_pll *pll = to_clk_pll(clk);

  val = clk_read(pll->cfg0_reg);
  dsmpd = (val >> PLL_DSMPD_SHIFT) & PLL_DSMPD_MASK;
  refdiv = (val >> PLL_REFDIV_SHIFT) & PLL_REFDIV_MASK;
  fbdiv = (val >> PLL_FBDIV_SHIFT) & PLL_FBDIV_MASK;
  postdiv1 = (val >> PLL_POSTDIV1_SHIFT) & PLL_POSTDIV1_MASK;
  postdiv2 = (val >> PLL_POSTDIV2_SHIFT) & PLL_POSTDIV2_MASK;

  div = refdiv * postdiv1 * postdiv2;

  if (dsmpd)
    rate = ((uint64_t)parent_rate * fbdiv) / div;
  else
    {
      uint32_t frac;

      val = clk_read(pll->cfg1_reg);
      frac = (val >> PLL_FRAC_SHIFT) & PLL_FRAC_MASK;
      rate = (parent_rate * (fbdiv * (1ull << 24) + frac)) / (div * (1ull << 24));
    }

  return rate;
}

/*****************************************************
 * fixed postdiv2 = 1 refdiv = 2, solve postdiv1, fbdiv
 *
 *           parent_rate * fbdiv
 * rate  = -----------------------
 *            postdiv1 * 1 * 2
 *****************************************************/

static uint32_t
clk_pll_round_rate(struct clk *clk, uint32_t rate, uint32_t *best_parent_rate)
{
  uint32_t refdiv, fbdiv, postdiv1, postdiv2, prate, tmprate;

  prate = *best_parent_rate;

  postdiv2 = 1, refdiv = 2;
  rate = rate * refdiv * postdiv2;

  tmprate = gcd(rate, prate);
  rate = rate / tmprate;
  prate = prate / tmprate;

  tmprate = (prate * rate) / (gcd(rate, prate));
  fbdiv = tmprate / prate;
  postdiv1 = tmprate / rate;

  rate = ((uint64_t)*best_parent_rate * fbdiv) / (postdiv1 * postdiv2 * refdiv);
  return rate;
}

static void clk_pll_endisable(struct clk *clk, int enable)
{
  struct clk_pll *pll = to_clk_pll(clk);
  uint32_t val, reg, status;

  if (enable)
    {
      reg    = 0;
      status = PLL_CTLST_WORK;
    }
  else
    {
      reg    = BIT(PLL_PLLPD_SHIFT);
      status = PLL_CTLST_PD;
    }

  val = clk_read(pll->cfg0_reg);
  val &= ~BIT(PLL_PLLPD_SHIFT);
  val |= reg;
  clk_write(pll->cfg0_reg, val);

  while((clk_read(pll->ctl_reg) >> PLL_CTLST_SHIFT & PLL_CTLST_MASK) != status);
}

static int clk_pll_enable(struct clk *clk)
{
  clk_pll_endisable(clk, 1);
  return 0;
}

static void clk_pll_disable(struct clk *clk)
{
  clk_pll_endisable(clk, 0);
}

static int clk_pll_is_enable(struct clk *clk)
{
  struct clk_pll *pll = to_clk_pll(clk);
  uint32_t val;

  val = clk_read(pll->cfg0_reg);
  val &= BIT(PLL_PLLPD_SHIFT);

  return val != BIT(PLL_PLLPD_SHIFT);
}

static int clk_pll_set_rate(struct clk *clk, uint32_t rate, uint32_t parent_rate)
{
  uint32_t val, refdiv, fbdiv, postdiv1, postdiv2, prate, tmprate;
  struct clk_pll *pll = to_clk_pll(clk);

  prate = parent_rate;

  postdiv2 = 1, refdiv = 2;
  rate = rate * refdiv * postdiv2;

  tmprate = gcd(rate, prate);
  rate = rate / tmprate;
  prate = prate / tmprate;

  tmprate = (prate * rate) / (gcd(rate, prate));
  fbdiv = tmprate / prate;
  postdiv1 = tmprate / rate;

  val = (1 << PLL_DSMPD_SHIFT) | (postdiv2 << PLL_POSTDIV2_SHIFT)
        | (postdiv1 << PLL_POSTDIV1_SHIFT) | (fbdiv << PLL_FBDIV_SHIFT)
        | (refdiv << PLL_REFDIV_SHIFT);

  clkerr("Start to adjust pll freq\n");
  clk_write(pll->cfg0_reg, val);
  val = (1 << (pll->ctl_shift + 16)) | (1 << pll->ctl_shift);
  clk_write(pll->ctl_reg, val);
  while (clk_read(pll->ctl_reg) & (1 << pll->ctl_shift));
  clkerr("Adjust pll freq done.\n");

  return 0;
}

/****************************************************************************
 * Public Data
 ****************************************************************************/

const struct clk_ops clk_pll_ops =
{
  .enable      = clk_pll_enable,
  .disable     = clk_pll_disable,
  .is_enabled  = clk_pll_is_enable,
  .recalc_rate = clk_pll_recalc_rate,
  .round_rate  = clk_pll_round_rate,
  .set_rate    = clk_pll_set_rate,
};

const struct clk_ops clk_pll_ro_ops =
{
  .is_enabled  = clk_pll_is_enable,
  .recalc_rate = clk_pll_recalc_rate,
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

struct clk *clk_register_pll(const char *name, const char *parent_name,
                             uint8_t flags, uint32_t cfg0_reg, uint32_t cfg1_reg,
                             uint32_t ctl_reg, uint8_t ctl_shift, uint8_t pll_flags)
{
  struct clk_pll pll;
  const char **parent_names;
  uint8_t num_parents;

  parent_names = parent_name ? &parent_name : NULL;
  num_parents = parent_name ? 1 : 0;

  pll.cfg0_reg = cfg0_reg;
  pll.cfg1_reg = cfg1_reg;
  pll.ctl_reg = ctl_reg;
  pll.ctl_shift = ctl_shift;

  if (pll_flags & CLK_PLL_READ_ONLY)
    return clk_register(name, parent_names, num_parents, flags,
                      &clk_pll_ro_ops, &pll, sizeof(pll));
  else
    return clk_register(name, parent_names, num_parents, flags,
                      &clk_pll_ops, &pll, sizeof(pll));
}
