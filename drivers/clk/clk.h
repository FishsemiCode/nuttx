/****************************************************************************
 * driver/clk/clk.h
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

#ifndef __DRIVER_CLK_CLK_H
#define __DRIVER_CLK_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <strings.h>

#ifdef CONFIG_CLK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* This describe the private macro used in clk driver */
#define BIT(nr)                     (1ULL << (nr))
#define MASK(width)                 (BIT(width) - 1)
#define MULT_ROUND_UP(r, m)         ((r) * (m) + (m) - 1)
#define DIV_ROUND_UP(n,d)           ((d) ? (((n) + (d) - 1) / (d)) : -1)
#define DIV_ROUND_CLOSEST(n, d)     ((((n) < 0) ^ ((d) < 0)) ? (((n) - (d)/2)/(d)) : (((n) + (d)/2)/(d)))
#define MIN(x, y)                   (x < y) ? x : y

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static inline void clk_write(uint32_t reg, uint32_t value)
{
  reg = reg / (CHAR_BIT / 8);
  *((volatile uint32_t *) (reg)) = value;
}

static inline uint32_t clk_read(uint32_t reg)
{
  reg = reg / (CHAR_BIT / 8);
  return *((volatile uint32_t *) (reg));
}

static inline uint32_t gcd(uint32_t a, uint32_t b)
{
  uint32_t r;
  uint32_t tmp;

  if (a < b)
    {
      tmp = a;
      a = b;
      b = tmp;
    }

  if (!b)
    return a;
  while ((r = a % b) != 0)
    {
      a = b;
      b = r;
    }
  return b;
}

static inline int is_power_of_2(uint32_t n)
{
  return (n != 0 && ((n & (n - 1)) == 0));
}

static inline uint32_t roundup_pow_of_two(uint32_t n)
{
  return 1 << fls(n - 1);
}

static inline uint32_t rounddown_pow_of_two(uint32_t n)
{
  return 1 << (fls(n) - 1);
}

static inline uint32_t roundup_double(double n)
{
  uint32_t intn = (uint32_t)n;
  if (n == (double)intn)
    {
      return intn;
    }
  return intn + 1;
}

#endif /* CONFIG_CLK */
#endif /* __DRIVER_CLK_CLK_H */
