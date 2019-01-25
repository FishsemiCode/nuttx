/****************************************************************************
 * include/nuttx/clk/clk.h
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

#ifndef __INCLUDE_NUTTX_CLK_CLK_H
#define __INCLUDE_NUTTX_CLK_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <errno.h>
#include <stdint.h>

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

struct clk;
struct clk_ops;

struct clk_rate
{
  const char *name;
  uint32_t   rate;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef CONFIG_CLK
/* Clk Operate API */
struct   clk *clk_get(const char *name);

struct   clk *clk_get_parent(struct clk *clk);
struct   clk *clk_get_parent_by_index(struct clk *clk, uint8_t index);
int      clk_set_parent(struct clk *clk, struct clk *parent);

int      clk_enable(struct clk *clk);
int      clk_disable(struct clk *clk);
int      clk_is_enabled(struct clk *clk);

uint32_t clk_round_rate(struct clk *clk, uint32_t rate);
int      clk_set_rate(struct clk *clk, uint32_t rate);
int      clk_set_rates(const struct clk_rate *rates);
uint32_t clk_get_rate(struct clk *clk);

int      clk_set_phase(struct clk *clk, int degrees);
int      clk_get_phase(struct clk *clk);

/* Clk helper API */

void clk_disable_unused(void);
const char *clk_get_name(const struct clk *clk);
#else
static inline struct clk *clk_get(const char *name)
  {
    return (struct clk *)(-ENOENT);
  }
static inline struct clk *clk_get_parent(struct clk *clk)
  {
    return (struct clk *)(-ENOENT);
  }
static inline struct clk *clk_get_parent_by_index(struct clk *clk, uint8_t index)
  {
    return (struct clk *)(-ENOENT);
  }
static inline int clk_set_parent(struct clk *clk, struct clk *parent)
  {
    return 0;
  }
static inline int clk_enable(struct clk *clk)
  {
    return 0;
  }
static inline void clk_disable(struct clk *clk)
  {
    return;
  }
static inline int clk_is_enabled(struct clk *clk)
  {
    return 0;
  }
static inline uint32_t clk_round_rate(struct clk *clk, uint32_t rate)
  {
    return 0;
  }
static inline int clk_set_rate(struct clk *clk, uint32_t rate)
  {
    return 0;
  }
static inline int clk_set_rates(const struct clk_rate *rates)
  {
    return 0;
  }
static inline uint32_t clk_get_rate(struct clk *clk)
  {
    return 0;
  }
static inline int clk_set_phase(struct clk *clk, int degrees)
  {
    return 0;
  }
static inline int clk_get_phase(struct clk *clk)
  {
    return 0;
  }
static inline void clk_disable_unused(void)
  {
    return;
  }
static inline const char *clk_get_name(const struct clk *clk)
  {
    return (const char *)(-ENOENT);
  }
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_CLK_CLK_H */
