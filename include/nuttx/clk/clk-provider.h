/****************************************************************************
 * include/nuttx/clk/clk-provider.h
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

#ifndef __INCLUDE_NUTTX_CLK_CLK_PROVIDER_H
#define __INCLUDE_NUTTX_CLK_CLK_PROVIDER_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/list.h>

#include <stdint.h>
#include <stddef.h>

#ifdef CONFIG_CLK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* This describe flags used across common struct clk driver */
#define CLK_SET_RATE_GATE               0x01 /* must be gated across rate change */
#define CLK_SET_PARENT_GATE             0x02 /* must be gated across re-parent */
#define CLK_SET_RATE_PARENT             0x04 /* propagate rate change up one level */
#define CLK_SET_RATE_NO_REPARENT        0x08 /* don't reparent on change rate */
#define CLK_GET_RATE_NOCACHE            0x10 /* do not use the cached clk rate */
#define CLK_NAME_IS_STATIC              0x20 /* static name: don't malloc in register */
#define CLK_PARENT_NAME_IS_STATIC       0x40 /* static parent name :don't malloc in register */
#define CLK_IS_CRITICAL                 0x80 /* critical clk do not gate ever */

/* gating clk flags */
#define CLK_GATE_SET_TO_DISABLE         0x01
#define CLK_GATE_HIWORD_MASK            0x02

/* divider clk flags */
#define CLK_DIVIDER_ONE_BASED           0x01
#define CLK_DIVIDER_HIWORD_MASK         0x02
#define CLK_DIVIDER_ROUND_CLOSEST       0x04
#define CLK_DIVIDER_READ_ONLY           0x08
#define CLK_DIVIDER_MAX_HALF            0x10 /* the max div is only half of the maximum one */
#define CLK_DIVIDER_DIV_NEED_EVEN       0x20
#define CLK_DIVIDER_POWER_OF_TWO        0x40
#define CLK_DIVIDER_MINDIV_OFF          8
#define CLK_DIVIDER_MINDIV_MSK          0xff00

/* fractional_divider clk flags */
#define CLK_FRAC_MUL_NEED_EVEN          0x01
#define CLK_FRAC_DIV_DOUBLE             0x02

/* multiplier clk flags */
#define CLK_MULT_ONE_BASED              0x01
#define CLK_MULT_ALLOW_ZERO             0x02
#define CLK_MULT_HIWORD_MASK            0x04
#define CLK_MULT_MAX_HALF               0x08
#define CLK_MULT_ROUND_CLOSEST          0x10

/* mux clk flags */
#define CLK_MUX_HIWORD_MASK             0x01
#define CLK_MUX_READ_ONLY               0x02
#define CLK_MUX_ROUND_CLOSEST           0x04

/* phase clk flags */
#define CLK_PHASE_HIWORD_MASK           0x01

/************************************************************************************
 * Public Type
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/* common clk struct used in clk-core */
struct clk
{
  const char              *name;
  const struct clk_ops    *ops;
  struct clk              *parent;
  uint8_t                 num_parents;
  uint8_t                 new_parent_index;
  uint8_t                 enable_count;
  uint8_t                 flags;
  uint32_t                rate;
  uint32_t                new_rate;
  struct clk              *new_parent;
  struct clk              *new_child;
  void                    *private_data;
  struct list_node        children;
  struct list_node        node;
  const char              *parent_names[0];
};

/* clk interface */
struct clk_ops
{
  int       (*enable)(struct clk *clk);
  void      (*disable)(struct clk *clk);
  int       (*is_enabled)(struct clk *clk);
  uint32_t  (*recalc_rate)(struct clk *clk, uint32_t parent_rate);
  uint32_t  (*round_rate)(struct clk *clk, uint32_t rate, uint32_t *parent_rate);
  uint32_t  (*determine_rate)(struct clk *clk, uint32_t rate,
            uint32_t *best_parent_rate, struct clk **best_parent_clk);
  int       (*set_parent)(struct clk *clk, uint8_t index);
  uint8_t   (*get_parent)(struct clk *clk);
  int       (*set_rate)(struct clk *clk, uint32_t rate, uint32_t parent_rate);
  int       (*set_rate_and_parent)(struct clk *clk, uint32_t rate,
            uint32_t parent_rate, uint8_t index);
  int       (*get_phase)(struct clk *clk);
  int       (*set_phase)(struct clk *clk, int degrees);
};

/* the individual clk struct */
struct clk_gate
{
  uint32_t            reg;
  uint8_t             bit_idx;
  uint8_t             flags;
};

struct clk_fixed_rate
{
  uint32_t            fixed_rate;
  uint8_t             flags;
};

struct clk_fixed_factor
{
  uint8_t             mult;
  uint8_t             div;
};

struct clk_divider
{
  uint32_t            reg;
  uint8_t             shift;
  uint8_t             width;
  uint16_t            flags;
};

struct clk_phase
{
  uint32_t            reg;
  uint8_t             shift;
  uint8_t             width;
  uint8_t             flags;
};

struct clk_fractional_divider
{
  uint32_t            reg;
  uint8_t             mwidth;
  uint8_t             nwidth;
  uint8_t             mshift;
  uint8_t             nshift;
  uint8_t             flags;
};

struct clk_multiplier
{
  uint32_t            reg;
  uint8_t             shift;
  uint8_t             width;
  uint8_t             flags;
};

struct clk_mux
{
  uint32_t            reg;
  uint8_t             width;
  uint8_t             shift;
  uint8_t             flags;
};


/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/* the individual clk ops declaration */
extern const struct clk_ops clk_gate_ops;
extern const struct clk_ops clk_fixed_rate_ops;
extern const struct clk_ops clk_fixed_factor_ops;
extern const struct clk_ops clk_divider_ops;
extern const struct clk_ops clk_phase_ops;
extern const struct clk_ops clk_fractional_divider_ops;
extern const struct clk_ops clk_multiplier_ops;
extern const struct clk_ops clk_mux_ops;
extern const struct clk_ops clk_mux_ro_ops;
#ifdef CONFIG_CLK_RPMSG
extern const struct clk_ops clk_rpmsg_ops;
#endif

struct clk *clk_register(const char *name, const char * const *parent_names,
                    uint8_t num_parents, uint8_t flags, const struct clk_ops *ops,
                    void *private_data, size_t private_size);

/* the individual clk register Prototypes */
struct clk *clk_register_gate(const char *name, const char *parent_name,
                    uint8_t flags, uint32_t reg, uint8_t bit_idx,
                    uint8_t clk_gate_flags);

struct clk *clk_register_fixed_rate(const char *name, const char *parent_name,
                    uint8_t flags, uint32_t fixed_rate);

struct clk *clk_register_fixed_factor(const char *name, const char *parent_name,
                    uint8_t flags, uint8_t mult, uint8_t div);

struct clk *clk_register_divider(const char *name, const char *parent_name,
                    uint8_t flags, uint32_t reg, uint8_t shift, uint8_t width,
                    uint16_t clk_divider_flags);

struct clk *clk_register_phase(const char *name, const char *parent_name,
                    uint8_t flags, uint32_t reg, uint8_t shift, uint8_t width,
                    uint8_t clk_phase_flags);

struct clk *clk_register_fractional_divider(const char *name, const char *parent_name,
                    uint8_t flags, uint32_t reg, uint8_t mshift, uint8_t mwidth,
                    uint8_t nshift, uint8_t nwidth, uint8_t clk_divider_flags);

struct clk *clk_register_multiplier(const char *name, const char *parent_name,
                    uint8_t flags, uint32_t reg, uint8_t shift, uint8_t width,
                    uint8_t clk_multiplier_flags);

struct clk *clk_register_mux(const char *name, const char * const *parent_names,
                    uint8_t num_parents, uint8_t flags, uint32_t reg, uint8_t shift,
                    uint8_t width, uint8_t clk_mux_flags);

#ifdef CONFIG_CLK_RPMSG
struct clk *clk_register_rpmsg(const char *name, uint8_t flags);

/* rpmsg clk must be initialize in board initialization */
int clk_rpmsg_initialize(void);
#endif

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_CLK */
#endif /* __INCLUDE_NUTTX_CLK_CLK_PROVIDER_H */
