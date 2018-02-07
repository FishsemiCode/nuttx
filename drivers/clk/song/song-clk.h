/****************************************************************************
 * drivers/clk/song/song-clk.h
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

#ifndef __DRIVER_CLK_SONG_SONG_CLK_H
#define __DRIVER_CLK_SONG_SONG_CLK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include "clk.h"

#ifdef CONFIG_SONG_CLK

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define SONG_CLK_FLAG_MASK              0xff
#define SONG_CLK_DIV_MASK               0xffff

/* This descibes individual clk register shift */
#define SONG_CLK_GR_FDIV_EN_SHIFT       0
#define SONG_CLK_GR_FDIV_GR_SHIFT       4
#define SONG_CLK_GR_FDIV_GR_WIDTH       3
#define SONG_CLK_GR_FDIV_MUL_SHIFT      16
#define SONG_CLK_GR_FDIV_MUL_WIDTH      16
#define SONG_CLK_GR_FDIV_DIV_SHIFT      0
#define SONG_CLK_GR_FDIV_DIV_WIDTH      16

#define SONG_CLK_SDIV_GR_EN_SHIFT       0
#define SONG_CLK_SDIV_GR_DIV_SHIFT      4
#define SONG_CLK_SDIV_GR_DIV_WIDTH      3
#define SONG_CLK_SDIV_GR_GR_SHIFT       8
#define SONG_CLK_SDIV_GR_GR_WIDTH       4

#define SONG_CLK_SDIV_SDIV_EN_SHIFT     0
#define SONG_CLK_SDIV_SDIV_DIV0_SHIFT   4
#define SONG_CLK_SDIV_SDIV_DIV0_WIDTH   3
#define SONG_CLK_SDIV_SDIV_DIV1_SHIFT   8
#define SONG_CLK_SDIV_SDIV_DIV1_WIDTH   3

#define SONG_CLK_GR_SDIV_EN_SHIFT       0
#define SONG_CLK_GR_SDIV_GR_SHIFT       4
#define SONG_CLK_GR_SDIV_GR_WIDTH       3
#define SONG_CLK_GR_SDIV_DIV_SHIFT      8
#define SONG_CLK_GR_SDIV_DIV_WIDTH      4

#define SONG_CLK_MUX_SDIV_GR_EN_SHIFT   0
#define SONG_CLK_MUX_SDIV_GR_DIV_SHIFT  4
#define SONG_CLK_MUX_SDIV_GR_GR_SHIFT   8
#define SONG_CLK_MUX_SDIV_GR_GR_WIDTH   4
#define SONG_CLK_MUX_SDIV_GR_MUX_SHIFT  12

#define SONG_CLK_SDMMC_EN_SHIFT         0
#define SONG_CLK_SDMMC_MUX_SHIFT        2
#define SONG_CLK_SDMMC_MUX_MASK         1
#define SONG_CLK_SDMMC_DIV_SHIFT        4
#define SONG_CLK_SDMMC_DIV_WIDTH        4

#define SONG_CLK_CLKOUT_MUX_WIDTH       3

#define SONG_CLK_TIMER_MUX_SHIFT        28
#define SONG_CLK_TIMER_MUX_WIDTH        3

#define SONG_CLK_NAME_MAX               50

/************************************************************************************
 * Public Data
 ************************************************************************************/

#undef EXTERN
#if defined(__cplusplus)
#  define EXTERN extern "C"
extern "C"
{
#else
#  define EXTERN extern
#endif

/* the song platform pll clk struct */
struct clk_pll
{
  uint32_t cfg_reg0;
  uint32_t cfg_reg1;
  uint32_t ctl_reg;
  uint32_t ctl_shift;
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/
/* the song platform pll ops declaration */
extern const struct clk_ops clk_pll_ops;

/* the song platform pll register declaration */
struct clk *clk_register_pll(const char *name, const char *parent_name, uint64_t flags,
    uint32_t cfg_reg0, uint32_t cfg_reg1, uint32_t ctl_reg, uint32_t ctl_shift);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* CONFIG_SONG_CLK */
#endif /* __DRIVER_CLK_SONG_SONG_CLK_H */
