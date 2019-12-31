/****************************************************************************
 * arch/ceva/src/xc5/up_relocate.c
 *
 *   Copyright (C) 2020 FishSemi Inc. All rights reserved.
 *   Author: Bo Zhang <zhangbo@fishsemi.com>
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

#include <nuttx/config.h>

#include "cpm.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MSS_PDEA            0x004
#define MSS_PDIA            0x008
#define MSS_PDTC            0x00c
#define MSS_PDTC_MASK       0xfffff
#define MSS_PDTC_PDST       (1 << 29)
#define MSS_DDEA             0x208
#define MSS_DDIA             0x20c
#define MSS_DDTC             0x210
#define MSS_DDTC_MASK        0x1fffff
#define MSS_DDTC_PDST        (1 << 29)
#define MSS_DDTC_BSZ_SHIFT   25
#define MSS_DDTC_DDIR_SHIFT  30

#define BSZ_1_TRANS          (0 << MSS_DDTC_BSZ_SHIFT)
#define BSZ_4_TRANS          (6 << MSS_DDTC_BSZ_SHIFT)
#define BSZ_8_TRANS          (10 << MSS_DDTC_BSZ_SHIFT)
#define BSZ_16_TRANS         (14 << MSS_DDTC_BSZ_SHIFT)

#define DDIR_EX2IN           (0 << MSS_DDTC_DDIR_SHIFT)
#define DDIR_IN2EX           (1 << MSS_DDTC_DDIR_SHIFT)

#define _START_INTTBL        ((void *)&_sinttbl)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern char _sinttbl;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void pdma_config(unsigned long iaddr, unsigned long eaddr, uint32_t count)
{
  putcpm(MSS_PDIA, iaddr);
  putcpm(MSS_PDEA, eaddr);
  putcpm(MSS_PDTC, count & MSS_PDTC_MASK);
}

static void pdma_wait_idle(void)
{
  while (getcpm(MSS_PDTC) & MSS_PDTC_PDST);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_relocate(void)
{
  pdma_config(0, (unsigned long)_START_INTTBL, 0x320);
  pdma_wait_idle();
}
