/****************************************************************************
 * arch/ceva/src/xc5/up_psu.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
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

#include <nuttx/irq.h>

#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//disable psu function temporily
#if CONFIG_XC5_PSU_ENABLE

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void up_cpu_pmod(uint32_t psvm)
{
  __asm__ __volatile__                                             \
  (                                                                \
    "mov #0x2,    mod2\n"                                          \
    "mov #0x3f80, modp\n"      /* Enable the interrupt */          \
    "mov %0,   r0\n"           /* Enter the low power mode */      \
    "mov #0x250,  r1\n"                                            \
    "out {dw,cpm} r0, (r1)\n"  /* output to cpm register psmv */   \
    "nop\nnop\nnop\nnop\nnop\nnop\nnop\nnop\n"                     \
    /* Core auto restore to DPS here after wakeup */               \
    "mov #0x0080, modp"        /* restore the interrupt */         \
	: : "r"(psvm)                                              \
  );
}
#else
static void up_cpu_pmod(uint32_t psvm)
{
}
#endif /* CONFIG_XC5_PSU_ENABLE */
/****************************************************************************
 * Public Functions
 ****************************************************************************/
#define XC5_DOZE  0x1ff2
#define XC5_IDLE  0x1ff3

void up_cpu_doze(void)
{
  up_cpu_pmod(XC5_DOZE);
}

void up_cpu_idle(void)
{
  up_cpu_pmod(XC5_IDLE);
}
