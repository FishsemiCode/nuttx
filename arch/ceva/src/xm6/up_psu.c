/****************************************************************************
 * arch/ceva/src/xm6/up_psu.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Xiang Xiao <xiaoxiang@pinecone.net>
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

//disable psu function temporily
#define CEVAXM6_PSU_ENABLE 0
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#if CEVAXM6_PSU_ENABLE
#define up_cpu_pmod(pmod_inst)                                        \
  __asm__ __volatile__                                                \
  (                                                                   \
    "nop #0x04\nnop\n"                                                \
    "movp %0.ui, moda.ui\n"       /* Enable the interrupt */          \
    pmod_inst                     /* Enter the low power mode */      \
    "nop #0x04\nnop #0x04\nnop\n" /* Clear the pipe of instruction */ \
    /* Core auto restore to DPS here after wakeup */                  \
    "movp %1.ui, moda.ui"         /* restore the interrupt */         \
     : : "r"(REG_MODA_ENABLE), "r"(REG_MODA_DISABLE)                  \
  )
#else
#define up_cpu_pmod(pmod_inst)
#endif
/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_cpu_doze(void)
{
  up_cpu_pmod("psu {lightsleep}\n");
}

void up_cpu_idle(void)
{
  up_cpu_pmod("psu {standby}\n");
}
