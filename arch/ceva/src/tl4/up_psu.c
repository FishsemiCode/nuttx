/****************************************************************************
 * arch/ceva/src/tl4/up_psu.c
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

#include "cpm.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PSVM                                    0x0250

#define PSVM_PMOD_FREERUN                       0x00000000
#define PSVM_PMOD_DYNAMIC                       0x00000001
#define PSVM_PMOD_LIGHTSLEEP                    0x00000002
#define PSVM_PMOD_STANDBY                       0x00000003
#define PSVM_PMOD_MASK                          0x00000003

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void up_cpu_pmod(uint32_t pmod)
{
  uint32_t psvm;

  psvm  = getcpm(PSVM);
  psvm &= ~PSVM_PMOD_MASK;
  psvm |= pmod;

  __asm__ __volatile__
  (
    "mov %0, mod1\n"          /* Enable the interrupt */
    "out{cpm} %1, (%2).dw\n"  /* Enter the low power mode */
    "nop 0x07\n"              /* Clear the pipe of instruction */
    /* Core auto restore to DPS here after wakeup */
    "mov %3, mod1"            /* restore the interrupt */
     : : "r"(REG_MOD1_ENABLE), "d"(psvm)
     , "r"(PSVM), "r"(REG_MOD1_DISABLE)
  );
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_cpu_doze(void)
{
  up_cpu_pmod(PSVM_PMOD_LIGHTSLEEP);
}

void up_cpu_idle(void)
{
  up_cpu_pmod(PSVM_PMOD_STANDBY);
}
