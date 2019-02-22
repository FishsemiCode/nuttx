/****************************************************************************
 * arch/arm/src/song/arm_gic.c
 *
 *   Copyright (C) 2018 Pinecone Inc. All rights reserved.
 *   Author: Yuan Zhang <zhangyuan7@pinecone.net>
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

#ifdef CONFIG_ARMV7A_HAVE_GICv2

#include <nuttx/arch.h>

#include "up_internal.h"
#include "sctlr.h"
#include "gic.h"

/****************************************************************************
 * Public Define
 ****************************************************************************/

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_SMP
volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];
#else
volatile uint32_t *g_current_regs[1];
#endif

extern uint32_t _vector_start; /* Beginning of vector block */
extern uint32_t _vector_end;   /* End+1 of vector block */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   This function is called by up_initialize() during the bring-up of the
 *   system.  It is the responsibility of this function to but the interrupt
 *   subsystem into the working and ready state.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* The following operations need to be atomic, but since this function is
   * called early in the initialization sequence, we expect to have exclusive
   * access to the GIC.
   */

  /* Initialize the Generic Interrupt Controller (GIC) for CPU0 */

  arm_gic0_initialize();  /* Initialization unique to CPU0 */
  arm_gic_initialize();   /* Initialization common to all CPUs */

#ifdef CONFIG_ARCH_LOWVECTORS
  /* If CONFIG_ARCH_LOWVECTORS is defined, then the vectors located at the
   * beginning of the .text region must appear at address at the address
   * specified in the VBAR.  There are two ways to accomplish this:
   *
   *   1. By explicitly mapping the beginning of .text region with a page
   *      table entry so that the virtual address zero maps to the beginning
   *      of the .text region.  VBAR == 0x0000:0000.
   *
   *   2. Set the Cortex-A5 VBAR register so that the vector table address
   *      is moved to a location other than 0x0000:0000.
   *
   *  The second method is used by this logic.
   */
  /* Set the VBAR register to the address of the vector table */

  DEBUGASSERT((((uintptr_t)&_vector_start) & ~VBAR_MASK) == 0);
  cp15_wrvbar((uint32_t)&_vector_start);
#endif /* CONFIG_ARCH_LOWVECTORS */

  /* And finally, enable interrupts */

  up_irq_enable();
}

#endif

