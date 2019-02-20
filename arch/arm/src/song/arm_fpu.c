/****************************************************************************
 * arch/arm/src/song/arm_fpu.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
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

#if defined(CONFIG_ARCH_CORTEXM4) && defined(CONFIG_ARCH_FPU)

#include <nuttx/irq.h>

#include "chip.h"
#include "nvic.h"
#include "up_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define CONTROL_FPCA                    (1 << 2) /* Bit 2: Floating Point Context Active */

#define NVIC_CPACR_FPU_SHIFT            (20) /* Bit 23-20: FPU Access Control */
#define NVIC_CPACR_FPU_MASK             (0xf << NVIC_CPACR_FPU_SHIFT)
#define NVIC_CPACR_FPU_DENIED           (0x0 << NVIC_CPACR_FPU_SHIFT)
#define NVIC_CPACR_FPU_PRIVILEGED       (0x5 << NVIC_CPACR_FPU_SHIFT)
#define NVIC_CPACR_FPU_FULL             (0xf << NVIC_CPACR_FPU_SHIFT)

#define NVIC_FPCCR_LSPEN                (1 << 30) /* Bit 30: Lazy Stacking */
#define NVIC_FPCCR_ASPEN                (1 << 31) /* Bit 31: Automatic Setting FPCA */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: arm_fpuconfig
 *
 * Description:
 *   Configure the FPU.  Relative bit settings:
 *
 *     CPACR:  Enables access to CP10 and CP11
 *     CONTROL.FPCA: Determines whether the FP extension is active in the
 *       current context:
 *     FPCCR.ASPEN:  Enables automatic FP state preservation, then the
 *       processor sets FPCA bit to 1 on successful completion of any FP
 *       instruction.
 *     FPCCR.LSPEN:  Enables lazy context save of FP state. When this is
 *       done, the processor reserves space on the stack for the FP state,
 *       but does not save that state information to the stack.
 *
 *  Software must not change the value of the ASPEN bit or LSPEN bit while either:
 *   - the CPACR permits access to CP10 and CP11, that give access to the FP
 *     extension, or
 *   - the CONTROL.FPCA bit is set to 1
 *
 ****************************************************************************/

void arm_fpuconfig(void)
{
  uint32_t regval;

#ifdef CONFIG_ARMV7M_LAZYFPU
  /* Clear CONTROL.FPCA so that we do not get the extended context frame
   * with the volatile FP registers stacked in the saved context.
   */

  regval = getcontrol();
  regval &= ~CONTROL_FPCA;
  setcontrol(regval);
#else
  /* Set CONTROL.FPCA so that we always get the extended context frame
   * with the volatile FP registers stacked above the basic context.
   */

  regval = getcontrol();
  regval |= CONTROL_FPCA;
  setcontrol(regval);
#endif

  /* Ensure that FPCCR.LSPEN is disabled, so that we don't have to contend
   * with the lazy FP context save behavior.  Clear FPCCR.ASPEN since we
   * are going to keep CONTROL.FPCA on/off for all contexts.
   */

  modifyreg32(NVIC_FPCCR, NVIC_FPCCR_LSPEN | NVIC_FPCCR_ASPEN, 0);

  /* Enable full access to FPU */

  modifyreg32(NVIC_CPACR, NVIC_CPACR_FPU_MASK, NVIC_CPACR_FPU_FULL);
}
#endif
