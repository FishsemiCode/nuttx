/****************************************************************************
 * arch/ceva/src/common/up_doirq.c
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

#include <nuttx/arch.h>

#include "sched/sched.h"
#include "up_internal.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

#ifdef CONFIG_SMP
uint32_t  volatile g_current_irqs[CONFIG_SMP_NCPUS] = {[0...CONFIG_SMP_NCPUS-1] = REG_IRQS_IE};
uint32_t *volatile g_current_regs[CONFIG_SMP_NCPUS];
#else
uint32_t  volatile g_current_irqs[1] = {REG_IRQS_IE};
uint32_t *volatile g_current_regs[1];
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

uint32_t *up_doirq(int irq, uint32_t *regs)
{
  /* Is it the outermost interrupt? */

  if (CURRENT_REGS != NULL)
    {
      /* No, simply deliver the IRQ because only the outermost nested
       * interrupt can result in a context switch.
       */

      irq_dispatch(irq, regs);
    }
  else
    {

      /* Hardware automatically clear IE bit at the interrupt entry,
       * update the software state(CURRENT_IRQS) to reflect the truth.
       */

      CURRENT_IRQS &= ~REG_IRQS_IE;

      struct tcb_s *rtcb;

      /* Current regs non-zero indicates that we are processing an interrupt;
       * CURRENT_REGS is also used to manage interrupt level context switches.
       */

      CURRENT_REGS = regs;

      /* Deliver the IRQ */

      irq_dispatch(irq, regs);

      /* If a context switch occurred while processing the interrupt then
       * CURRENT_REGS may have change value.  If we return any value different
       * from the input regs, then the lower level will know that a context
       * switch occurred during interrupt processing.
       */

      regs = CURRENT_REGS;

      /* Restore the previous value of CURRENT_REGS.  NULL would indicate that
       * we are no longer in an interrupt handler.  It will be non-NULL if we
       * are returning from a nested interrupt.
      */

      CURRENT_REGS = NULL;

      if (regs != (uint32_t *)regs[REG_SP])
        {
          /* We are returning with a pending context switch. This case is different
           * because in this case, the register save structure does not lie on the
           * stack but, rather within other storage. We'll have to copy some values
           * to the stack.
           */

          memcpy((uint32_t *)regs[REG_SP], regs, XCPTCONTEXT_SIZE);
          regs = (uint32_t *)regs[REG_SP];
        }

      /* Restore the new task irq state */

      rtcb = this_task();
      up_irq_restore(rtcb->xcp.irqflags);

      CURRENT_IRQS |= REG_IRQS_IE;
    }

  return regs;
}
