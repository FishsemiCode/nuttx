/****************************************************************************
 * arch/ceva/src/tl4/up_intc.c
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
#include <nuttx/irq.h>

#include "up_internal.h"
#include "vintc.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define INTX_MASK_BIT(x)          ((x) - IRQ_INT0  +  1)
#define TRAPX_MASK_BIT(x)         ((x) - IRQ_TRAP0 + 28)

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   This function implements disabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_save() supports the global level, the device level is hardware
 *   specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  if (irq < IRQ_VINT_FIRST)
    {
#if 0 /* Why comment out? see:
       * 1.arch/ceva/include/tl4/irq.h
       * 2.arch/ceva/src/tl4/up_initialstate.c
       */
      irqstate_t flags;
      uint32_t mode;

      flags = up_irq_save();
      if (irq >= IRQ_INT0 && irq <= IRQ_VINT)
        {
          __asm__ __volatile__("mov mod1, %0\nnop" : "=r"(mode));
          mode &= ~(1 << INTX_MASK_BIT(irq));
          __asm__ __volatile__("nop\nmov %0, mod1" : : "r"(mode));
        }
      else if (irq >= IRQ_TRAP0 && irq <= IRQ_TRAP3)
        {
          __asm__ __volatile__("mov mod2, %0\nnop" : "=r"(mode));
          mode &= ~(1 << TRAPX_MASK_BIT(irq));
          __asm__ __volatile__("nop\nmov %0, mod2" : : "r"(mode));
        }
      up_irq_restore(flags);
#endif
    }
  else
    {
      /* Forward to the secondary interrupt controller */
      up_vintc_disable_irq(irq);
    }
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   On many architectures, there are three levels of interrupt enabling: (1)
 *   at the global level, (2) at the level of the interrupt controller,
 *   and (3) at the device level.  In order to receive interrupts, they
 *   must be enabled at all three levels.
 *
 *   This function implements enabling of the device specified by 'irq'
 *   at the interrupt controller level if supported by the architecture
 *   (up_irq_restore() supports the global level, the device level is hardware
 *   specific).
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  if (irq < IRQ_VINT_FIRST)
    {
#if 0 /* Why comment out? see:
       * 1.arch/ceva/include/tl4/irq.h
       * 2.arch/ceva/src/tl4/up_initialstate.c
       */
      irqstate_t flags;
      uint32_t mode;

      flags = up_irq_save();
      if (irq >= IRQ_INT0 && irq <= IRQ_VINT)
        {
          __asm__ __volatile__("mov mod1, %0\nnop" : "=r"(mode));
          mode |= 1 << INTX_MASK_BIT(irq);
          __asm__ __volatile__("nop\nmov %0, mod1" : : "r"(mode));
        }
      else if (irq >= IRQ_TRAP0 && irq <= IRQ_TRAP3)
        {
          __asm__ __volatile__("mov mod2, %0\nnop" : "=r"(mode));
          mode |= 1 << TRAPX_MASK_BIT(irq);
          __asm__ __volatile__("nop\nmov %0, mod2" : : "r"(mode));
        }
      up_irq_restore(flags);
#endif
    }
  else
    {
      /* Forward to the secondary interrupt controller */
      up_vintc_enable_irq(irq);
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 *   Since this API is not supported on all architectures, it should be
 *   avoided in common implementations where possible.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  int ret = -EINVAL;

  if (irq >= IRQ_VINT_FIRST)
    {
      /* Forward to the secondary interrupt controller */
      ret = up_vintc_prioritize_irq(irq, priority);
    }

  return ret;
}
#endif

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  /* Initialize the secondary interrupt controller */
  up_vintc_initialize();

  /* Attach and enable SVCall exception handler */
  irq_attach(IRQ_TRAP0, up_svcall, NULL);
  up_enable_irq(IRQ_TRAP0);

  /* Attach and enable Hard Fault exception handler */
#if CONFIG_ARCH_HARDFAULT_IRQ >= 0
  irq_attach(CONFIG_ARCH_HARDFAULT_IRQ, up_hardfault, NULL);
  up_enable_irq(CONFIG_ARCH_HARDFAULT_IRQ);
#endif

  /* And finally, enable interrupts */
  up_irq_enable();
}
