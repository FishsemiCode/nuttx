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
 * Private Functions
 ****************************************************************************/

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: This function should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

/* Get/set the MOD1 register, here is the irq related bits:
 *   Bit  [0] Interrupt enable            (RW)
 *   Bit  [1] Interrupt mask for INT0     (RW)
 *   Bit  [2] Interrupt mask for INT1     (RW)
 *   Bit  [3] Interrupt mask for INT2     (RW)
 *   Bit  [4] Interrupt mask for INT3     (RW)
 *   Bit  [5] Interrupt mask for VINT     (RW)
 *   Bit  [6] Interrupt context for INT0  (RW)
 *   Bit  [7] Interrupt context for INT1  (RW)
 *   Bit  [8] Interrupt context for INT2  (RW)
 *   Bit  [9] Interrupt context for INT3  (RW)
 *   Bit [10] Interrupt context for INTV  (RO)
 *   Bit [11] Interrupt context for NMI   (RW)
 *   Bit [16] Interrupt pending for INT0  (RO)
 *   Bit [17] Interrupt pending for INT1  (RO)
 *   Bit [18] Interrupt pending for INT2  (RO)
 *   Bit [19] Interrupt pending for INT3  (RO)
 *   Bit [20] Interrupt pending for INTV  (RO)
 * All writable bits are clear by hardware during reset.
 *
 * We manipulate the individual mask bits instead of global enable bit since:
 * 1.Global IE not only mask INTX request but also mask TRAPX instruction.
 * 2.Hardware always enable global IE after the interrupt return.
 * Both behavior don't match the nuttx requirement.
 */

static inline uint32_t getmod1(void)
{
  uint32_t mod1;
  __asm__ __volatile__("mov mod1, %0\nnop" : "=r"(mod1));
  return mod1;
}

static inline void setmod1(uint32_t mod1)
{
  __asm__ __volatile__("nop\nmov %0, mod1" : : "r"(mod1));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/* Disable IRQs */

void up_irq_disable(void)
{
  /* Enable IE, but disable INT0~3 and INTV */

  setmod1(REG_IRQS_IE);
  CURRENT_IRQS &= ~REG_IRQS_IE;
}

/* Save the current state & disable IRQs */

irqstate_t up_irq_save(void)
{
  irqstate_t flags;

  flags = CURRENT_IRQS;
  up_irq_disable();
  return flags;
}

/* Enable IRQs */

void up_irq_enable(void)
{
  /* Restore INT0~3 and INTV */

  CURRENT_IRQS |= REG_IRQS_IE;
  setmod1(CURRENT_IRQS);
}

/* Restore saved IRQ state */

void up_irq_restore(irqstate_t flags)
{
  if (flags & REG_IRQS_IE)
    {
      up_irq_enable();
    }
}

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
      irqstate_t flags;

      flags = up_irq_save();
      if (irq >= IRQ_INT0 && irq <= IRQ_VINT)
        {
          CURRENT_IRQS &= ~(1 << INTX_MASK_BIT(irq));
        }
#if 0 /* All TRAPx is enabled by default(REG_MOD2_DEFAULT) */
      else if (irq >= IRQ_TRAP0 && irq <= IRQ_TRAP3)
        {
          uint32_t mod2;

          __asm__ __volatile__("mov mod2, %0\nnop" : "=r"(mod2));
          mod2 &= ~(1 << TRAPX_MASK_BIT(irq));
          __asm__ __volatile__("nop\nmov %0, mod2" : : "r"(mod2));
        }
#endif
      up_irq_restore(flags);
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
      irqstate_t flags;

      flags = up_irq_save();
      if (irq >= IRQ_INT0 && irq <= IRQ_VINT)
        {
          CURRENT_IRQS |= 1 << INTX_MASK_BIT(irq);
        }
#if 0 /* All TRAPx is enabled by default(REG_MOD2_DEFAULT) */
      else if (irq >= IRQ_TRAP0 && irq <= IRQ_TRAP3)
        {
          uint32_t mod2;

          __asm__ __volatile__("mov mod2, %0\nnop" : "=r"(mod2));
          mod2 |= 1 << TRAPX_MASK_BIT(irq);
          __asm__ __volatile__("nop\nmov %0, mod2" : : "r"(mod2));
        }
#endif
      up_irq_restore(flags);
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
 * Name: up_trigger_irq
 *
 * Description:
 *   Trigger an IRQ by software. May not be supported by all architectures.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
void up_trigger_irq(int irq)
{
  if (irq >= IRQ_VINT_FIRST)
    {
      /* Forward to the secondary interrupt controller */
      up_vintc_trigger_irq(irq);
    }
}
#endif /* CONFIG_ARCH_HAVE_IRQTRIGGER */

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
