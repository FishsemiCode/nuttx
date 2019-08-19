/****************************************************************************
 * arch/risc-v/src/song/intc_pulp.c
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

#include "chip.h"
#include "sched/sched.h"
#include "up_arch.h"

#ifdef CONFIG_INTC_PULP

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IER                             (CONFIG_INTC_PULP_BASE + 0x00)
#define ICP                             (CONFIG_INTC_PULP_BASE + 0x0c)

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
static uint32_t g_curr_irqs;
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

static void up_ack_irq(int irq)
{
  putreg32(1 << irq, ICP);
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void weak_function up_wic_initialize(void)
{
}

void weak_function up_wic_enable_irq(int irq)
{
}

void weak_function up_wic_disable_irq(int irq)
{
}

/****************************************************************************
 * Name: up_irqinitialize
 *
 * Description:
 *   Initialize the INTC.
 *
 ****************************************************************************/

void up_irqinitialize(void)
{
  up_wic_initialize();
  up_irq_enable();
}

/****************************************************************************
 * Name: up_dispatch_irq
 *
 * Description:
 * Call interrupt controller to dispatch irqs
 *
 ****************************************************************************/

int up_dispatch_irq(int irq, FAR void *context)
{
  irq_dispatch(irq, context);
  up_ack_irq(irq);
  return 0;
}

#ifndef CONFIG_ARCH_HIPRI_INTERRUPT
/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  modifyreg32(IER, 1 << irq, 0);
  up_wic_disable_irq(irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  modifyreg32(IER, 0, 1 << irq);
  up_wic_enable_irq(irq);
}

#else

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Return the current enabled irqs and disabled all except mask
 *
 ***************************************************************************/

irqstate_t up_irq_save(void)
{
  struct tcb_s *rtcb = this_task();
  irqstate_t flags = 1;

  if (!rtcb || up_interrupt_context())
    {
      return flags;
    }

  flags = rtcb->xcp.irqflags;
  putreg32(g_curr_irqs & CONFIG_HIPRI_INTERRUPT_PRIORITY, IER);
  rtcb->xcp.irqflags = 1;

  return flags;
}

/****************************************************************************
 * Name: up_irq_restore
 *
 * Description:
 *   Restore previous IRQ enable state
 *
 ****************************************************************************/

void up_irq_restore(irqstate_t flags)
{
  struct tcb_s *rtcb = this_task();

  if (!rtcb || up_interrupt_context())
    {
      return;
    }

  if(flags)
    {
      putreg32(g_curr_irqs & CONFIG_HIPRI_INTERRUPT_PRIORITY, IER);
      rtcb->xcp.irqflags = 1;
    }
  else
    {
      rtcb->xcp.irqflags = 0;
      putreg32(g_curr_irqs, IER);
    }
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   Disable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  irqstate_t flags;

  flags = up_irq_save();
  g_curr_irqs &= ~(1 << irq);
  up_irq_restore(flags);

  up_wic_disable_irq(irq);
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *   Enable the IRQ specified by 'irq'
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  irqstate_t flags;

  flags = up_irq_save();
  g_curr_irqs |= 1 << irq;
  up_irq_restore(flags);

  up_wic_enable_irq(irq);
}
#endif /* CONFIG_ARCH_HIPRI_INTERRUPT */
#endif /* CONFIG_INTC_PULP */
