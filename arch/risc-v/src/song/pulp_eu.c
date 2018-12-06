/****************************************************************************
 * arch/risc-v/src/song/pulp_eu.c
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

#include <nuttx/arch.h>
#include <nuttx/config.h>
#include <nuttx/irq.h>

#include "chip.h"
#include "up_arch.h"

#ifdef CONFIG_EVENT_UNIT_PULP
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IER                             CONFIG_EVENT_UNIT_PULP_BASE
#define ICP                             (CONFIG_EVENT_UNIT_PULP_BASE + 0xc)

/****************************************************************************
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
  putreg32(1 << irq, ICP);
}

/****************************************************************************
 * Name: song_irq_dispatch
 *
 * Description:
 * Call interrupt controller to dispatch irqs
 *
 ****************************************************************************/

void song_irq_dispatch(int irq, FAR void *context)
{
  irq_dispatch(irq, context);
  up_ack_irq(irq);
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
}

#else

uint32_t curr_irqs;
int irq_disabled;

/****************************************************************************
 * Name: up_restore_irqs
 *
 * Description:
 * restore the enabled irqs
 *
 ****************************************************************************/

void up_restore_irqs()
{
  putreg32(curr_irqs, IER);
}

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Return the current enabled irqs and disabled all except bt
 *
 ***************************************************************************/

irqstate_t up_irq_save(void)
{
  putreg32(curr_irqs & NVIC_SYSH_HIGH_PRIORITY, IER);

  if(irq_disabled) {
    return 0;
  } else {
    irq_disabled = 1;
    return 1;
  }
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
  if(flags) {
    irq_disabled = 0;
    putreg32(curr_irqs, IER);
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

  curr_irqs &= ~(1 << irq);

  up_irq_restore(flags);
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

  curr_irqs |= 1 << irq;

  up_irq_restore(flags);
}
#endif/* CONFIG_ARCH_HIPRI_INTERRUPT */
#endif/* CONFIG_EVENT_UNIT_PULP */
