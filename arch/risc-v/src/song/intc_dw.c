/****************************************************************************
 * arch/ceva/src/song/intc_dw.c
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

#include <errno.h>

#include "chip.h"
#include "sched/sched.h"

#ifdef CONFIG_INTC_DW

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define INTC_DW_PRIORITY_COUNT    16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct intc_dw_s
{
  volatile uint32_t IRQ_INTEN[2];
  volatile uint32_t IRQ_INTMASK[2];
  volatile uint32_t IRQ_INTFORCE[2];
  volatile uint32_t IRQ_RAWSTATUS[2];
  volatile uint32_t IRQ_STATUS[2];
  volatile uint32_t IRQ_MASKSTATUS[2];
  volatile uint32_t IRQ_FINALSTATUS[2];
  volatile uint32_t IRQ_VECTOR;
  volatile uint32_t RESERVED0;
  struct
  {
    volatile uint32_t VECTOR;
    volatile uint32_t RESERVED;
  } IRQ_VECTORX[INTC_DW_PRIORITY_COUNT];
  volatile uint32_t FIQ_INTEN;
  volatile uint32_t FIQ_INTMASK;
  volatile uint32_t FIQ_INTFORCE;
  volatile uint32_t FIQ_RAWSTATUS;
  volatile uint32_t FIQ_STATUS;
  volatile uint32_t FIQ_FINALSTATUS;
  volatile uint32_t IRQ_PLEVEL;
  volatile uint32_t IRQ_INTERNAL_PLEVEL;
  volatile uint32_t RESERVED1[2];
  volatile uint32_t IRQ_PRX[64];
  volatile uint32_t RESERVED2[390];
  volatile uint32_t ACK_CLR; /* 0x800 */
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct intc_dw_s * const g_intc_dw
  = (struct intc_dw_s *)CONFIG_INTC_DW_BASE;

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int intc_find_highprio_irq(void)
{
  uint32_t status;
  int hprio, hirq;
  int i, j;

  hprio = -1;
  hirq  = -1;

  for (i = 0; i < NR_IRQS; i += 32)
    {
      status = g_intc_dw->IRQ_FINALSTATUS[i/32];

      for (j = 0; status; j++)
        {
          if (status & (1 << j))
            {
              int cur = g_intc_dw->IRQ_PRX[i+j];

              if (cur > hprio)
                {
                  hprio = cur;
                  hirq  = i + j;
                }

              status &= ~(1 << j);
            }
        }
    }

  return hirq;
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
  int i;

  /* Make all interrupt to vector 11 */
  for (i = 0; i < INTC_DW_PRIORITY_COUNT; i++)
    {
      g_intc_dw->IRQ_VECTORX[i].VECTOR = 11;
    }

  /* Disable and umask all interrupt */
  for (i = 0; 32*i < NR_IRQS; i++)
    {
      g_intc_dw->IRQ_INTEN[i] = 0;
      g_intc_dw->IRQ_INTMASK[i] = 0;
    }

  for (i = 0; i < 64; i++)
    {
      g_intc_dw->IRQ_PRX[i] = 0;
    }

  /* And reset the priority level */
  g_intc_dw->IRQ_INTERNAL_PLEVEL = 0;

  up_wic_initialize();

  up_irq_enable();
}

/****************************************************************************
 * up_dispatch_irq
 ****************************************************************************/

int up_dispatch_irq(int irq, FAR void *context)
{
  int ret = 0;
  int hirq;

  /* Dispatch highest priority irq */

  hirq = intc_find_highprio_irq();
  if (hirq >= 0)
    {
      g_intc_dw->IRQ_INTFORCE[hirq / 32] &= ~(1 << (hirq % 32));
      irq_dispatch(hirq, context);
      ret = g_intc_dw->IRQ_PRX[hirq] >= CONFIG_HIPRI_INTERRUPT_PRIORITY;
    }

  /* Clear ACK */

  g_intc_dw->ACK_CLR = 1;

  /* Reset the priority level */

  g_intc_dw->IRQ_INTERNAL_PLEVEL = g_intc_dw->IRQ_PLEVEL;

  return ret;
}

/****************************************************************************
 * Name: up_enable_irq
 *
 * Description:
 *
 *   This function implements enabling of the device specified by 'irq'
 *   at the INTC level if supported by the architecture.
 *
 ****************************************************************************/

void up_enable_irq(int irq)
{
  irqstate_t flags;

  if (irq < NR_IRQS)
    {
      flags = spin_lock_irqsave();
      g_intc_dw->IRQ_INTEN[irq/32] |= 1 << irq%32;
      spin_unlock_irqrestore(flags);

      up_wic_enable_irq(irq);
    }
}

/****************************************************************************
 * Name: up_disable_irq
 *
 * Description:
 *   This function implements disabling of the device specified by 'irq'
 *   at the INTC level if supported by the architecture(up_irq_save()
 *   supports the global level, the device level is hardware specific).
 *
 ****************************************************************************/

void up_disable_irq(int irq)
{
  irqstate_t flags;

  if (irq < NR_IRQS)
    {
      flags = spin_lock_irqsave();
      g_intc_dw->IRQ_INTEN[irq/32] &= ~(1 << irq%32);
      spin_unlock_irqrestore(flags);

      up_wic_disable_irq(irq);
    }
}

/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_prioritize_irq(int irq, int priority)
{
  if (irq >= NR_IRQS || priority >= INTC_DW_PRIORITY_COUNT)
    {
      return -EINVAL;
    }

  g_intc_dw->IRQ_PRX[irq] = priority;

  return OK;
}
#endif /* CONFIG_ARCH_IRQPRIO */

/****************************************************************************
 * Name: up_trigger_irq
 *
 * Description:
 *   Trigger an irq by software.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
void up_trigger_irq(int irq)
{
  if (irq < NR_IRQS)
    {
      g_intc_dw->IRQ_INTFORCE[irq/32] |= 1 << irq%32;
    }
}
#endif /* CONFIG_ARCH_HAVE_IRQTRIGGER */

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT

/****************************************************************************
 * Name: up_irq_save
 *
 * Description:
 *   Return the current enabled irqs and disabled all except bt
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

  g_intc_dw->IRQ_PLEVEL = CONFIG_HIPRI_INTERRUPT_PRIORITY;
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
      g_intc_dw->IRQ_PLEVEL = CONFIG_HIPRI_INTERRUPT_PRIORITY;
      rtcb->xcp.irqflags = 1;
    }
  else
    {
      rtcb->xcp.irqflags = 0;
      g_intc_dw->IRQ_PLEVEL = 0;
    }
}
#endif /* CONFIG_ARCH_HIPRI_INTERRUPT */
#endif /* CONFIG_INTC_DW */
