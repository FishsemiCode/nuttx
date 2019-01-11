/****************************************************************************
 * arch/ceva/src/song/dw_vintc.c
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

#include <errno.h>
#include <string.h>

#include "vintc.h"

#ifdef CONFIG_VINTC_DW

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define DW_VINTC_PRIORITY_COUNT   16

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct dw_vintc_s
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
  } IRQ_VECTORX[DW_VINTC_PRIORITY_COUNT];
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
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static FAR struct dw_vintc_s * const g_dw_vintc
  = (FAR struct dw_vintc_s *)B2C(CONFIG_VINTC_DW_BASE);

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static int dw_vintc_interrupt(int irq, FAR void *context, FAR void *arg)
{
  uint32_t status;
  int i, j;

  /* Dispatch request one by one */
  for (i = 0; i < NR_IRQS; i += 32)
    {
      status = g_dw_vintc->IRQ_MASKSTATUS[i/32];
      for (j = 0; status; j++)
        {
          if (status & (1 << j))
            {
              g_dw_vintc->IRQ_INTFORCE[i/32] &= ~(1 << j);
              irq_dispatch(IRQ_VINT_FIRST + i + j, context);
              status &= ~(1 << j);
           }
        }
    }

  /* Reset the priority level */
  g_dw_vintc->IRQ_INTERNAL_PLEVEL = 0;

  return 0;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_vintc_initialize
 *
 * Description:
 *   Initialize the VINTC.
 *
 ****************************************************************************/

void up_vintc_initialize(void)
{
  int i;

  /* Disable all interrupt */
  for (i = 0; IRQ_VINT_FIRST + 32*i < NR_IRQS; i++)
    {
      g_dw_vintc->IRQ_INTEN[i] = 0;
    }

  /* And reset the priority level */
  g_dw_vintc->IRQ_INTERNAL_PLEVEL = 0;

  /* Make all priority send up_vintc_handler on VECTOR input */
  for (i = 0; i < DW_VINTC_PRIORITY_COUNT; i++)
    {
      g_dw_vintc->IRQ_VECTORX[i].VECTOR = (uintptr_t)up_vintc_handler;
    }

  /* Enable the VINTC self interrupt */
  irq_attach(IRQ_VINT, dw_vintc_interrupt, NULL);
  up_enable_irq(IRQ_VINT);
}

/****************************************************************************
 * Name: up_vintc_enable_irq
 *
 * Description:
 *   On CEVA architectures, there are four levels of interrupt enabling:
 *   (1) at the global level(up_irq_enable)
 *   (2) at the DSP level(up_enable_irq)
 *   (2) at the VINTC level
 *   (3) at the device level
 *   In order to receive interrupts, they must be enabled at all four levels.
 *
 *   This function implements enabling of the device specified by 'irq'
 *   at the VINTC level if supported by the architecture.
 *
 ****************************************************************************/

void up_vintc_enable_irq(int irq)
{
  irqstate_t flags;

  if (irq < NR_IRQS)
    {
      irq  -= IRQ_VINT_FIRST;
      flags = enter_critical_section();
      g_dw_vintc->IRQ_INTEN[irq/32] |= 1 << irq%32;
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: up_vintc_disable_irq
 *
 * Description:
 *   This function implements disabling of the device specified by 'irq'
 *   at the VINTC level if supported by the architecture(up_irq_save()
 *   supports the global level, the device level is hardware specific).
 *
 ****************************************************************************/

void up_vintc_disable_irq(int irq)
{
  irqstate_t flags;

  if (irq < NR_IRQS)
    {
      irq  -= IRQ_VINT_FIRST;
      flags = enter_critical_section();
      g_dw_vintc->IRQ_INTEN[irq/32] &= ~(1 << irq%32);
      leave_critical_section(flags);
    }
}

/****************************************************************************
 * Name: up_vintc_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQPRIO
int up_vintc_prioritize_irq(int irq, int priority)
{
  if (irq >= NR_IRQS || priority >= DW_VINTC_PRIORITY_COUNT)
    {
      return -EINVAL;
    }

  irq -= IRQ_VINT_FIRST;
  g_dw_vintc->IRQ_PRX[irq] = priority;

  return OK;
}
#endif /* CONFIG_ARCH_IRQPRIO */

/****************************************************************************
 * Name: up_vintc_trigger_irq
 *
 * Description:
 *   Trigger an irq by software.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_IRQTRIGGER
void up_vintc_trigger_irq(int irq)
{
  irqstate_t flags;

  if (irq < NR_IRQS)
    {
      flags = enter_critical_section();
      g_dw_vintc->IRQ_INTFORCE[irq/32] |= 1 << irq%32;
      leave_critical_section(flags);
    }
}
#endif /* CONFIG_ARCH_HAVE_IRQTRIGGER */

#endif /* CONFIG_VINTC_DW */
