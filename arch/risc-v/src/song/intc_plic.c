/****************************************************************************
 * arch/risc-v/src/song/intc_plic.c
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Guiding Li <liguiding@pinecone.net>
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
#include <nuttx/power/pm.h>

#include "chip.h"
#include "sched/sched.h"
#include "up_arch.h"

#ifdef CONFIG_INTC_PLIC

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define PLIC_PRIORITY               (CONFIG_INTC_PLIC_BASE + 0x00000000)
#define PLIC_ENABLE                 (CONFIG_INTC_PLIC_BASE + 0x00002000)
#define PLIC_THRESHOLD              (CONFIG_INTC_PLIC_BASE + 0x00200000)
#define PLIC_CLAIM                  (CONFIG_INTC_PLIC_BASE + 0x00200004)

#define PLIC_ENABLE_MASK            0x1f
#define PLIC_ENABLE_SHIFT           5
#define PLIC_PRIO_SHIFT             2

#define PLIC_PRIORITY_ADDR(n)       (PLIC_PRIORITY + ((n) << PLIC_PRIO_SHIFT))
#define PLIC_ENABLE_ADDR(n)         (PLIC_ENABLE + (((n) >> PLIC_ENABLE_SHIFT) * 4))

#ifndef ARRAY_SIZE
#  define ARRAY_SIZE(x)             (sizeof(x) / sizeof((x)[0]))
#endif

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

static void up_plic_pm_notify(struct pm_callback_s *cb, int domain,
                              enum pm_state_e pmstate);

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PM

/* Saved PLIC registers. */

static const uint32_t g_plic_regaddr[] =
{
  PLIC_THRESHOLD,
};

#define PLIC_REGSAVE_SCB_NUM    (ARRAY_SIZE(g_plic_regaddr))
#define PLIC_REGSAVE_PRIO_NUM   (NR_IRQS)
#define PLIC_REGSAVE_EN_NUM     ((NR_IRQS + 31) / 32)
#define PLIC_REGSAVE_CSR_NUM    (1)

static uint32_t g_plic_regsave[PLIC_REGSAVE_SCB_NUM + \
                               PLIC_REGSAVE_PRIO_NUM + \
                               PLIC_REGSAVE_EN_NUM + \
                               PLIC_REGSAVE_CSR_NUM];

static struct pm_callback_s g_plic_pm_cb =
{
  .notify  = up_plic_pm_notify,
};

#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void up_plic_backup(void)
{
  int i = 0;
  int j;

  for (j = 0; j < PLIC_REGSAVE_SCB_NUM; j++)
    {
      g_plic_regsave[i++] = getreg32(g_plic_regaddr[j]);
    }

  for (j = 0; j < NR_IRQS; j++)
    {
      g_plic_regsave[i++] = getreg32(PLIC_PRIORITY_ADDR(j));
    }

  for (j = 0; j < NR_IRQS; j += 32)
    {
      g_plic_regsave[i++] = getreg32(PLIC_ENABLE_ADDR(j));
    }

  g_plic_regsave[i] = READ_CSR(mie);
}

static void up_plic_restore(void)
{
  int i = 0;
  int j;

  for (j = 0; j < PLIC_REGSAVE_SCB_NUM; j++)
    {
      putreg32(g_plic_regsave[i++], g_plic_regaddr[j]);
    }

  for (j = 0; j < NR_IRQS; j++)
    {
      putreg32(g_plic_regsave[i++], PLIC_PRIORITY_ADDR(j));
    }

  for (j = 0; j < NR_IRQS; j += 32)
    {
      putreg32(g_plic_regsave[i++], PLIC_ENABLE_ADDR(j));
    }

  WRITE_CSR(mie, g_plic_regsave[i]);
}

static void up_plic_pm_notify(struct pm_callback_s *cb, int domain,
                              enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case PM_STANDBY:
      case PM_SLEEP:
        up_plic_backup();
        break;

      case PM_RESTORE:
        if (pm_querystate(PM_IDLE_DOMAIN) >= PM_STANDBY)
          {
            up_plic_restore();
          }
        break;

      default:
        break;
    }
}

static inline void plic_set_enable(int irq, int enable)
{
  int addr = PLIC_ENABLE_ADDR(irq);

  if (enable)
    {
      modifyreg32(addr, 0, (1 << (irq & PLIC_ENABLE_MASK)));
    }
  else
    {
      modifyreg32(addr, (1 << (irq & PLIC_ENABLE_MASK)), 0);
    }
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
  int irq;

  for (irq = 0; irq < NR_IRQS; irq++)
    {
      plic_set_enable(irq, 0);
      putreg32(1, PLIC_PRIORITY_ADDR(irq));
    }

  putreg32(0, PLIC_THRESHOLD);

  SET_CSR(mie, (1 << 11));

  up_wic_initialize();
  up_irq_enable();

#ifdef CONFIG_PM
  pm_register(&g_plic_pm_cb);
#endif
}

#ifdef CONFIG_ARCH_IRQPRIO
/****************************************************************************
 * Name: up_prioritize_irq
 *
 * Description:
 * Set interrupt priority
 *
 ****************************************************************************/

int up_prioritize_irq(int irq, int priority)
{
  putreg32(priority, PLIC_PRIORITY_ADDR(irq));
  return OK;
}
#endif

/****************************************************************************
 * Name: up_dispatch_irq
 *
 * Description:
 * Call interrupt controller to dispatch irqs
 *
 ****************************************************************************/

int up_dispatch_irq(int irq, FAR void *context)
{
  irq = getreg32(PLIC_CLAIM);
  irq_dispatch(irq, context);
  putreg32(irq, PLIC_CLAIM);

  return (getreg32(PLIC_PRIORITY_ADDR(irq)) > CONFIG_HIPRI_INTERRUPT_PRIORITY);
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
  plic_set_enable(irq, 0);
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
  plic_set_enable(irq, 1);
  up_wic_enable_irq(irq);
}

#ifdef CONFIG_ARCH_HIPRI_INTERRUPT
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

  putreg32(CONFIG_HIPRI_INTERRUPT_PRIORITY, PLIC_THRESHOLD);

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
      putreg32(CONFIG_HIPRI_INTERRUPT_PRIORITY, PLIC_THRESHOLD);
      rtcb->xcp.irqflags = 1;
    }
  else
    {
      rtcb->xcp.irqflags = 0;
      putreg32(0, PLIC_THRESHOLD);
    }
}
#endif /* CONFIG_ARCH_HIPRI_INTERRUPT */
#endif /* CONFIG_INTC_PLIC */
