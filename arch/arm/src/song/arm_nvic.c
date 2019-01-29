/****************************************************************************
 * arch/arm/src/song/arm_nvic.c
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

#ifdef CONFIG_ARCH_CORTEXM4

#include <assert.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>
#include <nuttx/power/pm.h>

#include "sched/sched.h"

#include "chip.h"
#include "nvic.h"
#include "ram_vectors.h"
#include "up_arch.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Get a 32-bit version of the default priority */

#define NVIC_PRIORITY_DEFAULT32   (NVIC_SYSH_PRIORITY_DEFAULT << 24 | \
                                   NVIC_SYSH_PRIORITY_DEFAULT << 16 | \
                                   NVIC_SYSH_PRIORITY_DEFAULT << 8  | \
                                   NVIC_SYSH_PRIORITY_DEFAULT)

/* Given the address of a NVIC ENABLE register, this is the offset to
 * the corresponding NVIC CLEAR register.
 */

#define NVIC_ENABLE_OFFSET        (NVIC_IRQ0_31_ENABLE - NVIC_IRQ0_31_ENABLE)
#define NVIC_CLEAR_OFFSET         (NVIC_IRQ0_31_CLEAR  - NVIC_IRQ0_31_ENABLE)

/****************************************************************************
 * Public Data
 ****************************************************************************/

/* g_current_regs[] holds a references to the current interrupt level
 * register storage structure.  If is non-NULL only during interrupt
 * processing.  Access to g_current_regs[] must be through the macro
 * CURRENT_REGS for portability.
 */

#ifdef CONFIG_SMP
volatile uint32_t *g_current_regs[CONFIG_SMP_NCPUS];
#else
volatile uint32_t *g_current_regs[1];
#endif

extern uint32_t _vectors[];

/****************************************************************************
 * Private Function Declarations
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_nvic_pm_notify(struct pm_callback_s *cb, int domain,
                           enum pm_state_e pmstate);
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef CONFIG_PM
/* Saved SCB registers. */

static const uint32_t g_nvic_scb_regaddr[] =
{
    NVIC_VECTAB,
    NVIC_SYSCON,
    NVIC_SYSH4_7_PRIORITY,
    NVIC_SYSH8_11_PRIORITY,
    NVIC_SYSH12_15_PRIORITY,
    NVIC_SYSHCON,
};

#define NVIC_REGSAVE_SCB_NUM \
            (sizeof(g_nvic_scb_regaddr) / sizeof(g_nvic_scb_regaddr[0]))

/* Saved NVIC registers: ISER, IPR. */

#define NVIC_REGSAVE_ISER_NUM  (((NR_IRQS - NVIC_IRQ_FIRST) + 31) / 32)
#define NVIC_REGSAVE_IPR_NUM   (((NR_IRQS - NVIC_IRQ_FIRST) + 3) / 4)

static uint32_t g_nvic_regsave[NVIC_REGSAVE_ISER_NUM + NVIC_REGSAVE_IPR_NUM + \
                               NVIC_REGSAVE_SCB_NUM];

static struct pm_callback_s g_up_nvic_pm_cb =
{
  .notify  = up_nvic_pm_notify,
};
#endif

/****************************************************************************
 * Private Functions
 ****************************************************************************/

#ifdef CONFIG_PM
static void up_nvic_backup(void)
{
  int i = 0;
  int j;

  for (j = 0; j < NVIC_REGSAVE_SCB_NUM; j++)
    {
      g_nvic_regsave[i++] = getreg32(g_nvic_scb_regaddr[j]);
    }

  for (j = 0; j < NR_IRQS - NVIC_IRQ_FIRST; j += 4)
    {
      g_nvic_regsave[i++] = getreg32(NVIC_IRQ_PRIORITY(j));
    }

  for (j = 0; j < NR_IRQS - NVIC_IRQ_FIRST; j += 32)
    {
      g_nvic_regsave[i++] = getreg32(NVIC_IRQ_ENABLE(j));
    }
}

static void up_nvic_restore(void)
{
  int i = 0;
  int j;

  for (j = 0; j < NVIC_REGSAVE_SCB_NUM; j++)
    {
      putreg32(g_nvic_regsave[i++], g_nvic_scb_regaddr[j]);
    }

  for (j = 0; j < NR_IRQS - NVIC_IRQ_FIRST; j += 4)
    {
      putreg32(g_nvic_regsave[i++], NVIC_IRQ_PRIORITY(j));
    }

  for (j = 0; j < NR_IRQS - NVIC_IRQ_FIRST; j += 32)
    {
      putreg32(g_nvic_regsave[i++], NVIC_IRQ_ENABLE(j));
    }
}

static void up_nvic_pm_notify(struct pm_callback_s *cb, int domain,
                           enum pm_state_e pmstate)
{
  switch (pmstate)
    {
      case PM_STANDBY:
      case PM_SLEEP:
        up_nvic_backup();
        break;

      case PM_RESTORE:
        if (pm_querystate(PM_IDLE_DOMAIN) >= PM_STANDBY)
          {
            up_nvic_restore();
          }
        break;

      default:
        break;
    }
}
#endif

/****************************************************************************
 * Name: nvic_irqinfo
 *
 * Description:
 *   Given an IRQ number, provide the register and bit setting to enable or
 *   disable the irq.
 *
 ****************************************************************************/

static int nvic_irqinfo(int irq, uintptr_t *regaddr, uint32_t *bit,
                        uintptr_t offset)
{
  DEBUGASSERT(irq >= NVIC_IRQ_MEMFAULT && irq < NR_IRQS);

  /* Check for external interrupt */

  if (irq >= NVIC_IRQ_FIRST)
    {
      irq      = irq - NVIC_IRQ_FIRST;
      *regaddr = NVIC_IRQ_ENABLE(irq) + offset;
      *bit     = (uint32_t)1 << (irq & 0x1f);
    }

  /* Handle processor exceptions.  Only a few can be disabled */

  else
    {
      *regaddr = NVIC_SYSHCON;
      if (irq == NVIC_IRQ_MEMFAULT)
        {
          *bit = NVIC_SYSHCON_MEMFAULTENA;
        }
      else if (irq == NVIC_IRQ_BUSFAULT)
        {
          *bit = NVIC_SYSHCON_BUSFAULTENA;
        }
      else if (irq == NVIC_IRQ_USAGEFAULT)
        {
          *bit = NVIC_SYSHCON_USGFAULTENA;
        }
      else if (irq == NVIC_IRQ_DBGMONITOR)
        {
          *regaddr = NVIC_DEMCR;
          *bit = NVIC_DEMCR_MONEN;
        }
      else if (irq == NVIC_IRQ_SYSTICK)
        {
          *regaddr = NVIC_SYSTICK_CTRL;
          *bit = NVIC_SYSTICK_CTRL_TICKINT;
        }
      else
        {
          return ERROR; /* Invalid or unsupported exception */
        }
    }

  return OK;
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
 * Name: up_ack_irq
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

void up_ack_irq(int irq)
{
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
  uintptr_t regaddr;
  uint32_t bit;

  if (nvic_irqinfo(irq, &regaddr, &bit, NVIC_CLEAR_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to disable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Clear Enable register.  For other exceptions, we need to
       * clear the bit in the System Handler Control and State Register.
       */

      if (irq >= NVIC_IRQ_FIRST)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          modifyreg32(regaddr, bit, 0);
        }
      up_wic_disable_irq(irq);
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
  uintptr_t regaddr;
  uint32_t bit;

  if (nvic_irqinfo(irq, &regaddr, &bit, NVIC_ENABLE_OFFSET) == 0)
    {
      /* Modify the appropriate bit in the register to enable the interrupt.
       * For normal interrupts, we need to set the bit in the associated
       * Interrupt Set Enable register.  For other exceptions, we need to
       * set the bit in the System Handler Control and State Register.
       */

      if (irq >= NVIC_IRQ_FIRST)
        {
          putreg32(bit, regaddr);
        }
      else
        {
          modifyreg32(regaddr, 0, bit);
        }
      up_wic_enable_irq(irq);
    }
}

/****************************************************************************
 * Name: up_trigger_irq
 *
 * Description:
 *   Trigger an IRQ by software.
 *
 ****************************************************************************/

void up_trigger_irq(int irq)
{
  uint32_t pend_bit = 0;

  DEBUGASSERT(irq >= NVIC_IRQ_NMI && irq < NR_IRQS);

  if (irq >= NVIC_IRQ_FIRST)
    {
      putreg32(irq - NVIC_IRQ_FIRST, NVIC_STIR);
    }
  else
    {
      switch (irq)
        {
          case NVIC_IRQ_PENDSV:
            pend_bit = NVIC_INTCTRL_PENDSVSET;
            break;
          case NVIC_IRQ_NMI:
            pend_bit = NVIC_INTCTRL_NMIPENDSET;
            break;
          case NVIC_IRQ_SYSTICK:
            pend_bit = NVIC_INTCTRL_PENDSTSET;
            break;
        }

      if (pend_bit)
        {
          modifyreg32(NVIC_INTCTRL, 0, pend_bit);
        }
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

int up_prioritize_irq(int irq, int priority)
{
  uint32_t regaddr;
  int shift;

  DEBUGASSERT(irq >= NVIC_IRQ_MEMFAULT && irq < NR_IRQS &&
              priority >= NVIC_SYSH_PRIORITY_MAX &&
              priority <= NVIC_SYSH_PRIORITY_MIN);

  if (irq < NVIC_IRQ_FIRST)
    {
      /* NVIC_SYSH_PRIORITY() maps {0..15} to one of three priority
       * registers (0-3 are invalid)
       */

      regaddr = NVIC_SYSH_PRIORITY(irq);
      irq    -= NVIC_IRQ_MEMFAULT;
    }
  else
    {
      /* NVIC_IRQ_PRIORITY() maps {0..} to one of many priority registers */

      irq    -= NVIC_IRQ_FIRST;
      regaddr = NVIC_IRQ_PRIORITY(irq);
    }

  shift = (irq & 3) << 3;
  modifyreg32(regaddr, 0xff << shift, priority << shift);

  return OK;
}

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  struct tcb_s *idle;
  int i;

  /* Initialize the idle task stack info */

  idle = this_task(); /* It should be idle task */
  idle->stack_alloc_ptr = _END_BSS;
  idle->adj_stack_ptr   = (FAR void *)g_idle_topstack;
  idle->adj_stack_size  = CONFIG_IDLETHREAD_STACKSIZE;

  /* Disable all interrupts */

  for (i = 0; i < NR_IRQS - NVIC_IRQ_FIRST; i += 32)
    {
      putreg32(0xffffffff, NVIC_IRQ_CLEAR(i));
    }

  /* Set the NVIC vector location in case _vectors not equal zero. */

  putreg32((uint32_t)_vectors, NVIC_VECTAB);

#ifdef CONFIG_ARCH_RAMVECTORS
  /* If CONFIG_ARCH_RAMVECTORS is defined, then we are using a RAM-based
   * vector table that requires special initialization.
   */

  up_ramvec_initialize();
#endif

  /* Set all interrupts (and exceptions) to the default priority */

  putreg32(NVIC_PRIORITY_DEFAULT32, NVIC_SYSH4_7_PRIORITY);
  putreg32(NVIC_PRIORITY_DEFAULT32, NVIC_SYSH8_11_PRIORITY);
  putreg32(NVIC_PRIORITY_DEFAULT32, NVIC_SYSH12_15_PRIORITY);

  for (i = 0; i < NR_IRQS - NVIC_IRQ_FIRST; i += 4)
    {
      putreg32(NVIC_PRIORITY_DEFAULT32, NVIC_IRQ_PRIORITY(i));
    }

  up_wic_initialize();

  /* Attach the SVCall and Hard Fault exception handlers.  The SVCall
   * exception is used for performing context switches; The Hard Fault
   * must also be caught because a SVCall may show up as a Hard Fault
   * under certain conditions.
   */

  irq_attach(NVIC_IRQ_SVCALL, up_svcall, NULL);
  up_prioritize_irq(NVIC_IRQ_SVCALL, NVIC_SYSH_SVCALL_PRIORITY);

  irq_attach(NVIC_IRQ_HARDFAULT, up_hardfault, NULL);

  /* Attach and enable the Memory Management Fault handler */

  irq_attach(NVIC_IRQ_MEMFAULT, up_memfault, NULL);
  up_enable_irq(NVIC_IRQ_MEMFAULT);

#ifdef CONFIG_PM
  pm_register(&g_up_nvic_pm_cb);
#endif

  /* And finally, enable interrupts */

  up_irq_enable();
}
#endif
