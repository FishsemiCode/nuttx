/****************************************************************************
 * arch/ceva/include/xm6/spinlock.h
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

#ifndef __ARCH_CEVA_INCLUDE_XM6_SPINLOCK_H
#define __ARCH_CEVA_INCLUDE_XM6_SPINLOCK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <arch/xm6/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SP_SECTION __attribute__ ((section(".DSECT spinlock")))

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

static inline void up_dsb(void)
{
  /* MSS_BARRIER(0x638):
   * Bit [7] Internal Barrier Activation
   */
#define MSS_BARRIER 0x638

  uint32_t barrier = 0x80;

  __asm__ __volatile__
  (
    "out {cpm} %0.ui, (%1.ui).ui"
     : : "r"(barrier), "r"(MSS_BARRIER)
  );

  do
    {
      __asm__ __volatile__
      (
        "in {cpm} (%1.ui).ui, %0.ui\n"
        "nop #0x04\nnop #0x02"
        : "=r"(barrier)
        : "r"(MSS_BARRIER)
      );

      /* Wait unitl the barrier operation complete */
    }
  while ((barrier & 0x80) != 0);
#undef MSS_BARRIER
}

static inline void up_dmb(void)
{
  up_dsb(); /* use dsb instead since dmb doesn't exist on xm6 */
}

/****************************************************************************
 * Name: up_testset
 *
 * Description:
 *   Perform an atomic test and set operation on the provided spinlock.
 *
 *   This function must be provided via the architecture-specific logoic.
 *
 * Input Parameters:
 *   lock - The address of spinlock object.
 *
 * Returned Value:
 *   The spinlock is always locked upon return.  The value of previous value
 *   of the spinlock variable is returned, either SP_LOCKED if the spinlock
 *   as previously locked (meaning that the test-and-set operation failed to
 *   obtain the lock) or SP_UNLOCKED if the spinlock was previously unlocked
 *   (meaning that we successfully obtained the lock)
 *
 ****************************************************************************/

static inline spinlock_t up_testset(volatile FAR spinlock_t *lock)
{
  irqstate_t flags;
  spinlock_t old;

  /* Disable the interrupt */
  flags = up_irq_save();

  while (1)
    {
      uint32_t modc = 0;

      /* Issue exclusive read */
      __asm__ __volatile__
      (
        "nop\n"
        "LS0.ld (%1.ui).ui, %0.ui || monitor {on}\n"
        "nop #0x02"
        : "=r"(old)
        : "r"(lock)
      );

      /* Is it already locked by other? */
      if (old == SP_LOCKED)
        {
          break; /* Yes, exit */
        }

      /* Not yet, issue exclusive write */
      __asm__ __volatile__
      (
        "LS1.st %2.ui, (%1.ui).ui || monitor {off}\n"
        "mov modc.ui, %0.ui\n"
        "nop"
        : "=r"(modc)
        : "r"(lock), "r"(SP_LOCKED)
        : "memory"
      );

      /* Exclusive write success? */
      if ((modc & 0x01) == 0) /* Bit[0] Monitor status */
        {
          break; /* Yes, we are done */
        }

      /* Fail, let's try again */
    }

  /* Restore the interrupt */
  up_irq_restore(flags);

  return old;
}

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_CEVA_INCLUDE_XM6_SPINLOCK_H */
