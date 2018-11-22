/****************************************************************************
 * arch/risc-v/src/song/song_irq.c
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
#include <nuttx/irq.h>

#include "up_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IER                             0xa0400000
#define ICP                             0xa040000c
/* BTDM_IRQ, BLE_IRQ, BT_IRQ, RFPHY: 17, 18, 19, 20 */
#define BT_IRQS_MK                      0x1e0000

uint32_t curr_irqs;
int irq_disabled;

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Declarations
 ****************************************************************************/

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/
/****************************************************************************
 * Name: song_irq_save
 *
 * Description:
 *   Return the current interrupt state and disable interrupts
 *
 ****************************************************************************/

static irqstate_t song_irq_save(void)
{
  irqstate_t flags;

  __asm__ volatile("csrrci %0, %1, %2" : "=r"(flags) : "i"(0x300), "i"(0x8));
  return flags & 0x8;
}

/****************************************************************************
 * Name: song_irq_restore
 *
 * Description:
 *   Restore previous IRQ mask state
 *
 ****************************************************************************/

static void song_irq_restore(irqstate_t flags)
{
  __asm__ volatile("csrs %0, %1" :: "i"(0x300), "r"(flags));
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

extern uint32_t _vectors;

/****************************************************************************
 * Name: song_restore_irqs
 *
 * Description:
 * restore the enabled irqs
 *
 ****************************************************************************/

void song_restore_irqs()
{
  putreg32(curr_irqs, IER);
}

/****************************************************************************
 * Name: up_irqinitialize
 ****************************************************************************/

void up_irqinitialize(void)
{
  __asm__ volatile("csrw %0, %1" : : "i"(0x305), "r"(&_vectors));
  up_irq_enable();
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
  putreg32(1 << irq, ICP);
}

/****************************************************************************
 * Name: up_get_newintctx
 *
 * Description:
 *   Acknowledge the IRQ
 *
 ****************************************************************************/

uint32_t up_get_newintctx(void)
{
  return 0x1880;
}

/****************************************************************************
 * Name: up_irq_enable
 *
 * Description:
 *   Return the current interrupt state and enable interrupts
 *
 ****************************************************************************/

irqstate_t up_irq_enable(void)
{
  irqstate_t flags;

  __asm__ volatile("csrrsi %0, %1, %2" : "=r"(flags) : "i"(0x300), "i"(0x8));
  return flags & 0x8;
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
  irqstate_t flags;

  flags = song_irq_save();

  putreg32(curr_irqs & BT_IRQS_MK, IER);

  if(irq_disabled) {
    song_irq_restore(flags);
    return 0;
  } else {
    irq_disabled = 1;
    song_irq_restore(flags);
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
  irqstate_t flags1;
  flags1 = song_irq_save();

  if(flags) {
    putreg32(curr_irqs, IER);
    irq_disabled = 0;
  }

  song_irq_restore(flags1);
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
 // putreg32(curr_irqs, IER);

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
 // putreg32(curr_irqs, IER);

  up_irq_restore(flags);
}
