/****************************************************************************
 * arch/ceva/include/irq.h
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

/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_CEVA_INCLUDE_IRQ_H
#define __ARCH_CEVA_INCLUDE_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/* Include chip-specific IRQ definitions (including IRQ numbers) */

#include <arch/chip/irq.h>

/* Include CEVA architecture-specific IRQ definitions (including register
 * save structure and up_irq_save()/up_irq_restore() functions)
 */

#if defined(CONFIG_ARCH_TL420) || defined(CONFIG_ARCH_TL421)
#  include <arch/tl4/irq.h>
#elif defined(CONFIG_ARCH_XM6)
#  include <arch/xm6/irq.h>
#elif defined(CONFIG_ARCH_XC5)
#  include <arch/xc5/irq.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define IRQ_VINT0           (IRQ_VINT_FIRST + 0)
#define IRQ_VINT1           (IRQ_VINT_FIRST + 1)
#define IRQ_VINT2           (IRQ_VINT_FIRST + 2)
#define IRQ_VINT3           (IRQ_VINT_FIRST + 3)
#define IRQ_VINT4           (IRQ_VINT_FIRST + 4)
#define IRQ_VINT5           (IRQ_VINT_FIRST + 5)
#define IRQ_VINT6           (IRQ_VINT_FIRST + 6)
#define IRQ_VINT7           (IRQ_VINT_FIRST + 7)
#define IRQ_VINT8           (IRQ_VINT_FIRST + 8)
#define IRQ_VINT9           (IRQ_VINT_FIRST + 9)
#define IRQ_VINT10          (IRQ_VINT_FIRST + 10)
#define IRQ_VINT11          (IRQ_VINT_FIRST + 11)
#define IRQ_VINT12          (IRQ_VINT_FIRST + 12)
#define IRQ_VINT13          (IRQ_VINT_FIRST + 13)
#define IRQ_VINT14          (IRQ_VINT_FIRST + 14)
#define IRQ_VINT15          (IRQ_VINT_FIRST + 15)
#define IRQ_VINT16          (IRQ_VINT_FIRST + 16)
#define IRQ_VINT17          (IRQ_VINT_FIRST + 17)
#define IRQ_VINT18          (IRQ_VINT_FIRST + 18)
#define IRQ_VINT19          (IRQ_VINT_FIRST + 19)
#define IRQ_VINT20          (IRQ_VINT_FIRST + 20)
#define IRQ_VINT21          (IRQ_VINT_FIRST + 21)
#define IRQ_VINT22          (IRQ_VINT_FIRST + 22)
#define IRQ_VINT23          (IRQ_VINT_FIRST + 23)
#define IRQ_VINT24          (IRQ_VINT_FIRST + 24)
#define IRQ_VINT25          (IRQ_VINT_FIRST + 25)
#define IRQ_VINT26          (IRQ_VINT_FIRST + 26)

#endif /* __ARCH_CEVA_INCLUDE_IRQ_H */
