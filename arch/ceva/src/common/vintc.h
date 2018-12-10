/****************************************************************************
 * rch/ceva/src/common/vintc.h
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

#ifndef __ARCH_CEVA_SRC_COMMON_VINTC_H
#define __ARCH_CEVA_SRC_COMMON_VINTC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#undef EXTERN
#if defined(__cplusplus)
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: up_vintc_initialize
 *
 * Description:
 *   Initialize the VINTC.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_VINTC
void up_vintc_initialize(void);
#else
static inline void up_vintc_initialize(void)
{
}
#endif

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

#ifdef CONFIG_ARCH_HAVE_VINTC
void up_vintc_enable_irq(int irq);
#else
static inline void up_vintc_enable_irq(int irq)
{
}
#endif

/****************************************************************************
 * Name: up_vintc_disable_irq
 *
 * Description:
 *   This function implements disabling of the device specified by 'irq'
 *   at the VINTC level if supported by the architecture(up_irq_save()
 *   supports the global level, the device level is hardware specific).
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_HAVE_VINTC
void up_vintc_disable_irq(int irq);
#else
static inline void up_vintc_disable_irq(int irq)
{
}
#endif

/****************************************************************************
 * Name: up_vintc_prioritize_irq
 *
 * Description:
 *   Set the priority of an IRQ.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_HAVE_VINTC) && defined(CONFIG_ARCH_IRQPRIO)
int up_vintc_prioritize_irq(int irq, int priority);
#else
static inline int up_vintc_prioritize_irq(int irq, int priority)
{
  return 0; /* Not a critical error */
}
#endif

/****************************************************************************
 * Name: up_vintc_trigger_irq
 *
 * Description:
 *   Trigger an IRQ by software.
 *
 ****************************************************************************/

#if defined(CONFIG_ARCH_HAVE_VINTC) && defined(CONFIG_ARCH_HAVE_IRQTRIGGER)
void up_vintc_trigger_irq(int irq);
#else
static inline void up_vintc_trigger_irq(int irq)
{
}
#endif

/****************************************************************************
 * Name: up_vintc_handler
 *
 * Description:
 *   This function address must be sent from VINTC on VECTOR input in order to
 *   let DSP could jump to the appropriate interrupt handler location.
 *   Note that VINTC may pass this address to the hardware register, but ARCH
 *   specific code is responsible to implement this function.
 *
 ****************************************************************************/

void up_vintc_handler(void);

#undef EXTERN
#if defined(__cplusplus)
}
#endif

#endif /* __ARCH_CEVA_SRC_COMMON_VINTC_H */
