/****************************************************************************
 * arch/ceva/include/xm6/irq.h
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

#ifndef __ARCH_CEVA_INCLUDE_XM6_IRQ_H
#define __ARCH_CEVA_INCLUDE_XM6_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Included implementation-dependent register save structure layouts */

#include <arch/xm6/reg.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* If this is a kernel build, how many nested system calls should we support? */

#ifndef CONFIG_SYS_NNEST
#  define CONFIG_SYS_NNEST 2
#endif

/* Alternate register names *************************************************/

#define REG_A0              REG_R0
#define REG_A1              REG_R1
#define REG_A2              REG_R2
#define REG_A3              REG_R3
#define REG_A4              REG_R4
#define REG_A5              REG_R5
#define REG_A6              REG_R6
#define REG_FP              REG_R8
//#define REG_SP            REG_SP
#define REG_LR              REG_RETREG
#define REG_PC              REG_RETREGI
#define REG_OM              REG_MODC  /* Operation Mode */

/* MODA IE bit */
#define REG_IRQS_IE         0x10

#define REG_OM_DEFAULT      0x20      /* PI and Supervisor */

/* Note: this is POM field not OM field */
#define REG_OM_KERNEL       0x00      /* Supervisor Mode */
#define REG_OM_USER         0x08      /* User0 Mode */
#define REG_OM_MASK         0x18      /* Mode Mask */

/* First Level Interrupt (vectors 0-15) */

#define IRQ_RESET           0x00 /* Vector  0: Reset(not handler as an IRQ) */
#define IRQ_BOOT            0x01 /* Vector  1: Boot(not handler as an IRQ) */
#define IRQ_TRAP            0x02 /* Vector  2: Software Interrupt */
#define IRQ_TRAPE           0x03 /* Vector  3: Emulation Software Interrupt */
#define IRQ_BI              0x03 /* Vector  3: Breakpoint Interrupt */
#define IRQ_NMI             0x04 /* Vector  4: Non-Maskable Interrupt */
#define IRQ_INT0            0x05 /* Vector  5: Maskable Interrupt 0 */
#define IRQ_INT1            0x06 /* Vector  6: Maskable Interrupt 1 */
#define IRQ_INT2            0x07 /* Vector  7: Maskable Interrupt 2 */
#define IRQ_INT3            0x08 /* Vector  8: Maskable Interrupt 3 */
#define IRQ_INT4            0x09 /* Vector  9: Maskable Interrupt 4 */
#define IRQ_VINT            0x0a /* Vector 10: Vectored Interrupt */
#define IRQ_TRAP0           0x0b /* Vector 11: Software Interrupt 0 */
#define IRQ_TRAP1           0x0c /* Vector 12: Software Interrupt 1 */
#define IRQ_TRAP2           0x0d /* Vector 13: Software Interrupt 2 */
#define IRQ_TRAP3           0x0e /* Vector 14: Software Interrupt 3 */
#define IRQ_PABP            0x03 /* Vector  3: Program Address Breakpoint */

/* Second Level interrupts (vectors >= 16).  These definitions are chip-specific */

#define IRQ_VINT_FIRST      16 /* Vector number of the first VINT interrupt */

/****************************************************************************
 * Public Types
 ****************************************************************************/
#ifndef __ASSEMBLY__

/* This structure represents the return state from a system call */

#ifdef CONFIG_LIB_SYSCALL
struct xcpt_syscall_s
{
  uint32_t saved_pc;
  uint32_t saved_om;
};
#endif

/* The following structure is included in the TCB and defines the complete
 * state of the thread.
 */

struct xcptcontext
{
#ifndef CONFIG_DISABLE_SIGNALS
  /* The following function pointer is non-zero if there
   * are pending signals to be processed.
   */

  FAR void *sigdeliver; /* Actual type is sig_deliver_t */

  /* These are saved copies of the context used during
   * signal processing.
   */

  uint32_t *saved_regs;

# ifdef CONFIG_BUILD_PROTECTED
  /* This is the saved address to use when returning from a user-space
   * signal handler.
   */

  uint32_t sigreturn;

# endif
#endif

#ifdef CONFIG_LIB_SYSCALL
  /* The following array holds the return address and operation mode
   * needed to return from each nested system call.
   */

  uint8_t nsyscalls;
  struct xcpt_syscall_s syscall[CONFIG_SYS_NNEST];

#endif

  irqstate_t irqflags;

  /* Register save area with XCPTCONTEXT_SIZE, only valid when:
   * 1.The task isn't running or
   * 2.The task is interrupted
   * otherwise task is running, and regs contain the stale value.
   */

  uint32_t *regs;
};
#endif

/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: up_getsp
 ****************************************************************************/

static inline uint32_t up_getsp(void)
{
  uint32_t sp;
  __asm__ __volatile__("mov sp.ui, %0.ui" : "=r"(sp));
  return sp;
}

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_CEVA_INCLUDE_XM6_IRQ_H */
