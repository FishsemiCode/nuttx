/****************************************************************************
 * arch/ceva/include/xc5/irq.h
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Bo Zhang <zhangbo@fishsemi.com>
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

#ifndef __ARCH_CEVA_INCLUDE_XC5_IRQ_H
#define __ARCH_CEVA_INCLUDE_XC5_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#ifndef __ASSEMBLY__
#  include <stdint.h>
#endif

/* Included implementation-dependent register save structure layouts */

#include <arch/xc5/reg.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/
/* If this is a kernel build, how many nested system calls should we support? */

#ifndef CONFIG_SYS_NNEST
#  define CONFIG_SYS_NNEST 2
#endif

/* Alternate register names *************************************************/

//#define REG_A0            REG_A0
//#define REG_A1            REG_A1
//#define REG_A2            REG_A2
//#define REG_A3            REG_A3
//#define REG_A4            REG_A4
//#define REG_A5            REG_A5
//#define REG_A6            REG_A6
//#define REG_A7            REG_A7
#define REG_FP              REG_G0
//#define REG_SP            REG_SP
#define REG_LR              REG_RETREG
#define REG_PC              REG_RETREGI
#define REG_OM              REG_MODQ

/* MODG: satuation */
#define REG_MODG_DEFAULT    0x001b

/* MODP: IRQ enable/disable */
#define REG_MODP_DEFAULT    0x3f80

#define REG_MODP_ENABLE     0x3f80
#define REG_MODP_DISABLE    0x0080

/* MOD2: Confirm C compiler assumption */
#define REG_MODPB_DEFAULT    0xf0 /* TRAPx */

/* MODQ: Operation mode */
#define REG_OM_DEFAULT       0x20 /* PI and Supervisor */
/* Note: this is POM filed not OM field */
#define REG_OM_KERNEL        0x00 /* Supervisor Mode */
#define REG_OM_USER          0x08 /* User0 Mode */
#define REG_OM_MASK          0x18 /* Mode mask */

/* First Level Interrupt (vectors 0-15) */

#define IRQ_RESET           0x00 /* Vector  0: Reset(not handler as an IRQ) */
#define IRQ_BOOT            0x01 /* Vector  1: Boot(not handler as an IRQ) */
#define IRQ_TRAP            0x02 /* Vector  2: Software interrupt */
#define IRQ_TRAPE           0x03 /* Vector  3: Emulation Software Interrupt */
#define IRQ_BI              0x03 /* Vector  3: Breakpoint Interrupt */
#define IRQ_CRCALL          0x03 /* Vector  3: Code Replacement Call */
#define IRQ_NMI             0x04 /* Vector  4: Non-Maskable Interrupt */
#define IRQ_INT0            0x05 /* Vector  5: Maskable Interrupt 0 */
#define IRQ_INT1            0x06 /* Vector  6: Maskable Interrupt 1 */
#define IRQ_INT2            0x07 /* Vector  7: Maskable Interrupt 2 */
#define IRQ_INT3            0x08 /* Vector  8: Maskable Interrupt 3 */
#define IRQ_INT4            0x09 /* Vector  9: Maskable Interrupt 4 */
#define IRQ_VINT            0x0a /* Vector 10: Vectored Interrupt */
#define IRQ_TRAP0           0x0b /* Vector 10: Software Interrupt 0 */
#define IRQ_TRAP1           0x0c /* Vector 11: Software Interrupt 1 */
#define IRQ_TRAP2           0x0d /* Vector 12: Software Interrupt 2 */
#define IRQ_TRAP3           0x0e /* Vector 13: Software Interrupt 3 */
#define IRQ_PABP            0x0f /* Vector 15: Program Address Breakpoint */

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
  /* The following array holds the return address
   * needed to return from each nested system call.
   */

  uint8_t nsyscalls;
  struct xcpt_syscall_s syscall[CONFIG_SYS_NNEST];

#endif

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

/* Name: up_irq_save, up_irq_restore, and friends.
 *
 * NOTE: This function should never be called from application code and,
 * as a general rule unless you really know what you are doing, this
 * function should not be called directly from operation system code either:
 * Typically, the wrapper functions, enter_critical_section() and
 * leave_critical section(), are probably what you really want.
 */

/* Get/set the MODp register, here is the irq related bits:
 *   Bit  [0] Interrupt context for NMI   (RW)
 *   Bit  [1] Interrupt context for INT0  (RW)
 *   Bit  [2] Interrupt context for INT1  (RW)
 *   Bit  [3] Interrupt context for INT2  (RW)
 *   Bit  [4] Interrupt context for INT3  (RW)
 *   Bit  [5] Interrupt context for INT4  (RW)
 *   Bit  [6]                             (Reserved)
 *   Bit  [7] Interrupt Enable            (RW)
 *   Bit  [8] Interrupt mask for INT0     (RW)
 *   Bit  [9] Interrupt mask for INT1     (RW)
 *   Bit [10] Interrupt mask for INT2     (RW)
 *   Bit [11] Interrupt mask for INT3     (RW)
 *   Bit [12] Interrupt mask for INT4     (RW)
 *   Bit [13] Interrupt mask for VINT     (RW)
 *   Bit [14]                             (Reserved)
 *   Bit [15] Interrupt pending for INT0  (RO)
 *   Bit [16] Interrupt pending for INT1  (RO)
 *   Bit [17] Interrupt pending for INT2  (RO)
 *   Bit [18] Interrupt pending for INT3  (RO)
 *   Bit [19] Interrupt pending for INT3  (RO)
 *   Bit [20] Interrupt pending for VINT  (RO)
 * All writable bits are clear by hardware during reset.
 *
 * We manipulate the individual mask bits instead of global enable bit since:
 * 1.Global IE not only mask INTX request but also mask TRAPX instruction.
 * 2.Hardware always enable global IE after the interrupt return.
 * Both behavior don't match the nuttx requirement.
 */

static inline uint32_t getmodp(void)
{
  register uint32_t modp __asm__ ("r0");
  __asm__ __volatile__("mov modp, %0\nnop\nnop" : "=r"(modp));
  return modp;
}

static inline void setmodp(uint32_t modp_v)
{
  __asm__ __volatile__("nop\nnop\nmov %0, r0\nnop\nnop" : : "r"(modp_v));
  __asm__ __volatile__("mov r0, modp\nnop\nnop\nnop\nnop\nnop\nnop\nnop\nnop");
}

static inline void up_irq_disable(void)
{
  setmodp(REG_MODP_DISABLE);
}

static inline irqstate_t up_irq_save(void)
{
  irqstate_t flags = getmodp();
  up_irq_disable();
  return flags;
}

static inline void up_irq_enable(void)
{
  setmodp(REG_MODP_ENABLE);
}

static inline void up_irq_restore(irqstate_t flags)
{
  setmodp(flags);
}

/****************************************************************************
 * Name: up_getsp
 ****************************************************************************/

static inline uint32_t up_getsp(void)
{
  uint32_t sp;
  __asm__ __volatile__("nop\nmov sp, %0" : "=r"(sp));
  return sp;
}

#endif /* __ASSEMBLY__ */

#endif /* __ARCH_CEVA_INCLUDE_XC5_IRQ_H */
