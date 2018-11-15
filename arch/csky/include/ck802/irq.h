/************************************************************************************
 *arch/ck802/include/irq.h
 *
 * Copyright (C) 2015 The YunOS Open Source Project
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ************************************************************************************/


/* This file should never be included directed but, rather, only indirectly
 * through nuttx/irq.h
 */

#ifndef __ARCH_CSKY_INCLUDE_CK802_IRQ_H
#define __ARCH_CSKY_INCLUDE_CK802_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <nuttx/irq.h>
#ifndef __ASSEMBLY__
#  include <nuttx/compiler.h>
#  include <arch/chip/chip.h>
#  include <stdint.h>
#  include <arch/ck802/csi_core.h>
#  include <arch/ck802/csi_gcc.h>
#endif

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* IRQ numbers.  The IRQ number corresponds vector number and hence map directly to
 * bits in the NVIC.  This does, however, waste several words of memory in the IRQ
 * to handle mapping tables.
 *
 *
 * External interrupts (vectors >= 16)
 */
#define  NR_IRQS             32

/* Configuration ************************************************************/
/* If this is a kernel build, how many nested system calls should we support? */

#ifndef CONFIG_SYS_NNEST
#  define CONFIG_SYS_NNEST 1
#endif

/* Alternate register names *************************************************/

#define REG_R0              (0)
#define REG_R1              (1)
#define REG_R2              (2)
#define REG_R3              (3)
#define REG_R4              (4)
#define REG_R5              (5)
#define REG_R6              (6)
#define REG_R7              (7)
#define REG_R8              (8)
#define REG_R9              (9)
#define REG_R10             (10)
#define REG_R11             (11)
#define REG_R12             (12)
#define REG_R13             (13)
#define REG_R14             (14)
#define REG_R15             (15)
#define REG_PSR             (16)
#define REG_PC              (17)
#define XCPTCONTEXT_REGS    (18)

#define XCPTCONTEXT_SIZE    (4 * XCPTCONTEXT_REGS)

#define REG_A0              REG_R0
#define REG_A1              REG_R1
#define REG_A2              REG_R2
#define REG_A3              REG_R3
#define REG_SP              REG_R14
#define REG_LR              REG_R15

/****************************************************************************
 * Public Types
 ****************************************************************************/
#ifndef __ASSEMBLY__

/* This structure represents the return state from a system call */

#ifdef CONFIG_LIB_SYSCALL
struct xcpt_syscall_s {
    uint32_t excreturn;   /* The EXC_RETURN value */
    uint32_t sysreturn;   /* The return PC */
};
#endif

/* The following structure is included in the TCB and defines the complete
 * state of the thread.
 */

struct xcptcontext {
    /* The following function pointer is non-zero if there
     * are pending signals to be processed.
     */

#ifndef CONFIG_DISABLE_SIGNALS
    void *sigdeliver; /* Actual type is sig_deliver_t */

    /* These are saved copies of LR and CPSR used during
     * signal processing.
     */

    uint32_t saved_pc;
    uint32_t saved_cpsr;
#endif

    /* Register save area */

    uint32_t regs[XCPTCONTEXT_REGS];

    /* Extra fault address register saved for common paging logic.  In the
     * case of the prefetch abort, this value is the same as regs[REG_R15];
     * For the case of the data abort, this value is the value of the fault
     * address register (FAR) at the time of data abort exception.
     */

#ifdef CONFIG_PAGING
    uintptr_t far;
#endif
};
#endif


/****************************************************************************
 * Inline functions
 ****************************************************************************/

#ifndef __ASSEMBLY__

/* Disable IRQs */

static inline void irqdisable(void) inline_function;
static inline void irqdisable(void)
{
    asm volatile(
        "psrclr ie     \n"
        : : :"memory");
}

/* Save the current primask state & disable IRQs */
static inline irqstate_t irqsave(void) inline_function;
static inline irqstate_t irqsave(void)
{
    unsigned long flags;
    asm volatile(
        "mfcr   %0, psr  \n"
        "psrclr ie      \n"
        :"=r"(flags) : :"memory" );
    return flags;
}

/* Enable IRQs */

static inline void irqenable(void) inline_function;
static inline void irqenable(void)
{
    asm volatile(
        "psrset ee, ie \n"
        : : :"memory" );
}

/* Restore saved primask state */

static inline void irqrestore(irqstate_t flags) inline_function;
static inline void irqrestore(irqstate_t flags)
{
    asm volatile(
        "mtcr    %0, psr  \n"
        : :"r" (flags) :"memory" );
}

/* Save the current primask state & disable IRQs */
static inline irqstate_t up_irq_save(void) inline_function;
static inline irqstate_t up_irq_save(void)
{
    return csi_irq_save();
}

/* Restore saved primask state */

static inline void up_irq_restore(irqstate_t pri) inline_function;
static inline void up_irq_restore(irqstate_t pri)
{
    csi_irq_restore(pri);
}

#endif /* __ASSEMBLY__ */

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__
#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

#undef EXTERN
#ifdef __cplusplus
}
#endif
#endif

#endif /* __ARCH_CSKY_INCLUDE_CK802_IRQ_H */

