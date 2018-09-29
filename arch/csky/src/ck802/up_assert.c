/************************************************************************************
 * arch/csky/src/ck802/up_assert.c
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


/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

/* Output debug info if stack dump is selected -- even if debug is not
 * selected.
 */

#ifdef CONFIG_ARCH_STACKDUMP
# undef  CONFIG_DEBUG
# undef  CONFIG_DEBUG_VERBOSE
# define CONFIG_DEBUG 1
# define CONFIG_DEBUG_VERBOSE 1
#endif

#include <stdint.h>
#include <stdlib.h>
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/usb/usbdev_trace.h>

#include <arch/board/board.h>

#include "up_arch.h"
//#include "sched/sched.h"
#include "up_internal.h"

#define THIS_MODULE MODULE_ARCH

#if 0
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* USB trace dumping */

#ifndef CONFIG_USBDEV_TRACE
#  undef CONFIG_ARCH_USBDUMP
#endif

/* The following is just intended to keep some ugliness out of the mainline
 * code.  We are going to print the task name if:
 *
 *  CONFIG_TASK_NAME_SIZE > 0 &&    <-- The task has a name
 *  (defined(CONFIG_DEBUG)    ||    <-- And the debug is enabled (sinfo used)
 *   defined(CONFIG_ARCH_STACKDUMP) <-- Or lowsyslog() is used
 */

#undef CONFIG_PRINT_TASKNAME
#if CONFIG_TASK_NAME_SIZE > 0 && (defined(CONFIG_DEBUG) || defined(CONFIG_ARCH_STACKDUMP))
#  define CONFIG_PRINT_TASKNAME 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_getsp
 ****************************************************************************/

/* I don't know if the builtin to get SP is enabled */

static inline uint32_t up_getsp(void)
{
    uint32_t sp=0;//set initial value to 0

//hge delete
#if 0
    __asm__
    (
        "\tmov %0, sp\n\t"
        : "=r"(sp)
    );
#endif
    return sp;
}

/****************************************************************************
 * Name: up_stackdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void up_stackdump(uint32_t sp, uint32_t stack_base)
{
    uint32_t stack ;

    for (stack = sp & ~0x1f; stack < stack_base; stack += 32) {
        uint32_t *ptr = (uint32_t *)stack;
        sinfo("%08x: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              stack, ptr[0], ptr[1], ptr[2], ptr[3],
              ptr[4], ptr[5], ptr[6], ptr[7]);
    }
}
#else
#  define up_stackdump(sp,stack_base)
#endif

/****************************************************************************
 * Name: up_taskdump
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static void up_taskdump(FAR struct tcb_s *tcb, FAR void *arg)
{
    /* Dump interesting properties of this task */

#ifdef CONFIG_PRINT_TASKNAME
    sinfo("%s: PID=%d Stack Used=%lu of %lu\n",
          tcb->name, tcb->pid, (unsigned long)up_check_tcbstack(tcb),
          (unsigned long)tcb->adj_stack_size);
#else
    sinfo("PID: %d Stack Used=%lu of %lu\n",
          tcb->pid, (unsigned long)up_check_tcbstack(tcb),
          (unsigned long)tcb->adj_stack_size);
#endif
}
#endif

/****************************************************************************
 * Name: up_showtasks
 ****************************************************************************/

#ifdef CONFIG_STACK_COLORATION
static inline void up_showtasks(void)
{
    /* Dump interesting properties of each task in the crash environment */

    sched_foreach(up_taskdump, NULL);
}
#else
#  define up_showtasks()
#endif

/****************************************************************************
 * Name: up_registerdump
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static inline void up_registerdump(void)
{
    /* Are user registers available from interrupt processing? */

    if (current_regs) {
        /* Yes.. dump the interrupt registers */

        sinfo("R0: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              current_regs[REG_R0],  current_regs[REG_R1],
              current_regs[REG_R2],  current_regs[REG_R3],
              current_regs[REG_R4],  current_regs[REG_R5],
              current_regs[REG_R6],  current_regs[REG_R7]);
        sinfo("R8: %08x %08x %08x %08x %08x %08x %08x %08x\n",
              current_regs[REG_R8],  current_regs[REG_R9],
              current_regs[REG_R10], current_regs[REG_R11],
              current_regs[REG_R12], current_regs[REG_R13],
              current_regs[REG_R14], current_regs[REG_R15]);

#ifdef CONFIG_ARMV7M_USEBASEPRI
        sinfo("xPSR: %08x BASEPRI: %08x CONTROL: %08x\n",
              current_regs[REG_XPSR],  current_regs[REG_BASEPRI],
              getcontrol());
#else
        sinfo("xPSR: %08x PRIMASK: %08x CONTROL: %08x\n",
              current_regs[REG_XPSR],  current_regs[REG_PRIMASK],
              getcontrol());
#endif

#ifdef REG_EXC_RETURN
        sinfo("EXC_RETURN: %08x\n", current_regs[REG_EXC_RETURN]);
#endif
    }
}
#else
# define up_registerdump()
#endif

/****************************************************************************
 * Name: assert_tracecallback
 ****************************************************************************/

#ifdef CONFIG_ARCH_USBDUMP
static int usbtrace_syslog(FAR const char *fmt, ...)
{
    va_list ap;
    int ret;

    /* Let vsyslog do the real work */

    va_start(ap, fmt);
    ret = lowvsyslog(LOG_INFO, fmt, ap);
    va_end(ap);
    return ret;
}

static int assert_tracecallback(FAR struct usbtrace_s *trace, FAR void *arg)
{
    usbtrace_trprintf(usbtrace_syslog, trace->event, trace->value);
    return 0;
}
#endif

/****************************************************************************
 * Name: up_dumpstate
 ****************************************************************************/

#ifdef CONFIG_ARCH_STACKDUMP
static void up_dumpstate(void)
{
    struct tcb_s *rtcb = (struct tcb_s *)g_readytorun.head;
    uint32_t sp = up_getsp();
    uint32_t ustackbase;
    uint32_t ustacksize;
#if CONFIG_ARCH_INTERRUPTSTACK > 3
    uint32_t istackbase;
    uint32_t istacksize;
#endif

    /* Get the limits on the user stack memory */

    if (rtcb->pid == 0) {
        ustackbase = g_idle_topstack - 4;
        ustacksize = CONFIG_IDLETHREAD_STACKSIZE;
    } else {
        ustackbase = (uint32_t)rtcb->adj_stack_ptr;
        ustacksize = (uint32_t)rtcb->adj_stack_size;
    }

#if CONFIG_ARCH_INTERRUPTSTACK > 3
    /* Get the limits on the interrupt stack memory */

    istackbase = (uint32_t)&g_intstackbase;
    istacksize = (CONFIG_ARCH_INTERRUPTSTACK & ~3);

    /* Show interrupt stack info */

    sinfo("sp:     %08x\n", sp);
    sinfo("IRQ stack:\n");
    sinfo("  base: %08x\n", istackbase);
    sinfo("  size: %08x\n", istacksize);
#ifdef CONFIG_STACK_COLORATION
    sinfo("  used: %08x\n", up_check_intstack());
#endif

    /* Does the current stack pointer lie within the interrupt
     * stack?
     */

    if (sp <= istackbase && sp > istackbase - istacksize) {
        /* Yes.. dump the interrupt stack */

        up_stackdump(sp, istackbase);
    }

    /* Extract the user stack pointer if we are in an interrupt handler.
     * If we are not in an interrupt handler.  Then sp is the user stack
     * pointer (and the above range check should have failed).
     */

    if (current_regs) {
        sp = current_regs[REG_R13];
        sinfo("sp:     %08x\n", sp);
    }

    sinfo("User stack:\n");
    sinfo("  base: %08x\n", ustackbase);
    sinfo("  size: %08x\n", ustacksize);
#ifdef CONFIG_STACK_COLORATION
    sinfo("  used: %08x\n", up_check_tcbstack(rtcb));
#endif

    /* Dump the user stack if the stack pointer lies within the allocated user
     * stack memory.
     */

    if (sp <= ustackbase && sp > ustackbase - ustacksize) {
        up_stackdump(sp, ustackbase);
    }

#else

    /* Show user stack info */

    sinfo("sp:         %08x\n", sp);
    sinfo("stack base: %08x\n", ustackbase);
    sinfo("stack size: %08x\n", ustacksize);
#ifdef CONFIG_STACK_COLORATION
    sinfo("stack used: %08x\n", up_check_tcbstack(rtcb));
#endif

    /* Dump the user stack if the stack pointer lies within the allocated user
     * stack memory.
     */

    if (sp > ustackbase || sp <= ustackbase - ustacksize) {
        sinfo("ERROR: Stack pointer is not within the allocated stack\n");
    } else {
        up_stackdump(sp, ustackbase);
    }

#endif

    /* Then dump the registers (if available) */

    up_registerdump();

    /* Dump the state of all tasks (if available) */

    up_showtasks();

#ifdef CONFIG_ARCH_USBDUMP
    /* Dump USB trace data */

    (void)usbtrace_enumerate(assert_tracecallback, NULL);
#endif
}
#else
# define up_dumpstate()
#endif

/****************************************************************************
 * Name: _up_assert
 ****************************************************************************/

static void _up_assert(int errorcode) noreturn_function;
static void _up_assert(int errorcode)
{
    /* Are we in an interrupt handler or the idle task? */

    if (current_regs || ((struct tcb_s *)g_readytorun.head)->pid == 0) {
        (void)irqsave();
        for (; ; ) {
#ifdef CONFIG_ARCH_LEDS
            board_autoled_on(LED_PANIC);
            up_mdelay(250);
            board_autoled_off(LED_PANIC);
            up_mdelay(250);
#endif
        }
    } else {
        while(1);
        exit(errorcode);
    }
}
#endif
/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_assert
 ****************************************************************************/

void up_assert(const uint8_t *filename, int lineno)
{
#ifdef CONFIG_PRINT_TASKNAME
    struct tcb_s *rtcb = (struct tcb_s *)g_readytorun.head;
#endif

    board_autoled_on(LED_ASSERTION);

#ifdef CONFIG_PRINT_TASKNAME
    sinfo("Assertion failed at file:%s line: %d task: %s\n",
          filename, lineno, rtcb->name);
#else
    sinfo("Assertion failed at file:%s line: %d\n",
          filename, lineno);
#endif

#ifndef CONFIG_DEBUG
   // syslog(LOG_ERR, "Assertion failed at file:%s line: %d\n",   filename, lineno);
#endif

    //up_dumpstate();

#ifdef CONFIG_BOARD_CRASHDUMP
    board_crashdump(up_getsp(), g_readytorun.head, filename, lineno);
#endif

    //_up_assert(EXIT_FAILURE);
     while(1);
}
