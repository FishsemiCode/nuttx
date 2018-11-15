/************************************************************************************
 *arch/csky/src/common/up_schedulesigaction.c
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

#include <stdint.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>

#include "csky.h"
#include "sched/sched.h"
#include "up_internal.h"
#include "up_arch.h"

#ifndef CONFIG_DISABLE_SIGNALS

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define THIS_MODULE MODULE_ARCH

/****************************************************************************
 * Private Data
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_schedule_sigaction
 *
 * Description:
 *   This function is called by the OS when one or more
 *   signal handling actions have been queued for execution.
 *   The architecture specific code must configure things so
 *   that the 'igdeliver' callback is executed on the thread
 *   specified by 'tcb' as soon as possible.
 *
 *   This function may be called from interrupt handling logic.
 *
 *   This operation should not cause the task to be unblocked
 *   nor should it cause any immediate execution of sigdeliver.
 *   Typically, a few cases need to be considered:
 *
 *   (1) This function may be called from an interrupt handler
 *       During interrupt processing, all xcptcontext structures
 *       should be valid for all tasks.  That structure should
 *       be modified to invoke sigdeliver() either on return
 *       from (this) interrupt or on some subsequent context
 *       switch to the recipient task.
 *   (2) If not in an interrupt handler and the tcb is NOT
 *       the currently executing task, then again just modify
 *       the saved xcptcontext structure for the recipient
 *       task so it will invoke sigdeliver when that task is
 *       later resumed.
 *   (3) If not in an interrupt handler and the tcb IS the
 *       currently executing task -- just call the signal
 *       handler now.
 *
 ****************************************************************************/

void up_schedule_sigaction(struct tcb_s *tcb, sig_deliver_t sigdeliver)
{
    irqstate_t flags;

    sinfo("tcb=0x%p sigdeliver=0x%p\n", tcb, sigdeliver);

    /* Make sure that interrupts are disabled */

    flags = irqsave();

    /* Refuse to handle nested signal actions */

    if (!tcb->xcp.sigdeliver) {
        /* First, handle some special cases when the signal is
         * being delivered to the currently executing task.
         */

        sinfo("rtcb=0x%p current_regs=0x%p\n", g_readytorun.head, current_regs);

        if (tcb == (struct tcb_s *)g_readytorun.head) {
            /* CASE 1:  We are not in an interrupt handler and
             * a task is signalling itself for some reason.
             */

            if (!current_regs) {
                /* In this case just deliver the signal now. */

                sigdeliver(tcb);
            }

            /* CASE 2:  We are in an interrupt handler AND the
             * interrupted task is the same as the one that
             * must receive the signal, then we will have to modify
             * the return state as well as the state in the TCB.
             *
             * Hmmm... there looks like a latent bug here: The following
             * logic would fail in the strange case where we are in an
             * interrupt handler, the thread is signalling itself, but
             * a context switch to another task has occurred so that
             * current_regs does not refer to the thread at g_readytorun.head!
             */

            else {
                /* Save the return lr and cpsr and one scratch register
                 * These will be restored by the signal trampoline after
                 * the signals have been delivered.
                 */

                tcb->xcp.sigdeliver       = sigdeliver;
                tcb->xcp.saved_pc         = current_regs[REG_PC];
                tcb->xcp.saved_cpsr       = current_regs[REG_PSR];

                /* Then set up to vector to the trampoline with interrupts
                 * disabled
                 */

                current_regs[REG_PC]      = (uint32_t)up_sigdeliver;
                current_regs[REG_PSR]    = SVC_MODE & PSR_I_BIT & PSR_F_BIT;

                /* And make sure that the saved context in the TCB
                 * is the same as the interrupt return context.
                 */

                up_savestate(tcb->xcp.regs);
            }
        }

        /* Otherwise, we are (1) signaling a task is not running
         * from an interrupt handler or (2) we are not in an
         * interrupt handler and the running task is signalling
         * some non-running task.
         */

        else {
            /* Save the return lr and cpsr and one scratch register
             * These will be restored by the signal trampoline after
             * the signals have been delivered.
             */

            tcb->xcp.sigdeliver       = sigdeliver;
            tcb->xcp.saved_pc         = tcb->xcp.regs[REG_PC];
            tcb->xcp.saved_cpsr       = tcb->xcp.regs[REG_PSR];

            /* Then set up to vector to the trampoline with interrupts
             * disabled
             */

            tcb->xcp.regs[REG_PC]      = (uint32_t)up_sigdeliver;
            tcb->xcp.regs[REG_PSR]    = SVC_MODE & PSR_I_BIT & PSR_F_BIT;
        }
    }

    irqrestore(flags);
}

#endif /* !CONFIG_DISABLE_SIGNALS */
