/************************************************************************************
 *arch/csky/src/common/up_sigdeliver.c
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

#include <nuttx/irq.h>
#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

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
 * Name: up_sigdeliver
 *
 * Description:
 *   This is the a signal handling trampoline.  When a signal action was
 *   posted.  The task context was mucked with and forced to branch to this
 *   location with interrupts disabled.
 *
 ****************************************************************************/

void up_sigdeliver(void)
{
    struct tcb_s  *rtcb = (struct tcb_s *)g_readytorun.head;
    uint32_t regs[XCPTCONTEXT_REGS];
    sig_deliver_t sigdeliver;

    /* Save the errno.  This must be preserved throughout the signal handling
     * so that the user code final gets the correct errno value (probably
     * EINTR).
     */

    int saved_errno = rtcb->pterrno;

    board_autoled_on(LED_SIGNAL);

    sinfo("rtcb=%p sigdeliver=%p sigpendactionq.head=%p\n",
         rtcb, rtcb->xcp.sigdeliver, rtcb->sigpendactionq.head);
    ASSERT(rtcb->xcp.sigdeliver != NULL);

    /* Save the real return state on the stack. */

    up_copyfullstate(regs, rtcb->xcp.regs);
    regs[REG_PC]         = rtcb->xcp.saved_pc;
    regs[REG_PSR]       = rtcb->xcp.saved_cpsr;

    /* Get a local copy of the sigdeliver function pointer. we do this so that
     * we can nullify the sigdeliver function pointer in the TCB and accept
     * more signal deliveries while processing the current pending signals.
     */

    sigdeliver           = rtcb->xcp.sigdeliver;
    rtcb->xcp.sigdeliver = NULL;

    /* Then restore the task interrupt state */

    irqrestore(regs[REG_PSR]);

    /* Deliver the signals */

    sigdeliver(rtcb);

    /* Output any debug messages BEFORE restoring errno (because they may
     * alter errno), then disable interrupts again and restore the original
     * errno that is needed by the user logic (it is probably EINTR).
     */

    sinfo("Resuming\n");
    (void)irqsave();
    rtcb->pterrno = saved_errno;

    /* Then restore the correct state for this thread of execution. */

    board_autoled_off(LED_SIGNAL);
    up_fullcontextrestore(regs);
}

#endif /* !CONFIG_DISABLE_SIGNALS */

