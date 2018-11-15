/************************************************************************************
 *arch/csky/src/common/up_releasepending.c
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

#include <sched.h>
#include <debug.h>
#include <nuttx/arch.h>
#include <nuttx/sched.h>
#include <nuttx/clock.h>

#include "sched/sched.h"
#include "group/group.h"
#include "up_internal.h"

#define THIS_MODULE MODULE_ARCH

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_release_pending
 *
 * Description:
 *   Release and ready-to-run tasks that have
 *   collected in the pending task list.  This can call a
 *   context switch if a new task is placed at the head of
 *   the ready to run list.
 *
 ****************************************************************************/

void up_release_pending(void)
{
    struct tcb_s *rtcb = (struct tcb_s *)g_readytorun.head;

    sinfo("From TCB=%p\n", rtcb);

    /* Merge the g_pendingtasks list into the g_readytorun task list */

    /* sched_lock(); */
    if (sched_mergepending()) {
        /* The currently active task has changed!  We will need to switch
         * contexts.
         *
         * Update scheduler parameters.
         */

        sched_suspend_scheduler(rtcb);

        /* Are we operating in interrupt context? */

        if (current_regs) {
            /* Yes, then we have to do things differently.
             * Just copy the current_regs into the OLD rtcb.
             */

            up_savestate(rtcb->xcp.regs);

            /* Restore the exception context of the rtcb at the (new) head
             * of the g_readytorun task list.
             */

            rtcb = (struct tcb_s *)g_readytorun.head;

            /* Update scheduler parameters */

            sched_resume_scheduler(rtcb);

            /* Then switch contexts.  Any necessary address environment
             * changes will be made when the interrupt returns.
             */

            up_restorestate(rtcb->xcp.regs);
        }

        /* Copy the exception context into the TCB of the task that
         * was currently active. if up_saveusercontext returns a non-zero
         * value, then this is really the previously running task
         * restarting!
         */

        else if (!up_saveusercontext(rtcb->xcp.regs)) {
            /* Restore the exception context of the rtcb at the (new) head
             * of the g_readytorun task list.
             */

            rtcb = (struct tcb_s *)g_readytorun.head;

#ifdef CONFIG_ARCH_ADDRENV
            /* Make sure that the address environment for the previously
             * running task is closed down gracefully (data caches dump,
             * MMU flushed) and set up the address environment for the new
             * thread at the head of the ready-to-run list.
             */

            (void)group_addrenv(rtcb);
#endif
            /* Update scheduler parameters */

            sched_resume_scheduler(rtcb);

            /* Then switch contexts */

            up_fullcontextrestore(rtcb->xcp.regs);
        }
    }
}
