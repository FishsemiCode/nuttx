/****************************************************************************
 *  arch/csky/src/common/up_initialstate.c
 *
 *
   * Copyright (C) 2015 The YunOS Open Source Project
  *
  * Licensed under the Apache License, Version 2.0 (the "License");
  * you may not use this file except in compliance with the License.
  * You may obtain a copy of the License at
  *
  * 	 http://www.apache.org/licenses/LICENSE-2.0
  *
  * Unless required by applicable law or agreed to in writing, software
  * distributed under the License is distributed on an "AS IS" BASIS,
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.

 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>
#include <string.h>

#include <nuttx/arch.h>

#include "csky.h"
#include "up_internal.h"
#include "up_arch.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

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
 * Name: up_initial_state
 *
 * Description:
 *   A new thread is being started and a new TCB
 *   has been created. This function is called to initialize
 *   the processor specific portions of the new TCB.
 *
 *   This function must setup the intial architecture registers
 *   and/or  stack so that execution will begin at tcb->start
 *   on the next context switch.
 *
 ****************************************************************************/

void up_initial_state(struct tcb_s *tcb)
{
    struct xcptcontext *xcp = &tcb->xcp;
    uint32_t psr;

    /* Initialize the initial exception register context structure */

    memset(xcp, 0, sizeof(struct xcptcontext));

    /* Save the initial stack pointer */

    xcp->regs[REG_SP]      = (uint32_t)tcb->adj_stack_ptr;

    /* Save the task entry point */

    xcp->regs[REG_PC]      = (uint32_t)tcb->start;

    /* If this task is running PIC, then set the PIC base register to the
     * address of the allocated D-Space region.
     */

#ifdef CONFIG_PIC
    if (tcb->dspace != NULL) {
        /* Set the PIC base register (probably R10) to the address of the
         * alloacated D-Space region.
         */

        xcp->regs[REG_PIC] = (uint32_t)tcb->dspace->region;
    }
#endif

    /* Set supervisor- or user-mode, depending on how NuttX is configured and
     * what kind of thread is being started.  Disable FIQs in any event
     */

#ifdef CONFIG_BUILD_KERNEL
    if ((tcb->flags & TCB_FLAG_TTYPE_MASK) == TCB_FLAG_TTYPE_KERNEL) {
        /* It is a kernel thread.. set supervisor mode */

        psr               = SVC_MODE & PSR_F_BIT;
    } else {
        /* It is a normal task or a pthread.  Set user mode */

        psr               = USR_MODE & PSR_F_BIT;
    }
#else
    /* If the kernel build is not selected, then all threads run in
     * supervisor-mode.
     */

    psr                   = SVC_MODE & PSR_F_BIT;
#endif

    /* Enable or disable interrupts, based on user configuration */

# ifdef CONFIG_SUPPRESS_INTERRUPTS
    psr                  &= PSR_I_BIT;
# endif

    xcp->regs[REG_PSR]    = psr;
}

