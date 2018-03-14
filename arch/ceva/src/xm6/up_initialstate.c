/****************************************************************************
 * arch/ceva/src/xm6/up_initialstate.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <string.h>

#include <nuttx/arch.h>
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define REG_MODE_DEFAULT          0x40004000 /* PR14H and PR14L */

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

  /* Initialize the initial exception register context structure */

  memset(xcp, 0, sizeof(struct xcptcontext));

  if (tcb->adj_stack_ptr)
    {
      xcp->regs = tcb->adj_stack_ptr - XCPTCONTEXT_SIZE;
      memset(xcp->regs, 0, XCPTCONTEXT_SIZE);

      /* Save the task entry point */

      xcp->regs[REG_PC]   = (uint32_t)tcb->start;

#ifdef CONFIG_ARCH_XM6_STACKCHECK
      /* Set the stack memory region for debugging */

      xcp->regs[REG_SB]   = (uint32_t)tcb->stack_alloc_ptr;
      xcp->regs[REG_SL]   = (uint32_t)tcb->adj_stack_ptr;
#endif

      /* Initialize all no zero registers */

      /* All tasks start via a stub function in kernel space.  So all
       * tasks must start in privileged thread mode.  If CONFIG_BUILD_PROTECTED
       * is defined, then that stub function will switch to unprivileged
       * mode before transferring control to the user task.
       */

      xcp->regs[REG_OM]   = REG_OM_DEFAULT;

#ifdef CONFIG_ARCH_XM6_BUG001
      /* See BUG 001 in CEVA-XM6 Bug List */

      xcp->regs[REG_MODE] = REG_MODE_DEFAULT;
#endif
    }
}
