/****************************************************************************
 * arch/ceva/src/common/up_svcall.c
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
#include <assert.h>
#include <debug.h>

#include <nuttx/irq.h>
#include <nuttx/sched.h>
#include <nuttx/userspace.h>

#include "svcall.h"
#include "up_internal.h"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_svcall
 *
 * Description:
 *   This is SVCall exception handler that performs context switching
 *
 ****************************************************************************/

int up_svcall(int irq, FAR void *context, FAR void *arg)
{
  uint32_t *regs = (uint32_t *)context;
  uint32_t cmd;

  DEBUGASSERT(regs && regs == CURRENT_REGS);
  cmd = regs[REG_A6];

  /* The SVCall software interrupt is called with A6 = system call command
   * and A0..A5 =  variable number of arguments depending on the system call.
   */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
# ifndef CONFIG_DEBUG_SVCALL
  if (cmd > SYS_switch_context)
# endif
    {
      svcinfo("SVCALL Entry: regs: %p cmd: %d\n", regs, cmd);
      svcinfo("A0: %08x %08x %08x %08x %08x %08x %08x\n",
              regs[REG_A0], regs[REG_A1], regs[REG_A2], regs[REG_A3],
              regs[REG_A4], regs[REG_A5], regs[REG_A6]);
      svcinfo("FP: %08x LR: %08x PC: %08x IRQ: %08x OM: %08x\n",
              regs[REG_FP], regs[REG_LR], regs[REG_PC], regs[REG_IRQ],
# ifdef REG_OM
              regs[REG_OM]);
#else
              0x00000000);
#endif
    }
#endif

  /* Handle the SVCall according to the command in A6 */

  switch (cmd)
    {
      /* A6=SYS_save_context:  This is a save context command:
       *
       *   int up_saveusercontext(uint32_t *saveregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A6 = SYS_save_context
       *   A0 = saveregs
       *
       * In this case, we simply need to copy the current regsters to the
       * save register space references in the saved A0 and return.
       */

      case SYS_save_context:
        {
          DEBUGASSERT(regs[REG_A0] != 0);
          memcpy((uint32_t *)regs[REG_A0], regs, XCPTCONTEXT_SIZE);
        }
        break;

      /* A6=SYS_restore_context:  This a restore context command:
       *
       *   void up_fullcontextrestore(uint32_t *restoreregs) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A6 = SYS_restore_context
       *   A0 = restoreregs
       *
       * In this case, we simply need to set CURRENT_REGS to restore register
       * area referenced in the saved A0. context == CURRENT_REGS is the normal
       * exception return.  By setting CURRENT_REGS = context[A0], we force
       * the return to the saved context referenced in A0.
       */

      case SYS_restore_context:
        {
          DEBUGASSERT(regs[REG_A0] != 0);
          CURRENT_REGS = (uint32_t *)regs[REG_A0];
        }
        break;

      /* A6=SYS_switch_context:  This a switch context command:
       *
       *   void up_switchcontext(uint32_t **saveregs, uint32_t *restoreregs);
       *
       * At this point, the following values are saved in context:
       *
       *   A6 = SYS_switch_context
       *   A0 = saveregs
       *   A1 = restoreregs
       *
       * In this case, we do both: We save the context registers to the save
       * register area reference by the saved contents of A0 and then set
       * CURRENT_REGS to to the save register area referenced by the saved
       * contents of A1.
       */

      case SYS_switch_context:
        {
          DEBUGASSERT(regs[REG_A0] != 0 && regs[REG_A1] != 0);
          *(uint32_t **)regs[REG_A0] = regs;
          CURRENT_REGS = (uint32_t *)regs[REG_A1];
        }
        break;

      /* A6=SYS_syscall_return:  This a syscall return command:
       *
       *   void up_syscall_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   A6 = SYS_syscall_return
       *
       * We need to restore the saved return address and return in
       * unprivileged thread mode.
       */

#ifdef CONFIG_LIB_SYSCALL
      case SYS_syscall_return:
        {
          struct tcb_s *rtcb = sched_self();
          int index = rtcb->xcp.nsyscalls - 1;

          /* Make sure that there is a saved syscall return address. */

          DEBUGASSERT(index >= 0);

          /* Setup to return to the saved syscall return address in
           * the original mode.
           */

          regs[REG_PC] = rtcb->xcp.syscall[index].saved_pc;
#ifdef REG_OM
          regs[REG_OM] = rtcb->xcp.syscall[index].saved_om;
#endif
          rtcb->xcp.nsyscalls = index;
        }
        break;
#endif

      /* A6=SYS_task_start:  This a user task start
       *
       *   void up_task_start(main_t taskentry, int argc, FAR char *argv[]) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A6 = SYS_task_start
       *   A0 = taskentry
       *   A1 = argc
       *   A2 = argv
       */

#ifdef CONFIG_BUILD_PROTECTED
      case SYS_task_start:
        {
          /* Set up to return to the user-space task start-up function in
           * unprivileged mode.
           */

          regs[REG_PC]  = (uint32_t)USERSPACE->task_startup;
#ifdef REG_OM
          regs[REG_OM] &= ~REG_OM_MASK;
          regs[REG_OM] |=  REG_OM_USER;
#endif
        }
        break;
#endif

      /* A6=SYS_pthread_start:  This a user pthread start
       *
       *   void up_pthread_start(pthread_startroutine_t entrypt, pthread_addr_t arg) noreturn_function;
       *
       * At this point, the following values are saved in context:
       *
       *   A6 = SYS_pthread_start
       *   A0 = entrypt
       *   A1 = arg
       */

#if defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_DISABLE_PTHREAD)
      case SYS_pthread_start:
        {
          /* Set up to return to the user-space pthread start-up function in
           * unprivileged mode.
           */

          regs[REG_PC]  = (uint32_t)USERSPACE->pthread_startup;
#ifdef REG_OM
          regs[REG_OM] &= ~REG_OM_MASK;
          regs[REG_OM] |=  REG_OM_USER;
#endif
        }
        break;
#endif

      /* A6=SYS_signal_handler:  This a user signal handler callback
       *
       * void signal_handler(_sa_sigaction_t sighand, int signo,
       *                     FAR siginfo_t *info, FAR void *ucontext);
       *
       * At this point, the following values are saved in context:
       *
       *   A6 = SYS_signal_handler
       *   A0 = sighand
       *   A1 = signo
       *   A2 = info
       *   A3 = ucontext
       */

#if defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_DISABLE_SIGNALS)
      case SYS_signal_handler:
        {
          struct tcb_s *rtcb = sched_self();

          /* Remember the caller's return address */

          DEBUGASSERT(rtcb->xcp.sigreturn == 0);
          rtcb->xcp.sigreturn = regs[REG_PC];

          /* Set up to return to the user-space signal handler function in
           * unprivileged mode.
           */

          regs[REG_PC]  = (uint32_t)USERSPACE->signal_handler;
#ifdef REG_OM
          regs[REG_OM] &= ~REG_OM_MASK;
          regs[REG_OM] |=  REG_OM_USER;
#endif
        }
        break;
#endif

      /* R6=SYS_signal_handler_return:  This a user signal handler callback
       *
       *   void signal_handler_return(void);
       *
       * At this point, the following values are saved in context:
       *
       *   A6 = SYS_signal_handler_return
       */

#if defined(CONFIG_BUILD_PROTECTED) && !defined(CONFIG_DISABLE_SIGNALS)
      case SYS_signal_handler_return:
        {
          struct tcb_s *rtcb = sched_self();

          /* Set up to return to the kernel-mode signal dispatching logic. */

          DEBUGASSERT(rtcb->xcp.sigreturn != 0);

          regs[REG_PC]  = rtcb->xcp.sigreturn;
#ifdef REG_OM
          regs[REG_OM] &= ~REG_OM_MASK;
          regs[REG_OM] |=  REG_OM_KERNEL;
#endif
          rtcb->xcp.sigreturn = 0;
        }
        break;
#endif

      /* This is not an architecture-specific system call.  If NuttX is built
       * as a standalone kernel with a system call interface, then all of the
       * additional system calls must be handled as in the default case.
       */

      default:
        {
#ifdef CONFIG_LIB_SYSCALL
          FAR struct tcb_s *rtcb = sched_self();
          int index = rtcb->xcp.nsyscalls;

          /* Verify that the SYS call number is within range */

          DEBUGASSERT(cmd >= CONFIG_SYS_RESERVED && cmd < SYS_maxsyscall);
          DEBUGASSERT(index < CONFIG_SYS_NNEST);

          /* Setup to return to up_svcall_handler in privileged mode. */

          rtcb->xcp.syscall[index].saved_pc = regs[REG_PC];
#ifdef REG_OM
          rtcb->xcp.syscall[index].saved_om = regs[REG_OM];
#endif
          rtcb->xcp.nsyscalls = index + 1;

          regs[REG_PC]  = (uint32_t)up_svcall_handler;
#ifdef REG_OM
          regs[REG_OM] &= ~REG_OM_MASK;
          regs[REG_OM] |=  REG_OM_KERNEL;
#endif

          /* Rotate A6(cmd) to A0 to account for the stub prototype
           * Offset A6(cmd) to account for the reserved values
           */

          regs[REG_A6] = regs[REG_A5];
          regs[REG_A5] = regs[REG_A4];
          regs[REG_A4] = regs[REG_A3];
          regs[REG_A3] = regs[REG_A2];
          regs[REG_A2] = regs[REG_A1];
          regs[REG_A1] = regs[REG_A0];
          regs[REG_A0] = cmd - CONFIG_SYS_RESERVED;
#else
          svcerr("ERROR: Bad SYS call: %d\n", regs[REG_A6]);
#endif
        }
        break;
    }

  /* Report what happened.  That might be different in the case of a context switch */

#ifdef CONFIG_DEBUG_SYSCALL_INFO
# ifndef CONFIG_DEBUG_SVCALL
  if (cmd > SYS_switch_context)
# else
  if (regs != CURRENT_REGS)
# endif
    {
      svcinfo("SVCall Return:\n");
      svcinfo("A0: %08x %08x %08x %08x %08x %08x %08x\n",
              CURRENT_REGS[REG_A0], CURRENT_REGS[REG_A1],
              CURRENT_REGS[REG_A2], CURRENT_REGS[REG_A3],
              CURRENT_REGS[REG_A4], CURRENT_REGS[REG_A5],
              CURRENT_REGS[REG_A6]);
      svcinfo("FP: %08x LR: %08x PC: %08x IRQ: %08x OM: %08x\n",
              CURRENT_REGS[REG_FP], CURRENT_REGS[REG_LR],
              CURRENT_REGS[REG_PC], CURRENT_REGS[REG_IRQ],
# ifdef REG_OM
              CURRENT_REGS[REG_OM]);
#else
              0x00000000);
#endif
    }
# ifdef CONFIG_DEBUG_SVCALL
  else
    {
      svcinfo("SVCall Return: %d\n", regs[REG_A0]);
    }
# endif
#endif

  return OK;
}
