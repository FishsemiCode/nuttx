/****************************************************************************
 *  arch/ceva/src/common/up_releasestack.c
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

#include <assert.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/kmalloc.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration */

#undef HAVE_KERNEL_HEAP
#if (defined(CONFIG_BUILD_PROTECTED) || defined(CONFIG_BUILD_KERNEL)) && \
     defined(CONFIG_MM_KERNEL_HEAP)
#  define HAVE_KERNEL_HEAP 1
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

#ifdef HAVE_KERNEL_HEAP
static sq_queue_t g_delayed_kfree;
#endif

#ifndef CONFIG_BUILD_KERNEL
static sq_queue_t g_delayed_ufree;
#endif

/****************************************************************************
 * Public Functions
 ****************************************************************************/

bool up_sched_have_garbage(void)
{
#ifdef HAVE_KERNEL_HEAP
  if (g_delayed_kfree.head != NULL)
    {
      return true;
    }
#endif
#ifndef CONFIG_BUILD_KERNEL
  if (g_delayed_ufree.head != NULL)
  {
    return true;
  }
#endif
  return false;
}

void up_sched_garbage_collection(void)
{
#ifdef HAVE_KERNEL_HEAP
  while (g_delayed_kfree.head)
    {
      irqstate_t flags;
      FAR void *address;

      /* Remove the first delayed deallocation.  This is not atomic and so
       * we must disable interrupts around the queue operation.
       */

      flags = spin_lock_irqsave();
      address = sq_remfirst(&g_delayed_kfree);
      spin_unlock_irqrestore(flags);

      /* Return the memory to the kernel heap
       * which may come from the default heap due to up_use_stack.
       */

      if (kmm_heapmember(address))
        {
          kmm_free(address);
        }
      else
        {
          mm_free(KMM_HEAP(CONFIG_ARCH_KERNEL_STACK_HEAP), address);
        }
    }
#endif
#ifndef CONFIG_BUILD_KERNEL
  while (g_delayed_ufree.head)
    {
      irqstate_t flags;
      FAR void *address;

      /* Remove the first delayed deallocation.  This is not atomic and so
       * we must disable interrupts around the queue operation.
       */

      flags = spin_lock_irqsave();
      address = sq_remfirst(&g_delayed_ufree);
      spin_unlock_irqrestore(flags);

      /* Return the memory to the user heap
       * which may come from the default heap due to up_use_stack.
       */

      if (umm_heapmember(address))
        {
          kumm_free(address);
        }
      else
        {
          mm_free(UMM_HEAP(CONFIG_ARCH_STACK_HEAP), address);
        }
    }
#endif
}

/****************************************************************************
 * Name: up_release_stack
 *
 * Description:
 *   A task has been stopped. Free all stack related resources retained in
 *   the defunct TCB.
 *
 * Input Parameters:
 *   - dtcb:  The TCB containing information about the stack to be released
 *   - ttype:  The thread type.  This may be one of following (defined in
 *     include/nuttx/sched.h):
 *
 *       TCB_FLAG_TTYPE_TASK     Normal user task
 *       TCB_FLAG_TTYPE_PTHREAD  User pthread
 *       TCB_FLAG_TTYPE_KERNEL   Kernel thread
 *
 *     This thread type is normally available in the flags field of the TCB,
 *     however, there are certain error recovery contexts where the TCB may
 *     not be fully initialized when up_release_stack is called.
 *
 *     If either CONFIG_BUILD_PROTECTED or CONFIG_BUILD_KERNEL are defined,
 *     then this thread type may affect how the stack is freed.  For example,
 *     kernel thread stacks may have been allocated from protected kernel
 *     memory.  Stacks for user tasks and threads must have come from memory
 *     that is accessible to user code.
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void up_release_stack(FAR struct tcb_s *dtcb, uint8_t ttype)
{
  /* Is there a stack allocated? */

  if (dtcb->stack_alloc_ptr)
    {
#ifdef HAVE_KERNEL_HEAP
      /* Use the kernel allocator if this is a kernel thread */

      if (ttype == TCB_FLAG_TTYPE_KERNEL)
        {
          if (kmm_heapmember(dtcb->stack_alloc_ptr) ||
              mm_heapmember(KMM_HEAP(CONFIG_ARCH_KERNEL_STACK_HEAP),
                            dtcb->stack_alloc_ptr))
          {
            irqstate_t flags;

            /* Delay the deallocation until a more appropriate time. */

            flags = spin_lock_irqsave();
            sq_addlast(dtcb->stack_alloc_ptr, &g_delayed_kfree);
            spin_unlock_irqrestore(flags);

            sched_signal_free();
          }
        }
      else
#endif
        {
          /* Use the user-space allocator if this is a task or pthread */

#ifdef CONFIG_BUILD_KERNEL
          /* REVISIT:  It is not safe to defer user allocation in the kernel mode
           * build.  Why?  Because the correct user context is in place now but
           * will not be in place when the deferred de-allocation is performed.  In
           * order to make this work, we would need to do something like:  (1) move
           * g_delayed_kufree into the group structure, then traverse the groups to
           * collect garbage on a group-by-group basis.
           */

          DEBUGASSERT(!up_interrupt_context());
          if (umm_heapmember(dtcb->stack_alloc_ptr))
            {
              kumm_free(dtcb->stack_alloc_ptr);
            }
          else if (mm_heapmember(UMM_HEAP(CONFIG_ARCH_STACK_HEAP),
                                 dtcb->stack_alloc_ptr))
            {
              mm_free(UMM_HEAP(CONFIG_ARCH_STACK_HEAP), dtcb->stack_alloc_ptr);
            }
#else
          if (umm_heapmember(dtcb->stack_alloc_ptr) ||
              mm_heapmember(UMM_HEAP(CONFIG_ARCH_STACK_HEAP),
                            dtcb->stack_alloc_ptr))
          {
            irqstate_t flags;

            /* Delay the deallocation until a more appropriate time. */

            flags = spin_lock_irqsave();
            sq_addlast(dtcb->stack_alloc_ptr, &g_delayed_ufree);
            spin_unlock_irqrestore(flags);

            sched_signal_free();
          }
#endif
        }

      /* Mark the stack freed */

      dtcb->stack_alloc_ptr = NULL;
    }

  /* The size of the allocated stack is now zero */

  dtcb->adj_stack_size = 0;
}
