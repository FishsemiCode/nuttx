/************************************************************************************
 *arch/csky/src/common/up_checkstack.c
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

#include <sys/types.h>
#include <stdint.h>
#include <sched.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>

#include "sched/sched.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
#define THIS_MODULE MODULE_ARCH

#ifdef CONFIG_STACK_COLORATION

/****************************************************************************
 * Public Data
 ****************************************************************************/

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/
static size_t do_stackcheck(uintptr_t alloc, size_t size);
static int do_stackcheck_overflow(uintptr_t alloc, size_t size);

/****************************************************************************
 * Name: do_stackcheck_overflow
 *
 * Description:
 *   Check the stack is overflow
 *
 * Input Parameters:
 *   alloc - Allocation base address of the stack
 *   size - The size of the stack in bytes
 *
 * Returned value:
 *   -1: overflow
 *   0 :  OK
 *
 ****************************************************************************/

static int do_stackcheck_overflow(uintptr_t alloc, size_t size)
{
    FAR uint32_t *ptr;

    /* Get aligned addresses and adjusted sizes */

    ptr  = (uint32_t *)(alloc & ~3);

    if (ptr && *ptr != STACK_COLOR) {
        return -1;
    } else {
        return OK;
    }
}

/****************************************************************************
 * Name: do_stackcheck
 *
 * Description:
 *   Determine (approximately) how much stack has been used be searching the
 *   stack memory for a high water mark.  That is, the deepest level of the
 *   stack that clobbered some recognizable marker in the stack memory.
 *
 * Input Parameters:
 *   alloc - Allocation base address of the stack
 *   size - The size of the stack in bytes
 *
 * Returned value:
 *   The estimated amount of stack space used.
 *
 ****************************************************************************/

static size_t do_stackcheck(uintptr_t alloc, size_t size)
{
    FAR uintptr_t start;
    FAR uintptr_t end;
    FAR uint32_t *ptr;
    size_t mark;

    /* Get aligned addresses and adjusted sizes */

    start  = alloc & ~3;
    end    = (alloc + size + 3) & ~3;
    size   = end - start;

  /* The CSKY uses a push-down stack:  the stack grows toward lower addresses
   * in memory.  We need to start at the lowest address in the stack memory
   * allocation and search to higher addresses.  The first word we encounter
   * that does not have the magic value is the high water mark.
   */

    for (ptr = (FAR uint32_t *)start, mark = (size >> 2);
         *ptr == STACK_COLOR && mark > 0;
         ptr++, mark--);

    /* If the stack is completely used, then this might mean that the stack
     * overflowed from above (meaning that the stack is too small), or may
     * have been overwritten from below meaning that some other stack or data
     * structure overflowed.
     *
     * If you see returned values saying that the entire stack is being used
     * then enable the following logic to see it there are unused areas in the
     * middle of the stack.
     */

#if 0
    if (mark + 16 > nwords) {
        int i, j;

        ptr = (FAR uint32_t *)start;
        for (i = 0; i < size; i += 4*64) {
            for (j = 0; j < 64; j++) {
                int ch;
                if (*ptr++ == STACK_COLOR) {
                    ch = '.';
                } else {
                    ch = 'X';
                }

                up_putc(ch);
            }

            up_putc('\n');
        }
    }
#endif

    /* Return our guess about how much stack space was used */

    return mark << 2;
}

/****************************************************************************
 * Name: do_check_tasklist_stack_remain
 *
 * Description:
 *   Check remaining of task stack in tasklists
 *
 * Input Parameters:
 *   tcb: First tcb in tasklists
 *
 * Returned value:
 *   None
 *
 ****************************************************************************/

static void do_check_tasklist_stack_remain(struct tcb_s *tcb)
{
    while(1) {
        if (!tcb || (tcb && tcb->pid == 0)) {
            break;
        }
        sinfo("pid: %d\t, entry: 0x%x, stack remain: %d bytes\n",
               tcb->pid, tcb->entry, up_check_tcbstack_remain(tcb));
        tcb = (struct tcb_s *)tcb->flink;
    }
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_check_stack and friends
 *
 * Description:
 *   Determine (approximately) how much stack has been used be searching the
 *   stack memory for a high water mark.  That is, the deepest level of the
 *   stack that clobbered some recognizable marker in the stack memory.
 *
 * Input Parameters:
 *   None
 *
 * Returned value:
 *   The estimated amount of stack space used.
 *
 ****************************************************************************/

int up_check_tcbstack_overflow(FAR struct tcb_s *tcb)
{
    return do_stackcheck_overflow((uintptr_t)tcb->stack_alloc_ptr, tcb->adj_stack_size);
}

size_t up_check_tcbstack(FAR struct tcb_s *tcb)
{
    return do_stackcheck((uintptr_t)tcb->stack_alloc_ptr, tcb->adj_stack_size);
}

ssize_t up_check_tcbstack_remain(FAR struct tcb_s *tcb)
{
    return (ssize_t)tcb->adj_stack_size - (ssize_t)up_check_tcbstack(tcb);
}

size_t up_check_stack(void)
{
    return up_check_tcbstack((FAR struct tcb_s *)g_readytorun.head);
}

int up_check_stack_overflow(void)
{
    return up_check_tcbstack_overflow((FAR struct tcb_s *)g_readytorun.head);
}

ssize_t up_check_stack_remain(void)
{
    return up_check_tcbstack_remain((FAR struct tcb_s *)g_readytorun.head);
}

#if CONFIG_ARCH_INTERRUPTSTACK > 3
size_t up_check_intstack(void)
{
    return do_stackcheck((uintptr_t)&g_intstackalloc, (CONFIG_ARCH_INTERRUPTSTACK & ~3));
}

size_t up_check_intstack_remain(void)
{
    return (CONFIG_ARCH_INTERRUPTSTACK & ~3) - up_check_intstack();
}
#endif

void up_check_alltask_stack_remain(void)
{
    sinfo("\n");
    sinfo("Remaining of all threads stack:\n");

    do_check_tasklist_stack_remain((struct tcb_s *)g_readytorun.head);
    do_check_tasklist_stack_remain((struct tcb_s *)g_pendingtasks.head);
    do_check_tasklist_stack_remain((struct tcb_s *)g_waitingforsemaphore.head);

#ifndef CONFIG_DISABLE_SIGNALS
    do_check_tasklist_stack_remain((struct tcb_s *)g_waitingforsignal.head);
#endif

#ifndef CONFIG_DISABLE_MQUEUE
    do_check_tasklist_stack_remain((struct tcb_s *)g_waitingformqnotempty.head);
#endif

#ifndef CONFIG_DISABLE_MQUEUE
    do_check_tasklist_stack_remain((struct tcb_s *)g_waitingformqnotfull.head);
#endif

#ifdef CONFIG_PAGING
    do_check_tasklist_stack_remain((struct tcb_s *)g_waitingforfill.head);
#endif

    do_check_tasklist_stack_remain((struct tcb_s *)g_inactivetasks.head);

    sinfo("\n");
}

#endif /* CONFIG_STACK_COLORATION */
