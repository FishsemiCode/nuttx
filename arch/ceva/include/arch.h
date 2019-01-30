/****************************************************************************
 * arch/ceva/include/arch.h
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

/* This file should never be included directed but, rather,
 * only indirectly through nuttx/arch.h
 */

#ifndef __ARCH_CEVA_INCLUDE_ARCH_H
#define __ARCH_CEVA_INCLUDE_ARCH_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/chip/chip.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK == 0
/* The interrupt stack is required for CEVA porting */

#  undef CONFIG_ARCH_INTERRUPTSTACK
#  define CONFIG_ARCH_INTERRUPTSTACK CONFIG_IDLETHREAD_STACKSIZE
#endif

#if defined(CONFIG_ARCH_ADDRENV) && defined(CONFIG_BUILD_KERNEL)
/* In the kernel build, there a multiple user heaps; one for each task
 * group.  In this build configuration, the user heap structure lies
 * in a reserved region at the beginning of the .bss/.data address
 * space (CONFIG_ARCH_DATA_VBASE).  The size of that region is given by
 * ARCH_DATA_RESERVE_SIZE
 */

#  define UMM_HEAP(i) ((i) ? NULL : &ARCH_DATA_RESERVE->ar_usrheap)

#elif defined(CONFIG_BUILD_PROTECTED) && defined(__KERNEL__)
/* In the protected mode, there are two heaps:  A kernel heap and a single
 * user heap.  Kernel code must obtain the address of the user heap data
 * structure from the userspace interface.
 */

#  define UMM_HEAP(i) ((struct mm_heap_s *const *)USERSPACE->us_heap)[i]

#else
/* Otherwise, the user heap data structures are in common .bss */

#  define UMM_HEAP(i) g_mm_heap[i]
#endif

#ifdef CONFIG_MM_KERNEL_HEAP
/* Kernel has the dedicated heap data structures */

#  define KMM_HEAP(i) g_mm_heap[i]
#else
/* Otherwise, kernel allocate the memory from the user heap data structures */

#  define KMM_HEAP(i) UMM_HEAP(i)
#endif

#ifdef __KERNEL__
#  define MM_HEAP(i) KMM_HEAP(i)
#else
#  define MM_HEAP(i) UMM_HEAP(i)
#endif

#define PM_IDLE_DOMAIN                0

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

EXTERN struct mm_heap_s *const g_mm_heap[];

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ARCH_CEVA_INCLUDE_ARCH_H */
