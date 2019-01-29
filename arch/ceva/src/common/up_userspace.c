/****************************************************************************
 * arch/ceva/src/common/up_userspace.c
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

#include <nuttx/arch.h>
#include <nuttx/init.h>
#include <nuttx/mm/mm.h>
#include <nuttx/wqueue.h>
#include <nuttx/userspace.h>

#include "up_internal.h"

#ifdef CONFIG_BUILD_PROTECTED

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_ARCH_USER_DEFAULT_HEAP == 0
#  define MM_HEAP1      &g_mmheap
#else
static struct mm_heap_s g_mmheap1;
#  define MM_HEAP1      &g_mmheap1
#endif

#if CONFIG_ARCH_NR_USER_MEMORY >= 2
#  if CONFIG_ARCH_USER_DEFAULT_HEAP == 1
#    define MM_HEAP2    &g_mmheap
#  else
static struct mm_heap_s g_mmheap2;
#    define MM_HEAP2    &g_mmheap2
#  endif
#  define _START_TEXT2  ((const void *)&_stext2)
#  define _END_TEXT2    ((const void *)&_etext2)
#  define _START_BSS2   ((void *)&_sbss2)
#  define _END_BSS2     ((void *)&_ebss2)
#  define _DATA_INIT2   ((const void *)&_eronly2)
#  define _START_DATA2  ((void *)&_sdata2)
#  define _END_DATA2    ((void *)&_edata2)
#  define _END_HEAP2    ((void *)&_eheap2)
#else
#  define MM_HEAP2      NULL
#  define _START_TEXT2  NULL
#  define _END_TEXT2    NULL
#  define _START_BSS2   NULL
#  define _END_BSS2     NULL
#  define _DATA_INIT2   NULL
#  define _START_DATA2  NULL
#  define _END_DATA2    NULL
#  define _END_HEAP2    NULL
#endif

#if CONFIG_ARCH_NR_USER_MEMORY >= 3
#  if CONFIG_ARCH_USER_DEFAULT_HEAP == 2
#    define MM_HEAP3    &g_mmheap
#  else
static struct mm_heap_s g_mmheap3;
#    define MM_HEAP3    &g_mmheap3
#  endif
#  define _START_TEXT3  ((const void *)&_stext3)
#  define _END_TEXT3    ((const void *)&_etext3)
#  define _START_BSS3   ((void *)&_sbss3)
#  define _END_BSS3     ((void *)&_ebss3)
#  define _DATA_INIT3   ((const void *)&_eronly3)
#  define _START_DATA3  ((void *)&_sdata3)
#  define _END_DATA3    ((void *)&_edata3)
#  define _END_HEAP3    ((void *)&_eheap3)
#else
#  define MM_HEAP3      NULL
#  define _START_TEXT3  NULL
#  define _END_TEXT3    NULL
#  define _START_BSS3   NULL
#  define _END_BSS3     NULL
#  define _DATA_INIT3   NULL
#  define _START_DATA3  NULL
#  define _END_DATA3    NULL
#  define _END_HEAP3    NULL
#endif

#if CONFIG_ARCH_NR_USER_MEMORY >= 4
#  if CONFIG_ARCH_USER_DEFAULT_HEAP == 3
#    define MM_HEAP4    &g_mmheap
#  else
static struct mm_heap_s g_mmheap4;
#    define MM_HEAP4    &g_mmheap4
#  endif
#  define _START_TEXT4  ((const void *)&_stext4)
#  define _END_TEXT4    ((const void *)&_etext4)
#  define _START_BSS4   ((void *)&_sbss4)
#  define _END_BSS4     ((void *)&_ebss4)
#  define _DATA_INIT4   ((const void *)&_eronly4)
#  define _START_DATA4  ((void *)&_sdata4)
#  define _END_DATA4    ((void *)&_edata4)
#  define _END_HEAP4    ((void *)&_eheap4)
#else
#  define MM_HEAP4      NULL
#  define _START_TEXT4  NULL
#  define _END_TEXT4    NULL
#  define _START_BSS4   NULL
#  define _END_BSS4     NULL
#  define _DATA_INIT4   NULL
#  define _START_DATA4  NULL
#  define _END_DATA4    NULL
#  define _END_HEAP4    NULL
#endif

#if CONFIG_ARCH_NR_USER_MEMORY >= 5
#  error CONFIG_ARCH_NR_USER_MEMORY must between 1 to 4
#endif

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const void *const g_textstart[] =
{
  _START_TEXT, _START_TEXT2, _START_TEXT3, _START_TEXT4, _END_MEM,
};

static const void *const g_textend[] =
{
  _END_TEXT, _END_TEXT2, _END_TEXT3, _END_TEXT4, _END_MEM,
};

static const void *const g_datasource[] =
{
  _DATA_INIT, _DATA_INIT2, _DATA_INIT3, _DATA_INIT4, _END_MEM,
};

static void *const g_datastart[] =
{
  _START_DATA, _START_DATA2, _START_DATA3, _START_DATA4, _END_MEM,
};

static void *const g_dataend[] =
{
  _END_DATA, _END_DATA2, _END_DATA3, _END_DATA4, _END_MEM,
};

static void *const g_bssstart[] =
{
  _START_BSS, _START_BSS2, _START_BSS3, _START_BSS4, _END_MEM,
};

static void *const g_bssend[] =
{
  _END_BSS, _END_BSS2, _END_BSS3, _END_BSS4, _END_MEM,
};

static void *const g_heapend[] =
{
  _END_HEAP, _END_HEAP2, _END_HEAP3, _END_HEAP4, _END_MEM,
};

/****************************************************************************
 * Public Data
 ****************************************************************************/

struct mm_heap_s *const g_mm_heap[] =
{
  MM_HEAP1, MM_HEAP2, MM_HEAP3, MM_HEAP4, NULL,
};

const struct userspace_s userspace __attribute__ ((section ("userspace"))) =
{
  /* General memory map */
  .us_base            =
  {
    .us_entrypoint    = CONFIG_USER_ENTRYPOINT,
    .us_textstart     = (uintptr_t)g_textstart,
    .us_textend       = (uintptr_t)g_textend,
    .us_datasource    = (uintptr_t)g_datasource,
    .us_datastart     = (uintptr_t)g_datastart,
    .us_dataend       = (uintptr_t)g_dataend,
    .us_bssstart      = (uintptr_t)g_bssstart,
    .us_bssend        = (uintptr_t)g_bssend,
    .us_heapend       = (uintptr_t)g_heapend,

    /* Memory manager heap structure */

    .us_heap          = (struct mm_heap_s *)g_mm_heap,

    /* Task/thread startup routines */

    .task_startup     = task_startup,
#ifndef CONFIG_DISABLE_PTHREAD
    .pthread_startup  = pthread_startup,
#endif

    /* Signal handler trampoline */

#ifndef CONFIG_DISABLE_SIGNALS
    .signal_handler   = up_signal_handler,
#endif

    /* User-space work queue support (declared in include/nuttx/wqueue.h) */

#ifdef CONFIG_LIB_USRWORK
    .work_usrstart    = work_usrstart,
#endif
  },
};

#endif /* CONFIG_BUILD_PROTECTED */
