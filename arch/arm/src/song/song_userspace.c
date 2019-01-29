/****************************************************************************
 * arch/arm/src/song/song_userspace.c
 *
 *   Copyright (C) 2017 Pinecone Inc. All rights reserved.
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

#if defined(CONFIG_BUILD_PROTECTED) && !defined(__KERNEL__)

/****************************************************************************
 * Public Data
 ****************************************************************************/

extern uint32_t _eheap;

const struct userspace_s userspace __attribute__ ((section(".userspace"))) =
{
  /* General memory map */
  .us_base            =
  {
    .us_entrypoint    = CONFIG_USER_ENTRYPOINT,
    .us_textstart     = (uintptr_t)_START_TEXT,
    .us_textend       = (uintptr_t)_END_TEXT,
    .us_datasource    = (uintptr_t)_DATA_INIT,
    .us_datastart     = (uintptr_t)_START_DATA,
    .us_dataend       = (uintptr_t)_END_DATA,
    .us_bssstart      = (uintptr_t)_START_BSS,
    .us_bssend        = (uintptr_t)_END_BSS,
    .us_heapend       = (uintptr_t)(&_eheap),

    /* Memory manager heap structure */

    .us_heap          = &g_mmheap,

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

#endif /* CONFIG_BUILD_PROTECTED && !__KERNEL__ */
