/****************************************************************************
 * arch/arm/src/song/arm_mpu.c
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

#include <assert.h>

#include <nuttx/userspace.h>

#include "arm_mpu.h"
#include "mpu.h"
#include "up_internal.h"

#ifdef CONFIG_ARM_MPU

/****************************************************************************
 * Public Functions
 ****************************************************************************/

void up_mpuinitialize(void)
{
  /* Show MPU information */

  mpu_showtype();

  /* Configure kernel flash and SRAM space */

  DEBUGASSERT((uintptr_t)_END_TEXT >= (uintptr_t)_START_TEXT);
  mpu_priv_flash((uintptr_t)_START_TEXT,
                 (uintptr_t)_END_TEXT - (uintptr_t)_START_TEXT);

#ifdef CONFIG_ARCH_RAMFUNCS
  DEBUGASSERT((uintptr_t)&_eramfuncs >= (uintptr_t)&_sramfuncs);
  mpu_priv_flash((uintptr_t)&_sramfuncs,
                 (uintptr_t)&_eramfuncs - (uintptr_t)&_sramfuncs);
#endif

  DEBUGASSERT((uintptr_t)_END_BSS >= (uintptr_t)_START_DATA);
  mpu_priv_intsram((uintptr_t)_START_DATA, (uintptr_t)_END_BSS +
                    CONFIG_IDLETHREAD_STACKSIZE - (uintptr_t)_START_DATA);

#ifdef CONFIG_BUILD_PROTECTED
  /* Configure user flash and SRAM space */

  DEBUGASSERT(USERSPACE->us_textend >= USERSPACE->us_textstart);
  mpu_user_flash(USERSPACE->us_textstart,
                 USERSPACE->us_textend - USERSPACE->us_textstart);

  DEBUGASSERT(USERSPACE->us_bssend >= USERSPACE->us_datastart);
  mpu_user_intsram(USERSPACE->us_datastart,
                   USERSPACE->us_bssend - USERSPACE->us_datastart);
#endif

  /* Then enable the MPU */

  mpu_control(true, false, true);
}

void up_mpu_user_heap(uintptr_t start, size_t size)
{
  mpu_user_intsram(start, size);
}

void up_mpu_priv_heap(uintptr_t start, size_t size)
{
  mpu_priv_intsram(start, size);
}
#endif
