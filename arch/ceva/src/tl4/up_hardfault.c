/****************************************************************************
 * arch/ceva/src/tl4/up_hardfault.c
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
#include <debug.h>

#include "cpm.h"
#include "up_internal.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define ERRPC                                   0x0330
#define DPCON                                   0x03b8

#ifdef CONFIG_DEBUG_HARDFAULT
# define hfalert(format, ...)  _alert(format, ##__VA_ARGS__)
#else
# define hfalert(x...)
#endif

#define hfdumpreg(reg)                      \
  hfalert("%s: %08x", #reg, getcpm(reg))

#define hfdumpreg1(reg1)                    \
  hfdumpreg(reg1); hfalert("\n")

#define hfdumpreg2(reg1, reg2)              \
  hfdumpreg(reg1); hfalert(" ");            \
  hfdumpreg(reg2); hfalert("\n")

#define hfdumpreg3(reg1, reg2, reg3)        \
  hfdumpreg(reg1); hfalert(" ");            \
  hfdumpreg(reg2); hfalert(" ");            \
  hfdumpreg(reg3); hfalert("\n")

#define hfdumpreg4(reg1, reg2, reg3, reg4)  \
  hfdumpreg(reg1); hfalert(" ");            \
  hfdumpreg(reg2); hfalert(" ");            \
  hfdumpreg(reg3); hfalert(" ");            \
  hfdumpreg(reg4); hfalert("\n")

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_hardfault
 *
 * Description:
 *   This is Hard Fault exception handler.
 *
 ****************************************************************************/

int up_hardfault(int irq, FAR void *context, FAR void *arg)
{
  /* Dump some hard fault info */

  hfalert("Hard Fault:\n");
  hfdumpreg2(ERRPC, DPCON);

  PANIC();
  return OK;
}
