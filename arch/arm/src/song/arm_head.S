/****************************************************************************
 * arch/arm/src/song/arm_head.S
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

#include "up_internal.h"

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.syntax		unified
	.thumb
	.file		"arm_head.S"
	.globl		up_start
	.globl		_ebss

/****************************************************************************
 * Name: __start
 ****************************************************************************/

	.text
	.thumb_func
	.globl		__start
	.type		__start, function

__start:
#if CONFIG_EARLY_STACKSIZE > 0
	/* Use a dedicated stack for early process to avoid
	 * corrupt the idle stack during restoring from the
	 * partial powerdown state.
	 */

	mov		r4, sp
	ldr		sp, =g_early_stack
	bl		up_earlystart
	mov		sp, r4
#endif

#ifdef CONFIG_STACK_COLORATION
	/* Set the IDLE stack to the stack coloration value then jump to
	 * up_start().  We take extreme care here because were currently
	 * executing on this stack.
	 */

	ldr		r0, =#_ebss
	ldr		r1, =#(CONFIG_IDLETHREAD_STACKSIZE >> 2)
	ldr		r2, =#STACK_COLOR
1:
	sub		r1, r1, #1
	cmp		r1, #0
	str		r2, [r0], #4
	bne		1b
#endif

#ifdef CONFIG_ARMV7M_STACKCHECK
	/* Set the stack limit before we attempt to call any functions */

	sub 		r10, sp, #(CONFIG_IDLETHREAD_STACKSIZE - 64)
#endif
	bl		up_start
	.size		__start, .-__start

#if CONFIG_EARLY_STACKSIZE > 0
/***************************************************************************
 * Name: g_early_stack
 ***************************************************************************/

	.bss
	.local		g_early_stack
	.type		g_early_stack, object

	.align		8
	.space		CONFIG_EARLY_STACKSIZE
g_early_stack:
	.size		g_early_stack, CONFIG_EARLY_STACKSIZE
	.end
#endif
