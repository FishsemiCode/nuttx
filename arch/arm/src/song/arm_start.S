/****************************************************************************
 * arch/arm/src/song/arm_start.S
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
	.file		"arm_start.S"
	.globl		up_start
	.globl		_ebss
	.globl		__aeabi_d2iz
	.globl		__aeabi_d2uiz
	.globl		__aeabi_ddiv
	.globl		__aeabi_dmul
	.globl		__aeabi_ldivmod
	.globl		__aeabi_ui2d
	.globl		__aeabi_uldivmod

/****************************************************************************
 * Name: __start
 ****************************************************************************/

	.text
	.thumb_func
	.globl		__start
	.type		__start, function

__start:
	bl		up_earlystart
	sub		sp, 0x100	/* top bytes are reserved for up_earlystart() */

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

/****************************************************************************
 *   GCC in LTO mode interoperates poorly with non-standard libraries that
 *   provide implementations of compiler intrinsics such as memcpy/memset
 *   or the stack protector entry points.
 *
 *   By referencing these functions from a non-LTO object that can be passed
 *   to the linker via the -plugin-opt=-pass-through=-lxxx options, the
 *   intrinsics are included in the link in a way that allows them to be
 *   pruned again if no other references to them exist.
 ****************************************************************************/

	bl		__aeabi_d2iz
	bl		__aeabi_d2uiz
	bl		__aeabi_ddiv
	bl		__aeabi_dmul
	bl		__aeabi_ldivmod
	bl		__aeabi_ui2d
	bl		__aeabi_uldivmod

	.size		__start, .-__start

/****************************************************************************
 * Name: up_earlystart
 ****************************************************************************/

	.text
	.thumb_func
	.weak		up_earlystart
	.type		up_earlystart, function

up_earlystart:
	bx		lr

	.size		up_earlystart, .-up_earlystart
	.end
