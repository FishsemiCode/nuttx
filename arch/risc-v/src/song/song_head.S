/****************************************************************************
 * arch/risc-v/src/song/song_head.S
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

	.file		"song_head.S"
	.globl		up_start
	.globl		_ebss

/****************************************************************************
 * Name: __start
 ****************************************************************************/

	.text
	.globl		__start
	.type		__start, function

__start:
	/* Set mie */

	csrw		mie, zero

	/* Set mtvec */

	lui		t0, %hi(_vectors)
	addi		t0, t0, %lo(_vectors)
	csrw		mtvec, t0

	/* Set stack pointer to earlystack if support warmreset */

#if CONFIG_EARLY_STACKSIZE > 0
	/* Use a dedicated stack for early process to avoid
	 * corrupt the idle stack during restoring from the
	 * partial powerdown state.
	 */

	la		sp, g_early_stack
	call		up_earlystart
#endif

	/* Set stack pointer */

	la		sp, _ebss + CONFIG_IDLETHREAD_STACKSIZE

#ifdef CONFIG_STACK_COLORATION
	/* Set the IDLE stack to the stack coloration value then jump to
	 * up_start().  We take extreme care here because were currently
	 * executing on this stack.
	 */
	la		t0, _ebss
	la		t1, _ebss + CONFIG_IDLETHREAD_STACKSIZE - 4
	la		t3, STACK_COLOR
1:
	sw		t3, 0(t0)
	addi		t0, t0, 4
	bltu		t0, t1, 1b
#endif

	/* Now jump to the up_start for future initialization */

	call		up_start

	.size		__start, .-__start

/****************************************************************************
 * Name: irq_handler
 ****************************************************************************/

	.globl		irq_handler
	.type		irq_handler, function

irq_handler:
	addi		x2, x2, -33*4
	sw		x1, 1*4(x2)
	addi		x1, x2, 33*4
	sw		x1, 2*4(x2)
	sw		x3, 3*4(x2)
	sw		x4, 4*4(x2)
	sw		x5, 5*4(x2)
	sw		x6, 6*4(x2)
	sw		x7, 7*4(x2)
	sw		x8, 8*4(x2)
	sw		x9, 9*4(x2)
	sw		x10, 10*4(x2)
	sw		x11, 11*4(x2)
	sw		x12, 12*4(x2)
	sw		x13, 13*4(x2)
	sw		x14, 14*4(x2)
	sw		x15, 15*4(x2)
	sw		x16, 16*4(x2)
	sw		x17, 17*4(x2)
	sw		x18, 18*4(x2)
	sw		x19, 19*4(x2)
	sw		x20, 20*4(x2)
	sw		x21, 21*4(x2)
	sw		x22, 22*4(x2)
	sw		x23, 23*4(x2)
	sw		x24, 24*4(x2)
	sw		x25, 25*4(x2)
	sw		x26, 26*4(x2)
	sw		x27, 27*4(x2)
	sw		x28, 28*4(x2)
	sw		x29, 29*4(x2)
	sw		x30, 30*4(x2)
	sw		x31, 31*4(x2)
	csrr		x10, mepc
	sw		x10, 0*4(x2)
	csrr		x10, mstatus
	sw		x10, 32*4(x2)
	mv		x10, x2
	call		up_dispatch_all
	mv		x1, x10
	lw		x10, 32*4(x1)
	csrw		mstatus, x10
	lw		x10, 0*4(x1)
	csrw		mepc, x10
	lw		x31, 31*4(x1)
	lw		x30, 30*4(x1)
	lw		x29, 29*4(x1)
	lw		x28, 28*4(x1)
	lw		x27, 27*4(x1)
	lw		x26, 26*4(x1)
	lw		x25, 25*4(x1)
	lw		x24, 24*4(x1)
	lw		x23, 23*4(x1)
	lw		x22, 22*4(x1)
	lw		x21, 21*4(x1)
	lw		x20, 20*4(x1)
	lw		x19, 19*4(x1)
	lw		x18, 18*4(x1)
	lw		x17, 17*4(x1)
	lw		x16, 16*4(x1)
	lw		x15, 15*4(x1)
	lw		x14, 14*4(x1)
	lw		x13, 13*4(x1)
	lw		x12, 12*4(x1)
	lw		x11, 11*4(x1)
	lw		x10, 10*4(x1)
	lw		x9, 9*4(x1)
	lw		x8, 8*4(x1)
	lw		x7, 7*4(x1)
	lw		x6, 6*4(x1)
	lw		x5, 5*4(x1)
	lw		x4, 4*4(x1)
	lw		x3, 3*4(x1)
	lw		x2, 2*4(x1)
	lw		x1, 1*4(x1)
	mret

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
