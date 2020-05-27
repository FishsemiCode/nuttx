/****************************************************************************
 * arch/csky/src/song/csky_head.S
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

	.section	.vectors, "aw"
	.align		10
	.globl		__Vectors
	.type		__Vectors, @object
__Vectors:
	.long		Reset_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		Default_Handler
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.long		NOVIC_IRQ_Default_Handler    /*  default interrupt entry  */
	.size		__Vectors, . - __Vectors

	.text
	.globl		__start
	.type		__start, %function
__start:
Reset_Handler:
	lrw		r0, 0x80000200
	mtcr		r0, psr

	/* Initialize the normal stack pointer from the linker definition. */

	lrw		r0, _ebss
	addi		sp, r0, CONFIG_IDLETHREAD_STACKSIZE

#ifdef CONFIG_STACK_COLORATION
	/* Set the IDLE stack to the stack coloration value then jump to
	 * up_start().  We take extreme care here because were currently
	 * executing on this stack.
	 */

	lrw		r2, STACK_COLOR
1:
	stw		r2, (r0)
	addi		r0, 4
	cmpne		r0, sp
	bt		1b
#endif

	jbsr		up_start

	.size		Reset_Handler, . - Reset_Handler

	.section	.bss
	.align		2
	.globl		g_intstackalloc
	.global		g_intstackbase
	.global		g_top_irqstack
g_intstackalloc:
g_intstackbase:
	.space		CONFIG_ARCH_INTERRUPTSTACK
g_top_irqstack:
