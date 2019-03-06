/****************************************************************************
 * arch/ceva/src/xm6/up_head.S
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
#include <nuttx/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#if CONFIG_ARCH_INTERRUPTSTACK == 0
#  undef CONFIG_ARCH_INTERRUPTSTACK
#  define CONFIG_ARCH_INTERRUPTSTACK CONFIG_IDLETHREAD_STACKSIZE
#endif

	.EQU		DBG_STACK_START 0xd44
	.EQU		DBG_STACK_END   0xd48

.IF CONFIG_ARCH_XM6_BUG001
	.EQU		prx pr14
.ELSE
	.EQU		prx pr15
.ENDIF

.MACRO	IRQ_HANDLER	irq retx
	push		{auxreg10}
	push		{auxreg2}
	mov		#irq, r0.ui
	callr		#exception_common, ?prx.b
	pop		{auxreg10}
	nop		#0x04
	nop		#0x02
	retx
.ENDM

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file		"up_head.S"
	.extern		_g_idle_topstack
	.extern		_g_idle_basestack
	.extern		_up_doirq
	.extern		_up_start

/****************************************************************************
 * Interrupt Functions
 ****************************************************************************/

	.CSECT		inttbl
.IF CONFIG_ARCH_XM6_BUG001
	cmp {eq}	r0.ui, r0.ui, prx.b2, pr15.b2
	nop
.ENDIF
	br		#reset_handler, #0x00, #0x00, ?prx.b

	.ORG		0x20
	IRQ_HANDLER	IRQ_TRAPE, retb

	.ORG		0x40
	IRQ_HANDLER	IRQ_TRAP, reti

	.ORG		0x60
	IRQ_HANDLER	IRQ_NMI, retn

	.ORG		0x80
	IRQ_HANDLER	IRQ_INT0, reti

	.ORG		0xc0
	IRQ_HANDLER	IRQ_INT1, reti

	.ORG		0x100
	IRQ_HANDLER	IRQ_INT2, reti

	.ORG		0x140
	IRQ_HANDLER	IRQ_INT3, reti

	.ORG		0x180
	IRQ_HANDLER	IRQ_INT4, reti

	.ORG		0x1c0
	.GLOBAL		_up_vintc_handler
_up_vintc_handler:
	IRQ_HANDLER	IRQ_VINT, reti

	.ORG		0x200
	IRQ_HANDLER	IRQ_TRAP0, reti

	.ORG		0x240
	IRQ_HANDLER	IRQ_TRAP1, reti

	.ORG		0x280
	IRQ_HANDLER	IRQ_TRAP2, reti

	.ORG		0x2c0
	IRQ_HANDLER	IRQ_TRAP3, reti

reset_handler:
	/* Tricky for compiler bug: when codes are placed onto ddr memory, label
	* address will be place at the back of its first code line, so when branch
	* to this label, its first code line will be skipped.
	*/
	nop
	nop
	nop
	nop

	/* Initialize the C language environment */
	ld		(#_g_idle_topstack).ui, r0.ui
	ld		(#_g_idle_basestack).ui, r1.ui
	nop		#0x01
	mov		r0.ui, sp.ui

#ifdef CONFIG_ARCH_XM6_STACKCHECK
	/* Setup the stack violation exception registers */

	out {cpm}	r0.ui, (#DBG_STACK_START).ui
	out {cpm}	r1.ui, (#DBG_STACK_END).ui
#endif

	br		#_up_start, #0x00, #0x00, ?prx.b

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Common exception handling logic, need sync with:
 * arch/ceva/include/xm6/reg.h
 */

	.func_start	3 exception_common

exception_common:
	/* Tricky for compiler bug: when codes are placed onto ddr memory, label address
	* will be placed at the back of its first code line, so when branch to
	* to this label, its first code line will be skipped.
	*/
	nop
	nop
	nop
	nop

	/* Note: r0 contain exception number
	 * Complete the context save
	 */

	push		{auxreg3}
	push		{auxreg4}
	push		{auxreg5}
	push		{auxreg6}
	push		{auxreg7}
	push		{auxreg8}
	push		{auxreg9}

	push		modc.ui
	push		modd.ui
	push		mode.ui
	push		modg.ui
	push		modi.ui

#ifdef CONFIG_ARCH_XM6_STACKCHECK
	/* Save the stack violation exception registers */

	in {cpm}	(#DBG_STACK_START).ui, r1.ui
	in {cpm}	(#DBG_STACK_END).ui, r2.ui
	nop		#0x02
	push {in1}	r1.ui
	push {in1}	r2.ui
#endif

	modr		(sp.ui).ui +#-4
	mov		sp.ui, r1.ui
	nop		#0x02
	st		r1.ui, (r1.ui).ui

	/* Prepare the C language environment */

	lbf {sv}	#0x00

	/* There are two arguments to up_doirq:
	 *
	 *   r0 = The IRQ number
	 *   r1 = The top of the stack points to the saved state
	 */

	/* Switch to the dedicated stack */

	mov		#_g_intstackbase, r2.ui
	mov		#_g_intstackalloc, r3.ui
	mov		r2.ui, sp.ui

#ifdef CONFIG_ARCH_XM6_STACKCHECK
	/* Setup the stack violation exception registers */

	out {cpm}	r2.ui, (#DBG_STACK_START).ui
	out {cpm}	r3.ui, (#DBG_STACK_END).ui
#else
	nop		#0x02
#endif
	nop		#0x04

	push		retreg.ui
	callr		#_up_doirq, ?prx.b
	pop		retreg.ui

	/* On return from up_doirq, r0 will hold a pointer to register context
	 * array to use for the interrupt return.
	 */

	modr		(r0.ui).ui +#4

#ifdef CONFIG_ARCH_XM6_STACKCHECK
	/* Restore the stack violation exception registers */

	ld		(r0.ui).ui +#4, r2.ui
	ld		(r0.ui).ui +#4, r1.ui
	nop
	out {cpm}	r2.ui, (#DBG_STACK_END).ui
	out {cpm}	r1.ui, (#DBG_STACK_START).ui
#endif

	/* Restore the stack pointer */

	mov		r0.ui, sp.ui
	nop		#0x04
	nop		#0x02

	/* Unwind the same stack frame that we created at entry */

	pop		modi.ui
	pop		modg.ui
	pop		mode.ui
	pop		modd.ui
	pop		modc.ui
	nop

	pop		{auxreg9}
	pop		{auxreg8}
	pop		{auxreg7}
	pop		{auxreg6}
	pop		{auxreg5}
	pop		{auxreg4}
	pop		{auxreg3}
	pop		{auxreg2}

	ret		?prx.b

	.func_end	3 exception_common

	.bss
	.public		_g_intstackalloc
	.public		_g_intstackbase
_g_intstackalloc:
	DD		CONFIG_ARCH_INTERRUPTSTACK/4 dup ?
_g_intstackbase:
