/****************************************************************************
 * arch/ceva/src/tl4/up_head.S
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

.MACRO	IRQ_HANDLER	irq retx
	pushdw		retreg
	pushdw		r0
	mov		#irq, r0
	16.callr	#exception_common
	popdw		retreg
	retx
.ENDM

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file		"up_head.S"
	.extern		_g_idle_topstack
	.extern		_up_doirq
	.extern		_up_start

/****************************************************************************
 * Interrupt Functions
 ****************************************************************************/

	.CSECT		inttbl
	br		reset_handler

	.ORG		0x02
	br		trape_handler

	.ORG		0x04
	br		nmi_handler

	.ORG		0x06
	IRQ_HANDLER	IRQ_INT0, reti

	.ORG		0x0e
	IRQ_HANDLER	IRQ_INT1, reti

	.ORG		0x16
	IRQ_HANDLER	IRQ_INT2, reti

	.ORG		0x20
	IRQ_HANDLER	IRQ_INT3, reti

	.ORG		0x30
	.GLOBAL		_up_vintc_handler
_up_vintc_handler:
	IRQ_HANDLER	IRQ_VINT, reti

	.ORG		0x40
	pushdw		crpc
	IRQ_HANDLER	IRQ_CRCALL, popdw pc

	.ORG		0x50
	IRQ_HANDLER	IRQ_TRAP0, reti

	.ORG		0x60
	IRQ_HANDLER	IRQ_TRAP1, reti

	.ORG		0x70
	IRQ_HANDLER	IRQ_TRAP2, reti

	.ORG		0x80
	IRQ_HANDLER	IRQ_TRAP3, reti

reset_handler:
	/* Initialize the C language environment */

	mov		#REG_MOD2_DEFAULT, r0
	mov		r0, mod2
	ld		[_g_idle_topstack].dw, sp

	brf		_up_start

trape_handler:
	IRQ_HANDLER	IRQ_TRAPE, retb

nmi_handler:
	IRQ_HANDLER	IRQ_NMI, retn

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/* Common exception handling logic, need sync with:
 * arch/ceva/include/tl4/reg_tl420.h
 * arch/ceva/include/tl4/reg_tl421.h
 */

	.func_start	3 exception_common

exception_common:

	/* Note: r0 contain exception number
	 * Complete the context save
	 */

	pushdw		r1
	pushdw		r2
	pushdw		r3
	pushdw		r4
	pushdw		r5
	pushdw		r6
	pushdw		r7
	pushdw		a0e
	pushdw		a0
	pushdw		a1e
	pushdw		a1
	pushdw		b0e
	pushdw		b0
	pushdw		b1e
	pushdw		b1
	pushw		sv
	pushw		sk
	pushdw		y
	pushdw		p0e
	pushdw		p0
	pushdw		p1e
	pushdw		p1
	pushw		lc
	pushw		streamptr
	pushdw		mixp
	pushdw		mcfgi
	pushdw		mcfgj
	bkstfia
	bkstlc
	bkstfia
	bkstlc
	bkstfia
	bkstlc
	bkstfia
	bkstlc
	pushdw		sts0
	pushdw		mod1
	pushdw		mod2
	mov		x, y
	pushdw		y
	pushdw		streambuff0
	pushdw		streambuff1
	pushdw		tstreambuff

.IF CONFIG_ARCH_TL421
	pushdw		c0e
	pushdw		c0
	pushdw		c1e
	pushdw		c1
	pushdw		p2e
	pushdw		p2
	pushdw		p3e
	pushdw		p3
	pushdw		f0
	pushdw		f1
	pushdw		f2
	pushdw		f3
	pushdw		modg
.ENDIF

	add		#-0x02, sp
	nop
	mov		sp, r1
	st		r1, (r1).dw

	/* Prepare the C language environment */

	mov		#REG_MOD2_DEFAULT, r1
	mov		r1, mod2
	mov		#0x00, r1
	mov {u}		r1, sts0

	/* There are two arguments to up_doirq:
	 *
	 *   a0 = The IRQ number
	 *   a1 = The top of the stack points to the saved state
	 */

	mov		r0, a0
	mov		sp, a1

	/* Switch to the dedicated stack and go */

	mov		#_g_intstackbase, sp
	nop
	pushdw		retreg
	callr		#_up_doirq
	popdw		retreg

	/* On return from up_doirq, a0 will hold a pointer to register context
	 * array to use for the interrupt return.
	 */

	mov		a0, sp
	nop		0x02
	add		#0x02, sp
	nop

	/* Unwind the same stack frame that we created at entry */

.IF CONFIG_ARCH_TL421
	popdw		modg
	popdw		f3
	popdw		f2
	popdw		f1
	popdw		f0
	popab		p3
	popab		p2
	popab		c1
	popab		c0
.ENDIF

	popdw		tstreambuff
	popdw		streambuff1
	popdw		streambuff0
	popdw		y
	mov		y, x
	popdw		mod2
	popdw		mod1
	popdw		sts0
	bkrest
	bkrest
	bkrest
	bkrest
	popdw		mcfgj
	popdw		mcfgi
	popdw		mixp
	popw		streamptr
	popw		lc
	popab		p1
	popab		p0
	popdw		y
	popw		sk
	popw		sv
	popab		b1
	popab		b0
	popab		a1
	popab		a0
	popdw		r7
	popdw		r6
	popdw		r5
	popdw		r4
	popdw		r3
	popdw		r2
	popdw		r1
	popdw		r0
	ret

	.func_end	3 exception_common

	.bss
	.public		_g_intstackalloc
	.public		_g_intstackbase
_g_intstackalloc:
	dd		CONFIG_ARCH_INTERRUPTSTACK/4 dup ?
_g_intstackbase:
