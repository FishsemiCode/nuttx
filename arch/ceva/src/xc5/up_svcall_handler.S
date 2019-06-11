/****************************************************************************
 * arch/ceva/src/xc5/up_svcall_handler.S
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Bo Zhang <zhangbo@fishsemi.com>
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

#include "svcall.h"

#ifdef CONFIG_LIB_SYSCALL

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file		"up_svcall_handler.S"
	.extern		_g_stublookup

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_svcall_handler
 *
 * Description:
 *   This function is the kernel-space, syscall handler trampoline function.  It
 *   is called from up_svcall() in interrupt handler.
 *
 *   Call the stub function corresponding to the system call.  NOTE the non-
 *   standard parameter passing:
 *
 *     r0 = SYS_ call number
 *     r1 = parm0
 *     r2 = parm1
 *     r3 = parm2
 *     r4 = parm3
 *     r5 = parm4
 *     r6 = parm5
 *
 * Returned Value:
 *   None.  This function does not return in the normal sense.  It returns
 *   via the SYS_syscall_return (see svcall.h)
 *
 ****************************************************************************/

	.text
	.public		_up_svcall_handler
	.func_start	3 _up_svcall_handler

_up_svcall_handler:

	/* Create a stack frame to hold LR */

	push {dw}	retreg

	/* Call the stub function */

	mov		#_g_stublookup, a8
	shift		#0x02, a8
	add		a8, a8
	nop
	nop
	mov		a8, r4
	ld {dw}		(r4), a8
	nop
	nop
	nop
	nop
	nop
	mov		a8, r4
	callar		r4

	/* Destroy the stack frame */

	pop {dw}	retreg

	/* Execute the SYS_syscall_return SVCall (will not return) */
						/* Save return value in r2 */
	mov		a0, a2			/* will restore in up_svcall */
	mov		#SYS_syscall_return, a0
	trap

	.func_end	3 _up_svcall_handler

#else	/* Add dummy symbol to avoid cofflib crash */

	.text
dummy_svcall_handler:

#endif /* CONFIG_LIB_SYSCALL */
