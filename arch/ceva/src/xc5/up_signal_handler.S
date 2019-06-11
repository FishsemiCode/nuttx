/****************************************************************************
 * arch/ceva/src/xc5/up_signal_handler.S
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

#if (defined(CONFIG_BUILD_PROTECTED) && !defined(__KERNEL__)) && \
    !defined(CONFIG_DISABLE_SIGNALS)

/****************************************************************************
 * File info
 ****************************************************************************/

	.file		"up_signal_handler.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: up_signal_handler
 *
 * Description:
 *   This function is the user-space, signal handler trampoline function.  It
 *   is called from up_signal_dispatch() in user-mode.
 *
 * Input Parameters:
 *   r0 = sighand
 *     The address user-space signal handling function
 *   r1, r2, r3 = signo, info, and ucontext
 *     Standard arguments to be passed to the signal handling function.
 *
 * Returned Value:
 *   None.  This function does not return in the normal sense.  It returns
 *   via the SYS_signal_handler_return (see svcall.h)
 *
 ****************************************************************************/

	.text
	.public		_up_signal_handler
	.func_start	3 _up_signal_handler

_up_signal_handler:

	/* Save some register */

	push		retreg.ui		/* Save LR on the stack */
	push {dw}	retreg
	/* Call the signal handler */

	mov		a0, a4			/* r4=sighand */
	mov		a1, a0			/* r0=signo */
	mov		a2, a1			/* r1=info */
	mov		a3, a2			/* r2=ucontext */
	mov		a4, r4
	nop
	nop
	nop
	callar		r4			/* Call the signal handler */

	/* Restore the registers */

	pop {dw}	retreg

	/* Execute the SYS_signal_handler_return SVCall (will not return) */

	mov		#SYS_signal_handler_return, a0
	trap

	.func_end	3 _up_signal_handler

#else	/* Add dummy symbol to avoid cofflib crash */

	.text
dummy_signal_handler:

#endif
