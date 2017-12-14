/****************************************************************************
 * arch/ceva/src/xm6/up_signal_handler.S
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
 * Inputs:
 *   r0 = sighand
 *     The address user-space signal handling function
 *   r1, r2, r3 = signo, info, and ucontext
 *     Standard arguments to be passed to the signal handling function.
 *
 * Return:
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

	/* Call the signal handler */

	mov		r0.ui, r4.ui		/* r4=sighand */
	mov		r1.ui, r0.ui		/* r0=signo */
	mov		r2.ui, r1.ui		/* r1=info */
	mov		r3.ui, r2.ui		/* r2=ucontext */
	nop		#0x03
	callar		r4.ui			/* Call the signal handler */

	/* Restore the registers */

	pop		retreg.ui

	/* Execute the SYS_signal_handler_return SVCall (will not return) */

	mov		#SYS_signal_handler_return, r0.ui
	trap		{t0}

	.func_end	3 _up_signal_handler

#else	/* Add dummy symbol to avoid cofflib crash */

	.text
dummy_signal_handler:

#endif
