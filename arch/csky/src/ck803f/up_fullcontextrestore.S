/**************************************************************************
 * arch/csky/src/ck803f/up_fullcontextrestore.S
 *
 *   Copyright (C) 2007, 2009-2010 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
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
 **************************************************************************/

/**************************************************************************
 * Included Files
 **************************************************************************/

#include <nuttx/irq.h>
#include <up_internal.h>

/**************************************************************************
 * Pre-processor Definitions
 **************************************************************************/

/**************************************************************************
 * Private Types
 **************************************************************************/

/**************************************************************************
 * Private Function Prototypes
 **************************************************************************/

/**************************************************************************
 * Public Data
 **************************************************************************/

/**************************************************************************
 * Private Variables
 **************************************************************************/

/**************************************************************************
 * Private Functions
 **************************************************************************/

/**************************************************************************
 * Public Functions
 **************************************************************************/

/**************************************************************************
 * Name: up_fullcontextrestore
 **************************************************************************/

	.globl	up_fullcontextrestore
	.type	up_fullcontextrestore, function
up_fullcontextrestore:

	/* On entry, a0 (r0) holds address of the register save area */

	/* Recover all registers except for r0, r1, r15, and PSR */

	addi    r1, r0, (4*REG_R2) /* Offset to REG_R2 storage */
	ldm     r2-r15, (r1)
	addi    r1, r0, (4*REG_VR0)
	fldms   vr0-vr15, (r1)

	/* Create a stack frame to hold the PC */

	subi    sp, sp, (3*4)          /* Frame for three registers */
	ldw     r1, (r0, (4*REG_R0))   /* Fetch the stored r0 value */
	stw     r1, (sp)               /* Save it at the top of the stack */
	ldw     r1, (r0, (4*REG_R1))   /* Fetch the stored r1 value */
	stw     r1, (sp, 4)            /* Save it in the stack */
	ldw     r1, (r0, (4*REG_PC))   /* Fetch the stored pc value */
	stw     r1, (sp, 8)            /* Save it at the bottom of the frame */

	/* Now we can restore the PSR.  We wait until we are completely
	 * finished with the context save data to do this. Restore the CPSR
	 * may re-enable and interrupts and we could be in a context
	 * where the save structure is only protected by interrupts being
	 * disabled.
	 */

	ldw     r1, (r0, (4*REG_PSR)) /* Fetch the stored PSR value */
	mtcr    r1, epsr              /* Set the PSR */
	ldw     r1, (sp, 8)
	mtcr    r1, epc


	/* Now recover r0 and r1 */

	ldw     r0, (sp)
	ldw     r1, (sp, 4)
	addi    sp, sp, (3*4)
	rte

	.size up_fullcontextrestore, . - up_fullcontextrestore

