/****************************************************************************
 * arch/risc-v/src/song/song_vectors.S
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

/****************************************************************************
 * Public Symbols
 ****************************************************************************/

	.file		"song_vectors.S"
	.globl		__start
	.globl		irq_handler

/****************************************************************************
 * Name: _vectors
 ****************************************************************************/

	.section	.vectors
	.option		norvc
	.globl		_vectors
	.type		_vectors, function

_vectors:
	j		irq_handler	/* Interrupt 0 */
	j		irq_handler	/* Interrupt 1 */
	j		irq_handler	/* Interrupt 2 */
	j		irq_handler	/* Interrupt 3 */
	j		irq_handler	/* Interrupt 4 */
	j		irq_handler	/* Interrupt 5 */
	j		irq_handler	/* Interrupt 6 */
	j		irq_handler	/* Interrupt 7 */
	j		irq_handler	/* Interrupt 8 */
	j		irq_handler	/* Interrupt 9 */
	j		irq_handler	/* Interrupt 10 */
	j		irq_handler	/* Interrupt 11 */
	j		irq_handler	/* Interrupt 12 */
	j		irq_handler	/* Interrupt 13 */
	j		irq_handler	/* Interrupt 14 */
	j		irq_handler	/* Interrupt 15 */
	j		irq_handler	/* Interrupt 16 */
	j		irq_handler	/* Interrupt 17 */
	j		irq_handler	/* Interrupt 18 */
	j		irq_handler	/* Interrupt 19 */
	j		irq_handler	/* Interrupt 20 */
	j		irq_handler	/* Interrupt 21 */
	j		irq_handler	/* Interrupt 22 */
	j		irq_handler	/* Interrupt 23 */
	j		irq_handler	/* Interrupt 24 */
	j		irq_handler	/* Interrupt 25 */
	j		irq_handler	/* Interrupt 26 */
	j		irq_handler	/* Interrupt 27 */
	j		irq_handler	/* Interrupt 28 */
	j		irq_handler	/* Interrupt 29 */
	j		irq_handler	/* Interrupt 30 */
	j		irq_handler	/* Interrupt 31 */
	j		__start		/* Reset */
	j		irq_handler	/* Illegal Instruction */
	j		irq_handler	/* ECALL */

	.size		_vectors, .-_vectors
