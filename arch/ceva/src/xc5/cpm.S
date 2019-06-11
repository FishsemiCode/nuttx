/****************************************************************************
 * arch/ceva/src/xc5/cpm.S
 *
 *   Copyright (C) 2019 FishSemi Inc. All rights reserved.
 *   Author: Bo Zhang <zhangbo@fishsemi.net>
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

	.file		"cpm.S"

	.text
	.public		_getcpm
	.func_start	2 _getcpm
_getcpm:
	push {dw}	r0
	mov		a0, r0
	nop
	in {dw,cpm}	(r0), r0
	nop
	nop
	nop
	mov		r0, a0
	pop {dw}	r0
	ret
	.func_end	2 _getcpm

	.public		_putcpm
	.func_start	2 _putcpm
_putcpm:
	push {dw}	r0
	push {dw}	r1
	mov		a0, r0
	mov		a1, r1
	nop
	out {dw,cpm}	r1, (r0)
	pop {dw}	r1
	pop {dw}	r0
	ret
	.func_end	2 _putcpm

	.public		_modifycpm
	.func_start	2 _modifycpm
_modifycpm:
	push {dw}	r0
	push {dw}	r1
	mov		a0, r0
	nop
	in {dw,cpm}	(r0), r1
	nop
	nop
	nop
	mov		r1, a0
	not		a1, a1
	and		a0, a1, a0
	or		a0, a2, a0
	nop
	mov		a0, r1
	nop
	out {dw,cpm}	r1, (r0)
	pop {dw}	r1
	pop {dw}	r0
	ret
	.func_end	2 _modifycpm
