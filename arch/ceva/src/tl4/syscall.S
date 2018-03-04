/****************************************************************************
 * arch/ceva/src/tl4/syscall.S
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
 * Public Symbols
 ****************************************************************************/

	.file		"syscall.S"

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sys_callx
 *
 * Description:
 *   This function generate the trap exception with the specified number of
 *   parameters.
 *
 *     a0 = SYS_ call number
 *     a1 = parm0
 *     b0 = parm1
 *     b1 = parm2
 *     r0 = parm3(pop from stack)
 *     r1 = parm4(pop from stack)
 *     r2 = parm5(pop from stack)
 *
 ****************************************************************************/

	.text

	.public		_sys_call0
	.public		_sys_call1
	.public		_sys_call2
	.public		_sys_call3

	.func_start	3 _sys_call0

_sys_call0:
_sys_call1:
_sys_call2:
_sys_call3:
	trap0
	ret

	.func_end	3 _sys_call0

	.public		_sys_call4
	.func_start	3 _sys_call4

_sys_call4:
	popdw		r0
	trap0
	ret

	.func_end	3 _sys_call4

	.public		_sys_call5
	.func_start	3 _sys_call5

_sys_call5:
	popdw		r0
	popdw		r1
	trap0
	ret

	.func_end	3 _sys_call5

	.public		_sys_call6
	.func_start	3 _sys_call6

_sys_call6:
	popdw		r0
	popdw		r1
	popdw		r2
	trap0
	ret

	.func_end	3 _sys_call6
