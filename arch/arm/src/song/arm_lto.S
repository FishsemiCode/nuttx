/****************************************************************************
 * arch/arm/src/song/arm_lto.S
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

	.syntax		unified
	.thumb
	.file		"arm_lto.S"
	.globl		__aeabi_ldivmod
	.globl		__aeabi_uldivmod

/****************************************************************************
 * Name: arm_lto
 *
 * Description:
 *   GCC in LTO mode interoperates poorly with non-standard libraries that
 *   provide implementations of compiler intrinsics such as memcpy/memset
 *   or the stack protector entry points.
 *
 *   By referencing these functions from a non-LTO object that can be passed
 *   to the linker via the -plugin-opt=-pass-through=-lxxx options, the
 *   intrinsics are included in the link in a way that allows them to be
 *   pruned again if no other references to them exist.
 ****************************************************************************/

	.text
	.thumb_func
	.globl		arm_lto
	.type		arm_lto, function

arm_lto:
	bx		lr
	bl		__aeabi_ldivmod
	bl		__aeabi_uldivmod

	.size		arm_lto, .-arm_lto
	.end
